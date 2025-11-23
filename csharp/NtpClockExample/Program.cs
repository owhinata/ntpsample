// Copyright (c) 2025 <Your Name>
using System.Runtime.InteropServices;
using NtpClock;
using NtpClock.Internal;

// Parse command-line arguments
string ip = "127.0.0.1";
ushort port = 9123;
bool optUtc = false; // default: JST
bool debug = false;

var builder = new NtpClock.Internal.Options.Builder();

for (int i = 0; i < args.Length; i++)
{
    string arg = args[i];
    switch (arg)
    {
        case "--ip" when i + 1 < args.Length:
            ip = args[++i];
            break;
        case "--port" when i + 1 < args.Length:
            port = ushort.Parse(args[++i]);
            break;
        case "--poll" when i + 1 < args.Length:
            builder.PollIntervalMs(int.Parse(args[++i]));
            break;
        case "--step" when i + 1 < args.Length:
            builder.StepThresholdMs(int.Parse(args[++i]));
            break;
        case "--slew" when i + 1 < args.Length:
            builder.SlewRateMsPerSec(double.Parse(args[++i]));
            break;
        case "--max-rtt" when i + 1 < args.Length:
            builder.MaxRttMs(int.Parse(args[++i]));
            break;
        case "--min-samples" when i + 1 < args.Length:
            builder.MinSamplesToLock(int.Parse(args[++i]));
            break;
        case "--offset-window" when i + 1 < args.Length:
            builder.OffsetWindow(int.Parse(args[++i]));
            break;
        case "--skew-window" when i + 1 < args.Length:
            builder.SkewWindow(int.Parse(args[++i]));
            break;
        case "--utc":
            optUtc = true;
            break;
        case "--debug":
            debug = true;
            break;
        case "-h":
        case "--help":
            PrintUsage();
            return 0;
        default:
            Console.Error.WriteLine($"Unknown or incomplete option: {arg}");
            PrintUsage();
            return 2;
    }
}

// Create logger
var logger = new Logger(debug);
builder.LogSink(msg => logger.Log(msg));

using var svc = new ClockService();
var opt = builder.Build();

Console.WriteLine($"Starting sync to {ip}:{port}");
EnableVirtualTerminal();
HideCursor();

// Setup Ctrl+C handler to restore cursor
Console.CancelKeyPress += (sender, e) =>
{
    ShowCursor();
    Environment.Exit(0);
};

if (!svc.Start(ip, port, opt))
{
    Console.Error.WriteLine("Failed to start ClockService");
    return 1;
}

// Double-buffer: update only changed lines to reduce flicker
var prev = new string[11];
for (int i = 0; i < prev.Length; i++)
    prev[i] = "";

Console.Write("\x1b[H"); // Move to home position

while (true)
{
    int cols = GetTerminalWidth();
    var st = svc.GetStatus();
    double nowS = svc.NowUnix().ToDouble();

    // Format current time
    string nowLine = FormatTime(nowS, optUtc);

    // Format error message (single line, truncated if needed)
    string err = string.IsNullOrEmpty(st.LastError) ? "" : OneLine(st.LastError);
    int cap = Math.Max(0, cols - 7);
    if (err.Length > cap)
        err = err.Substring(0, cap);

    // Build current display lines
    var cur = new string[11];
    cur[0] = nowLine;
    cur[1] = $"sync={st.Synchronized}";
    cur[2] = $"rtt_ms={st.RttMs}  delay_s={st.LastDelayS:F3}";
    cur[3] = $"offset_s={st.OffsetS:F6}  skew_ppm={st.SkewPpm:F1}";
    cur[4] = $"samples={st.Samples}  last_update={st.LastUpdate.ToDouble():F3}";

    string corrStr =
        st.LastCorrection == NtpClock.Internal.Status.CorrectionType.Step
            ? "Step"
            : (st.LastCorrection == NtpClock.Internal.Status.CorrectionType.Slew ? "Slew" : "None");
    cur[5] = $"last_corr={corrStr}  amount={st.LastCorrectionAmountS:F6}";
    cur[6] =
        $"offset_window={st.OffsetWindow}  skew_window={st.SkewWindow}  window_count={st.WindowCount}";
    cur[7] = $"median={st.OffsetMedianS:F6}  min={st.OffsetMinS:F6}  max={st.OffsetMaxS:F6}";
    cur[8] = $"applied={st.OffsetAppliedS:F6}  target={st.OffsetTargetS:F6}";
    cur[9] = $"error={err}";
    cur[10] = "";

    // Truncate lines to terminal width
    for (int i = 0; i < cur.Length; i++)
    {
        if (cur[i].Length > cols)
            cur[i] = cur[i].Substring(0, cols);
    }

    // Update only changed lines
    for (int i = 0; i < cur.Length; i++)
    {
        if (prev[i] != cur[i])
        {
            MoveToRow(i + 1);
            Console.Write("\x1b[2K"); // Clear line
            Console.Write(cur[i]);
            Console.WriteLine();
        }
    }

    Console.Out.Flush();
    Array.Copy(cur, prev, cur.Length);

    Thread.Sleep(100);
}

// Local functions
static void PrintUsage()
{
    Console.Error.WriteLine(
        "Usage: ntpclock_example --ip A.B.C.D --port N [options]\n"
            + "Options:\n"
            + "  --ip A.B.C.D         (default 127.0.0.1)\n"
            + "  --port N             (default 9123)\n"
            + "  --poll ms            (default 10000)\n"
            + "  --step ms            (default 200)\n"
            + "  --slew ms_per_s      (default 5.0)\n"
            + "  --max-rtt ms         (default 100)\n"
            + "  --min-samples n      (default 3)\n"
            + "  --offset-window n    (default 5)\n"
            + "  --skew-window n      (default 10)\n"
            + "  --utc                (default: JST)\n"
            + "  --debug              Enable debug logging"
    );
}

static string OneLine(string s)
{
    var sb = new System.Text.StringBuilder(s.Length);
    foreach (char c in s)
    {
        if (c == '\n' || c == '\r')
            sb.Append(' ');
        else
            sb.Append(c);
    }
    return sb.ToString();
}

static void HideCursor()
{
    Console.Write("\x1b[?25l");
}

static void ShowCursor()
{
    Console.Write("\x1b[?25h");
    Console.Out.Flush();
}

static void MoveToRow(int row)
{
    Console.Write($"\x1b[{row};1H");
}

static void EnableVirtualTerminal()
{
    if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
    {
        // Enable ANSI escape sequences on Windows
        var handle = NativeMethods.GetStdHandle(-11); // STD_OUTPUT_HANDLE
        if (handle != IntPtr.Zero)
        {
            NativeMethods.GetConsoleMode(handle, out uint mode);
            mode |= 0x0004; // ENABLE_VIRTUAL_TERMINAL_PROCESSING
            NativeMethods.SetConsoleMode(handle, mode);
        }
    }
}

static int GetTerminalWidth()
{
    if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
    {
        try
        {
            var handle = NativeMethods.GetStdHandle(-11); // STD_OUTPUT_HANDLE
            if (
                handle != IntPtr.Zero
                && NativeMethods.GetConsoleScreenBufferInfo(handle, out var info)
            )
            {
                return info.srWindow.Right - info.srWindow.Left + 1;
            }
        }
        catch
        {
            // Fallback
        }
    }
    return 120; // Default width
}

static string FormatTime(double nowS, bool utc)
{
    int offset = utc ? 0 : 9 * 3600; // JST = UTC+9
    string tzLabel = utc ? "UTC" : "JST";

    long isec = (long)nowS;
    long adj = isec + offset;

    var dt = DateTimeOffset.FromUnixTimeSeconds(adj).UtcDateTime;
    double frac = nowS - isec;
    if (frac < 0)
        frac = 0;
    int ms = (int)(frac * 1000.0 + 0.5);
    if (ms >= 1000)
        ms -= 1000;

    return $"now={dt:yyyy-MM-dd HH:mm:ss}.{ms:D3} {tzLabel}";
}

// Thread-safe logger for debug messages
class Logger
{
    private readonly bool _enabled;
    private readonly object _lock = new();

    public Logger(bool enabled)
    {
        _enabled = enabled;
    }

    public void Log(string msg)
    {
        if (!_enabled)
            return;

        lock (_lock)
        {
            Console.Error.WriteLine(msg);
        }
    }
}

// Windows console API interop - must be in a class for P/Invoke
static partial class NativeMethods
{
    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern IntPtr GetStdHandle(int nStdHandle);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool GetConsoleMode(IntPtr hConsoleHandle, out uint lpMode);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool SetConsoleMode(IntPtr hConsoleHandle, uint dwMode);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern bool GetConsoleScreenBufferInfo(
        IntPtr hConsoleOutput,
        out CONSOLE_SCREEN_BUFFER_INFO lpConsoleScreenBufferInfo
    );

    [StructLayout(LayoutKind.Sequential)]
    public struct COORD
    {
        public short X;
        public short Y;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct SMALL_RECT
    {
        public short Left;
        public short Top;
        public short Right;
        public short Bottom;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct CONSOLE_SCREEN_BUFFER_INFO
    {
        public COORD dwSize;
        public COORD dwCursorPosition;
        public ushort wAttributes;
        public SMALL_RECT srWindow;
        public COORD dwMaximumWindowSize;
    }
}
