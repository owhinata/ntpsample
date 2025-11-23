// Copyright (c) 2025 <Your Name>
using System.Text;
using NtpServer;
using NtpServer.Internal;

// Parse command-line arguments
ushort port = 9123;
double initRate = 1.0;
double initAbs = 0.0; // 0: do not set
bool debug = false;

for (int i = 0; i < args.Length; i++)
{
    string arg = args[i];
    switch (arg)
    {
        case "--port" when i + 1 < args.Length:
            port = ushort.Parse(args[++i]);
            break;
        case "--rate" when i + 1 < args.Length:
            initRate = double.Parse(args[++i]);
            break;
        case "--abs" when i + 1 < args.Length:
            initAbs = double.Parse(args[++i]);
            break;
        case "--debug":
            debug = true;
            break;
        case "-h":
        case "--help":
            PrintUsage();
            return 0;
        default:
            Console.Error.WriteLine($"Unknown option: {arg}");
            PrintUsage();
            return 2;
    }
}

// Create logger
var logger = new Logger(debug);

// Create time source (using StopwatchClock)
var timeSource = new StopwatchClock();
timeSource.SetRate(initRate);
if (initAbs != 0.0)
{
    timeSource.SetAbsolute(TimeSpec.FromDouble(initAbs));
}

// Build options with logger
var opts = new NtpServer.Internal.Options.Builder().LogSink(msg => logger.Log(msg)).Build();

// Create and start server
using var server = new NtpServer.NtpServer();
if (!server.Start(port, timeSource, opts))
{
    Console.Error.WriteLine("failed to start ntp server");
    return 1;
}

Console.WriteLine($"ntp server running on UDP {port}");
Console.WriteLine("stdin commands: help | now | rate R | abs SEC | add SEC | reset | quit");

// Control loop on stdin
string? line;
while ((line = Console.ReadLine()) != null)
{
    line = line.Trim();
    if (string.IsNullOrEmpty(line))
        continue;

    if (line == "help")
    {
        PrintUsage();
        continue;
    }

    if (line == "quit" || line == "exit")
    {
        break;
    }

    if (line == "now")
    {
        double now = timeSource.NowUnix().ToDouble();
        Console.WriteLine($"now={now:F6}");
        continue;
    }

    if (line.StartsWith("rate "))
    {
        double r = double.Parse(line.Substring(5));
        double before = timeSource.NowUnix().ToDouble();

        // Stop -> change -> Start
        server.Stop();
        timeSource.SetRate(r);
        if (!server.Start(port, timeSource, opts))
        {
            Console.Error.WriteLine("Failed to restart server");
            return 1;
        }

        double after = timeSource.NowUnix().ToDouble();
        logger.Log($"Server restarted with rate={r} before={before} after={after}");
        continue;
    }

    if (line.StartsWith("abs "))
    {
        double t = double.Parse(line.Substring(4));
        double before = timeSource.NowUnix().ToDouble();

        // Stop -> change -> Start
        server.Stop();
        timeSource.SetAbsolute(TimeSpec.FromDouble(t));
        if (!server.Start(port, timeSource, opts))
        {
            Console.Error.WriteLine("Failed to restart server");
            return 1;
        }

        double after = timeSource.NowUnix().ToDouble();
        logger.Log($"Server restarted with absolute={t} before={before} after={after}");
        continue;
    }

    if (line.StartsWith("add "))
    {
        double d = double.Parse(line.Substring(4));
        double before = timeSource.NowUnix().ToDouble();

        // Stop -> change -> Start
        server.Stop();
        // Use absolute update to avoid cumulative rounding and ensure exact step
        timeSource.SetAbsolute(TimeSpec.FromDouble(before + d));
        if (!server.Start(port, timeSource, opts))
        {
            Console.Error.WriteLine("Failed to restart server");
            return 1;
        }

        double after = timeSource.NowUnix().ToDouble();
        string sign = d >= 0 ? "+" : "";
        logger.Log(
            $"Server restarted with offset {sign}{d} before={before} after={after} delta={after - before}"
        );
        continue;
    }

    if (line == "reset")
    {
        double before = timeSource.NowUnix().ToDouble();
        double rateBefore = timeSource.GetRate();

        // Stop -> change -> Start
        server.Stop();
        timeSource.ResetToRealTime();
        if (!server.Start(port, timeSource, opts))
        {
            Console.Error.WriteLine("Failed to restart server");
            return 1;
        }

        double after = timeSource.NowUnix().ToDouble();
        logger.Log(
            $"Server restarted (reset to real-time) before={before} (rate={rateBefore}) after={after} (rate=1.0)"
        );
        continue;
    }

    Console.Error.WriteLine($"unknown command: {line}");
}

server.Stop();
return 0;

// Local functions
static void PrintUsage()
{
    Console.Error.WriteLine(
        "Usage: ntpserver_example [--port N] [--rate R] [--abs SEC] [--debug]\n"
            + "       Commands on stdin: help | now | rate R | abs SEC | add SEC | reset | quit"
    );
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
