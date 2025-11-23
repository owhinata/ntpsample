// Copyright (c) 2025 The NTP Sample Authors
using NtpClock;
using NtpServer;
using NtpServer.Internal;

// Parse command-line arguments
string upstreamIp = "127.0.0.1";
ushort upstreamPort = 9123;
ushort servePort = 9124;
bool debug = false;

var builder = new NtpClock.Internal.Options.Builder();

for (int i = 0; i < args.Length; i++)
{
    string arg = args[i];
    switch (arg)
    {
        case "--upstream-ip" when i + 1 < args.Length:
            upstreamIp = args[++i];
            break;
        case "--upstream-port" when i + 1 < args.Length:
            upstreamPort = ushort.Parse(args[++i]);
            break;
        case "--serve-port" when i + 1 < args.Length:
            servePort = ushort.Parse(args[++i]);
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
        case "--min-samples" when i + 1 < args.Length:
            builder.MinSamplesToLock(int.Parse(args[++i]));
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
            return 1;
    }
}

// Create logger
var logger = new Logger(debug);
var logCallback = new Action<string>(msg => logger.Log(msg));

builder.LogSink(msg => logger.Log(msg));
var opts = builder.Build();

// Also use logger for NtpServer
var serverOpts = new NtpServer.Internal.Options.Builder()
    .Stratum(2)
    .LogSink(msg => logger.Log(msg))
    .Build();

// Setup signal handler for graceful shutdown
bool running = true;
Console.CancelKeyPress += (sender, e) =>
{
    e.Cancel = true;
    running = false;
};

// Create shared TimeSource for both ClockService and NtpServer
// This allows vendor hints (rate/abs) to propagate from upstream to downstream
var timeSource = new StopwatchClock();

// Create ClockService to sync with upstream server
using var clockService = new ClockService();
if (!clockService.Start(timeSource, upstreamIp, upstreamPort, opts))
{
    Console.Error.WriteLine("Failed to start ClockService");
    return 1;
}
Console.WriteLine($"ClockService started, syncing with {upstreamIp}:{upstreamPort}");

// Wait a bit for initial synchronization
Thread.Sleep(500);

// Create NtpServer to serve downstream clients using the same TimeSource
using var server = new NtpServer.NtpServer();
if (!server.Start(servePort, timeSource, serverOpts))
{
    Console.Error.WriteLine($"Failed to start NtpServer on port {servePort}");
    clockService.Stop();
    return 1;
}
Console.WriteLine($"NtpServer started on UDP port {servePort} (stratum 2)");
Console.WriteLine("Press Ctrl+C to stop...\n");

// Main status display loop with change detection
TimeSpec lastUpdate = new TimeSpec();
uint lastEpoch = 0;

while (running)
{
    Thread.Sleep(100);

    var st = clockService.GetStatus();

    // Detect ClockService updates and propagate to downstream clients
    if (st.LastUpdate.ToDouble() > 0.0 && st.LastUpdate != lastUpdate)
    {
        lastUpdate = st.LastUpdate;

        // Restart server when upstream server epoch changes
        if (st.Epoch != 0 && st.Epoch != lastEpoch)
        {
            lastEpoch = st.Epoch;
            server.Stop();
            // timeSource is already updated by ClockService
            if (!server.Start(servePort, timeSource, serverOpts))
            {
                Console.Error.WriteLine("\nFailed to restart NtpServer");
                clockService.Stop();
                return 1;
            }
            logger.Log($"[Gateway] NtpServer restarted due to upstream epoch change: {st.Epoch}");
        }
    }

    Console.Write(
        $"\r[Gateway Status] sync={st.Synchronized, -3} rtt={st.RttMs, 3}ms offset={st.OffsetS, 9:F6}s samples={st.Samples, 3}   "
    );
}

Console.WriteLine("\n\nStopping gateway...");
server.Stop();
clockService.Stop();
Console.WriteLine("Gateway stopped.");

return 0;

// Local functions
static void PrintUsage()
{
    Console.Error.WriteLine(
        "Usage: ntpclock_gateway [options]\n"
            + "Options:\n"
            + "  --upstream-ip IP     Upstream NTP server IP (default 127.0.0.1)\n"
            + "  --upstream-port N    Upstream NTP server port (default 9123)\n"
            + "  --serve-port N       Port to serve on (default 9124)\n"
            + "  --poll ms            Polling interval in ms (default 10000)\n"
            + "  --step ms            Step threshold in ms (default 200)\n"
            + "  --slew ms_per_s      Slew rate in ms/s (default 5.0)\n"
            + "  --min-samples n      Min samples to lock (default 3)\n"
            + "  --debug              Enable debug logging"
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
