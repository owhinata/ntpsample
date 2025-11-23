# NTP Implementation for .NET

A high-performance, production-ready NTP (Network Time Protocol) server and client implementation in C# (.NET 8.0), ported from the C++ reference implementation with full feature parity.

## Features

- ✅ **NTPv4 Protocol** - Full RFC 5905 compliant implementation
- ✅ **Vendor Extensions** - Custom ABS/RATE/PUSH notification support
- ✅ **High Precision** - Microsecond-level time synchronization
- ✅ **Cross-Platform** - Runs on Windows, Linux, and macOS
- ✅ **Production Ready** - Extensively tested with 228 unit tests
- ✅ **C++ Compatible** - 100% interoperable with C++ reference implementation
- ✅ **High Performance** - Only 3.5% slower than native C++ version

## Quick Start

### Prerequisites

- .NET 8.0 SDK or later
- Linux, macOS, or Windows

### Build

```bash
# Build all projects
dotnet build

# Run tests
dotnet test
```

### Run NTP Server

```bash
# Basic usage (default port 9123)
dotnet run --project NtpServerExample

# Custom port
dotnet run --project NtpServerExample -- --port 9123

# With debug logging
dotnet run --project NtpServerExample -- --port 9123 --debug
```

**Server Commands (stdin):**
```
help          - Show available commands
now           - Display current server time
abs <SEC>     - Set absolute time (Unix seconds)
add <SEC>     - Add offset to current time (seconds)
rate <RATE>   - Set time progression rate (1.0 = normal)
reset         - Reset to real system time
quit          - Stop server
```

### Run NTP Client

```bash
# Sync with server (default: localhost:9123)
dotnet run --project NtpClockExample

# Custom server
dotnet run --project NtpClockExample -- --server 192.168.1.100 --port 9123

# With debug logging
dotnet run --project NtpClockExample -- --debug
```

### Run Gateway (Client + Server)

```bash
# Gateway mode: sync from upstream and serve to downstream clients
dotnet run --project NtpClockGateway -- \
  --upstream-server 192.168.1.100 \
  --upstream-port 9123 \
  --server-port 9124
```

## Project Structure

```
csharp/
├── NtpServer/              # Server library
│   ├── NtpServer.cs        # Public API
│   └── Internal/           # Internal implementation
├── NtpClock/               # Client library
│   ├── ClockService.cs     # Public API
│   └── Internal/           # Internal implementation
├── NtpServer.Tests/        # Server unit tests (68 tests)
├── NtpClock.Tests/         # Client unit tests (160 tests)
├── NtpServerExample/       # Server sample application
├── NtpClockExample/        # Client sample application
└── NtpClockGateway/        # Gateway sample application
```

## Library Usage

### NTP Server

```csharp
using NtpServer;
using NtpServer.Internal;

// Create server with options
var options = new Options.Builder()
    .Stratum(1)
    .RefId(RefIdHelper.MakeRefId("LOCL"))
    .LogSink(msg => Console.WriteLine(msg))
    .Build();

var server = new NtpServer.NtpServer();
server.Start(port: 9123, options: options);

// Control time
var clock = new StopwatchClock();
clock.SetAbsolute(TimeSpec.FromDouble(1234567890.0));
server.Start(port: 9123, timeSource: clock, options: options);

// Stop server
server.Stop();
```

### NTP Client

```csharp
using NtpClock;
using NtpClock.Internal;

// Create client with options
var options = new Options.Builder()
    .ServerEndpoint(new IPEndPoint(IPAddress.Loopback, 9123))
    .PollIntervalMs(10000)
    .LogSink(msg => Console.WriteLine(msg))
    .Build();

var client = new ClockService();
client.Start(options);

// Get current synchronized time
TimeSpec now = client.NowUnix();
Console.WriteLine($"Current time: {now.Sec}.{now.Nsec:D9}");

// Get synchronization status
var status = client.GetStatus();
Console.WriteLine($"Synchronized: {status.Synchronized}");
Console.WriteLine($"Offset: {status.OffsetS:F6} seconds");
Console.WriteLine($"RTT: {status.RttMs:F3} ms");

// Stop client
client.Stop();
```

## Performance

Benchmarked against the C++ reference implementation:

| Metric | C++ | C# | Difference |
|--------|-----|----|-----------|
| **Throughput** | 581 req/s | 560 req/s | -3.5% ✅ |
| **Memory (RSS)** | 2.84 MB | 169.7 MB | +167 MB* |
| **Latency** | ~0.5 ms | ~0.5 ms | ≈0% ✅ |

*Includes .NET runtime overhead (~160 MB), acceptable for server applications

For detailed benchmarks, see [../BENCHMARK.md](../BENCHMARK.md).

## Testing

```bash
# Run all tests
dotnet test

# Run specific test project
dotnet test NtpServer.Tests/
dotnet test NtpClock.Tests/

# Run with coverage
dotnet test --collect:"XPlat Code Coverage"
```

**Test Results:**
- ✅ 68 NtpServer tests (100% pass)
- ✅ 160 NtpClock tests (100% pass)
- ✅ 77.7% line coverage (core logic: 100%)

## Configuration

### Server Options

```csharp
var options = new Options.Builder()
    .Stratum(1)                          // Stratum level (default: 1)
    .Precision(-20)                      // Clock precision (default: -20)
    .RefId(0x4C4F434C)                   // Reference ID "LOCL"
    .ClientRetention(TimeSpan.FromMinutes(60))  // Client tracking time
    .LogSink(msg => Console.WriteLine(msg))     // Log callback
    .Build();
```

### Client Options

```csharp
var options = new Options.Builder()
    .ServerEndpoint(new IPEndPoint(IPAddress.Parse("192.168.1.100"), 9123))
    .PollIntervalMs(10000)               // Polling interval (default: 10s)
    .StepThresholdMs(200)                // Step vs slew threshold (default: 200ms)
    .SlewRateMsPerSec(5.0)               // Slew rate (default: 5 ms/s)
    .MaxRttMs(100)                       // Max acceptable RTT (default: 100ms)
    .MinSamplesToLock(3)                 // Min samples for lock (default: 3)
    .OffsetWindow(5)                     // Offset window size (default: 5)
    .SkewWindow(10)                      // Skew window size (default: 10)
    .LogSink(msg => Console.WriteLine(msg))
    .Build();
```

## Architecture

### Key Components

**NtpServer:**
- `NtpServer` - Main server class (UDP listener, packet processing)
- `StopwatchClock` - Adjustable time source with rate control
- `ClientTracker` - Client endpoint tracking for push notifications
- `NtpVendorExt` - Vendor extension field handling

**NtpClock:**
- `ClockService` - Main synchronization service
- `SyncEstimatorState` - Offset/skew estimation with OLS regression
- `ClockCorrector` - Time correction (slew vs step decision)
- `VendorHintProcessor` - Vendor hint processing and epoch detection
- `UdpSocket` - UDP communication and message classification

### Vendor Extensions

Custom NTP extension fields (Type: 0xFF01) support:

- **ABS** - Absolute time setting
- **RATE** - Time progression rate adjustment
- **PUSH** - Server-initiated client notifications
- **Epoch tracking** - Server restart detection

## Interoperability

Fully compatible with the C++ reference implementation:

```bash
# C++ Server ↔ C# Client
./build/ntpserver/ntpserver_example --port 9123 &
dotnet run --project NtpClockExample

# C# Server ↔ C++ Client
dotnet run --project NtpServerExample -- --port 9123 &
./build/ntpclock/ntpclock_example --server localhost --port 9123

# 3-tier: C# Server → C# Gateway → C# Client
dotnet run --project NtpServerExample -- --port 9123 &
dotnet run --project NtpClockGateway -- --upstream-port 9123 --server-port 9124 &
dotnet run --project NtpClockExample -- --port 9124
```

## Migration from C++

This is a faithful port of the C++ implementation with:

- ✅ Same algorithms and logic
- ✅ Same configuration defaults
- ✅ Same protocol behavior
- ✅ Same vendor extension format
- ✅ Near-identical performance

See [MIGRATION_PLAN.md](MIGRATION_PLAN.md) for detailed migration documentation.

## Troubleshooting

### Port already in use

```bash
# Find process using port
lsof -i :9123

# Kill process
pkill -f NtpServerExample
```

### Permission denied (ports < 1024)

Run with sudo or use ports ≥ 1024:

```bash
dotnet run --project NtpServerExample -- --port 9123  # OK
sudo dotnet run --project NtpServerExample -- --port 123  # Requires sudo
```

### Clock not synchronizing

1. Check server is running: `netstat -an | grep 9123`
2. Enable debug logging: `--debug` flag
3. Check firewall settings
4. Verify network connectivity: `ping <server>`

## Development

### Code Style

- C# 10 features
- Nullable reference types enabled
- LINQ for collections
- Builder pattern for configuration
- IDisposable for resource management

### Adding Tests

```csharp
[TestClass]
public class MyTests
{
    [TestMethod]
    public void TestSomething()
    {
        // Arrange
        var server = new NtpServer.NtpServer();

        // Act
        bool started = server.Start(9123);

        // Assert
        Assert.IsTrue(started);

        // Cleanup
        server.Stop();
    }
}
```

## License

See [../LICENSE](../LICENSE) for details.

## References

- RFC 5905 - Network Time Protocol Version 4
- [C++ Reference Implementation](../README.md)
- [Migration Plan](MIGRATION_PLAN.md)
- [Benchmark Results](../BENCHMARK.md)

## Support

For issues and questions:
- Open an issue on GitHub
- See [MIGRATION_PLAN.md](MIGRATION_PLAN.md) for implementation details
- Check [../BENCHMARK.md](../BENCHMARK.md) for performance data
