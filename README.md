# NTP Sample

A cross-platform high-precision NTP server and client implementation featuring vendor extensions for instant time synchronization and gateway capabilities.

**Supported Platforms**: Windows, Linux, macOS

## Features

- **High-precision time representation**: Platform-independent `TimeSpec` (64-bit seconds + 32-bit nanoseconds)
- **NTP Server** (`ntpserver`): Minimal NTPv4 server with vendor extension support
- **NTP Client** (`ntpclock`): Clock synchronization service with adaptive correction
- **Push notifications**: Instant time change propagation via vendor extension fields
- **Gateway mode**: Multi-tier NTP deployment with upstream sync and downstream serving
- **Adaptive clock correction**: Automatic slew/step selection based on offset magnitude

## Project Structure

```
ntpsample/
├── ntpserver/          # NTP server library
│   ├── include/        # Public headers (TimeSource, NtpServer, TimeSpec)
│   ├── src/            # Server implementation
│   └── example/        # Server example application
├── ntpclock/           # NTP client library
│   ├── include/        # Public headers (ClockService, Options)
│   ├── src/            # Client implementation and sync logic
│   └── example/        # Client and gateway example applications
└── cmake/              # Build configuration
```

## Build Requirements

- CMake 3.20 or higher
- C++17 compatible compiler
  - **Windows**: Visual Studio 2022 or MinGW
  - **Linux**: GCC 7+ or Clang 5+
  - **macOS**: Xcode 10+ (Apple Clang)

## Build Instructions

### Windows (Visual Studio)

```bash
# Configure
cmake -S . -B build -G "Visual Studio 17 2022" -A x64

# Build
cmake --build build --config Release
```

### Linux / macOS

```bash
# Configure
cmake -B build

# Build
cmake --build build -j

# Run tests
ctest --test-dir build --output-on-failure
```

### Build Outputs

**Windows (Release configuration)**:
- Libraries: `build/ntpserver/Release/ntpserver.lib`, `build/ntpclock/Release/ntpclock.lib`
- Executables: `build/ntpserver/Release/ntpserver_example.exe`, etc.

**Linux / macOS**:
- Libraries: `build/ntpserver/libntpserver.a`, `build/ntpclock/libntpclock.a`
- Executables: `build/ntpserver/ntpserver_example`, `build/ntpclock/ntpclock_example`, `build/ntpclock/ntpclock_gateway`

### Running Tests

**Windows**:
```bash
ctest --test-dir build -C Release
```

**Linux / macOS**:
```bash
ctest --test-dir build --output-on-failure
```

### Offline GoogleTest Dependency

Both ntpserver and ntpclock fetch GoogleTest via CMake’s `FetchContent`. In environments without Internet access, pass a pre-downloaded archive to CMake:

```bash
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DNTP_GTEST_ARCHIVE=C:/deps/googletest-1.17.0.tar.gz
```

When `NTP_GTEST_ARCHIVE` is set, the build uses that archive instead of cloning from GitHub.

## Usage Examples

> **Note**: On Windows, executables are in `build/<project>/Release/`. On Linux/macOS, they're in `build/<project>/`.

### 1. Simple NTP Server

Start an NTP server on UDP port 9123:

**Windows**:
```bash
./build/ntpserver/Release/ntpserver_example.exe --port 9123
```

**Linux / macOS**:
```bash
./build/ntpserver/ntpserver_example --port 9123
```

The server uses the platform-appropriate high-resolution clock as the time source by default (`QueryPerformanceCounter` on Windows, `clock_gettime(CLOCK_MONOTONIC)` on POSIX).

### 2. NTP Client

Synchronize with an NTP server:

```bash
./build/ntpclock/Release/ntpclock_example.exe --ip 127.0.0.1 --port 9123 --poll 10000
```

Options:
- `--ip IP`: NTP server IP address (default: 127.0.0.1)
- `--port N`: NTP server port (default: 123)
- `--poll ms`: Polling interval in milliseconds (default: 10000)
- `--step ms`: Step threshold in milliseconds (default: 200)
- `--slew ms_per_s`: Slew rate in ms/second (default: 5.0)

### 3. Multi-tier NTP Gateway

Create an NTP gateway that syncs with an upstream server and serves downstream clients:

```bash
# Start upstream server on port 9123
./build/ntpserver/Release/ntpserver_example.exe --port 9123

# Start gateway: sync from 9123, serve on 9124
./build/ntpclock/Release/ntpclock_gateway.exe --upstream-ip 127.0.0.1 --upstream-port 9123 --serve-port 9124 --poll 10000

# Connect downstream client to gateway
./build/ntpclock/Release/ntpclock_example.exe --ip 127.0.0.1 --port 9124
```

The gateway will:
1. Synchronize with the upstream server using `ClockService`
2. Serve the synchronized time to downstream clients using `NtpServer`
3. Propagate Push notifications from upstream to downstream clients

### 4. Testing Push Notifications

Push notifications enable instant time change propagation without waiting for polling intervals.

In the server console, trigger a time change:
```
add 31536000    # Add one year (31536000 seconds)
```

The server will automatically restart (Stop/Start) and send Push notifications (mode=5) to all known clients. Connected clients detect the new epoch and synchronize immediately, without waiting for the next poll interval.

## Architecture

### Platform Abstraction Layer

The implementation provides cross-platform compatibility through several abstraction layers:

1. **Socket Abstraction** (`ISocket` interface):
   - Windows: Winsock2 (`socket_win32.cc`)
   - POSIX (Linux/macOS): BSD sockets with `poll()` (`socket_posix.cc`)
   - Platform-independent factory: `CreatePlatformSocket()`

2. **Time Source Abstraction**:
   - Windows: `QpcClock` using `QueryPerformanceCounter`
   - POSIX: `MonotonicClock` using `clock_gettime(CLOCK_MONOTONIC)`
   - Platform-independent access: `platform::GetDefaultTimeSource()`

3. **Build System**:
   - CMake automatically detects the platform and compiles the appropriate implementations
   - Conditional compilation for platform-specific headers and system calls

### TimeSpec

High-precision time representation using separate integer fields:
- `int64_t sec`: Seconds since epoch
- `uint32_t nsec`: Nanoseconds (0-999999999)

This avoids precision loss from floating-point arithmetic while supporting the full NTP timestamp range.

### ClockService (NTP Client)

Core synchronization logic:
1. **Exchange protocol**: Client sends request, server responds with timestamps (T1-T4)
2. **Offset calculation**: `offset = ((T2-T1) + (T3-T4)) / 2` using TimeSpec arithmetic
3. **Clock correction**: Adaptive slew (gradual) or step (immediate) based on offset magnitude
4. **Monotonicity enforcement**: Prevents backward time jumps (except after step corrections)
5. **Push support**: Receives vendor extension fields and executes immediate Exchange

### NtpServer

Lightweight NTPv4 server with epoch-based synchronization:
1. **Epoch-based sync**: Each Start() creates a new epoch; clients automatically detect and synchronize
2. **Client tracking**: Maintains list of recently seen client endpoints across restarts
3. **Vendor extensions**: Includes absolute time and rate captured at epoch start
4. **Automatic Push**: Sends Push notifications (mode=5) to known clients on Start()
5. **TimeSource abstraction**: Can serve from any time source (system clock, synchronized clock, etc.)
6. **Persistent statistics**: Server statistics accumulate across Stop/Start cycles

### Gateway Architecture

The gateway combines ClockService and NtpServer:
1. **ClockService** syncs with upstream server and maintains corrected time
2. **TimeSource adapter** exposes ClockService time to NtpServer
3. **Status monitoring** detects corrections and restarts NtpServer (Stop/Start)
4. **Automatic Push** on server restart propagates changes to downstream clients
5. **Epoch increment** ensures all downstream clients detect and synchronize with changes

## Configuration

### ClockService Options

```cpp
auto opts = ntpclock::Options::Builder()
    .PollIntervalMs(10000)      // Poll every 10 seconds
    .StepThresholdMs(200)       // Step if offset > 200ms
    .SlewRateMsPerSec(5.0)      // Slew at 5ms/second
    .Build();
```

### NtpServer Settings

```cpp
auto server_opts = ntpserver::Options::Builder()
    .Stratum(2)                                // Stratum level
    .Precision(-20)                            // Precision (2^-20 seconds)
    .RefId(ntpserver::MakeRefId("LOCL"))       // Reference ID ("LOCL")
    .Build();

ntpserver::NtpServer server;
server.Start(port, &time_source, server_opts);  // Start with time source
```

## API Overview

### ClockService

```cpp
#include "ntpclock/clock_service.hpp"

ntpclock::ClockService clock;
clock.Start("127.0.0.1", 9123, options);
ntpserver::TimeSpec now = clock.NowUnix();
ntpclock::Status status = clock.GetStatus();
clock.Stop();
```

### NtpServer

```cpp
#include "ntpserver/ntp_server.hpp"

auto server_opts =
    ntpserver::Options::Builder().Stratum(1).RefId(ntpserver::MakeRefId("GPS"))
        .Build();
ntpserver::NtpServer server;
server.Start(9123, &time_source, server_opts);

// To apply configuration changes (e.g., time source updates):
// Stop and restart the server - Push notifications are sent automatically
server.Stop();
time_source.SetAbsolute(new_time);
server.Start(9123, &time_source, server_opts);  // Sends Push to known clients

server.Stop();
```

### TimeSource Interface

```cpp
class MyTimeSource : public ntpserver::TimeSource {
 public:
  ntpserver::TimeSpec NowUnix() override {
    // Return current UNIX time
  }
  double GetRate() const override {
    // Return clock rate (1.0 = nominal)
  }
};
```

## License

Copyright (c) 2025

## Author

Created as a demonstration of high-precision NTP implementation with vendor extensions for instant synchronization.
