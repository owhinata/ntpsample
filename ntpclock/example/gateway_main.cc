// Copyright (c) 2025 The NTP Sample Authors
/**
 * @file gateway_main.cc
 * @brief NTP Gateway: Sync with upstream server and serve to downstream
 * clients.
 *
 * This application combines ClockService (NTP client) and NtpServer to create
 * an NTP gateway that:
 * 1. Synchronizes with an upstream NTP server using ClockService
 * 2. Serves the synchronized time to downstream clients using NtpServer
 * 3. Propagates vendor hints (rate/abs adjustments) from upstream to downstream
 *    by sharing a single QpcClock instance between both components
 *
 * Usage:
 *   ntpclock_gateway --upstream-ip 127.0.0.1 --upstream-port 9123 \
 *                    --serve-port 9124 --poll 10000 --min-samples 3
 */

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "ntpclock/clock_service.hpp"
#include "ntpserver/ntp_server.hpp"
#include "ntpserver/platform/default_time_source.hpp"

namespace {

/**
 * @brief Thread-safe logger for debug messages.
 */
class Logger {
 public:
  explicit Logger(bool enabled) : enabled_(enabled) {}

  void Log(const std::string& msg) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::fprintf(stderr, "%s\n", msg.c_str());
  }

 private:
  bool enabled_;
  std::mutex mutex_;
};

std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running.store(false); }

void PrintUsage() {
  std::fprintf(
      stderr,
      "Usage: ntpclock_gateway [options]\n"
      "Options:\n"
      "  --upstream-ip IP     Upstream NTP server IP (default 127.0.0.1)\n"
      "  --upstream-port N    Upstream NTP server port (default 9123)\n"
      "  --serve-port N       Port to serve on (default 9124)\n"
      "  --poll ms            Polling interval in ms (default 10000)\n"
      "  --step ms            Step threshold in ms (default 200)\n"
      "  --slew ms_per_s      Slew rate in ms/s (default 5.0)\n"
      "  --min-samples n      Min samples to lock (default 3)\n"
      "  --debug              Enable debug logging\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string upstream_ip = "127.0.0.1";
  uint16_t upstream_port = 9123;
  uint16_t serve_port = 9124;
  bool debug = false;

  auto builder = ntpclock::Options::Builder();

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto need = [&](int more) { return i + more < argc; };

    if (a == "--upstream-ip" && need(1)) {
      upstream_ip = argv[++i];
    } else if (a == "--upstream-port" && need(1)) {
      upstream_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--serve-port" && need(1)) {
      serve_port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--poll" && need(1)) {
      builder.PollIntervalMs(std::atoi(argv[++i]));
    } else if (a == "--step" && need(1)) {
      builder.StepThresholdMs(std::atoi(argv[++i]));
    } else if (a == "--slew" && need(1)) {
      builder.SlewRateMsPerSec(std::atof(argv[++i]));
    } else if (a == "--min-samples" && need(1)) {
      builder.MinSamplesToLock(std::atoi(argv[++i]));
    } else if (a == "--debug") {
      debug = true;
    } else if (a == "--help" || a == "-h") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
      PrintUsage();
      return 1;
    }
  }

  // Create logger
  Logger logger(debug);
  auto log_callback = [&logger](const std::string& msg) { logger.Log(msg); };

  builder.LogSink(log_callback);
  auto opts = builder.Build();

  // Also use logger for NtpServer
  auto server_opts =
      ntpserver::Options::Builder().Stratum(2).LogSink(log_callback).Build();

  // Setup signal handler for graceful shutdown
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  // Create shared TimeSource for both ClockService and NtpServer
  // This allows vendor hints (rate/abs) to propagate from upstream to
  // downstream
  auto time_source = ntpserver::platform::CreateDefaultTimeSource();

  // Create ClockService to sync with upstream server
  ntpclock::ClockService clock_service;
  if (!clock_service.Start(time_source.get(), upstream_ip, upstream_port,
                           opts)) {
    std::fprintf(stderr, "Failed to start ClockService\n");
    return 1;
  }
  std::printf("ClockService started, syncing with %s:%u\n", upstream_ip.c_str(),
              upstream_port);

  // Wait a bit for initial synchronization
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Create NtpServer to serve downstream clients using the same TimeSource
  ntpserver::NtpServer server;
  if (!server.Start(serve_port, time_source.get(), server_opts)) {
    std::fprintf(stderr, "Failed to start NtpServer on port %u\n", serve_port);
    clock_service.Stop();
    return 1;
  }
  std::printf("NtpServer started on UDP port %u (stratum 2)\n", serve_port);
  std::printf("Press Ctrl+C to stop...\n\n");

  // Main status display loop with change detection
  ntpserver::TimeSpec last_update{};
  uint32_t last_epoch = 0;
  while (g_running.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ntpclock::Status st = clock_service.GetStatus();

    // Detect ClockService updates and propagate to downstream clients
    if (st.last_update.ToDouble() > 0.0 && st.last_update != last_update) {
      last_update = st.last_update;
      // Restart server when upstream server epoch changes
      if (st.epoch != 0 && st.epoch != last_epoch) {
        last_epoch = st.epoch;
        server.Stop();
        // time_source is already updated by ClockService
        if (!server.Start(serve_port, time_source.get(), server_opts)) {
          std::fprintf(stderr, "\nFailed to restart NtpServer\n");
          clock_service.Stop();
          return 1;
        }
        std::ostringstream oss;
        oss << "[Gateway] NtpServer restarted due to upstream epoch change: "
            << st.epoch;
        logger.Log(oss.str());
      }
    }

    std::printf(
        "\r[Gateway Status] sync=%s rtt=%dms offset=%.6fs samples=%d   ",
        st.synchronized ? "YES" : "NO ", st.rtt_ms, st.offset_s, st.samples);
    std::fflush(stdout);
  }

  std::printf("\n\nStopping gateway...\n");
  server.Stop();
  clock_service.Stop();
  std::printf("Gateway stopped.\n");

  return 0;
}
