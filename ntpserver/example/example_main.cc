// Copyright (c) 2025 <Your Name>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "ntpserver/ntp_server.hpp"
#include "ntpserver/qpc_clock.hpp"

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

void PrintUsage() {
  std::fprintf(
      stderr,
      "Usage: ntpserver_example [--port N] [--rate R] [--abs SEC] "
      "[--debug]\n"
      "       Commands on stdin: help | now | rate R | abs SEC | add SEC | "
      "reset | quit\n");
}
}  // namespace

int main(int argc, char** argv) {
  uint16_t port = 9123;
  double init_rate = 1.0;
  double init_abs = 0.0;  // 0: do not set
  bool debug = false;

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto need = [&](int n) { return i + n < argc; };
    if (a == "--port" && need(1)) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--rate" && need(1)) {
      init_rate = std::atof(argv[++i]);
    } else if (a == "--abs" && need(1)) {
      init_abs = std::atof(argv[++i]);
    } else if (a == "--debug") {
      debug = true;
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
      PrintUsage();
      return 2;
    }
  }

  // Create logger
  Logger logger(debug);
  auto log_callback = [&logger](const std::string& msg) { logger.Log(msg); };

  ntpserver::NtpServer server;
  auto& ts = ntpserver::QpcClock::Instance();
  ts.SetRate(init_rate);
  if (init_abs != 0.0)
    ts.SetAbsolute(ntpserver::TimeSpec::FromDouble(init_abs));

  // Build options with logger (used for Start/Stop cycles)
  auto opts = ntpserver::Options::Builder().LogSink(log_callback).Build();

  if (!server.Start(port, &ts, opts)) {
    std::fprintf(stderr, "failed to start ntp server\n");
    return 1;
  }
  std::printf("ntp server running on UDP %u\n", port);
  std::printf(
      "stdin commands: help | now | rate R | abs SEC | add SEC | reset | "
      "quit\n");

  // Control loop on stdin
  char line[256];
  while (std::fgets(line, sizeof(line), stdin)) {
    // Trim
    size_t len = std::strlen(line);
    while (len && (line[len - 1] == '\n' || line[len - 1] == '\r'))
      line[--len] = 0;
    if (len == 0) continue;
    if (std::strcmp(line, "help") == 0) {
      PrintUsage();
      continue;
    }
    if (std::strcmp(line, "quit") == 0 || std::strcmp(line, "exit") == 0) {
      break;
    }
    if (std::strcmp(line, "now") == 0) {
      double now = ts.NowUnix().ToDouble();
      std::printf("now=%.6f\n", now);
      continue;
    }
    if (std::strncmp(line, "rate ", 5) == 0) {
      double r = std::atof(line + 5);
      double before = ts.NowUnix().ToDouble();

      // Stop -> change -> Start
      server.Stop();
      ts.SetRate(r);
      if (!server.Start(port, &ts, opts)) {
        std::fprintf(stderr, "Failed to restart server\n");
        return 1;
      }

      double after = ts.NowUnix().ToDouble();
      std::ostringstream oss;
      oss << "Server restarted with rate=" << r << " before=" << before
          << " after=" << after;
      logger.Log(oss.str());
      continue;
    }
    if (std::strncmp(line, "abs ", 4) == 0) {
      double t = std::atof(line + 4);
      double before = ts.NowUnix().ToDouble();

      // Stop -> change -> Start
      server.Stop();
      ts.SetAbsolute(ntpserver::TimeSpec::FromDouble(t));
      if (!server.Start(port, &ts, opts)) {
        std::fprintf(stderr, "Failed to restart server\n");
        return 1;
      }

      double after = ts.NowUnix().ToDouble();
      std::ostringstream oss;
      oss << "Server restarted with absolute=" << t << " before=" << before
          << " after=" << after;
      logger.Log(oss.str());
      continue;
    }
    if (std::strncmp(line, "add ", 4) == 0) {
      double d = std::atof(line + 4);
      double before = ts.NowUnix().ToDouble();

      // Stop -> change -> Start
      server.Stop();
      // Use absolute update to avoid cumulative rounding and ensure exact step.
      ts.SetAbsolute(ntpserver::TimeSpec::FromDouble(before + d));
      if (!server.Start(port, &ts, opts)) {
        std::fprintf(stderr, "Failed to restart server\n");
        return 1;
      }

      double after = ts.NowUnix().ToDouble();
      std::ostringstream oss;
      oss << "Server restarted with offset " << (d >= 0 ? "+" : "") << d
          << " before=" << before << " after=" << after
          << " delta=" << (after - before);
      logger.Log(oss.str());
      continue;
    }
    if (std::strcmp(line, "reset") == 0) {
      double before = ts.NowUnix().ToDouble();
      double rate_before = ts.GetRate();

      // Stop -> change -> Start
      server.Stop();
      ts.ResetToRealTime();
      if (!server.Start(port, &ts, opts)) {
        std::fprintf(stderr, "Failed to restart server\n");
        return 1;
      }

      double after = ts.NowUnix().ToDouble();
      std::ostringstream oss;
      oss << "Server restarted (reset to real-time) before=" << before
          << " (rate=" << rate_before << ") after=" << after << " (rate=1.0)";
      logger.Log(oss.str());
      continue;
    }
    std::fprintf(stderr, "unknown command: %s\n", line);
  }

  server.Stop();
  return 0;
}
