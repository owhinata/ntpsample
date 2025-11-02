/**
 * @file
 * @brief Example program for ClockService with simple CLI options.
 *
 * Usage:
 *   ntpclock_example --ip 127.0.0.1 --port 9123 \
 *     --poll 1000 --step 200 --slew 5.0 --max-rtt 100 \
 *     --offset-window 5 --skew-window 20 --duration 10
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>

#include "ntpclock/clock_service.hpp"

namespace {
void PrintUsage() {
  std::fprintf(stderr,
               "Usage: ntpclock_example --ip A.B.C.D --port N [options]\n"
               "Options:\n"
               "  --poll ms            (default 1000)\n"
               "  --step ms            (default 200)\n"
               "  --slew ms_per_s      (default 5.0)\n"
               "  --max-rtt ms         (default 100)\n"
               "  --offset-window n    (default 5)\n"
               "  --skew-window n      (default 20)\n"
               "  --duration sec       (default 10)\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::string ip = "127.0.0.1";
  uint16_t port = 9123;
  int duration_sec = 10;

  auto builder = ntpclock::Options::Builder();

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto need = [&](int more) { return i + more < argc; };
    if (a == "--ip" && need(1)) {
      ip = argv[++i];
    } else if (a == "--port" && need(1)) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--poll" && need(1)) {
      builder.PollIntervalMs(std::atoi(argv[++i]));
    } else if (a == "--step" && need(1)) {
      builder.StepThresholdMs(std::atoi(argv[++i]));
    } else if (a == "--slew" && need(1)) {
      builder.SlewRateMsPerSec(std::atof(argv[++i]));
    } else if (a == "--max-rtt" && need(1)) {
      builder.MaxRttMs(std::atoi(argv[++i]));
    } else if (a == "--offset-window" && need(1)) {
      builder.OffsetWindow(std::atoi(argv[++i]));
    } else if (a == "--skew-window" && need(1)) {
      builder.SkewWindow(std::atoi(argv[++i]));
    } else if (a == "--duration" && need(1)) {
      duration_sec = std::atoi(argv[++i]);
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown or incomplete option: %s\n", a.c_str());
      PrintUsage();
      return 2;
    }
  }

  ntpclock::ClockService svc;
  auto opt = builder.Build();
  std::ostringstream oss;
  oss << opt;
  std::string opt_str = oss.str();
  std::printf("Starting sync to %s:%u with options: %s\n",
              ip.c_str(), port, opt_str.c_str());

  if (!svc.Start(ip, port, opt)) {
    std::fprintf(stderr, "Failed to start ClockService\n");
    return 1;
  }

  auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(
             std::chrono::steady_clock::now() - t0)
             .count() < duration_sec) {
    auto st = svc.GetStatus();
    double now = svc.NowUnix();
    std::ostringstream osst;
    osst << st;
    std::string st_str = osst.str();
    std::printf("now=%.6f | %s\n", now, st_str.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  svc.Stop();
  return 0;
}
