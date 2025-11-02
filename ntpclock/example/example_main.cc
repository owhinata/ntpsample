#include <cstdio>
#include <thread>
#include <chrono>

#include "ntpclock/clock_service.hpp"

int main() {
  ntpclock::ClockService svc;
  auto opt = ntpclock::Options::Builder().PollIntervalMs(1000).Build();
  // Change to your server IP/port as needed
  svc.Start("127.0.0.1", 9123, opt);
  for (int i = 0; i < 5; ++i) {
    double t = svc.NowUnix();
    std::printf("now=%.6f\n", t);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  svc.Stop();
  return 0;
}

