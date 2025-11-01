// Copyright (c) 2025 <Your Name>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <chrono>

#include "ntpserver/ntp_server.hpp"
#include "ntpserver/user_time.hpp"

int main() {
  ntpserver::NtpServer server;
  // Optional: tweak local time source
  auto& ut = ntpserver::UserTime::Instance();
  ut.SetRate(1.0);
  ut.AdjustOffset(0.0);
  server.SetTimeSource(&ut);

  if (!server.Start(9123)) {
    std::fprintf(stderr, "failed to start ntp server\n");
    return 1;
  }
  std::printf("ntp server running on UDP 9123 (Ctrl+C to exit)\n");

  // Run until killed.
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
