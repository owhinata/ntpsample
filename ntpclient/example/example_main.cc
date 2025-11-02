// Copyright (c) 2025 <Your Name>
#include <cstdio>
#include <string>

#include "ntpclient/ntp_client.hpp"
#include "ntpserver/qpc_clock.hpp"

int main(int argc, char** argv) {
  std::string host = "127.0.0.1";
  uint16_t port = 9123;
  if (argc >= 2) host = argv[1];
  if (argc >= 3) port = static_cast<uint16_t>(std::stoi(argv[2]));

  ntpserver::QpcClock clock;
  ntpclient::NtpClient client(&clock);

  if (!client.SyncOnce(host, port, 1000)) {
    std::fprintf(stderr, "ntpclient: sync failed (host=%s, port=%u)\n",
                 host.c_str(), port);
    return 1;
  }
  std::printf("ntpclient: synced with %s:%u, now=%.6f\n", host.c_str(), port,
              clock.NowUnix());
  return 0;
}
