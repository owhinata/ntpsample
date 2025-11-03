// Copyright (c) 2025 <Your Name>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "ntpserver/ntp_server.hpp"
#include "ntpserver/qpc_clock.hpp"

namespace {
void PrintUsage() {
  std::fprintf(stderr,
               "Usage: ntpserver_example [--port N] [--rate R] [--abs SEC]\n"
               "       Commands on stdin: help | now | rate R | abs SEC | add "
               "SEC | quit\n");
}
}  // namespace

int main(int argc, char** argv) {
  uint16_t port = 9123;
  double init_rate = 1.0;
  double init_abs = 0.0;  // 0: do not set

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto need = [&](int n) { return i + n < argc; };
    if (a == "--port" && need(1)) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--rate" && need(1)) {
      init_rate = std::atof(argv[++i]);
    } else if (a == "--abs" && need(1)) {
      init_abs = std::atof(argv[++i]);
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", a.c_str());
      PrintUsage();
      return 2;
    }
  }

  ntpserver::NtpServer server;
  auto& ts = ntpserver::QpcClock::Instance();
  ts.SetRate(init_rate);
  if (init_abs != 0.0) ts.SetAbsolute(init_abs);
  server.SetTimeSource(&ts);

  if (!server.Start(port)) {
    std::fprintf(stderr, "failed to start ntp server\n");
    return 1;
  }
  std::printf("ntp server running on UDP %u\n", port);
  std::printf(
      "stdin commands: help | now | rate R | abs SEC | add SEC | quit\n");

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
      double now = ts.NowUnix();
      std::printf("now=%.6f\n", now);
      continue;
    }
    if (std::strncmp(line, "rate ", 5) == 0) {
      double r = std::atof(line + 5);
      ts.SetRate(r);
      server.NotifyControlSnapshot();
      std::printf("rate set to %.6f (notify)\n", r);
      continue;
    }
    if (std::strncmp(line, "abs ", 4) == 0) {
      double t = std::atof(line + 4);
      ts.SetAbsolute(t);
      server.NotifyControlSnapshot();
      std::printf("absolute set to %.6f (notify)\n", t);
      continue;
    }
    if (std::strncmp(line, "add ", 4) == 0) {
      double d = std::atof(line + 4);
      ts.AdjustOffset(d);
      server.NotifyControlSnapshot();
      std::printf("offset adjusted by %+f (notify)\n", d);
      continue;
    }
    std::fprintf(stderr, "unknown command: %s\n", line);
  }

  server.Stop();
  return 0;
}
