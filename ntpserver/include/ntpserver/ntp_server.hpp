// Copyright (c) 2025 <Your Name>
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ntpserver/export.hpp"

namespace ntpserver {

// Abstract time source. Returns current time as UNIX seconds (UTC).
class TimeSource {
 public:
  virtual ~TimeSource() = default;
  virtual double NowUnix() = 0;  // seconds since 1970-01-01T00:00:00Z
};

// Minimal NTP server. UDP/IPv4, NTPv4 server mode (4), stratum 1 (local).
class NTP_SERVER_API NtpServer {
 public:
  NtpServer();
  ~NtpServer();

  NtpServer(const NtpServer&) = delete;
  NtpServer& operator=(const NtpServer&) = delete;

  // Start serving on `port` (default 9123). Returns true on success.
  bool Start(uint16_t port = 9123);

  // Stop the server. Safe to call multiple times.
  void Stop();

  // Set external time source. If not set, an internal monotonic-backed
  // user time source is used (see user_time.hpp).
  void SetTimeSource(TimeSource* src);

  // Optional tunables.
  void SetStratum(uint8_t stratum);     // default 1
  void SetPrecision(int8_t precision);  // default -20
  void SetRefId(uint32_t ref_id_be);    // network-order 4CC, default "LOCL"

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntpserver
