// Copyright (c) 2025 <Your Name>
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ntpserver/export.hpp"

namespace ntpserver {

/**
 * Interface for time sources.
 *
 * Provides current time as UNIX seconds (UTC, since 1970-01-01T00:00:00Z).
 */
class TimeSource {
 public:
  virtual ~TimeSource() = default;
  /** Returns the current time in seconds since the UNIX epoch. */
  virtual double NowUnix() = 0;
};

/**
 * Minimal NTPv4 server (UDP/IPv4).
 *
 * Server operates in mode 4 (server). Default stratum is 1 (local reference).
 */
class NTP_SERVER_API NtpServer {
 public:
  NtpServer();
  ~NtpServer();

  NtpServer(const NtpServer&) = delete;
  NtpServer& operator=(const NtpServer&) = delete;

  /** Starts serving on the given UDP port (default: 9123). */
  bool Start(uint16_t port = 9123);

  /** Stops the server. Safe to call multiple times. */
  void Stop();

  /** Sets an external time source used for timestamps. */
  void SetTimeSource(TimeSource* src);

  /** Sets stratum (default 1). */
  void SetStratum(uint8_t stratum);
  /** Sets precision exponent (default -20). */
  void SetPrecision(int8_t precision);
  /** Sets reference ID (network byte order, default: "LOCL"). */
  void SetRefId(uint32_t ref_id_be);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntpserver
