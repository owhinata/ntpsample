// Copyright (c) 2025 <Your Name>
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ntpserver/export.hpp"
#include "ntpserver/time_source.hpp"

namespace ntpserver {

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

  /**
   * @brief Starts serving on the given UDP port.
   * @param port UDP port to bind (default: 9123).
   * @param time_source Time source for timestamps (default: QpcClock).
   * @return true on success, false on failure.
   */
  bool Start(uint16_t port = 9123, TimeSource* time_source = nullptr);

  /** Stops the server. Safe to call multiple times. */
  void Stop();

  /** Sets stratum (default 1). */
  void SetStratum(uint8_t stratum);
  /** Sets precision exponent (default -20). */
  void SetPrecision(int8_t precision);
  /** Sets reference ID (network byte order, default: "LOCL"). */
  void SetRefId(uint32_t ref_id_be);

  /**
   * @brief Sends a control snapshot (ABS/RATE via NTP EF) to known clients.
   *
   * The server tracks recently seen client endpoints (from requests). When
   * configuration changes (e.g., SetAbsolute/SetRate on the TimeSource), call
   * this method to actively notify clients between polls.
   */
  void NotifyControlSnapshot();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntpserver
