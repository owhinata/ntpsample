// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_client.hpp
 * @brief Simple SNTP client (NTPv4) over UDP for Windows.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "ntpserver/time_source.hpp"

// QpcClock forward-declare not needed due to include above

namespace ntpclient {

/**
 * Minimal SNTP client that sends one request and adjusts a QpcClock.
 *
 * Responsibilities
 * - Build SNTP request (client mode, VN=4) with t1 timestamp.
 * - Use transport to exchange a single datagram and capture local t4.
 * - Compute offset (theta) = ((t2 - t1) + (t3 - t4)) / 2.
 * - Apply step (>|0.2|s) or slew (<=|0.2|s within ~window seconds, ppm cap).
 */
class NtpClient {
 public:
  /** Default-construct with the library's default clock implementation. */
  NtpClient();

  /** Construct with an injected time source (also used for adjustments). */
  explicit NtpClient(ntpserver::TimeSource* time_source);

  /**
   * Send one SNTP request to host:port and apply step/slew.
   * @param host hostname or IPv4 address
   * @param port UDP port (default 9123)
   * @param timeout_ms receive timeout
   * @return true on success
   */
  ~NtpClient();

  bool SyncOnce(const std::string& host, uint16_t port = 9123,
                int timeout_ms = 1000);
  /** Sets maximum slew rate in ppm (default 500). */
  void SetSlewPpmMax(double ppm);
  /** Sets slew window target in seconds (default 10). */
  void SetSlewWindowSec(double sec);

 private:
  class Impl;  // defined in .cc
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntpclient
