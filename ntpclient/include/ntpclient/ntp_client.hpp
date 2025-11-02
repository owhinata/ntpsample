// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_client.hpp
 * @brief Simple SNTP client (NTPv4) over UDP for Windows.
 */
#pragma once

#include <cstdint>
#include <string>

namespace ntpserver {
class QpcClock;
}

namespace ntpclient {

/**
 * Minimal SNTP client that sends one request and adjusts a QpcClock.
 */
class NtpClient {
 public:
  /** Constructs a client with the given adjustable clock. */
  explicit NtpClient(ntpserver::QpcClock* clock);

  /**
   * Send one SNTP request to host:port and apply step/slew.
   * @param host hostname or IPv4 address
   * @param port UDP port (default 9123)
   * @param timeout_ms receive timeout
   * @return true on success
   */
  bool SyncOnce(const std::string& host, uint16_t port = 9123,
                int timeout_ms = 1000);

 private:
  ntpserver::QpcClock* clock_{};  // not owned
};

}  // namespace ntpclient
