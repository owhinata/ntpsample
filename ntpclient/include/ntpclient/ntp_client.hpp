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

/** Lightweight transport interface to allow test/server swapping. */
class INtpTransport {
 public:
  virtual ~INtpTransport() = default;
  /**
   * Sends request and receives response from host:port with timeout.
   * @return true if exactly resp_len bytes were received.
   */
  virtual bool Exchange(const std::string& host, uint16_t port, int timeout_ms,
                        const uint8_t* req, size_t req_len, uint8_t* resp,
                        size_t resp_len) = 0;
};

/**
 * Minimal SNTP client that sends one request and adjusts a QpcClock.
 */
class NtpClient {
 public:
  /** Constructs a client with the given adjustable clock. */
  explicit NtpClient(ntpserver::QpcClock* clock);

  /** Constructs a client with injected transport (for testing). */
  NtpClient(ntpserver::QpcClock* clock, INtpTransport* transport);

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
  INtpTransport* transport_{};    // not owned
};

}  // namespace ntpclient
