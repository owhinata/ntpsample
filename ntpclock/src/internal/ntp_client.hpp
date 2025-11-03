// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_client.hpp
 * @brief NTP client for single-server synchronization via UDP.
 *
 * Encapsulates NTP request/response exchange over UDP, including packet
 * construction, network I/O, timestamp parsing, and offset/RTT calculation.
 * Designed to preserve exact timing semantics of the original implementation.
 */
#pragma once

#include <winsock2.h>
#include <ws2tcpip.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace ntpclock {
namespace internal {

namespace {
constexpr uint32_t kNtpUnixEpochDiff = 2208988800UL;

#pragma pack(push, 1)
struct NtpPacket {
  uint8_t li_vn_mode;
  uint8_t stratum;
  uint8_t poll;
  int8_t precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  uint32_t ref_id;
  uint64_t ref_timestamp;
  uint64_t orig_timestamp;
  uint64_t recv_timestamp;
  uint64_t tx_timestamp;
};
#pragma pack(pop)

inline void WriteTimestamp(double unix_seconds, uint8_t* dst8) {
  uint32_t sec =
      static_cast<uint32_t>(std::floor(unix_seconds + kNtpUnixEpochDiff));
  double frac_d = (unix_seconds + kNtpUnixEpochDiff) - static_cast<double>(sec);
  uint32_t frac =
      static_cast<uint32_t>(frac_d * static_cast<double>(1ULL << 32));
  uint32_t sec_be = htonl(sec);
  uint32_t frac_be = htonl(frac);
  std::memcpy(dst8 + 0, &sec_be, 4);
  std::memcpy(dst8 + 4, &frac_be, 4);
}

inline double ReadTimestamp(const uint8_t* src8) {
  uint32_t sec_be = 0, frac_be = 0;
  std::memcpy(&sec_be, src8 + 0, 4);
  std::memcpy(&frac_be, src8 + 4, 4);
  uint32_t sec = ntohl(sec_be);
  uint32_t frac = ntohl(frac_be);
  return (static_cast<double>(sec) - static_cast<double>(kNtpUnixEpochDiff)) +
         (static_cast<double>(frac) / static_cast<double>(1ULL << 32));
}
}  // namespace

/**
 * @brief NTP client for UDP-based time synchronization.
 *
 * Performs a single NTP request/response exchange with a server, computes
 * offset and RTT, and optionally processes vendor extension fields via
 * callback. Uses callbacks for timestamp acquisition to preserve exact
 * timing semantics and mutex lock ordering.
 */
class NtpClient {
 public:
  /**
   * @brief NTP exchange result.
   */
  struct Result {
    bool success;
    double offset_s;
    int rtt_ms;
    std::vector<uint8_t> response_bytes;
    std::string error;
  };

  /**
   * @brief Perform a single NTP request/response exchange.
   *
   * Sends an NTP client request to the specified server, receives the
   * response, computes offset and RTT, and returns raw response bytes
   * for extension field processing.
   *
   * @param ip Server IPv4 address (numeric string, no DNS).
   * @param port Server UDP port.
   * @param get_timestamp Callback to get current time (UNIX seconds).
   *                      Called twice: before send (T1) and after receive (T4).
   * @param on_response_received Optional callback invoked after receiving
   *                             response with raw bytes (for vendor hints).
   * @return Result containing success status, offset, RTT, raw bytes, and error.
   */
  static Result Exchange(
      const std::string& ip, uint16_t port,
      std::function<double()> get_timestamp,
      std::function<void(const std::vector<uint8_t>&)> on_response_received =
          nullptr) {
    Result result{false, 0.0, 0, {}, ""};

    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      result.error = "WSAStartup failed";
      return result;
    }

    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) {
      result.error = "socket() failed";
      WSACleanup();
      return result;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
      result.error = "inet_pton failed (IPv4 only)";
      closesocket(s);
      WSACleanup();
      return result;
    }

    NtpPacket req{};
    std::memset(&req, 0, sizeof(req));
    req.li_vn_mode =
        static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);  // v4, client

    double T1 = get_timestamp();
    WriteTimestamp(T1, reinterpret_cast<uint8_t*>(&req.tx_timestamp));

    int sent = sendto(s, reinterpret_cast<const char*>(&req), sizeof(req), 0,
                      reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (sent != sizeof(req)) {
      result.error = "sendto failed";
      closesocket(s);
      WSACleanup();
      return result;
    }

    DWORD timeout = 500;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
               reinterpret_cast<const char*>(&timeout), sizeof(timeout));

    std::vector<uint8_t> rx(1500);
    sockaddr_in from{};
    int fromlen = sizeof(from);
    int recvd = recvfrom(s, reinterpret_cast<char*>(rx.data()),
                         static_cast<int>(rx.size()), 0,
                         reinterpret_cast<sockaddr*>(&from), &fromlen);
    double T4 = get_timestamp();

    if (recvd < static_cast<int>(sizeof(NtpPacket))) {
      result.error = "recvfrom timeout/failure";
      closesocket(s);
      WSACleanup();
      return result;
    }

    closesocket(s);
    WSACleanup();

    NtpPacket resp{};
    std::memcpy(&resp, rx.data(), sizeof(resp));

    if (recvd > static_cast<int>(sizeof(NtpPacket))) {
      rx.resize(recvd);
      if (on_response_received) {
        on_response_received(rx);
      }
    }

    double T2 = ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.recv_timestamp));
    double T3 = ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.tx_timestamp));

    double delay = (T4 - T1) - (T3 - T2);
    double offset = ((T2 - T1) + (T3 - T4)) / 2.0;

    result.success = true;
    result.offset_s = offset;
    result.rtt_ms = static_cast<int>(std::max(0.0, delay) * 1000.0 + 0.5);
    result.response_bytes = std::move(rx);

    return result;
  }
};

}  // namespace internal
}  // namespace ntpclock
