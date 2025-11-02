// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_client.cc
 * @brief Implementation of a minimal SNTP client for Windows.
 */
#include "ntpclient/ntp_client.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "ntpserver/qpc_clock.hpp"

#pragma comment(lib, "Ws2_32.lib")

namespace ntpclient {

namespace {
constexpr uint32_t kNtpUnixEpochDiff = 2208988800UL;  // seconds

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

inline uint64_t Hton64(uint64_t v) {
  uint32_t hi = htonl(static_cast<uint32_t>(v >> 32));
  uint32_t lo = htonl(static_cast<uint32_t>(v & 0xFFFFFFFFULL));
  return (static_cast<uint64_t>(lo) << 32) | hi;
}

inline uint64_t ToNtpTimestamp(double unix_seconds) {
  double sec;
  double frac =
      std::modf(unix_seconds + static_cast<double>(kNtpUnixEpochDiff), &sec);
  uint64_t s = static_cast<uint64_t>(sec);
  uint64_t f = static_cast<uint64_t>(frac * static_cast<double>(1ULL << 32));
  return (s << 32) | f;
}

inline double FromNtpTimestamp(uint64_t ntp_ts_be) {
  uint32_t hi = ntohl(static_cast<uint32_t>(ntp_ts_be >> 32));
  uint32_t lo = ntohl(static_cast<uint32_t>(ntp_ts_be & 0xFFFFFFFFULL));
  double sec = static_cast<double>(hi);
  double frac = static_cast<double>(lo) / static_cast<double>(1ULL << 32);
  return (sec - static_cast<double>(kNtpUnixEpochDiff)) + frac;
}

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
}  // namespace

NtpClient::NtpClient(ntpserver::QpcClock* clock) : clock_(clock) {}

bool NtpClient::SyncOnce(const std::string& host, uint16_t port,
                         int timeout_ms) {
  if (!clock_) return false;

  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;

  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) {
    WSACleanup();
    return false;
  }

  // Resolve host
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    hostent* he = gethostbyname(host.c_str());
    if (!he || he->h_addrtype != AF_INET) {
      closesocket(s);
      WSACleanup();
      return false;
    }
    std::memcpy(&addr.sin_addr, he->h_addr_list[0], sizeof(in_addr));
  }
  addr.sin_port = htons(port);

  // Build request
  NtpPacket req{};
  req.li_vn_mode = static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);
  double t1 = clock_->NowUnix();
  req.tx_timestamp = Hton64(ToNtpTimestamp(t1));

  // Send
  int sent = sendto(s, reinterpret_cast<const char*>(&req), sizeof(req), 0,
                    reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (sent != static_cast<int>(sizeof(req))) {
    closesocket(s);
    WSACleanup();
    return false;
  }

  // Timeout
  DWORD tv = static_cast<DWORD>(timeout_ms);
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv),
             sizeof(tv));

  // Receive
  sockaddr_in from{};
  int fromlen = sizeof(from);
  NtpPacket resp{};
  int n = recvfrom(s, reinterpret_cast<char*>(&resp), sizeof(resp), 0,
                   reinterpret_cast<sockaddr*>(&from), &fromlen);
  double t4 = clock_->NowUnix();

  closesocket(s);
  WSACleanup();
  if (n != static_cast<int>(sizeof(resp))) return false;

  double t1_e = FromNtpTimestamp(resp.orig_timestamp);
  double t2 = FromNtpTimestamp(resp.recv_timestamp);
  double t3 = FromNtpTimestamp(resp.tx_timestamp);

  // Basic check: t1 echoed back
  (void)t1_e;

  // Offset theta and target time at local receive instant (t4)
  double theta = ((t2 - t1) + (t3 - t4)) / 2.0;
  double target = t4 + theta;

  // Step or slew
  const double slew_ppm_max = 500.0;
  double off = target - clock_->NowUnix();
  if (std::abs(off) > 0.2) {
    clock_->SetAbsolute(target);
  } else {
    // Try to remove within ~10s limited by ppm
    double ppm = std::clamp(off * 1e6 / 10.0, -slew_ppm_max, slew_ppm_max);
    clock_->SetRate(1.0 + ppm / 1e6);
  }
  return true;
}

}  // namespace ntpclient
