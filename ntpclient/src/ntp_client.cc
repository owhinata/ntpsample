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
#include "ntpserver/time_source.hpp"

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

inline double FromNtpTimestamp(const uint64_t& ntp_ts_be) {
  uint32_t hi = 0, lo = 0;
  std::memcpy(&hi, &ntp_ts_be, 4);
  std::memcpy(&lo, reinterpret_cast<const uint8_t*>(&ntp_ts_be) + 4, 4);
  hi = ntohl(hi);
  lo = ntohl(lo);
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

namespace {
static bool UdpExchange(const std::string& host, uint16_t port, int timeout_ms,
                        const uint8_t* req, size_t req_len, uint8_t* resp,
                        size_t resp_len) {
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;

  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) {
    WSACleanup();
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  // Try numeric first, then resolve via getaddrinfo (IPv4 only).
  if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 ||
        res == nullptr) {
      closesocket(s);
      WSACleanup();
      return false;
    }
    auto* sin = reinterpret_cast<sockaddr_in*>(res->ai_addr);
    addr.sin_addr = sin->sin_addr;
    freeaddrinfo(res);
  }

  int sent =
      sendto(s, reinterpret_cast<const char*>(req), static_cast<int>(req_len),
             0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (sent != static_cast<int>(req_len)) {
    closesocket(s);
    WSACleanup();
    return false;
  }

  DWORD tv = static_cast<DWORD>(timeout_ms);
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv),
             sizeof(tv));

  sockaddr_in from{};
  int fromlen = sizeof(from);
  int n = recvfrom(s, reinterpret_cast<char*>(resp), static_cast<int>(resp_len),
                   0, reinterpret_cast<sockaddr*>(&from), &fromlen);
  closesocket(s);
  WSACleanup();
  return n == static_cast<int>(resp_len);
}
}  // namespace

class NtpClient::Impl {
 public:
  explicit Impl(ntpserver::TimeSource* ts)
      : time_source(ts ? ts : &ntpserver::QpcClock::Instance()) {}

  bool SyncOnce(const std::string& host, uint16_t port, int timeout_ms) {
    if (!time_source) return false;
    NtpPacket req{};
    req.li_vn_mode = static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);
    const double t1 = time_source->NowUnix();
    req.tx_timestamp = Hton64(ToNtpTimestamp(t1));

    NtpPacket resp{};
    const bool ok = UdpExchange(
        host, port, timeout_ms, reinterpret_cast<const uint8_t*>(&req),
        sizeof(req), reinterpret_cast<uint8_t*>(&resp), sizeof(resp));
    const double t4 = time_source->NowUnix();
    if (!ok) return false;

    // Parse server timestamps
    const double t2 = FromNtpTimestamp(resp.recv_timestamp);
    const double t3 = FromNtpTimestamp(resp.tx_timestamp);
    const double theta = ((t2 - t1) + (t3 - t4)) / 2.0;
    const double target = t4 + theta;
    const double off = target - time_source->NowUnix();
    if (std::abs(off) > 0.2) {
      time_source->SetAbsolute(target);
    } else {
      const double ppm =
          std::clamp(off * 1e6 / slew_window_sec, -slew_ppm_max, slew_ppm_max);
      time_source->SetRate(1.0 + ppm / 1e6);
    }
    return true;
  }

  ntpserver::TimeSource* time_source;  // not owned
  double slew_ppm_max{500.0};
  double slew_window_sec{10.0};

  void SetSlewPpmMax(double ppm) { slew_ppm_max = ppm; }
  void SetSlewWindowSec(double sec) { slew_window_sec = sec; }
};

NtpClient::NtpClient()
    : impl_(std::make_unique<Impl>(&ntpserver::QpcClock::Instance())) {}
NtpClient::NtpClient(ntpserver::TimeSource* time_source)
    : impl_(std::make_unique<Impl>(time_source)) {}

NtpClient::~NtpClient() = default;

bool NtpClient::SyncOnce(const std::string& host, uint16_t port,
                         int timeout_ms) {
  return impl_->SyncOnce(host, port, timeout_ms);
}

void NtpClient::SetSlewPpmMax(double ppm) { impl_->SetSlewPpmMax(ppm); }
void NtpClient::SetSlewWindowSec(double sec) { impl_->SetSlewWindowSec(sec); }

}  // namespace ntpclient
