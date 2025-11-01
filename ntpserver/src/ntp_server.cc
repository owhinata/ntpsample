// Copyright (c) 2025 <Your Name>
// Minimal NTPv4 server implementation for Windows (UDP/IPv4)
#include "ntpserver/ntp_server.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>

#include "ntpserver/user_time.hpp"

#pragma comment(lib, "Ws2_32.lib")

namespace ntpserver {

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

inline uint64_t hton64(uint64_t v) {
  uint32_t hi = htonl(static_cast<uint32_t>(v >> 32));
  uint32_t lo = htonl(static_cast<uint32_t>(v & 0xFFFFFFFFULL));
  return (static_cast<uint64_t>(lo) << 32) | hi;
}

inline uint64_t to_ntp_timestamp(double unix_seconds) {
  double sec;
  double frac =
      std::modf(unix_seconds + static_cast<double>(kNtpUnixEpochDiff), &sec);
  uint64_t s = static_cast<uint64_t>(sec);
  uint64_t f = static_cast<uint64_t>(frac * static_cast<double>(1ULL << 32));
  return (s << 32) | f;
}
}  // namespace

class NtpServer::Impl {
 public:
  Impl() = default;
  ~Impl() { Stop(); }

  bool Start(uint16_t port) {
    if (running_.load()) return true;

    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      return false;
    }
    wsa_started_ = true;

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
      CleanupWinsock();
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) ==
        SOCKET_ERROR) {
      closesocket(sock_);
      sock_ = INVALID_SOCKET;
      CleanupWinsock();
      return false;
    }

    running_.store(true);
    thread_ = std::thread([this]() { Loop(); });
    return true;
  }

  void Stop() {
    if (!running_.exchange(false)) return;

    if (thread_.joinable()) {
      // Wake select by setting a short timeout and using closesocket after
      // join.
      thread_.join();
    }
    if (sock_ != INVALID_SOCKET) {
      closesocket(sock_);
      sock_ = INVALID_SOCKET;
    }
    CleanupWinsock();
  }

  void SetTimeSource(TimeSource* src) { time_source_ = src; }
  void SetStratum(uint8_t s) { stratum_ = s; }
  void SetPrecision(int8_t p) { precision_ = p; }
  void SetRefId(uint32_t ref_be) { ref_id_be_ = ref_be; }

 private:
  void CleanupWinsock() {
    if (wsa_started_) {
      WSACleanup();
      wsa_started_ = false;
    }
  }

  void Loop() {
    TimeSource* ts = time_source_ ? time_source_ : &UserTime::Instance();

    while (running_.load()) {
      fd_set rfds;
      FD_ZERO(&rfds);
      if (sock_ == INVALID_SOCKET) break;
      FD_SET(sock_, &rfds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 200000;  // 200ms tick
      int ready = select(0, &rfds, nullptr, nullptr, &tv);
      if (!running_.load()) break;
      if (ready <= 0) continue;  // timeout or error; continue

      sockaddr_in cli{};
      int clen = sizeof(cli);
      NtpPacket req{};
      int n = recvfrom(sock_, reinterpret_cast<char*>(&req), sizeof(req), 0,
                       reinterpret_cast<sockaddr*>(&cli), &clen);
      if (n <= 0) continue;

      const double t_recv = ts->NowUnix();

      NtpPacket resp{};
      resp.li_vn_mode =
          static_cast<uint8_t>((0 << 6) | (4 << 3) | 4);  // LI=0,VN=4,Mode=4
      resp.stratum = stratum_;
      resp.poll = 4;
      resp.precision = precision_;
      resp.root_delay = htonl(0);
      resp.root_dispersion = htonl(0);
      resp.ref_id = ref_id_be_;

      const double t_ref = t_recv;
      resp.ref_timestamp = hton64(to_ntp_timestamp(t_ref));
      resp.orig_timestamp =
          req.tx_timestamp;  // echo client's transmit timestamp
      resp.recv_timestamp = hton64(to_ntp_timestamp(t_recv));

      const double t_tx = ts->NowUnix();
      resp.tx_timestamp = hton64(to_ntp_timestamp(t_tx));

      sendto(sock_, reinterpret_cast<const char*>(&resp), sizeof(resp), 0,
             reinterpret_cast<sockaddr*>(&cli), clen);
    }
  }

  std::thread thread_;
  std::atomic<bool> running_{false};
  SOCKET sock_{INVALID_SOCKET};
  bool wsa_started_{false};

  TimeSource* time_source_{nullptr};
  uint8_t stratum_{1};
  int8_t precision_{-20};
  uint32_t ref_id_be_{htonl(0x4C4F434C)};  // "LOCL"
};

NtpServer::NtpServer() : impl_(new Impl) {}
NtpServer::~NtpServer() = default;

bool NtpServer::Start(uint16_t port) { return impl_->Start(port); }
void NtpServer::Stop() { impl_->Stop(); }
void NtpServer::SetTimeSource(TimeSource* src) { impl_->SetTimeSource(src); }
void NtpServer::SetStratum(uint8_t s) { impl_->SetStratum(s); }
void NtpServer::SetPrecision(int8_t p) { impl_->SetPrecision(p); }
void NtpServer::SetRefId(uint32_t ref_id_be) { impl_->SetRefId(ref_id_be); }

}  // namespace ntpserver
