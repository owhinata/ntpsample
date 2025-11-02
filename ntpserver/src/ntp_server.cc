// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_server.cc
 * @brief Minimal NTPv4 server implementation for Windows (UDP/IPv4).
 */
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

/** Returns v converted to big-endian (network) order. */
inline uint64_t Hton64(uint64_t v) {
  uint32_t hi = htonl(static_cast<uint32_t>(v >> 32));
  uint32_t lo = htonl(static_cast<uint32_t>(v & 0xFFFFFFFFULL));
  return (static_cast<uint64_t>(lo) << 32) | hi;
}

/** Converts UNIX seconds to 64-bit NTP timestamp (seconds.fraction). */
inline uint64_t ToNtpTimestamp(double unix_seconds) {
  double sec;
  double frac =
      std::modf(unix_seconds + static_cast<double>(kNtpUnixEpochDiff), &sec);
  uint64_t s = static_cast<uint64_t>(sec);
  uint64_t f = static_cast<uint64_t>(frac * static_cast<double>(1ULL << 32));
  return (s << 32) | f;
}

/**
 * Fills an NTP response packet from request and times.
 * Responsibility: formatting header fields and timestamps only.
 */
inline void BuildResponsePacket(const NtpPacket& req, uint8_t stratum,
                                int8_t precision, uint32_t ref_id_be,
                                double t_ref, double t_recv, double t_tx,
                                NtpPacket* out) {
  out->li_vn_mode =
      static_cast<uint8_t>((0 << 6) | (4 << 3) | 4);  // LI=0,VN=4,Mode=4
  out->stratum = stratum;
  out->poll = 4;
  out->precision = precision;
  out->root_delay = htonl(0);
  out->root_dispersion = htonl(0);
  out->ref_id = ref_id_be;

  out->ref_timestamp = Hton64(ToNtpTimestamp(t_ref));
  out->orig_timestamp = req.tx_timestamp;  // echo client's transmit timestamp
  out->recv_timestamp = Hton64(ToNtpTimestamp(t_recv));
  out->tx_timestamp = Hton64(ToNtpTimestamp(t_tx));
}
}  // namespace

class NtpServer::Impl {
 public:
  Impl() = default;
  ~Impl() { Stop(); }

  bool Start(uint16_t port) {
    if (running_.load()) return true;
    if (!InitializeWinsock()) return false;
    if (!CreateAndBindSocket(port)) {
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
  /** Initializes WinSock. */
  bool InitializeWinsock() {
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
      return false;
    }
    wsa_started_ = true;
    return true;
  }

  /** Releases WinSock if it was initialized. */
  void CleanupWinsock() {
    if (wsa_started_) {
      WSACleanup();
      wsa_started_ = false;
    }
  }

  /** Creates UDP socket and binds to given port. */
  bool CreateAndBindSocket(uint16_t port) {
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
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
      return false;
    }
    return true;
  }

  /** Main loop: wait for datagrams and respond. */
  void Loop() {
    TimeSource* ts = time_source_ ? time_source_ : &UserTime::Instance();

    while (running_.load()) {
      if (!WaitReadable(/*timeout_us=*/200000)) continue;
      HandleSingleDatagram(ts);
    }
  }

  /** Waits for readability with a microsecond timeout. */
  bool WaitReadable(int64_t timeout_us) {
    if (sock_ == INVALID_SOCKET) return false;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_, &rfds);
    timeval tv{};
    tv.tv_sec = static_cast<decltype(tv.tv_sec)>(timeout_us / 1000000);
    tv.tv_usec = static_cast<decltype(tv.tv_usec)>(timeout_us % 1000000);
    int ready = select(0, &rfds, nullptr, nullptr, &tv);
    return ready > 0;
  }

  /** Receives one datagram and sends a response. */
  void HandleSingleDatagram(TimeSource* ts) {
    sockaddr_in cli{};
    int clen = sizeof(cli);
    NtpPacket req{};
    int n = recvfrom(sock_, reinterpret_cast<char*>(&req), sizeof(req), 0,
                     reinterpret_cast<sockaddr*>(&cli), &clen);
    if (n <= 0) return;

    const double t_recv = ts->NowUnix();
    const double t_ref = t_recv;
    const double t_tx = ts->NowUnix();

    NtpPacket resp{};
    BuildResponsePacket(req, stratum_, precision_, ref_id_be_, t_ref, t_recv,
                        t_tx, &resp);

    sendto(sock_, reinterpret_cast<const char*>(&resp), sizeof(resp), 0,
           reinterpret_cast<sockaddr*>(&cli), clen);
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
