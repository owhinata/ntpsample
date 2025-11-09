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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "internal/client_tracker.hpp"
#include "ntpserver/ntp_types.hpp"
#include "ntpserver/qpc_clock.hpp"

#pragma comment(lib, "Ws2_32.lib")

namespace ntpserver {

namespace {

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
                                const TimeSpec& t_ref, const TimeSpec& t_recv,
                                const TimeSpec& t_tx, NtpPacket* out) {
  out->li_vn_mode =
      static_cast<uint8_t>((0 << 6) | (4 << 3) | 4);  // LI=0,VN=4,Mode=4
  out->stratum = stratum;
  out->poll = 4;
  out->precision = precision;
  out->root_delay = htonl(0);
  out->root_dispersion = htonl(0);
  out->ref_id = ref_id_be;

  out->ref_timestamp = Hton64(t_ref.ToNtpTimestamp());
  out->orig_timestamp = req.tx_timestamp;  // echo client's transmit timestamp
  out->recv_timestamp = Hton64(t_recv.ToNtpTimestamp());
  out->tx_timestamp = Hton64(t_tx.ToNtpTimestamp());
}
}  // namespace

class NtpServer::Impl {
 public:
  Impl() = default;
  ~Impl() { Stop(); }

  bool Start(uint16_t port) {
    std::lock_guard<std::mutex> lock(start_stop_mtx_);
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
    std::lock_guard<std::mutex> lock(start_stop_mtx_);
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

  void NotifyControlSnapshot() {
    TimeSource* ts = time_source_ ? time_source_ : &QpcClock::Instance();
    if (sock_ == INVALID_SOCKET) return;
    const TimeSpec now = ts->NowUnix();

    // Minimal mode-4 response and EF using SRP helpers
    NtpPacket resp{};
    BuildResponsePacket({}, stratum_, precision_, ref_id_be_, now, now, now,
                        &resp);
    std::vector<uint8_t> ef = MakeVendorEf(ts, now, true);
    std::vector<uint8_t> buf = ComposeWithEf(resp, ef);

    // prune and send
    TimeSpec cutoff_offset = TimeSpec::FromDouble(60.0);
    TimeSpec cutoff = now - cutoff_offset;
    auto& clients = client_tracker_.GetAllMutable();
    auto it = clients.begin();
    while (it != clients.end()) {
      if (it->last_seen < cutoff) {
        it = clients.erase(it);
        continue;
      }
      sendto(sock_, reinterpret_cast<const char*>(buf.data()),
             static_cast<int>(buf.size()), 0,
             reinterpret_cast<sockaddr*>(&it->addr), sizeof(it->addr));
      ++it;
    }
  }

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
    TimeSource* ts = time_source_ ? time_source_ : &QpcClock::Instance();

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

  /**
   * @brief Receives one NTP datagram and sends a response.
   * @details Orchestrates receive -> build response -> append EF -> remember
   *          client -> send.
   */
  void HandleSingleDatagram(TimeSource* ts) {
    sockaddr_in cli{};
    int clen = sizeof(cli);
    NtpPacket req{};
    if (!ReceiveNtp(&cli, &clen, &req)) return;

    TimeSpec t_recv{};
    NtpPacket resp = MakeResponse(req, ts, &t_recv);
    std::vector<uint8_t> ef = MakeVendorEf(ts, ts->NowUnix(), false);
    std::vector<uint8_t> buf = ComposeWithEf(resp, ef);
    RememberClient(cli, t_recv);
    SendBuf(cli, clen, buf);
  }

  /**
   * @brief Receive one NTP request from socket.
   * @param cli  Output: client address.
   * @param clen Input/Output: address length.
   * @param req  Output: NTP request.
   * @return true if a datagram was received.
   */
  bool ReceiveNtp(sockaddr_in* cli, int* clen, NtpPacket* req) {
    int n = recvfrom(sock_, reinterpret_cast<char*>(req), sizeof(*req), 0,
                     reinterpret_cast<sockaddr*>(cli), clen);
    return n > 0;
  }

  /**
   * @brief Build an NTP response packet (no extension fields).
   * @param req     Client request packet.
   * @param ts      Time source used for timestamps.
   * @param t_recv  Output: receive time used in response.
   */
  NtpPacket MakeResponse(const NtpPacket& req, TimeSource* ts,
                         TimeSpec* t_recv) {
    const TimeSpec tr = ts->NowUnix();
    const TimeSpec t_ref = tr;
    const TimeSpec t_tx = ts->NowUnix();
    if (t_recv) *t_recv = tr;
    NtpPacket resp{};
    BuildResponsePacket(req, stratum_, precision_, ref_id_be_, t_ref, tr, t_tx,
                        &resp);
    return resp;
  }

  /**
   * @brief Build vendor extension field (ABS+RATE) bytes.
   * @param ts         Time source to query rate.
   * @param server_now Server absolute time to embed.
   * @param is_push    True for Push notifications, false for Exchange
   * responses.
   */
  std::vector<uint8_t> MakeVendorEf(TimeSource* ts, const TimeSpec& server_now,
                                    bool is_push) {
    NtpVendorExt::Payload v{};
    v.seq = ++ctrl_seq_;
    v.flags = NtpVendorExt::kFlagAbs | NtpVendorExt::kFlagRate;
    if (is_push) {
      v.flags |= NtpVendorExt::kFlagPush;
    }
    v.server_time = server_now;
    v.abs_time = server_now;
    v.rate_scale = ts->GetRate();
    std::vector<uint8_t> val = NtpVendorExt::Serialize(v);

    const uint16_t typ = NtpVendorExt::kEfTypeVendorHint;
    const uint16_t len = static_cast<uint16_t>(val.size() + 4);
    std::vector<uint8_t> ef;
    ef.reserve(4 + val.size());
    ef.push_back(static_cast<uint8_t>((typ >> 8) & 0xFF));
    ef.push_back(static_cast<uint8_t>(typ & 0xFF));
    ef.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
    ef.push_back(static_cast<uint8_t>(len & 0xFF));
    ef.insert(ef.end(), val.begin(), val.end());
    return ef;
  }

  /**
   * @brief Concatenate response header and EF bytes.
   */
  std::vector<uint8_t> ComposeWithEf(const NtpPacket& resp,
                                     const std::vector<uint8_t>& ef) {
    std::vector<uint8_t> buf(sizeof(resp) + ef.size());
    std::memcpy(buf.data(), &resp, sizeof(resp));
    if (!ef.empty())
      std::memcpy(buf.data() + sizeof(resp), ef.data(), ef.size());
    return buf;
  }

  /**
   * @brief Send raw response buffer to client.
   */
  void SendBuf(const sockaddr_in& cli, int clen,
               const std::vector<uint8_t>& buf) {
    sendto(sock_, reinterpret_cast<const char*>(buf.data()),
           static_cast<int>(buf.size()), 0,
           reinterpret_cast<const sockaddr*>(&cli), clen);
  }

  void RememberClient(const sockaddr_in& cli, const TimeSpec& now) {
    client_tracker_.Remember(cli, now);
  }

  std::thread thread_;
  std::atomic<bool> running_{false};
  SOCKET sock_{INVALID_SOCKET};
  bool wsa_started_{false};
  std::mutex start_stop_mtx_;

  TimeSource* time_source_{nullptr};
  uint8_t stratum_{1};
  int8_t precision_{-20};
  uint32_t ref_id_be_{htonl(0x4C4F434C)};  // "LOCL"
  uint32_t ctrl_seq_{0};

  internal::ClientTracker client_tracker_;
};

NtpServer::NtpServer() : impl_(new Impl) {}
NtpServer::~NtpServer() = default;

bool NtpServer::Start(uint16_t port) { return impl_->Start(port); }
void NtpServer::Stop() { impl_->Stop(); }
void NtpServer::SetTimeSource(TimeSource* src) { impl_->SetTimeSource(src); }
void NtpServer::SetStratum(uint8_t s) { impl_->SetStratum(s); }
void NtpServer::SetPrecision(int8_t p) { impl_->SetPrecision(p); }
void NtpServer::SetRefId(uint32_t ref_id_be) { impl_->SetRefId(ref_id_be); }
void NtpServer::NotifyControlSnapshot() { impl_->NotifyControlSnapshot(); }

}  // namespace ntpserver
