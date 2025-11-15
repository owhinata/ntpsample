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
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
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
                                int8_t precision, uint32_t ref_id_host,
                                const TimeSpec& t_ref, const TimeSpec& t_recv,
                                const TimeSpec& t_tx, NtpPacket* out) {
  out->li_vn_mode =
      static_cast<uint8_t>((0 << 6) | (4 << 3) | 4);  // LI=0,VN=4,Mode=4
  out->stratum = stratum;
  out->poll = 4;
  out->precision = precision;
  out->root_delay = htonl(0);
  out->root_dispersion = htonl(0);
  out->ref_id = htonl(ref_id_host);

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

  bool Start(uint16_t port, TimeSource* time_source, const Options& options) {
    std::lock_guard<std::mutex> lock(start_stop_mtx_);
    if (running_.load()) return true;

    // Set time source (use QpcClock if nullptr)
    time_source_ = time_source;

    stratum_ = options.Stratum();
    precision_ = options.Precision();
    ref_id_ = options.RefId();
    client_tracker_.SetRetention(options.ClientRetention());
    log_callback_ = options.LogSink();
    packets_received_.store(0, std::memory_order_relaxed);
    packets_sent_.store(0, std::memory_order_relaxed);
    recv_errors_.store(0, std::memory_order_relaxed);
    short_packets_.store(0, std::memory_order_relaxed);
    send_errors_.store(0, std::memory_order_relaxed);
    push_notifications_.store(0, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lk(status_mtx_);
      last_error_.clear();
    }

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

  void NotifyControlSnapshot() {
    TimeSource* ts = time_source_ ? time_source_ : &QpcClock::Instance();
    if (sock_ == INVALID_SOCKET) return;
    const TimeSpec now = ts->NowUnix();

    // Minimal mode-4 response and EF using SRP helpers
    NtpPacket resp{};
    BuildResponsePacket({}, stratum_, precision_, ref_id_, now, now, now,
                        &resp);
    std::vector<uint8_t> ef = MakeVendorEf(ts, now, true);
    std::vector<uint8_t> buf = ComposeWithEf(resp, ef);

    // Prune inactive clients using monotonic time to tolerate absolute jumps.
    client_tracker_.PruneStale(std::chrono::steady_clock::now());

    auto& clients = client_tracker_.GetAllMutable();
    for (auto& client : clients) {
      if (SendBuffer(client.addr, sizeof(client.addr), buf)) {
        push_notifications_.fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  ServerStats GetStats() const {
    ServerStats stats;
    stats.packets_received = packets_received_.load(std::memory_order_relaxed);
    stats.packets_sent = packets_sent_.load(std::memory_order_relaxed);
    stats.recv_errors = recv_errors_.load(std::memory_order_relaxed);
    stats.drop_short_packets = short_packets_.load(std::memory_order_relaxed);
    stats.send_errors = send_errors_.load(std::memory_order_relaxed);
    stats.push_notifications =
        push_notifications_.load(std::memory_order_relaxed);
    stats.active_clients =
        static_cast<uint64_t>(client_tracker_.GetAll().size());
    {
      std::lock_guard<std::mutex> lk(status_mtx_);
      stats.last_error = last_error_;
    }
    return stats;
  }

 private:
  /** Initializes WinSock. */
  bool InitializeWinsock() {
    WSADATA wsa{};
    int err = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (err != 0) {
      RecordError("WSAStartup failed", err);
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
      LogSocketError("socket");
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);
    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) ==
        SOCKET_ERROR) {
      LogSocketError("bind");
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
    if (ready == SOCKET_ERROR) {
      LogSocketError("select");
      return false;
    }
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
    SendBuffer(cli, clen, buf);
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
    if (n == sizeof(*req)) {
      packets_received_.fetch_add(1, std::memory_order_relaxed);
      return true;
    }
    if (n == SOCKET_ERROR) {
      recv_errors_.fetch_add(1, std::memory_order_relaxed);
      LogSocketError("recvfrom");
    } else if (n > 0) {
      short_packets_.fetch_add(1, std::memory_order_relaxed);
      RecordError("short packet", 0);
    }
    return false;
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
    BuildResponsePacket(req, stratum_, precision_, ref_id_, t_ref, tr, t_tx,
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
  bool SendBuffer(const sockaddr_in& cli, int clen,
                  const std::vector<uint8_t>& buf) {
    int sent = sendto(sock_, reinterpret_cast<const char*>(buf.data()),
                      static_cast<int>(buf.size()), 0,
                      reinterpret_cast<const sockaddr*>(&cli), clen);
    if (sent == static_cast<int>(buf.size())) {
      packets_sent_.fetch_add(1, std::memory_order_relaxed);
      return true;
    }
    send_errors_.fetch_add(1, std::memory_order_relaxed);
    LogSocketError("sendto");
    return false;
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
  uint32_t ref_id_{Options::kDefaultRefId};
  uint32_t ctrl_seq_{0};
  Options::LogCallback log_callback_;
  std::atomic<uint64_t> packets_received_{0};
  std::atomic<uint64_t> packets_sent_{0};
  std::atomic<uint64_t> recv_errors_{0};
  std::atomic<uint64_t> short_packets_{0};
  std::atomic<uint64_t> send_errors_{0};
  std::atomic<uint64_t> push_notifications_{0};
  mutable std::mutex status_mtx_;
  std::string last_error_;

  internal::ClientTracker client_tracker_;
  void RecordError(const std::string& msg, int err_code);
  void LogSocketError(const char* ctx);
};

void NtpServer::Impl::RecordError(const std::string& msg, int err_code) {
  std::ostringstream oss;
  if (err_code != 0) {
    oss << msg << " (err=" << err_code << ")";
  } else {
    oss << msg;
  }
  const std::string text = oss.str();
  if (log_callback_) {
    log_callback_(text);
  }
  {
    std::lock_guard<std::mutex> lk(status_mtx_);
    last_error_ = text;
  }
}

void NtpServer::Impl::LogSocketError(const char* ctx) {
  int err = WSAGetLastError();
  RecordError(std::string(ctx) + " failed", err);
}

NtpServer::NtpServer() : impl_(new Impl) {}
NtpServer::~NtpServer() = default;

bool NtpServer::Start(uint16_t port, TimeSource* time_source,
                      const Options& options) {
  return impl_->Start(port, time_source, options);
}
void NtpServer::Stop() { impl_->Stop(); }
ServerStats NtpServer::GetStats() const { return impl_->GetStats(); }
void NtpServer::NotifyControlSnapshot() { impl_->NotifyControlSnapshot(); }

}  // namespace ntpserver
