// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_server.cc
 * @brief Minimal NTPv4 server implementation (cross-platform UDP/IPv4).
 */
#include "ntpserver/ntp_server.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "internal/client_tracker.hpp"
#include "ntpserver/ntp_types.hpp"
#include "ntpserver/platform/default_time_source.hpp"
#include "ntpserver/platform/socket_interface.hpp"
#include "platform/common/socket_utils.hpp"

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
                                const TimeSpec& t_tx, NtpPacket* out,
                                uint8_t mode = 4) {
  out->li_vn_mode =
      static_cast<uint8_t>((0 << 6) | (4 << 3) | mode);  // LI=0,VN=4,Mode=mode
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

class StatsTracker {
 public:
  void Reset() {
    packets_received_.store(0, std::memory_order_relaxed);
    packets_sent_.store(0, std::memory_order_relaxed);
    recv_errors_.store(0, std::memory_order_relaxed);
    short_packets_.store(0, std::memory_order_relaxed);
    send_errors_.store(0, std::memory_order_relaxed);
    push_notifications_.store(0, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(last_error_mtx_);
    last_error_.clear();
  }

  void IncPacketsReceived() {
    packets_received_.fetch_add(1, std::memory_order_relaxed);
  }
  void IncPacketsSent() {
    packets_sent_.fetch_add(1, std::memory_order_relaxed);
  }
  void IncRecvErrors() { recv_errors_.fetch_add(1, std::memory_order_relaxed); }
  void IncShortPackets() {
    short_packets_.fetch_add(1, std::memory_order_relaxed);
  }
  void IncSendErrors() { send_errors_.fetch_add(1, std::memory_order_relaxed); }
  void IncPushNotifications() {
    push_notifications_.fetch_add(1, std::memory_order_relaxed);
  }
  void SetLastError(const std::string& text) {
    std::lock_guard<std::mutex> lk(last_error_mtx_);
    last_error_ = text;
  }

  ServerStats Snapshot(uint64_t active_clients) const {
    ServerStats stats;
    stats.packets_received = packets_received_.load(std::memory_order_relaxed);
    stats.packets_sent = packets_sent_.load(std::memory_order_relaxed);
    stats.recv_errors = recv_errors_.load(std::memory_order_relaxed);
    stats.drop_short_packets = short_packets_.load(std::memory_order_relaxed);
    stats.send_errors = send_errors_.load(std::memory_order_relaxed);
    stats.push_notifications =
        push_notifications_.load(std::memory_order_relaxed);
    stats.active_clients = active_clients;
    std::lock_guard<std::mutex> lk(last_error_mtx_);
    stats.last_error = last_error_;
    return stats;
  }

 private:
  std::atomic<uint64_t> packets_received_{0};
  std::atomic<uint64_t> packets_sent_{0};
  std::atomic<uint64_t> recv_errors_{0};
  std::atomic<uint64_t> short_packets_{0};
  std::atomic<uint64_t> send_errors_{0};
  std::atomic<uint64_t> push_notifications_{0};
  mutable std::mutex last_error_mtx_;
  std::string last_error_;
};

}  // namespace

class NtpServer::Impl {
 public:
  Impl() = default;
  ~Impl() { Stop(); }

  bool Start(uint16_t port, TimeSource* time_source, const Options& options) {
    std::lock_guard<std::mutex> lock(start_stop_mtx_);
    if (running_.load()) return true;

    // Increment epoch number
    epoch_++;

    // Set time source (use QpcClock if nullptr)
    time_source_ = time_source;

    // Capture ABS/RATE values for this epoch
    TimeSource* ts =
        time_source_ ? time_source_ : &platform::GetDefaultTimeSource();
    epoch_abs_ = ts->NowUnix();
    epoch_rate_ = ts->GetRate();

    if (log_callback_) {
      std::ostringstream oss;
      oss << "[DEBUG Server] Start: epoch=" << epoch_
          << " epoch_abs_=" << epoch_abs_.sec << "." << std::setw(9)
          << std::setfill('0') << epoch_abs_.nsec
          << " epoch_rate_=" << epoch_rate_;
      log_callback_(oss.str());
    }

    stratum_ = options.Stratum();
    precision_ = options.Precision();
    ref_id_ = options.RefId();
    client_tracker_.SetRetention(options.ClientRetention());
    log_callback_ = options.LogSink();
    // Don't call stats_.Reset() to persist statistics across Start/Stop
    if (!CreateAndBindSocket(port)) {
      return false;
    }

    // Push if ClientTracker not empty
    if (!client_tracker_.GetAll().empty()) {
      NotifyControlSnapshot();
    }

    running_.store(true);
    thread_ = std::thread([this]() { Loop(); });
    return true;
  }

  void Stop() {
    std::lock_guard<std::mutex> lock(start_stop_mtx_);
    if (!running_.exchange(false)) return;

    if (thread_.joinable()) {
      // Wake WaitReadable by setting a short timeout before join
      thread_.join();
    }
    if (socket_) {
      socket_->Close();
      socket_.reset();
    }
  }

  void NotifyControlSnapshot() {
    TimeSource* ts =
        time_source_ ? time_source_ : &platform::GetDefaultTimeSource();
    if (!socket_ || !socket_->IsValid()) return;
    const TimeSpec now = ts->NowUnix();

    // mode=5 (broadcast) for Push notifications
    NtpPacket resp{};
    BuildResponsePacket({}, stratum_, precision_, ref_id_, now, now, now, &resp,
                        5);
    std::vector<uint8_t> ef = MakeVendorEf(now, true);
    std::vector<uint8_t> buf = ComposeNtpPacketWithEf(resp, ef);

    // Prune inactive clients using monotonic time to tolerate absolute jumps.
    client_tracker_.PruneStale(std::chrono::steady_clock::now());

    auto clients = client_tracker_.GetAll();
    for (const auto& client : clients) {
      if (SendBuffer(client.addr, sizeof(client.addr), buf)) {
        stats_.IncPushNotifications();
      }
    }
  }

  ServerStats GetStats() const {
    return stats_.Snapshot(client_tracker_.GetAll().size());
  }

 private:
  /** Creates UDP socket and binds to given port. */
  bool CreateAndBindSocket(uint16_t port) {
    socket_ = platform::CreatePlatformSocket();
    if (!socket_->Initialize()) {
      RecordError("Socket initialization failed: " + socket_->GetLastError(),
                  0);
      return false;
    }

    if (!socket_->Bind(port)) {
      RecordError("Socket bind failed: " + socket_->GetLastError(), 0);
      socket_->Close();
      socket_.reset();
      return false;
    }
    return true;
  }

  /** Main loop: wait for datagrams and respond. */
  void Loop() {
    TimeSource* ts =
        time_source_ ? time_source_ : &platform::GetDefaultTimeSource();

    while (running_.load()) {
      if (!WaitReadable(/*timeout_us=*/200000)) continue;
      HandleSingleDatagram(ts);
    }
  }

  /** Waits for readability with a microsecond timeout. */
  bool WaitReadable(int64_t timeout_us) {
    if (!socket_ || !socket_->IsValid()) return false;
    return socket_->WaitReadable(timeout_us);
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
    std::vector<uint8_t> ef = MakeVendorEf(ts->NowUnix(), false);
    std::vector<uint8_t> buf = ComposeNtpPacketWithEf(resp, ef);
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
    if (!socket_ || !socket_->IsValid()) {
      stats_.IncRecvErrors();
      RecordError("Socket not valid", 0);
      return false;
    }

    platform::Endpoint from;
    std::vector<uint8_t> data;
    if (!socket_->Receive(&from, &data, sizeof(*req))) {
      stats_.IncRecvErrors();
      RecordError("Receive failed: " + socket_->GetLastError(), 0);
      return false;
    }

    if (data.size() == sizeof(*req)) {
      std::memcpy(req, data.data(), sizeof(*req));
      // Convert Endpoint back to sockaddr_in for ClientTracker
      platform::EndpointToSockaddr(from, cli);
      *clen = sizeof(*cli);
      stats_.IncPacketsReceived();
      return true;
    } else if (data.size() > 0) {
      stats_.IncShortPackets();
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
   * @param server_now Server absolute time to embed.
   * @param is_push    True for Push notifications, false for Exchange
   * responses.
   */
  std::vector<uint8_t> MakeVendorEf(const TimeSpec& server_now, bool is_push) {
    NtpVendorExt::Payload v{};
    v.seq = epoch_;  // Use epoch number (not incrementing)
    v.flags = NtpVendorExt::kFlagAbs | NtpVendorExt::kFlagRate;
    // Note: PUSH flag will be deprecated in favor of mode=5
    if (is_push) {
      v.flags |= NtpVendorExt::kFlagPush;
    }
    v.server_time = server_now;
    v.abs_time = epoch_abs_;     // Use ABS captured at epoch start
    v.rate_scale = epoch_rate_;  // Use RATE captured at epoch start

    if (log_callback_) {
      std::ostringstream oss;
      oss << "[DEBUG Server] MakeVendorEf: epoch=" << epoch_
          << " abs_time=" << v.abs_time.sec << "." << std::setw(9)
          << std::setfill('0') << v.abs_time.nsec
          << " server_time=" << v.server_time.sec << "." << std::setw(9)
          << std::setfill('0') << v.server_time.nsec
          << " rate=" << v.rate_scale;
      log_callback_(oss.str());
    }

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
   * @brief Send raw response buffer to client.
   */
  bool SendBuffer(const sockaddr_in& cli, int clen,
                  const std::vector<uint8_t>& buf) {
    (void)clen;  // Unused parameter
    if (!socket_ || !socket_->IsValid()) {
      stats_.IncSendErrors();
      RecordError("Socket not valid", 0);
      return false;
    }

    // Convert sockaddr_in to Endpoint
    platform::Endpoint to = platform::SockaddrToEndpoint(cli);

    if (!socket_->Send(to, buf)) {
      stats_.IncSendErrors();
      RecordError("Send failed: " + socket_->GetLastError(), 0);
      return false;
    }

    stats_.IncPacketsSent();
    return true;
  }

  void RememberClient(const sockaddr_in& cli, const TimeSpec& now) {
    client_tracker_.Remember(cli, now);
  }

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::unique_ptr<platform::ISocket> socket_;
  std::mutex start_stop_mtx_;

  TimeSource* time_source_{nullptr};
  uint8_t stratum_{1};
  int8_t precision_{-20};
  uint32_t ref_id_{Options::kDefaultRefId};
  uint32_t epoch_{0};       // Epoch number (incremented on Start())
  TimeSpec epoch_abs_{};    // ABS value captured at epoch start
  double epoch_rate_{1.0};  // RATE value captured at epoch start
  Options::LogCallback log_callback_;
  StatsTracker stats_;

  internal::ClientTracker client_tracker_;
  void RecordError(const std::string& msg, int err_code);
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
  stats_.SetLastError(text);
}

NtpServer::NtpServer() : impl_(new Impl) {}
NtpServer::~NtpServer() = default;

bool NtpServer::Start(uint16_t port, TimeSource* time_source,
                      const Options& options) {
  return impl_->Start(port, time_source, options);
}
void NtpServer::Stop() { impl_->Stop(); }
ServerStats NtpServer::GetStats() const { return impl_->GetStats(); }

}  // namespace ntpserver
