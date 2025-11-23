// Copyright (c) 2025 The NTP Sample Authors
#include "internal/udp_socket.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ntpserver/ntp_types.hpp"
#include "ntpserver/platform/socket_interface.hpp"

namespace ntpclock {
namespace internal {

namespace {
// Minimum size of an NTP packet (48 bytes)
constexpr size_t kNtpPacketSize = 48;
}  // namespace

UdpSocket::~UdpSocket() { Close(); }

bool UdpSocket::IsOpen() const { return socket_ && socket_->IsValid(); }

bool UdpSocket::Open(const std::string& server_ip, uint16_t server_port,
                     std::function<ntpserver::TimeSpec()> get_time,
                     LogCallback log_callback) {
  if (IsOpen()) {
    return false;  // Already open
  }

  get_time_ = get_time;
  log_callback_ = log_callback;

  // Create platform socket
  socket_ = ntpserver::platform::CreatePlatformSocket();
  if (!socket_->Initialize()) {
    LogError("Socket initialization failed: " + socket_->GetLastError());
    return false;
  }

  // Bind to any local address/ephemeral port (port 0 = OS chooses)
  if (!socket_->Bind(0)) {
    LogError("Socket bind failed: " + socket_->GetLastError());
    socket_->Close();
    socket_.reset();
    return false;
  }

  // Store server endpoint for Send()
  server_endpoint_.address = server_ip;
  server_endpoint_.port = server_port;

  // Start receive thread
  running_.store(true);
  recv_thread_ = std::thread(&UdpSocket::ReceiveLoop, this);

  return true;
}

void UdpSocket::Close() {
  if (!IsOpen()) {
    return;
  }

  // Signal thread to stop
  running_.store(false);

  // Close socket to unblock Receive()
  if (socket_) {
    socket_->Close();
    socket_.reset();
  }

  // Wait for thread to finish
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }

  // Clear message queue
  std::lock_guard<std::mutex> lock(queue_mtx_);
  while (!msg_queue_.empty()) {
    msg_queue_.pop();
  }
}

bool UdpSocket::Send(const std::vector<uint8_t>& data) {
  if (!IsOpen()) {
    return false;
  }

  if (!socket_->Send(server_endpoint_, data)) {
    LogError("Send failed: " + socket_->GetLastError());
    return false;
  }

  return true;
}

bool UdpSocket::WaitMessage(int timeout_ms, Message* out_msg) {
  if (!out_msg) {
    return false;
  }

  std::unique_lock<std::mutex> lock(queue_mtx_);

  // Wait with timeout for message availability
  if (msg_queue_.empty()) {
    if (timeout_ms <= 0) {
      return false;
    }
    auto result = queue_cv_.wait_for(
        lock, std::chrono::milliseconds(timeout_ms),
        [this]() { return !msg_queue_.empty() || !running_.load(); });
    if (!result || msg_queue_.empty()) {
      return false;
    }
  }

  // Pop message from queue
  *out_msg = std::move(msg_queue_.front());
  msg_queue_.pop();
  return true;
}

void UdpSocket::ReceiveLoop() {
  while (running_.load()) {
    // Wait for data with timeout to allow checking running_ flag
    if (!socket_ || !socket_->WaitReadable(200000)) {  // 200ms timeout
      if (!running_.load()) {
        break;  // Shutdown requested
      }
      continue;  // Timeout, check running_ again
    }

    // Receive packet
    ntpserver::platform::Endpoint from;
    std::vector<uint8_t> data;
    if (!socket_->Receive(&from, &data, 1500)) {  // Max UDP payload
      if (running_.load()) {
        // Unexpected error while still running
        LogError("Receive failed: " + socket_->GetLastError());
        continue;
      } else {
        // Socket closed intentionally during shutdown
        break;
      }
    }

    // Ignore packets smaller than minimum NTP packet size
    if (data.size() < kNtpPacketSize) {
      if (!data.empty()) {
        LogError("Received short packet (" + std::to_string(data.size()) +
                 " bytes)");
      }
      continue;
    }

    // Get receive timestamp
    ntpserver::TimeSpec recv_time =
        get_time_ ? get_time_() : ntpserver::TimeSpec{};

    // Classify message type
    MessageType type = ClassifyMessage(data);

    // Add to queue
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      msg_queue_.push(Message{type, std::move(data), recv_time});
    }
    queue_cv_.notify_one();
  }
}

UdpSocket::MessageType UdpSocket::ClassifyMessage(
    const std::vector<uint8_t>& data) {
  // Default to ExchangeResponse
  if (data.size() <= kNtpPacketSize) {
    return MessageType::ExchangeResponse;
  }

  // Check for vendor extension field
  const uint8_t* p = data.data() + kNtpPacketSize;
  size_t remain = data.size() - kNtpPacketSize;

  // Need at least 4 bytes for EF header (type + length)
  if (remain < 4) {
    return MessageType::ExchangeResponse;
  }

  // Parse EF header (big-endian)
  uint16_t ef_type = static_cast<uint16_t>((p[0] << 8) | p[1]);
  uint16_t ef_len = static_cast<uint16_t>((p[2] << 8) | p[3]);

  // Check if this is our vendor hint EF
  if (ef_type != ntpserver::NtpVendorExt::kEfTypeVendorHint || ef_len < 4 ||
      ef_len > remain) {
    return MessageType::ExchangeResponse;
  }

  // Extract vendor payload (skip 4-byte EF header)
  std::vector<uint8_t> payload(p + 4, p + ef_len);

  // Parse vendor extension
  ntpserver::NtpVendorExt::Payload vendor{};
  if (!ntpserver::NtpVendorExt::Parse(payload, &vendor)) {
    return MessageType::ExchangeResponse;
  }

  // Check for Push flag
  if ((vendor.flags & ntpserver::NtpVendorExt::kFlagPush) != 0) {
    return MessageType::Push;
  }

  return MessageType::ExchangeResponse;
}

void UdpSocket::LogError(const std::string& text) {
  if (log_callback_) {
    log_callback_(text);
  }
}

}  // namespace internal
}  // namespace ntpclock
