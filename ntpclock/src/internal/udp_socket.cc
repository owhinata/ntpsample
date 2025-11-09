// Copyright (c) 2025 <Your Name>
#include "internal/udp_socket.hpp"

#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "ntpserver/ntp_types.hpp"

namespace ntpclock {
namespace internal {

namespace {
// Minimum size of an NTP packet (48 bytes)
constexpr size_t kNtpPacketSize = 48;
}  // namespace

UdpSocket::~UdpSocket() { Close(); }

bool UdpSocket::IsOpen() const { return sock_ != INVALID_SOCKET; }

bool UdpSocket::Open(const std::string& server_ip, uint16_t server_port,
                     std::function<double()> get_time) {
  if (IsOpen()) {
    return false;  // Already open
  }

  get_time_ = get_time;

  // Initialize Winsock
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    return false;
  }
  wsa_started_ = true;

  // Create UDP socket
  sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock_ == INVALID_SOCKET) {
    WSACleanup();
    wsa_started_ = false;
    return false;
  }

  // Bind to any local address/ephemeral port
  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = 0;  // Let OS choose port
  if (bind(sock_, reinterpret_cast<sockaddr*>(&local_addr),
           sizeof(local_addr)) == SOCKET_ERROR) {
    closesocket(sock_);
    sock_ = INVALID_SOCKET;
    WSACleanup();
    wsa_started_ = false;
    return false;
  }

  // Store server address for Send()
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(server_port);
  if (inet_pton(AF_INET, server_ip.c_str(), &server_addr_.sin_addr) != 1) {
    closesocket(sock_);
    sock_ = INVALID_SOCKET;
    WSACleanup();
    wsa_started_ = false;
    return false;
  }

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

  // Close socket to unblock recvfrom()
  if (sock_ != INVALID_SOCKET) {
    closesocket(sock_);
    sock_ = INVALID_SOCKET;
  }

  // Wait for thread to finish
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }

  // Clean up Winsock
  if (wsa_started_) {
    WSACleanup();
    wsa_started_ = false;
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

  int sent =
      sendto(sock_, reinterpret_cast<const char*>(data.data()),
             static_cast<int>(data.size()), 0,
             reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));
  return sent == static_cast<int>(data.size());
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
  std::vector<uint8_t> buffer(1500);  // Max UDP payload

  while (running_.load()) {
    sockaddr_in from{};
    int fromlen = sizeof(from);
    int recvd = recvfrom(sock_, reinterpret_cast<char*>(buffer.data()),
                         static_cast<int>(buffer.size()), 0,
                         reinterpret_cast<sockaddr*>(&from), &fromlen);

    // Check if socket was closed or error occurred
    if (recvd < 0) {
      if (running_.load()) {
        // Unexpected error while still running
        continue;
      } else {
        // Socket closed intentionally during shutdown
        break;
      }
    }

    // Ignore packets smaller than minimum NTP packet size
    if (recvd < static_cast<int>(kNtpPacketSize)) {
      continue;
    }

    // Get receive timestamp
    double recv_time = get_time_ ? get_time_() : 0.0;

    // Classify message type
    std::vector<uint8_t> data(buffer.begin(), buffer.begin() + recvd);
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

}  // namespace internal
}  // namespace ntpclock
