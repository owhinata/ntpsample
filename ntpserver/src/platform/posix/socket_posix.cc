// Copyright (c) 2025
/**
 * @file socket_posix.cc
 * @brief POSIX (Linux/macOS) implementation of ISocket interface
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ntpserver/platform/socket_interface.hpp"

namespace ntpserver {
namespace platform {

class SocketPosix : public ISocket {
 public:
  SocketPosix() : sock_(-1) {}

  ~SocketPosix() override { Close(); }

  bool Initialize() override {
    // POSIX doesn't require explicit initialization like Winsock
    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ < 0) {
      CaptureErrno("socket creation failed");
      return false;
    }

    return true;
  }

  bool Bind(uint16_t port) override {
    if (sock_ < 0) {
      last_error_ = "Socket not initialized";
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      CaptureErrno("bind failed");
      return false;
    }

    return true;
  }

  bool WaitReadable(int64_t timeout_us) override {
    if (sock_ < 0) {
      last_error_ = "Socket not initialized";
      return false;
    }

    pollfd pfd{};
    pfd.fd = sock_;
    pfd.events = POLLIN;

    // Convert microseconds to milliseconds
    int timeout_ms = static_cast<int>(timeout_us / 1000);

    int ready = poll(&pfd, 1, timeout_ms);

    if (ready < 0) {
      CaptureErrno("poll failed");
      return false;
    }

    // Check if socket is readable
    return ready > 0 && (pfd.revents & POLLIN);
  }

  bool Receive(Endpoint* from, std::vector<uint8_t>* data,
               size_t max_size) override {
    if (sock_ < 0) {
      last_error_ = "Socket not initialized";
      return false;
    }

    data->resize(max_size);
    sockaddr_in addr{};
    socklen_t addrlen = sizeof(addr);

    ssize_t n = recvfrom(sock_, data->data(), max_size, 0,
                         reinterpret_cast<sockaddr*>(&addr), &addrlen);

    if (n < 0) {
      CaptureErrno("recvfrom failed");
      return false;
    }

    if (n == 0) {
      last_error_ = "Connection closed";
      return false;
    }

    // Resize to actual received size
    data->resize(n);

    // Convert address to string
    char ip[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip)) != nullptr) {
      from->address = ip;
    } else {
      from->address = "";
    }
    from->port = ntohs(addr.sin_port);

    return true;
  }

  bool Send(const Endpoint& to, const std::vector<uint8_t>& data) override {
    if (sock_ < 0) {
      last_error_ = "Socket not initialized";
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;

    if (inet_pton(AF_INET, to.address.c_str(), &addr.sin_addr) != 1) {
      last_error_ = "Invalid IP address: " + to.address;
      return false;
    }

    addr.sin_port = htons(to.port);

    ssize_t sent =
        sendto(sock_, data.data(), data.size(), 0,
               reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));

    if (sent < 0) {
      CaptureErrno("sendto failed");
      return false;
    }

    if (sent != static_cast<ssize_t>(data.size())) {
      std::ostringstream oss;
      oss << "Partial send: sent " << sent << " of " << data.size() << " bytes";
      last_error_ = oss.str();
      return false;
    }

    return true;
  }

  void Close() override {
    if (sock_ >= 0) {
      close(sock_);  // POSIX uses close(), not closesocket()
      sock_ = -1;
    }
  }

  std::string GetLastError() const override { return last_error_; }

  bool IsValid() const override { return sock_ >= 0; }

 private:
  void CaptureErrno(const std::string& context) {
    int err = errno;
    std::ostringstream oss;
    oss << context << " (errno " << err << ": " << std::strerror(err) << ")";
    last_error_ = oss.str();
  }

  int sock_;  // POSIX uses int for socket descriptors
  std::string last_error_;
};

// Factory function implementation for POSIX (Linux/macOS)
std::unique_ptr<ISocket> CreatePlatformSocket() {
  return std::unique_ptr<ISocket>(new SocketPosix());
}

}  // namespace platform
}  // namespace ntpserver
