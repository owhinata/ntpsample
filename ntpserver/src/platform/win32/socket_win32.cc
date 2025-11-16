// Copyright (c) 2025
/**
 * @file socket_win32.cc
 * @brief Windows Winsock2 implementation of ISocket interface
 */
#include <winsock2.h>
#include <ws2tcpip.h>

#include "ntpserver/platform/socket_interface.hpp"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ntpserver {
namespace platform {

namespace {

// RAII wrapper for Winsock initialization
class WinsockSession {
 public:
  WinsockSession() : initialized_(false), error_(0) {}

  ~WinsockSession() { Cleanup(); }

  bool Initialize() {
    if (initialized_) return true;

    WSADATA wsa{};
    int err = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (err != 0) {
      error_ = err;
      return false;
    }
    initialized_ = true;
    error_ = 0;
    return true;
  }

  void Cleanup() {
    if (initialized_) {
      WSACleanup();
      initialized_ = false;
    }
  }

  int GetLastError() const { return error_; }

 private:
  bool initialized_;
  int error_;
};

}  // namespace

class SocketWin32 : public ISocket {
 public:
  SocketWin32() : sock_(INVALID_SOCKET), winsock_() {}

  ~SocketWin32() override { Close(); }

  bool Initialize() override {
    if (!winsock_.Initialize()) {
      std::ostringstream oss;
      oss << "WSAStartup failed with error: " << winsock_.GetLastError();
      last_error_ = oss.str();
      return false;
    }

    sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET) {
      CaptureWinsockError("socket creation failed");
      return false;
    }

    return true;
  }

  bool Bind(uint16_t port) override {
    if (sock_ == INVALID_SOCKET) {
      last_error_ = "Socket not initialized";
      return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) ==
        SOCKET_ERROR) {
      CaptureWinsockError("bind failed");
      return false;
    }

    return true;
  }

  bool WaitReadable(int64_t timeout_us) override {
    if (sock_ == INVALID_SOCKET) {
      last_error_ = "Socket not initialized";
      return false;
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_, &rfds);

    timeval tv{};
    tv.tv_sec = static_cast<long>(timeout_us / 1000000);  // NOLINT(runtime/int)
    tv.tv_usec =
        static_cast<long>(timeout_us % 1000000);  // NOLINT(runtime/int)

    // Windows: first parameter to select() is ignored
    int ready = select(0, &rfds, nullptr, nullptr, &tv);

    if (ready == SOCKET_ERROR) {
      CaptureWinsockError("select failed");
      return false;
    }

    return ready > 0;
  }

  bool Receive(Endpoint* from, std::vector<uint8_t>* data,
               size_t max_size) override {
    if (sock_ == INVALID_SOCKET) {
      last_error_ = "Socket not initialized";
      return false;
    }

    data->resize(max_size);
    sockaddr_in addr{};
    int addrlen = sizeof(addr);

    int n = recvfrom(sock_, reinterpret_cast<char*>(data->data()),
                     static_cast<int>(max_size), 0,
                     reinterpret_cast<sockaddr*>(&addr), &addrlen);

    if (n == SOCKET_ERROR) {
      CaptureWinsockError("recvfrom failed");
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
    if (sock_ == INVALID_SOCKET) {
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

    int sent = sendto(sock_, reinterpret_cast<const char*>(data.data()),
                      static_cast<int>(data.size()), 0,
                      reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));

    if (sent == SOCKET_ERROR) {
      CaptureWinsockError("sendto failed");
      return false;
    }

    if (sent != static_cast<int>(data.size())) {
      std::ostringstream oss;
      oss << "Partial send: sent " << sent << " of " << data.size() << " bytes";
      last_error_ = oss.str();
      return false;
    }

    return true;
  }

  void Close() override {
    if (sock_ != INVALID_SOCKET) {
      closesocket(sock_);
      sock_ = INVALID_SOCKET;
    }
    winsock_.Cleanup();
  }

  std::string GetLastError() const override { return last_error_; }

  bool IsValid() const override { return sock_ != INVALID_SOCKET; }

 private:
  void CaptureWinsockError(const std::string& context) {
    int err = WSAGetLastError();
    std::ostringstream oss;
    oss << context << " (WSA error: " << err << ")";
    last_error_ = oss.str();
  }

  SOCKET sock_;
  WinsockSession winsock_;
  std::string last_error_;
};

// Factory function implementation for Windows
std::unique_ptr<ISocket> CreatePlatformSocket() {
  return std::unique_ptr<ISocket>(new SocketWin32());
}

}  // namespace platform
}  // namespace ntpserver
