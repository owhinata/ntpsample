#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ntpserver {
namespace platform {

// Endpoint information (IP address and port)
struct Endpoint {
  std::string address;  // IPv4 address string (e.g., "192.168.1.1")
  uint16_t port;

  Endpoint() : port(0) {}
  Endpoint(const std::string& addr, uint16_t p) : address(addr), port(p) {}
};

// Platform-independent socket interface
class ISocket {
 public:
  virtual ~ISocket() = default;

  // Initialize socket (including platform-specific initialization)
  // Returns: true on success, false on failure
  virtual bool Initialize() = 0;

  // Bind to specified port
  // port: Port number to bind to
  // Returns: true on success, false on failure
  virtual bool Bind(uint16_t port) = 0;

  // Wait for readable data (with timeout)
  // timeout_us: Timeout in microseconds
  // Returns: true if data is ready, false on timeout or error
  virtual bool WaitReadable(int64_t timeout_us) = 0;

  // Receive datagram
  // from: Stores sender endpoint information (output parameter)
  // data: Buffer to store received data (output parameter)
  // max_size: Maximum size to receive (bytes)
  // Returns: true on success, false on failure
  virtual bool Receive(Endpoint* from, std::vector<uint8_t>* data,
                       size_t max_size) = 0;

  // Send datagram
  // to: Destination endpoint information
  // data: Data to send
  // Returns: true on success, false on failure
  virtual bool Send(const Endpoint& to, const std::vector<uint8_t>& data) = 0;

  // Close socket (including platform-specific cleanup)
  virtual void Close() = 0;

  // Get description of last error
  // Returns: Error message string
  virtual std::string GetLastError() const = 0;

  // Check if socket is valid
  // Returns: true if valid, false if invalid
  virtual bool IsValid() const = 0;
};

// Factory function to create platform-specific socket implementation
// Returns: Instance of ISocket implementation for the platform
std::unique_ptr<ISocket> CreatePlatformSocket();

}  // namespace platform
}  // namespace ntpserver
