// Copyright (c) 2025 <Your Name>
/**
 * @file udp_socket.hpp
 * @brief UDP socket manager for NTP client with background receive thread.
 *
 * Encapsulates UDP socket lifecycle, message classification (Exchange response
 * vs Push notification), and thread-safe message queueing for asynchronous
 * server-initiated notifications.
 */
#pragma once

#include <winsock2.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace ntpclock {
namespace internal {

/**
 * @brief UDP socket manager with background receive thread.
 *
 * Manages a single UDP socket for NTP communication, keeping it open to
 * receive both Exchange responses (request/response) and Push notifications
 * (server-initiated). A background receive thread continuously reads from
 * the socket, classifies messages based on vendor extension flags, and
 * queues them for consumption by the main synchronization loop.
 */
class UdpSocket {
 public:
  /**
   * @brief Message type classification.
   */
  enum class MessageType {
    ExchangeResponse,  ///< Response to a client request
    Push,              ///< Server-initiated notification
  };

  /**
   * @brief Received message with metadata.
   */
  struct Message {
    MessageType type;           ///< Classified message type
    std::vector<uint8_t> data;  ///< Raw packet bytes
    double recv_time;           ///< Reception timestamp (UNIX seconds)
  };

  UdpSocket() = default;
  ~UdpSocket() { Close(); }

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  /**
   * @brief Open socket and start background receive thread.
   *
   * Creates a UDP socket, binds to an ephemeral port, connects to the
   * specified server address, and starts a background thread to receive
   * and classify incoming messages.
   *
   * @param server_ip Server IPv4 address (numeric string, no DNS).
   * @param server_port Server UDP port.
   * @param get_time Callback to get current UNIX time in seconds.
   * @return true on success, false on failure.
   */
  bool Open(const std::string& server_ip, uint16_t server_port,
            std::function<double()> get_time);

  /**
   * @brief Close socket and stop background thread.
   *
   * Signals the receive thread to stop, closes the socket (which unblocks
   * recvfrom), and waits for the thread to terminate. Safe to call
   * multiple times.
   */
  void Close();

  /**
   * @brief Send data to the connected server.
   *
   * @param data Bytes to send (typically an NTP request packet).
   * @return true if send succeeded, false otherwise.
   */
  bool Send(const std::vector<uint8_t>& data);

  /**
   * @brief Wait for a message with timeout.
   *
   * Blocks until a message is available in the queue or the timeout expires.
   * Messages are classified by the background receive thread.
   *
   * @param timeout_ms Maximum time to wait in milliseconds.
   * @param out_msg Pointer to store the received message.
   * @return true if a message was received, false on timeout.
   */
  bool WaitMessage(int timeout_ms, Message* out_msg);

  /**
   * @brief Check if socket is currently open.
   *
   * @return true if socket is open and receive thread is running.
   */
  bool IsOpen() const { return sock_ != INVALID_SOCKET; }

 private:
  /**
   * @brief Background thread loop for receiving and classifying messages.
   *
   * Continuously calls recvfrom() to receive UDP packets, timestamps them,
   * classifies them as ExchangeResponse or Push, and adds them to the
   * message queue. Runs until running_ is set to false.
   */
  void ReceiveLoop();

  /**
   * @brief Classify message type based on vendor extension flags.
   *
   * Parses the NTP packet and vendor extension field to determine if the
   * message is a Push notification (kFlagPush set) or an Exchange response.
   *
   * @param data Raw packet bytes.
   * @return Classified message type.
   */
  MessageType ClassifyMessage(const std::vector<uint8_t>& data);

  SOCKET sock_ = INVALID_SOCKET;      ///< Windows socket handle
  sockaddr_in server_addr_{};         ///< Server address for send()
  std::function<double()> get_time_;  ///< Timestamp callback
  std::atomic<bool> running_{false};  ///< Receive thread control flag
  std::thread recv_thread_;           ///< Background receive thread

  std::mutex queue_mtx_;                   ///< Protects msg_queue_
  std::condition_variable queue_cv_;       ///< Notifies message availability
  std::queue<Message> msg_queue_;          ///< Received message queue
  bool wsa_started_ = false;               ///< WSAStartup state
};

}  // namespace internal
}  // namespace ntpclock
