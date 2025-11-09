// Copyright (c) 2025 <Your Name>
/**
 * @file client_tracker.hpp
 * @brief Active NTP client endpoint tracking for server notifications.
 *
 * Maintains a list of recently seen client endpoints to support
 * server-initiated notifications when configuration changes occur
 * (e.g., time source updates via vendor extension fields).
 */
#pragma once

#include <winsock2.h>

#include <vector>

#include "ntpserver/time_spec.hpp"

namespace ntpserver {
namespace internal {

/**
 * @brief Tracks active NTP client endpoints.
 *
 * Records client socket addresses from received requests and updates
 * last-seen timestamps. Supports pruning of stale entries and iteration
 * over active clients for broadcasting control snapshots.
 */
class ClientTracker {
 public:
  /**
   * @brief Client endpoint record.
   */
  struct Client {
    sockaddr_in addr{};    ///< IPv4 socket address (IP + port).
    TimeSpec last_seen{};  ///< Last request time.
  };

  /**
   * @brief Record or update a client endpoint.
   *
   * If the client (identified by IP address and port) already exists,
   * updates its last_seen timestamp. Otherwise, adds a new entry to
   * the tracked client list.
   *
   * @param addr Client socket address (IPv4).
   * @param now Current time.
   */
  void Remember(const sockaddr_in& addr, const TimeSpec& now) {
    for (auto& c : clients_) {
      if (c.addr.sin_addr.s_addr == addr.sin_addr.s_addr &&
          c.addr.sin_port == addr.sin_port) {
        c.last_seen = now;
        return;
      }
    }
    clients_.push_back(Client{addr, now});
  }

  /**
   * @brief Get read-only access to all tracked clients.
   * @return Const reference to the client list.
   */
  const std::vector<Client>& GetAll() const { return clients_; }

  /**
   * @brief Get mutable access to client list for pruning.
   *
   * Allows caller to iterate and erase stale entries based on
   * last_seen timestamps. Used by NotifyControlSnapshot to prune
   * clients not seen within the retention window before broadcasting.
   *
   * @return Mutable reference to the client list.
   */
  std::vector<Client>& GetAllMutable() { return clients_; }

 private:
  std::vector<Client> clients_;
};

}  // namespace internal
}  // namespace ntpserver
