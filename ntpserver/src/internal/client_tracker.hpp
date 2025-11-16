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

#include <algorithm>
#include <chrono>
#include <mutex>
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
 *
 * Thread-safe: All public methods are protected by an internal mutex.
 */
class ClientTracker {
 public:
  /**
   * @brief Client endpoint record.
   */
  struct Client {
    sockaddr_in addr{};    ///< IPv4 socket address (IP + port).
    TimeSpec last_seen{};  ///< Last request absolute time (for diagnostics).
    std::chrono::steady_clock::time_point
        last_seen_mono{};  ///< Monotonic timestamp.
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
    std::lock_guard<std::mutex> lock(mtx_);
    const auto now_mono = std::chrono::steady_clock::now();
    for (auto& c : clients_) {
      if (c.addr.sin_addr.s_addr == addr.sin_addr.s_addr &&
          c.addr.sin_port == addr.sin_port) {
        c.last_seen = now;
        c.last_seen_mono = now_mono;
        return;
      }
    }
    clients_.push_back(Client{addr, now, now_mono});
  }

  /**
   * @brief Get a snapshot of all tracked clients.
   * @return Copy of the client list (thread-safe).
   */
  std::vector<Client> GetAll() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return clients_;
  }

  /**
   * @brief Remove clients that have not been seen within max_age.
   *
   * Uses steady_clock timestamps instead of absolute time so that
   * pruning remains correct even if the time source jumps.
   */
  void PruneStale(std::chrono::steady_clock::time_point now_mono) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto effective = retention_;
    if (effective <= std::chrono::steady_clock::duration::zero()) {
      effective = std::chrono::minutes(60);
    }
    const auto is_stale = [&](const Client& c) {
      return (now_mono - c.last_seen_mono) > effective;
    };
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(), is_stale),
                   clients_.end());
  }

  /**
   * @brief Configure retention duration for clients (default 60 minutes).
   */
  void SetRetention(std::chrono::steady_clock::duration retention) {
    std::lock_guard<std::mutex> lock(mtx_);
    retention_ = retention;
  }

 private:
  mutable std::mutex mtx_;
  std::vector<Client> clients_;
  std::chrono::steady_clock::duration retention_{std::chrono::minutes(60)};
};

}  // namespace internal
}  // namespace ntpserver
