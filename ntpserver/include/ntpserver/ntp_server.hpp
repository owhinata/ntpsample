// Copyright (c) 2025 <Your Name>
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "ntpserver/export.hpp"
#include "ntpserver/time_source.hpp"

namespace ntpserver {

uint32_t MakeRefId(const char (&tag)[5]);

/**
 * Immutable configuration options for NtpServer.
 */
class Options {
 public:
  class Builder {
   public:
    Builder();
    Builder& Stratum(uint8_t v);
    Builder& Precision(int8_t v);
    Builder& RefId(uint32_t v);
    Builder& ClientRetention(std::chrono::steady_clock::duration v);
    Options Build() const;

   private:
    uint8_t stratum_;
    int8_t precision_;
    uint32_t ref_id_;
    std::chrono::steady_clock::duration client_retention_;
  };

  Options();

  uint8_t Stratum() const;
  int8_t Precision() const;
  uint32_t RefId() const;
  std::chrono::steady_clock::duration ClientRetention() const;

  static constexpr uint8_t kDefaultStratum = 1;
  static constexpr int8_t kDefaultPrecision = -20;
  static constexpr uint32_t kDefaultRefId = 0x4C4F434C;  // "LOCL"
  static constexpr std::chrono::steady_clock::duration kDefaultClientRetention =
      std::chrono::minutes(60);

 private:
  Options(uint8_t stratum, int8_t precision, uint32_t ref_id,
          std::chrono::steady_clock::duration retention);

  uint8_t stratum_;
  int8_t precision_;
  uint32_t ref_id_;
  std::chrono::steady_clock::duration client_retention_;
};
/**
 * Minimal NTPv4 server (UDP/IPv4).
 *
 * Server operates in mode 4 (server). Default stratum is 1 (local reference).
 */
class NTP_SERVER_API NtpServer {
 public:
  NtpServer();
  ~NtpServer();

  NtpServer(const NtpServer&) = delete;
  NtpServer& operator=(const NtpServer&) = delete;

  /**
   * @brief Starts serving on the given UDP port.
   * @param port UDP port to bind (default: 9123).
   * @param time_source Time source for timestamps (default: QpcClock).
   * @param options Immutable configuration snapshot (defaults applied).
   * @return true on success, false on failure.
   */
  bool Start(uint16_t port = 9123, TimeSource* time_source = nullptr,
             const Options& options = Options());

  /** Stops the server. Safe to call multiple times. */
  void Stop();

  /**
   * @brief Sends a control snapshot (ABS/RATE via NTP EF) to known clients.
   *
   * The server tracks recently seen client endpoints (from requests). When
   * configuration changes (e.g., SetAbsolute/SetRate on the TimeSource), call
   * this method to actively notify clients between polls.
   */
  void NotifyControlSnapshot();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ntpserver
