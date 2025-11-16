// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief NTP vendor extension hint processor.
 *
 * Parses and applies vendor-specific extension fields from NTP responses,
 * specifically handling SetRate and SetAbsolute hints.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "ntpserver/ntp_types.hpp"
#include "ntpserver/time_source.hpp"
#include "ntpserver/time_spec.hpp"

namespace ntpclock {
namespace internal {

/**
 * @brief Processes NTP vendor extension hints.
 *
 * Parses vendor extension fields from received NTP packets and applies
 * rate/absolute time changes to a TimeSource. Provides deduplication
 * based on sequence numbers.
 */
class VendorHintProcessor {
 public:
  /** Callback for logging messages (thread-safe). */
  using LogCallback = std::function<void(const std::string&)>;

  /**
   * @brief Result of processing a vendor hint.
   */
  struct HintResult {
    bool reset_needed = false;        ///< Whether estimator reset is needed
    bool abs_applied = false;         ///< Whether SetAbsolute was applied
    ntpserver::TimeSpec step_amount;  ///< Amount of step (if abs_applied)
  };

  VendorHintProcessor() = default;

  /**
   * @brief Set log sink callback for debug logging.
   * @param cb Callback function for logging messages.
   */
  void SetLogSink(LogCallback cb);

  /**
   * @brief Process and apply vendor hints from an NTP response.
   *
   * @param rx_data Raw NTP packet bytes (header + optional extensions).
   * @param ntp_packet_size Size of the basic NTP packet structure.
   * @param time_source Target TimeSource to apply hints to.
   * @param step_threshold_s Threshold for applying absolute time changes
   * (seconds).
   * @return HintResult indicating what was applied and whether reset is needed.
   *
   * Parses vendor extension fields, deduplicates by sequence number,
   * and applies SetRate/SetAbsolute if values differ meaningfully.
   */
  HintResult ProcessAndApply(const std::vector<uint8_t>& rx_data,
                             size_t ntp_packet_size,
                             ntpserver::TimeSource* time_source,
                             double step_threshold_s);

  /**
   * @brief Process packet based on epoch and mode.
   *
   * @param pkt NTP packet header.
   * @param vendor Parsed vendor extension payload.
   * @param time_source Target TimeSource to update on new epoch.
   * @return true if packet should be used for NTP sync, false otherwise.
   *
   * Implements epoch-based synchronization:
   * - Old epoch (< current_epoch_): ignore packet, return false
   * - New epoch (> current_epoch_): update current_epoch_, apply ABS/RATE,
   * return true
   * - Same epoch, mode=5: ignore for NTP (Push notification), return false
   * - Same epoch, mode=4: process normally, return true
   */
  bool ProcessPacket(const ntpserver::NtpPacket& pkt,
                     const ntpserver::NtpVendorExt::Payload& vendor,
                     ntpserver::TimeSource* time_source);

  /**
   * @brief Process packet with epoch detection from raw data.
   *
   * @param rx_data Raw NTP packet bytes (header + optional extensions).
   * @param ntp_packet_size Size of the basic NTP packet structure.
   * @param time_source Target TimeSource to update on new epoch.
   * @param out_epoch_changed Output parameter set to true if epoch changed.
   * @return HintResult indicating what was applied and whether reset is needed.
   *
   * Combines epoch detection with hint processing. Parses NTP packet and
   * vendor extension, detects epoch changes, and updates TimeSource.
   */
  HintResult ProcessWithEpochDetection(const std::vector<uint8_t>& rx_data,
                                       size_t ntp_packet_size,
                                       ntpserver::TimeSource* time_source,
                                       bool* out_epoch_changed);

  /**
   * @brief Get current epoch number.
   * @return Current epoch number being tracked.
   */
  uint32_t GetCurrentEpoch() const { return current_epoch_; }

 private:
  /**
   * @brief Parse and validate vendor extension payload from raw packet.
   *
   * @param rx_data Raw NTP packet bytes.
   * @param ntp_packet_size Size of the basic NTP packet structure.
   * @param out_payload Output parsed payload on success.
   * @return true if parsing succeeded, false otherwise.
   */
  bool ParseVendorPayload(const std::vector<uint8_t>& rx_data,
                          size_t ntp_packet_size,
                          ntpserver::NtpVendorExt::Payload* out_payload);

  /**
   * @brief Apply rate change hint if present and different.
   *
   * @param payload Parsed vendor extension payload.
   * @param time_source Target TimeSource.
   * @return true if rate was changed, false otherwise.
   */
  bool ApplyRateHint(const ntpserver::NtpVendorExt::Payload& payload,
                     ntpserver::TimeSource* time_source);

  /**
   * @brief Apply absolute time hint if present and exceeds threshold.
   *
   * @param payload Parsed vendor extension payload.
   * @param time_source Target TimeSource.
   * @param step_threshold_s Threshold for applying absolute time changes
   * (seconds).
   * @param out_step_amount Output parameter for step amount.
   * @return true if absolute time was changed, false otherwise.
   */
  bool ApplyAbsoluteHint(const ntpserver::NtpVendorExt::Payload& payload,
                         ntpserver::TimeSource* time_source,
                         double step_threshold_s,
                         ntpserver::TimeSpec* out_step_amount);

  bool have_seq_ = false;  ///< Whether last_seq_ has been initialized
  uint32_t last_seq_ = 0;  ///< Last processed sequence number for deduplication
  uint32_t current_epoch_{0};  ///< Current epoch number tracked
  LogCallback log_callback_;   ///< Optional log sink for debug messages

  /**
   * @brief Return true if seq is newer than last_seq_, accounting for wrap.
   *
   * Uses signed arithmetic on the difference so that any forward delta
   * smaller than 2^31 is treated as newer, and wrap-around still works.
   */
  bool IsSeqNewer(uint32_t seq) const;
};

}  // namespace internal
}  // namespace ntpclock
