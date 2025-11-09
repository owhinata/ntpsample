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
#include <vector>

#include "ntpserver/ntp_extension.hpp"
#include "ntpserver/time_source.hpp"

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
  /**
   * @brief Result of processing a vendor hint.
   */
  struct HintResult {
    bool reset_needed = false;  ///< Whether estimator reset is needed
    bool abs_applied = false;   ///< Whether SetAbsolute was applied
    double step_amount_s = 0.0; ///< Amount of step (if abs_applied)
  };

  VendorHintProcessor() = default;

  /**
   * @brief Process and apply vendor hints from an NTP response.
   *
   * @param rx_data Raw NTP packet bytes (header + optional extensions).
   * @param ntp_packet_size Size of the basic NTP packet structure.
   * @param time_source Target TimeSource to apply hints to.
   * @param step_threshold_s Threshold for applying absolute time changes (seconds).
   * @return HintResult indicating what was applied and whether reset is needed.
   *
   * Parses vendor extension fields, deduplicates by sequence number,
   * and applies SetRate/SetAbsolute if values differ meaningfully.
   */
  HintResult ProcessAndApply(const std::vector<uint8_t>& rx_data,
                             size_t ntp_packet_size,
                             ntpserver::TimeSource* time_source,
                             double step_threshold_s);

 private:
  uint32_t last_seq_ = 0;  ///< Last processed sequence number for deduplication
};

}  // namespace internal
}  // namespace ntpclock
