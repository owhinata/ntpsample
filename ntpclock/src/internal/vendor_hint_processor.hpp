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
                             double step_threshold_s) {
    HintResult result;

    // Check if there's extension data beyond the NTP packet
    if (rx_data.size() <= ntp_packet_size) {
      return result;
    }

    // Parse extension field header
    const uint8_t* p = rx_data.data() + ntp_packet_size;
    size_t remain = rx_data.size() - ntp_packet_size;
    if (remain < 4U) {
      return result;
    }

    uint16_t typ = static_cast<uint16_t>((p[0] << 8) | p[1]);
    uint16_t len = static_cast<uint16_t>((p[2] << 8) | p[3]);

    // Validate extension field type and length
    if (typ != ntpserver::NtpVendorExt::kEfTypeVendorHint || len < 4U ||
        len > remain) {
      return result;
    }

    // Extract and parse vendor payload
    std::vector<uint8_t> val(p + 4, p + len);
    ntpserver::NtpVendorExt::Payload v{};
    if (!ntpserver::NtpVendorExt::Parse(val, &v)) {
      return result;
    }

    // Deduplicate by sequence number
    if (v.seq <= last_seq_) {
      return result;
    }
    last_seq_ = v.seq;

    if (time_source == nullptr) {
      return result;
    }

    // Apply rate change if flag is set and value differs
    const double rate_eps = 1e-12;  // ~1e-6 ppm
    if ((v.flags & ntpserver::NtpVendorExt::kFlagRate) != 0U) {
      double cur_rate = time_source->GetRate();
      if (std::abs(v.rate_scale - cur_rate) > rate_eps) {
        time_source->SetRate(v.rate_scale);
        result.reset_needed = true;
      }
    }

    // Apply absolute time change if flag is set and value differs
    if ((v.flags & ntpserver::NtpVendorExt::kFlagAbs) != 0U) {
      double cur = time_source->NowUnix();
      if (std::abs(v.abs_unix_s - cur) >= step_threshold_s) {
        time_source->SetAbsolute(v.abs_unix_s);
        result.reset_needed = true;
        result.abs_applied = true;
        result.step_amount_s = v.abs_unix_s - cur;
      }
    }

    return result;
  }

 private:
  uint32_t last_seq_ = 0;  ///< Last processed sequence number for deduplication
};

}  // namespace internal
}  // namespace ntpclock
