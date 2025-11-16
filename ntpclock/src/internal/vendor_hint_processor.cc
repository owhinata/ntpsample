// Copyright (c) 2025 <Your Name>
#include "internal/vendor_hint_processor.hpp"

#include <cmath>
#include <vector>

namespace ntpclock {
namespace internal {

bool VendorHintProcessor::ParseVendorPayload(
    const std::vector<uint8_t>& rx_data, size_t ntp_packet_size,
    ntpserver::NtpVendorExt::Payload* out_payload) {
  // Check if there's extension data beyond the NTP packet
  if (rx_data.size() <= ntp_packet_size) {
    return false;
  }

  // Parse extension field header
  const uint8_t* p = rx_data.data() + ntp_packet_size;
  size_t remain = rx_data.size() - ntp_packet_size;
  if (remain < 4U) {
    return false;
  }

  uint16_t typ = static_cast<uint16_t>((p[0] << 8) | p[1]);
  uint16_t len = static_cast<uint16_t>((p[2] << 8) | p[3]);

  // Validate extension field type and length
  if (typ != ntpserver::NtpVendorExt::kEfTypeVendorHint || len < 4U ||
      len > remain) {
    return false;
  }

  // Extract and parse vendor payload
  std::vector<uint8_t> val(p + 4, p + len);
  if (!ntpserver::NtpVendorExt::Parse(val, out_payload)) {
    return false;
  }

  // Deduplicate by sequence number using wrap-safe comparison
  if (!IsSeqNewer(out_payload->seq)) {
    return false;
  }
  last_seq_ = out_payload->seq;
  have_seq_ = true;

  return true;
}

bool VendorHintProcessor::ApplyRateHint(
    const ntpserver::NtpVendorExt::Payload& payload,
    ntpserver::TimeSource* time_source) {
  if ((payload.flags & ntpserver::NtpVendorExt::kFlagRate) == 0U) {
    return false;
  }

  const double rate_eps = 1e-12;  // ~1e-6 ppm
  double cur_rate = time_source->GetRate();
  if (std::abs(payload.rate_scale - cur_rate) <= rate_eps) {
    return false;
  }

  time_source->SetRate(payload.rate_scale);
  return true;
}

bool VendorHintProcessor::ApplyAbsoluteHint(
    const ntpserver::NtpVendorExt::Payload& payload,
    ntpserver::TimeSource* time_source, double step_threshold_s,
    ntpserver::TimeSpec* out_step_amount) {
  if ((payload.flags & ntpserver::NtpVendorExt::kFlagAbs) == 0U) {
    return false;
  }

  ntpserver::TimeSpec cur = time_source->NowUnix();
  ntpserver::TimeSpec diff = ntpserver::AbsDiff(payload.abs_time, cur);
  ntpserver::TimeSpec threshold =
      ntpserver::TimeSpec::FromDouble(step_threshold_s);
  if (diff < threshold) {
    return false;
  }

  time_source->SetAbsolute(payload.abs_time);
  *out_step_amount = payload.abs_time - cur;
  return true;
}

VendorHintProcessor::HintResult VendorHintProcessor::ProcessAndApply(
    const std::vector<uint8_t>& rx_data, size_t ntp_packet_size,
    ntpserver::TimeSource* time_source, double step_threshold_s) {
  HintResult result;

  ntpserver::NtpVendorExt::Payload payload{};
  if (!ParseVendorPayload(rx_data, ntp_packet_size, &payload)) {
    return result;
  }

  if (time_source == nullptr) {
    return result;
  }

  bool has_abs = (payload.flags & ntpserver::NtpVendorExt::kFlagAbs) != 0U;
  bool has_rate = (payload.flags & ntpserver::NtpVendorExt::kFlagRate) != 0U;

  // Atomic update when both abs and rate are present
  if (has_abs && has_rate) {
    const double rate_eps = 1e-12;  // ~1e-6 ppm
    double cur_rate = time_source->GetRate();
    ntpserver::TimeSpec cur_abs = time_source->NowUnix();

    bool rate_changed = std::abs(payload.rate_scale - cur_rate) > rate_eps;
    ntpserver::TimeSpec diff = ntpserver::AbsDiff(payload.abs_time, cur_abs);
    ntpserver::TimeSpec threshold =
        ntpserver::TimeSpec::FromDouble(step_threshold_s);
    bool abs_changed = diff >= threshold;

    if (rate_changed || abs_changed) {
      time_source->SetAbsoluteAndRate(payload.abs_time, payload.rate_scale);
      result.reset_needed = true;
      if (abs_changed) {
        result.abs_applied = true;
        result.step_amount = payload.abs_time - cur_abs;
      }
    }
  } else {
    // Apply individually if only one is present
    if (ApplyRateHint(payload, time_source)) {
      result.reset_needed = true;
    }

    if (ApplyAbsoluteHint(payload, time_source, step_threshold_s,
                          &result.step_amount)) {
      result.reset_needed = true;
      result.abs_applied = true;
    }
  }

  return result;
}

bool VendorHintProcessor::IsSeqNewer(uint32_t seq) const {
  if (!have_seq_) return true;
  if (seq == last_seq_) return false;
  // Signed diff captures wrap-around: positive diff => newer
  int32_t diff = static_cast<int32_t>(seq - last_seq_);
  return diff > 0;
}

bool VendorHintProcessor::ProcessPacket(
    const ntpserver::NtpPacket& pkt,
    const ntpserver::NtpVendorExt::Payload& vendor,
    ntpserver::TimeSource* time_source) {
  uint8_t mode = pkt.li_vn_mode & 0x07;
  uint32_t packet_epoch = vendor.seq;

  // Old epoch - ignore
  if (ntpserver::IsEpochOlder(packet_epoch, current_epoch_)) {
    return false;
  }

  // New epoch - update
  if (ntpserver::IsEpochNewer(packet_epoch, current_epoch_)) {
    current_epoch_ = packet_epoch;

    // Apply TimeSource update if ABS and RATE are present
    if (time_source) {
      bool has_abs = (vendor.flags & ntpserver::NtpVendorExt::kFlagAbs) != 0U;
      bool has_rate = (vendor.flags & ntpserver::NtpVendorExt::kFlagRate) != 0U;
      if (has_abs && has_rate) {
        time_source->SetAbsoluteAndRate(vendor.abs_time, vendor.rate_scale);
      }
    }

    // New epoch detected - caller should send new Exchange
    return true;
  }

  // Same epoch
  if (mode == 5) {
    // Push notification - don't use for NTP sync
    return false;
  }

  // mode=4, same epoch - process normally
  return true;
}

}  // namespace internal
}  // namespace ntpclock
