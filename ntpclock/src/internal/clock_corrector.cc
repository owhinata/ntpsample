// Copyright (c) 2025 The NTP Sample Authors
#include "internal/clock_corrector.hpp"

#include <algorithm>
#include <cmath>

namespace ntpclock {
namespace internal {

Status::Correction ClockCorrector::Apply(
    double current_offset, double target_offset, double step_threshold_s,
    double slew_rate_s_per_s, double poll_interval_s, double* out_amount_s) {
  double delta = target_offset - current_offset;

  if (std::abs(delta) >= step_threshold_s) {
    // Step correction: apply full delta immediately
    ntpserver::TimeSpec new_offset =
        ntpserver::TimeSpec::FromDouble(current_offset + delta);
    ntpserver::TimeSpec delta_ts = ntpserver::TimeSpec::FromDouble(delta);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      offset_applied_ = new_offset;
      // Adjust last_returned_ by the step amount to allow backward jump
      last_returned_ = last_returned_ + delta_ts;
      allow_backward_once_ = true;
    }
    *out_amount_s = delta;
    return Status::Correction::Step;

  } else {
    // Slew correction: apply bounded-rate change
    double max_change = slew_rate_s_per_s * poll_interval_s;
    double change = std::clamp(delta, -max_change, max_change);

    if (std::abs(change) > 0.0) {
      ntpserver::TimeSpec new_offset =
          ntpserver::TimeSpec::FromDouble(current_offset + change);
      {
        std::lock_guard<std::mutex> lk(mtx_);
        offset_applied_ = new_offset;
      }
      *out_amount_s = change;
      return Status::Correction::Slew;
    }
  }

  *out_amount_s = 0.0;
  return Status::Correction::None;
}

void ClockCorrector::AllowBackwardOnce() {
  std::lock_guard<std::mutex> lk(mtx_);
  allow_backward_once_ = true;
}

ntpserver::TimeSpec ClockCorrector::GetMonotonicTime(
    const ntpserver::TimeSpec& base_time) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Apply current offset correction
  ntpserver::TimeSpec candidate = base_time + offset_applied_;

  // Allow one backward jump right after a step correction
  if (allow_backward_once_) {
    allow_backward_once_ = false;
    last_returned_ = candidate;
    return candidate;
  }

  // Otherwise, enforce monotonicity with a small epsilon
  ntpserver::TimeSpec eps = ntpserver::TimeSpec::FromDouble(1e-9);
  ntpserver::TimeSpec min_next = last_returned_ + eps;

  if (candidate < min_next) {
    last_returned_ = min_next;
    return min_next;
  } else {
    last_returned_ = candidate;
    return candidate;
  }
}

double ClockCorrector::GetOffsetApplied() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return offset_applied_.ToDouble();
}

void ClockCorrector::ResetOffset() {
  std::lock_guard<std::mutex> lk(mtx_);
  offset_applied_ = ntpserver::TimeSpec{};
  last_returned_ = ntpserver::TimeSpec{};
  allow_backward_once_ = false;
}

}  // namespace internal
}  // namespace ntpclock
