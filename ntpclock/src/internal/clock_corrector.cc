// Copyright (c) 2025 <Your Name>
#include "internal/clock_corrector.hpp"

#include <algorithm>
#include <cmath>

namespace ntpclock {
namespace internal {

ClockCorrector::ClockCorrector(std::atomic<double>* offset_applied)
    : offset_applied_(offset_applied) {}

Status::Correction ClockCorrector::Apply(
    double current_offset, double target_offset, double step_threshold_s,
    double slew_rate_s_per_s, double poll_interval_s, double* out_amount_s) {
  double delta = target_offset - current_offset;

  if (std::abs(delta) >= step_threshold_s) {
    // Step correction: apply full delta immediately
    offset_applied_->store(current_offset + delta, std::memory_order_relaxed);
    allow_backward_once_.store(true, std::memory_order_release);
    *out_amount_s = delta;
    return Status::Correction::Step;

  } else {
    // Slew correction: apply bounded-rate change
    double max_change = slew_rate_s_per_s * poll_interval_s;
    double change = std::clamp(delta, -max_change, max_change);

    if (std::abs(change) > 0.0) {
      offset_applied_->store(current_offset + change,
                             std::memory_order_relaxed);
      *out_amount_s = change;
      return Status::Correction::Slew;
    }
  }

  *out_amount_s = 0.0;
  return Status::Correction::None;
}

void ClockCorrector::AllowBackwardOnce() {
  allow_backward_once_.store(true, std::memory_order_release);
}

double ClockCorrector::GetMonotonicTime(double base_time) {
  // Apply current offset correction
  double candidate =
      base_time + offset_applied_->load(std::memory_order_relaxed);

  // Allow one backward jump right after a step correction
  if (allow_backward_once_.exchange(false, std::memory_order_acq_rel)) {
    last_returned_s_.store(candidate, std::memory_order_relaxed);
    return candidate;
  }

  // Otherwise, enforce monotonicity with a small epsilon
  double last = last_returned_s_.load(std::memory_order_relaxed);
  const double eps = 1e-9;
  double clamped = std::max(candidate, last + eps);
  last_returned_s_.store(clamped, std::memory_order_relaxed);
  return clamped;
}

}  // namespace internal
}  // namespace ntpclock
