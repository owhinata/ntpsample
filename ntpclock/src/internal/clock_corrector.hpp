// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Clock correction with integrated monotonicity control.
 *
 * Manages offset correction decisions (slew vs step) and enforces monotonic
 * time progression with coordinated backward jump allowance after steps.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>

#include "ntpclock/clock_service.hpp"

namespace ntpclock {
namespace internal {

/**
 * @brief Unified clock correction and monotonicity management.
 *
 * Combines correction decision logic (slew vs step) with monotonic time
 * enforcement. When a step correction is applied, coordinates the one-time
 * backward jump allowance for NowUnix().
 */
class ClockCorrector {
 public:
  /**
   * @brief Construct corrector with reference to shared offset state.
   *
   * @param offset_applied Pointer to atomic offset variable (owned by caller).
   */
  explicit ClockCorrector(std::atomic<double>* offset_applied)
      : offset_applied_(offset_applied) {}

  /**
   * @brief Apply offset correction based on target vs current delta.
   *
   * Called periodically from the sync loop to update the offset correction
   * parameter. Does NOT read time; only updates the correction state.
   *
   * @param current_offset Currently applied offset (seconds).
   * @param target_offset Desired target offset (seconds).
   * @param step_threshold_s Threshold for step correction (seconds).
   * @param slew_rate_s_per_s Maximum slew rate (seconds per second).
   * @param poll_interval_s Polling interval (seconds).
   * @param out_amount_s Output parameter for actual correction amount applied.
   * @return Type of correction applied (None, Slew, or Step).
   *
   * Decision logic:
   * - If |delta| >= step_threshold: Apply step (full delta)
   * - Otherwise: Apply slew (rate-limited change)
   * - If step is applied, allows one backward jump in GetMonotonicTime()
   */
  Status::Correction Apply(double current_offset, double target_offset,
                           double step_threshold_s, double slew_rate_s_per_s,
                           double poll_interval_s, double* out_amount_s) {
    double delta = target_offset - current_offset;

    if (std::abs(delta) >= step_threshold_s) {
      // Step correction: apply full delta immediately
      offset_applied_->store(current_offset + delta,
                             std::memory_order_relaxed);
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

  /**
   * @brief Allow the next time reading to go backward once.
   *
   * Typically called after external time adjustments (e.g., vendor hints
   * applying SetAbsolute) to permit GetMonotonicTime() to reflect the backward jump.
   */
  void AllowBackwardOnce() {
    allow_backward_once_.store(true, std::memory_order_release);
  }

  /**
   * @brief Get monotonic corrected time.
   *
   * Called from NowUnix() at arbitrary times to read the current corrected time.
   * Applies the offset correction and enforces monotonic progression.
   *
   * @param base_time The raw base time from TimeSource.
   * @return The monotonic corrected time (base + offset, with monotonicity
   *         enforced, unless backward jump is explicitly allowed).
   *
   * Applies the current offset correction to base_time and enforces
   * monotonicity. If Apply() previously applied a step correction, allows
   * the result to go backward once, then reverts to enforcing monotonicity.
   */
  double GetMonotonicTime(double base_time) {
    // Apply current offset correction
    double candidate = base_time + offset_applied_->load(std::memory_order_relaxed);

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

 private:
  std::atomic<double>* offset_applied_;

  // Monotonicity state (integrated from MonotonicityGuard)
  std::atomic<double> last_returned_s_{0.0};
  std::atomic<bool> allow_backward_once_{false};
};

}  // namespace internal
}  // namespace ntpclock
