// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Clock correction with integrated monotonicity control.
 *
 * Manages offset correction decisions (slew vs step) and enforces monotonic
 * time progression with coordinated backward jump allowance after steps.
 */

#pragma once

#include <atomic>
#include <mutex>

#include "ntpclock/clock_service.hpp"
#include "ntpserver/time_spec.hpp"

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
  ClockCorrector() = default;

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
                           double poll_interval_s, double* out_amount_s);

  /**
   * @brief Allow the next time reading to go backward once.
   *
   * Typically called after external time adjustments (e.g., vendor hints
   * applying SetAbsolute) to permit GetMonotonicTime() to reflect the backward
   * jump.
   */
  void AllowBackwardOnce();

  /**
   * @brief Get monotonic corrected time.
   *
   * Called from NowUnix() at arbitrary times to read the current corrected
   * time. Applies the offset correction and enforces monotonic progression.
   *
   * @param base_time The raw base time from TimeSource.
   * @return The monotonic corrected time (base + offset, with monotonicity
   *         enforced, unless backward jump is explicitly allowed).
   *
   * Applies the current offset correction to base_time and enforces
   * monotonicity. If Apply() previously applied a step correction, allows
   * the result to go backward once, then reverts to enforcing monotonicity.
   */
  ntpserver::TimeSpec GetMonotonicTime(const ntpserver::TimeSpec& base_time);

  /**
   * @brief Get current offset correction value.
   * @return Current applied offset (in seconds as double for compatibility).
   */
  double GetOffsetApplied() const;

  /**
   * @brief Reset offset to zero.
   */
  void ResetOffset();

 private:
  // Correction and monotonicity state (single mutex for consistency)
  mutable std::mutex mtx_;
  ntpserver::TimeSpec offset_applied_;
  ntpserver::TimeSpec last_returned_;
  bool allow_backward_once_{false};
};

}  // namespace internal
}  // namespace ntpclock
