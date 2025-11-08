// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Monotonicity guard for clock time enforcement.
 *
 * Ensures that time readings are monotonic non-decreasing except when
 * explicitly allowed (e.g., after a step correction).
 */

#ifndef NTPCLOCK_INTERNAL_MONOTONICITY_GUARD_HPP_
#define NTPCLOCK_INTERNAL_MONOTONICITY_GUARD_HPP_

#include <algorithm>
#include <atomic>

namespace ntpclock {
namespace internal {

/**
 * @brief Enforces monotonic non-decreasing time progression.
 *
 * Maintains the last returned time value and ensures that subsequent
 * time readings never go backward, except for a single backward jump
 * that can be explicitly allowed (typically after a step correction).
 */
class MonotonicityGuard {
 public:
  MonotonicityGuard() = default;

  /**
   * @brief Enforce monotonic time progression.
   *
   * @param candidate_time The raw candidate time value.
   * @return The monotonic time value (>= last returned time, unless
   *         backward jump is explicitly allowed).
   *
   * If AllowBackwardOnce() was called previously, this method allows
   * the candidate time to go backward once, then reverts to enforcing
   * monotonicity.
   */
  double EnforceMonotonic(double candidate_time) {
    // Allow one backward jump right after a step correction.
    if (allow_backward_once_.exchange(false, std::memory_order_acq_rel)) {
      last_returned_s_.store(candidate_time, std::memory_order_relaxed);
      return candidate_time;
    }

    // Otherwise, enforce monotonicity with a small epsilon.
    double last = last_returned_s_.load(std::memory_order_relaxed);
    const double eps = 1e-9;
    double clamped = std::max(candidate_time, last + eps);
    last_returned_s_.store(clamped, std::memory_order_relaxed);
    return clamped;
  }

  /**
   * @brief Allow the next time reading to go backward once.
   *
   * Typically called immediately after applying a step correction
   * to allow NowUnix() to reflect the backward jump.
   */
  void AllowBackwardOnce() {
    allow_backward_once_.store(true, std::memory_order_release);
  }

 private:
  std::atomic<double> last_returned_s_{0.0};
  std::atomic<bool> allow_backward_once_{false};
};

}  // namespace internal
}  // namespace ntpclock

#endif  // NTPCLOCK_INTERNAL_MONOTONICITY_GUARD_HPP_
