// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Step inhibition management for clock corrections.
 *
 * Temporarily prevents step corrections after vendor hint application to
 * avoid double-stepping (vendor extension step + NTP step in same cycle).
 */

#ifndef NTPCLOCK_INTERNAL_STEP_INHIBITOR_HPP_
#define NTPCLOCK_INTERNAL_STEP_INHIBITOR_HPP_

#include <atomic>

namespace ntpclock {
namespace internal {

/**
 * @brief Manages temporary inhibition of step corrections.
 *
 * After applying a vendor hint (SetAbsolute/SetRate), step corrections
 * should be inhibited for one poll interval to prevent double-application
 * of the same underlying time change (once via vendor extension, once via
 * NTP offset estimation).
 */
class StepInhibitor {
 public:
  StepInhibitor() = default;

  /**
   * @brief Set inhibition period until specified time.
   *
   * @param until_time_s UNIX time (seconds) until which steps are inhibited.
   *
   * Typically called after applying a vendor hint with:
   * until_time_s = current_time + poll_interval
   */
  void InhibitUntil(double until_time_s) {
    inhibit_until_s_.store(until_time_s, std::memory_order_relaxed);
  }

  /**
   * @brief Check if step corrections are currently inhibited.
   *
   * @param current_time_s Current UNIX time (seconds).
   * @return true if current_time < inhibit_until, false otherwise.
   */
  bool IsInhibited(double current_time_s) const {
    double until = inhibit_until_s_.load(std::memory_order_relaxed);
    return current_time_s < until;
  }

  /**
   * @brief Clear inhibition (allow steps immediately).
   */
  void Clear() {
    inhibit_until_s_.store(0.0, std::memory_order_relaxed);
  }

 private:
  std::atomic<double> inhibit_until_s_{0.0};
};

}  // namespace internal
}  // namespace ntpclock

#endif  // NTPCLOCK_INTERNAL_STEP_INHIBITOR_HPP_
