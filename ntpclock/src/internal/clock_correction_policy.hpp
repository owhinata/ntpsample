// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Clock correction policy for slew vs step decisions.
 *
 * Determines whether to apply gradual slew correction or immediate step
 * correction based on offset magnitude and configured thresholds.
 */

#ifndef NTPCLOCK_INTERNAL_CLOCK_CORRECTION_POLICY_HPP_
#define NTPCLOCK_INTERNAL_CLOCK_CORRECTION_POLICY_HPP_

#include <algorithm>
#include <cmath>

namespace ntpclock {
namespace internal {

/**
 * @brief Determines clock correction strategy based on offset delta.
 *
 * Stateless policy that decides between slew (gradual) and step (immediate)
 * corrections. Slew maintains monotonicity while step may introduce a
 * single backward jump.
 */
class ClockCorrectionPolicy {
 public:
  /**
   * @brief Type of correction to apply.
   */
  enum class Type {
    None,  ///< No correction needed.
    Slew,  ///< Gradual bounded-rate correction.
    Step   ///< Immediate jump correction.
  };

  /**
   * @brief Correction decision result.
   */
  struct Decision {
    Type type = Type::None;       ///< Type of correction.
    double amount_s = 0.0;        ///< Amount to apply (seconds).
  };

  ClockCorrectionPolicy() = default;

  /**
   * @brief Decide correction type and amount.
   *
   * @param offset_delta Delta between target and current offset (seconds).
   * @param step_threshold_s Threshold for triggering step correction (seconds).
   * @param slew_rate_s_per_s Maximum slew rate (seconds per second).
   * @param poll_interval_s Polling interval (seconds).
   * @return Decision indicating correction type and amount.
   *
   * Decision logic:
   * - If |delta| >= step_threshold: Step by full delta
   * - Otherwise: Slew by rate-limited amount (clamped to max_change)
   * - If slew amount is zero: No correction
   */
  Decision Decide(double offset_delta, double step_threshold_s,
                  double slew_rate_s_per_s, double poll_interval_s) const {
    Decision decision;

    if (std::abs(offset_delta) >= step_threshold_s) {
      // Step correction: apply full delta immediately
      decision.type = Type::Step;
      decision.amount_s = offset_delta;
    } else {
      // Slew correction: apply bounded-rate change
      double max_change = slew_rate_s_per_s * poll_interval_s;
      double change = std::clamp(offset_delta, -max_change, max_change);

      if (std::abs(change) > 0.0) {
        decision.type = Type::Slew;
        decision.amount_s = change;
      } else {
        decision.type = Type::None;
        decision.amount_s = 0.0;
      }
    }

    return decision;
  }
};

}  // namespace internal
}  // namespace ntpclock

#endif  // NTPCLOCK_INTERNAL_CLOCK_CORRECTION_POLICY_HPP_
