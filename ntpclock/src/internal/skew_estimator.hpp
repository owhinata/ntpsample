// Copyright (c) 2025 <Your Name>
/**
 * @file skew_estimator.hpp
 * @brief Clock skew estimation via ordinary least squares regression.
 *
 * Estimates the rate at which the local clock drifts relative to the server
 * clock by performing linear regression on offset vs. time samples. The
 * slope of the fitted line represents the skew in parts per million (PPM).
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace ntpclock {
namespace internal {

/**
 * @brief Estimates clock skew using OLS regression.
 *
 * Performs ordinary least squares (OLS) linear regression on a sliding
 * window of (time, offset) pairs to determine the local clock's drift
 * rate. The computed slope (seconds per second) is converted to PPM
 * for typical clock accuracy representation.
 */
class SkewEstimator {
 public:
  /**
   * @brief Compute clock skew from time and offset samples.
   *
   * Applies OLS regression: slope = Cov(time, offset) / Var(time).
   * Uses only the most recent samples within the specified window size.
   * Returns 0.0 if fewer than 2 samples or if variance is zero.
   *
   * @param times Vector of time samples (UNIX seconds).
   * @param offsets Vector of offset samples (seconds), aligned with times.
   * @param window Maximum number of recent samples to use (>=1).
   * @return Estimated skew in parts per million (PPM), or 0.0 on failure.
   */
  static double ComputeSkewPpm(const std::vector<double>& times,
                               const std::vector<double>& offsets, int window) {
    if (times.size() < 2 || times.size() != offsets.size()) {
      return 0.0;
    }

    size_t n =
        std::min(times.size(), static_cast<size_t>(std::max(1, window)));

    // Compute means
    double mean_t = 0.0, mean_o = 0.0;
    for (size_t i = times.size() - n; i < times.size(); ++i) {
      mean_t += times[i];
      mean_o += offsets[i];
    }
    mean_t /= static_cast<double>(n);
    mean_o /= static_cast<double>(n);

    // OLS regression: slope = Cov(t, o) / Var(t)
    double num = 0.0, den = 0.0;
    for (size_t i = times.size() - n; i < times.size(); ++i) {
      double dt = times[i] - mean_t;
      double doff = offsets[i] - mean_o;
      num += dt * doff;
      den += dt * dt;
    }

    double slope = (den > 0.0) ? (num / den) : 0.0;  // sec offset per sec
    return slope * 1e6;                              // convert to ppm
  }
};

}  // namespace internal
}  // namespace ntpclock
