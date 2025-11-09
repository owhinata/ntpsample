// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Sliding window data management for clock sync estimators.
 *
 * Maintains offset and time sample vectors for offset and skew estimation,
 * providing thread-safe access and automatic window size management.
 * Integrates offset statistics (median, min, max) and skew estimation
 * (OLS regression) computations.
 */

#pragma once

#include <mutex>
#include <vector>

namespace ntpclock {
namespace internal {

/**
 * @brief Thread-safe storage for offset/time samples with sliding window.
 *
 * Manages vectors of offset and time samples and provides robust statistics
 * computation (median, min, max) and skew estimation (OLS regression).
 * Automatically maintains window size and provides synchronized access for
 * multi-threaded environments.
 */
class SyncEstimatorState {
 public:
  /**
   * @brief Computed offset statistics.
   */
  struct Stats {
    double median;  ///< Median offset in seconds.
    double min;     ///< Minimum offset in window (seconds).
    double max;     ///< Maximum offset in window (seconds).
  };

  SyncEstimatorState() = default;

  /**
   * @brief Add a new sample pair to the sliding windows.
   *
   * @param offset_s Offset measurement in seconds.
   * @param time_s Time of measurement (UNIX seconds).
   * @param max_window Maximum window size (keeps max of offset/skew windows).
   *
   * Appends samples and removes oldest if window size is exceeded.
   * Thread-safe.
   */
  void AddSample(double offset_s, double time_s, int max_window);

  /**
   * @brief Compute offset statistics from current samples.
   *
   * Selects the most recent samples up to the window size, computes
   * the median via nth_element, and finds min/max via minmax_element.
   *
   * @param window Window size for offset computation.
   * @param fallback Fallback value when no samples available.
   * @return Stats with median, min, max.
   *
   * Thread-safe.
   */
  Stats ComputeOffsetStats(int window, double fallback) const;

  /**
   * @brief Compute clock skew from current samples.
   *
   * Applies OLS regression: slope = Cov(time, offset) / Var(time).
   * Uses only the most recent samples within the specified window size.
   * Returns 0.0 if fewer than 2 samples or if variance is zero.
   *
   * @param window Window size for skew computation.
   * @return Estimated skew in parts per million (PPM), or 0.0 on failure.
   *
   * Thread-safe.
   */
  double ComputeSkewPpm(int window) const;

  /**
   * @brief Get current number of samples in the window.
   *
   * @return Number of samples stored.
   *
   * Thread-safe.
   */
  int GetSampleCount() const;

  /**
   * @brief Clear all stored samples.
   *
   * Typically called when estimator reset is needed (e.g., after
   * vendor hint application or configuration change).
   *
   * Thread-safe.
   */
  void Clear();

 private:
  mutable std::mutex mtx_;
  std::vector<double> offsets_;
  std::vector<double> times_;
};

}  // namespace internal
}  // namespace ntpclock
