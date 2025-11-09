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

#include <algorithm>
#include <cstddef>
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
  void AddSample(double offset_s, double time_s, int max_window) {
    std::lock_guard<std::mutex> lock(mtx_);

    offsets_.push_back(offset_s);
    times_.push_back(time_s);

    int maxw = std::max(1, max_window);
    size_t max_size = static_cast<size_t>(maxw);

    // Trim oldest samples if window exceeded
    while (offsets_.size() > max_size) {
      offsets_.erase(offsets_.begin());
    }
    while (times_.size() > max_size) {
      times_.erase(times_.begin());
    }
  }

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
  Stats ComputeOffsetStats(int window, double fallback) const {
    std::lock_guard<std::mutex> lock(mtx_);

    Stats result{fallback, fallback, fallback};
    size_t n = offsets_.size();
    if (n == 0) return result;

    size_t win = static_cast<size_t>(std::max(1, window));
    size_t start = (n > win) ? (n - win) : 0;
    std::vector<double> tmp(offsets_.begin() + start, offsets_.end());

    if (tmp.empty()) return result;

    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    result.median = tmp[tmp.size() / 2];

    auto mm = std::minmax_element(tmp.begin(), tmp.end());
    result.min = *mm.first;
    result.max = *mm.second;

    return result;
  }

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
  double ComputeSkewPpm(int window) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (times_.size() < 2 || times_.size() != offsets_.size()) {
      return 0.0;
    }

    size_t n = std::min(times_.size(), static_cast<size_t>(std::max(1, window)));

    // Compute means
    double mean_t = 0.0, mean_o = 0.0;
    for (size_t i = times_.size() - n; i < times_.size(); ++i) {
      mean_t += times_[i];
      mean_o += offsets_[i];
    }
    mean_t /= static_cast<double>(n);
    mean_o /= static_cast<double>(n);

    // OLS regression: slope = Cov(t, o) / Var(t)
    double num = 0.0, den = 0.0;
    for (size_t i = times_.size() - n; i < times_.size(); ++i) {
      double dt = times_[i] - mean_t;
      double doff = offsets_[i] - mean_o;
      num += dt * doff;
      den += dt * dt;
    }

    double slope = (den > 0.0) ? (num / den) : 0.0;  // sec offset per sec
    return slope * 1e6;                              // convert to ppm
  }

  /**
   * @brief Get current number of samples in the window.
   *
   * @return Number of samples stored.
   *
   * Thread-safe.
   */
  int GetSampleCount() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<int>(offsets_.size());
  }

  /**
   * @brief Clear all stored samples.
   *
   * Typically called when estimator reset is needed (e.g., after
   * vendor hint application or configuration change).
   *
   * Thread-safe.
   */
  void Clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    offsets_.clear();
    times_.clear();
  }

 private:
  mutable std::mutex mtx_;
  std::vector<double> offsets_;
  std::vector<double> times_;
};

}  // namespace internal
}  // namespace ntpclock
