// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Sliding window data management for clock sync estimators.
 *
 * Maintains offset and time sample vectors for offset and skew estimation,
 * providing thread-safe access and automatic window size management.
 */

#ifndef NTPCLOCK_INTERNAL_SYNC_ESTIMATOR_STATE_HPP_
#define NTPCLOCK_INTERNAL_SYNC_ESTIMATOR_STATE_HPP_

#include <algorithm>
#include <mutex>
#include <vector>

#include "internal/offset_estimator.hpp"
#include "internal/skew_estimator.hpp"

namespace ntpclock {
namespace internal {

/**
 * @brief Thread-safe storage for offset/time samples with sliding window.
 *
 * Manages vectors of offset and time samples used by OffsetEstimator and
 * SkewEstimator. Automatically maintains window size and provides
 * synchronized access for multi-threaded environments.
 */
class SyncEstimatorState {
 public:
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
   * @param window Window size for offset computation.
   * @param fallback Fallback value when no samples available.
   * @return OffsetEstimator::Stats with median, min, max.
   *
   * Thread-safe.
   */
  OffsetEstimator::Stats ComputeOffsetStats(int window, double fallback) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return OffsetEstimator::ComputeStats(offsets_, window, fallback);
  }

  /**
   * @brief Compute clock skew from current samples.
   *
   * @param window Window size for skew computation.
   * @return Estimated skew in PPM, or 0.0 if insufficient samples.
   *
   * Thread-safe.
   */
  double ComputeSkewPpm(int window) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return SkewEstimator::ComputeSkewPpm(times_, offsets_, window);
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

#endif  // NTPCLOCK_INTERNAL_SYNC_ESTIMATOR_STATE_HPP_
