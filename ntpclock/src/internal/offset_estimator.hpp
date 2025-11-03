// Copyright (c) 2025 <Your Name>
/**
 * @file offset_estimator.hpp
 * @brief Robust offset statistics computation for clock synchronization.
 *
 * Provides utilities to compute median, min, and max values from a sliding
 * window of offset samples. Used by ClockService to determine target offset
 * for correction decisions.
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace ntpclock {
namespace internal {

/**
 * @brief Computes robust statistics from offset samples.
 *
 * Extracts median, minimum, and maximum values from a sliding window of
 * clock offset measurements. The median is used as a robust estimator
 * less sensitive to outliers than the mean.
 */
class OffsetEstimator {
 public:
  /**
   * @brief Computed offset statistics.
   */
  struct Stats {
    double median;  ///< Median offset in seconds.
    double min;     ///< Minimum offset in window (seconds).
    double max;     ///< Maximum offset in window (seconds).
  };

  /**
   * @brief Compute robust statistics from offset samples.
   *
   * Selects the most recent samples up to the window size, computes
   * the median via nth_element, and finds min/max via minmax_element.
   *
   * @param offsets Vector of all offset samples (seconds).
   * @param window Maximum number of recent samples to use (>=1).
   * @param fallback Default value returned when offsets is empty.
   * @return Stats structure containing median, min, and max.
   */
  static Stats ComputeStats(const std::vector<double>& offsets, int window,
                            double fallback) {
    Stats result{fallback, fallback, fallback};

    size_t n = offsets.size();
    if (n == 0) return result;

    size_t win = static_cast<size_t>(std::max(1, window));
    size_t start = (n > win) ? (n - win) : 0;
    std::vector<double> tmp(offsets.begin() + start, offsets.end());

    if (tmp.empty()) return result;

    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    result.median = tmp[tmp.size() / 2];

    auto mm = std::minmax_element(tmp.begin(), tmp.end());
    result.min = *mm.first;
    result.max = *mm.second;

    return result;
  }
};

}  // namespace internal
}  // namespace ntpclock
