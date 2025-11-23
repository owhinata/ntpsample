// Copyright (c) 2025 The NTP Sample Authors
#include "internal/sync_estimator_state.hpp"

#include <algorithm>
#include <vector>

namespace ntpclock {
namespace internal {

void SyncEstimatorState::AddSample(double offset_s, double time_s,
                                   int max_window) {
  std::lock_guard<std::mutex> lock(mtx_);

  offsets_.push_back(offset_s);
  times_.push_back(time_s);

  int maxw = std::max(1, max_window);
  size_t max_size = static_cast<size_t>(maxw);

  // Trim oldest samples if window exceeded (O(1) with deque)
  while (offsets_.size() > max_size) {
    offsets_.pop_front();
  }
  while (times_.size() > max_size) {
    times_.pop_front();
  }
}

SyncEstimatorState::Stats SyncEstimatorState::ComputeOffsetStats(
    int window, double fallback) const {
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

double SyncEstimatorState::ComputeSkewPpm(int window) const {
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

int SyncEstimatorState::GetSampleCount() const {
  std::lock_guard<std::mutex> lock(mtx_);
  return static_cast<int>(offsets_.size());
}

void SyncEstimatorState::Clear() {
  std::lock_guard<std::mutex> lock(mtx_);
  offsets_.clear();
  times_.clear();
}

}  // namespace internal
}  // namespace ntpclock
