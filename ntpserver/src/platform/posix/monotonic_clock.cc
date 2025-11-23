// Copyright (c) 2025
/**
 * @file monotonic_clock.cc
 * @brief POSIX-specific implementation using clock_gettime().
 */
#include "ntpserver/monotonic_clock.hpp"

#include <time.h>

#include <atomic>
#include <chrono>
#include <mutex>

namespace ntpserver {

struct MonotonicClock::Impl {
  mutable std::mutex mtx_;
  std::atomic<int64_t> mono_t0_nsec_{0};  // Monotonic anchor (nanoseconds)
  TimeSpec start_time_{};                 // Absolute time at anchor
  double rate_{1.0};                      // Time rate multiplier

  // Get current CLOCK_REALTIME as TimeSpec
  static TimeSpec CurrentUnix() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nsec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec);
    return TimeSpec(sec.count(), static_cast<uint32_t>(nsec.count()));
  }

  // Get elapsed time since anchor (in seconds)
  double Elapsed() const {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    int64_t now_nsec = static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL +
                       static_cast<int64_t>(ts.tv_nsec);
    int64_t anchor_nsec = mono_t0_nsec_.load(std::memory_order_relaxed);
    return static_cast<double>(now_nsec - anchor_nsec) / 1e9;
  }

  // Get current CLOCK_MONOTONIC in nanoseconds
  static int64_t MonotonicNow() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1'000'000'000LL +
           static_cast<int64_t>(ts.tv_nsec);
  }
};

MonotonicClock::MonotonicClock() : impl_(std::make_unique<Impl>()) {
  ResetToRealTime();
}

MonotonicClock::~MonotonicClock() = default;

TimeSpec MonotonicClock::NowUnix() {
  std::lock_guard<std::mutex> lk(impl_->mtx_);
  double elapsed_s = impl_->Elapsed();
  // Compute: start_time_ + rate_ * elapsed_s
  TimeSpec elapsed_time = TimeSpec::FromDouble(elapsed_s);
  return impl_->start_time_ + (elapsed_time * impl_->rate_);
}

void MonotonicClock::SetRate(double new_rate) {
  std::lock_guard<std::mutex> lk(impl_->mtx_);

  // Get current monotonic time once
  int64_t mono_now = Impl::MonotonicNow();

  // Calculate current time (without calling Elapsed() to avoid second call)
  int64_t anchor_nsec = impl_->mono_t0_nsec_.load(std::memory_order_relaxed);
  double elapsed = static_cast<double>(mono_now - anchor_nsec) / 1e9;
  TimeSpec elapsed_time = TimeSpec::FromDouble(elapsed);
  TimeSpec now = impl_->start_time_ + (elapsed_time * impl_->rate_);

  // Update anchor point
  impl_->mono_t0_nsec_.store(mono_now, std::memory_order_relaxed);
  impl_->start_time_ = now;
  impl_->rate_ = new_rate;
}

void MonotonicClock::SetAbsolute(const TimeSpec& time) {
  std::lock_guard<std::mutex> lk(impl_->mtx_);

  // Get current monotonic time once
  int64_t mono_now = Impl::MonotonicNow();
  impl_->mono_t0_nsec_.store(mono_now, std::memory_order_relaxed);

  // Update anchor point (rate unchanged)
  impl_->start_time_ = time;
}

void MonotonicClock::SetAbsoluteAndRate(const TimeSpec& time, double new_rate) {
  std::lock_guard<std::mutex> lk(impl_->mtx_);

  // Get current monotonic time once
  int64_t mono_now = Impl::MonotonicNow();
  impl_->mono_t0_nsec_.store(mono_now, std::memory_order_relaxed);

  // Update anchor point atomically
  impl_->start_time_ = time;
  impl_->rate_ = new_rate;
}

void MonotonicClock::ResetToRealTime() {
  std::lock_guard<std::mutex> lk(impl_->mtx_);

  // Get current monotonic time once
  int64_t mono_now = Impl::MonotonicNow();
  impl_->mono_t0_nsec_.store(mono_now, std::memory_order_relaxed);

  // Reset to current system time with rate 1.0
  impl_->start_time_ = Impl::CurrentUnix();
  impl_->rate_ = 1.0;
}

double MonotonicClock::GetRate() const {
  std::lock_guard<std::mutex> lk(impl_->mtx_);
  return impl_->rate_;
}

}  // namespace ntpserver
