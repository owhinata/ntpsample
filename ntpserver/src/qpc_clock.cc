// Copyright (c) 2025 <Your Name>
/**
 * @file qpc_clock.cc
 * @brief Windows-specific implementation of QpcClock using QPC.
 */
#include "ntpserver/qpc_clock.hpp"

#include <windows.h>

#include <chrono>

namespace ntpserver {

QpcClock::QpcClock() {
  LARGE_INTEGER f{};
  QueryPerformanceFrequency(&f);
  qpc_freq_ = static_cast<double>(f.QuadPart);

  // Initialize to real-time with rate 1.0
  ResetToRealTime();
}

TimeSpec QpcClock::CurrentUnix() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
  auto nsec =
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec);

  return TimeSpec(sec.count(), static_cast<uint32_t>(nsec.count()));
}

double QpcClock::Elapsed() const {
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  return (static_cast<int64_t>(t.QuadPart) -
          qpc_t0_.load(std::memory_order_relaxed)) /
         qpc_freq_;
}

TimeSpec QpcClock::NowUnix() {
  std::lock_guard<std::mutex> lk(mtx_);
  double elapsed_s = Elapsed();
  // Compute: start_time_ + rate_ * elapsed_s
  TimeSpec elapsed_time = TimeSpec::FromDouble(elapsed_s);
  return start_time_ + (elapsed_time * rate_);
}

void QpcClock::SetRate(double new_rate) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  int64_t qpc_now = static_cast<int64_t>(t.QuadPart);

  // Calculate current time (without calling Elapsed() to avoid second QPC call)
  double elapsed =
      (qpc_now - qpc_t0_.load(std::memory_order_relaxed)) / qpc_freq_;
  TimeSpec elapsed_time = TimeSpec::FromDouble(elapsed);
  TimeSpec now = start_time_ + (elapsed_time * rate_);

  // Update anchor point
  qpc_t0_.store(qpc_now, std::memory_order_relaxed);
  start_time_ = now;
  rate_ = new_rate;
}

void QpcClock::SetAbsolute(const TimeSpec& time) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_.store(static_cast<int64_t>(t.QuadPart), std::memory_order_relaxed);

  // Update anchor point (rate unchanged)
  start_time_ = time;
}

void QpcClock::SetAbsoluteAndRate(const TimeSpec& time, double new_rate) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_.store(static_cast<int64_t>(t.QuadPart), std::memory_order_relaxed);

  // Update anchor point atomically
  start_time_ = time;
  rate_ = new_rate;
}

void QpcClock::ResetToRealTime() {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_.store(static_cast<int64_t>(t.QuadPart), std::memory_order_relaxed);

  // Reset to current system time with rate 1.0
  start_time_ = CurrentUnix();
  rate_ = 1.0;
}

double QpcClock::GetRate() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return rate_;
}

}  // namespace ntpserver
