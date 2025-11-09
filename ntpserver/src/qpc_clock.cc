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

double QpcClock::CurrentUnix() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

double QpcClock::Elapsed() const {
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  return (static_cast<int64_t>(t.QuadPart) - qpc_t0_) / qpc_freq_;
}

double QpcClock::NowUnix() {
  std::lock_guard<std::mutex> lk(mtx_);
  return start_unix_ + rate_ * Elapsed();
}

void QpcClock::SetRate(double new_rate) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  int64_t qpc_now = static_cast<int64_t>(t.QuadPart);

  // Calculate current time (without calling Elapsed() to avoid second QPC call)
  double elapsed = (qpc_now - qpc_t0_) / qpc_freq_;
  double now = start_unix_ + rate_ * elapsed;

  // Update anchor point
  qpc_t0_ = qpc_now;
  start_unix_ = now;
  rate_ = new_rate;
}

void QpcClock::SetAbsolute(double unix_sec) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_ = static_cast<int64_t>(t.QuadPart);

  // Update anchor point (rate unchanged)
  start_unix_ = unix_sec;
}

void QpcClock::SetAbsoluteAndRate(double unix_sec, double new_rate) {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_ = static_cast<int64_t>(t.QuadPart);

  // Update anchor point atomically
  start_unix_ = unix_sec;
  rate_ = new_rate;
}

void QpcClock::ResetToRealTime() {
  std::lock_guard<std::mutex> lk(mtx_);

  // Get current QPC once
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  qpc_t0_ = static_cast<int64_t>(t.QuadPart);

  // Reset to current system time with rate 1.0
  start_unix_ = CurrentUnix();
  rate_ = 1.0;
}

double QpcClock::GetRate() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return rate_;
}

QpcClock& QpcClock::Instance() {
  static QpcClock ut;
  return ut;
}

}  // namespace ntpserver
