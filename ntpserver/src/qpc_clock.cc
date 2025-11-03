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

  LARGE_INTEGER t0{};
  QueryPerformanceCounter(&t0);
  qpc_t0_ = static_cast<int64_t>(t0.QuadPart);

  start_unix_ = CurrentUnix();
  rate_ = 1.0;
  offset_ = 0.0;
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
  return start_unix_ + rate_ * Elapsed() + offset_;
}

void QpcClock::SetRate(double rate) {
  std::lock_guard<std::mutex> lk(mtx_);
  rate_ = rate;
}

void QpcClock::AdjustOffset(double delta) {
  std::lock_guard<std::mutex> lk(mtx_);
  offset_ += delta;
}

void QpcClock::SetAbsolute(double unix_sec) {
  std::lock_guard<std::mutex> lk(mtx_);
  offset_ = unix_sec - (start_unix_ + rate_ * Elapsed());
}

QpcClock& QpcClock::Instance() {
  static QpcClock ut;
  return ut;
}

}  // namespace ntpserver
