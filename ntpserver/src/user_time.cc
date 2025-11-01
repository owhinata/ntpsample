// Copyright (c) 2025 <Your Name>
// Windows-specific implementation of UserTime using QPC
#include "ntpserver/user_time.hpp"

#include <windows.h>

#include <chrono>

namespace ntpserver {

UserTime::UserTime() {
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

double UserTime::CurrentUnix() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

double UserTime::Elapsed() const {
  LARGE_INTEGER t{};
  QueryPerformanceCounter(&t);
  return (static_cast<int64_t>(t.QuadPart) - qpc_t0_) / qpc_freq_;
}

double UserTime::NowUnix() { return start_unix_ + rate_ * Elapsed() + offset_; }

void UserTime::SetRate(double rate) { rate_ = rate; }

void UserTime::AdjustOffset(double delta) { offset_ += delta; }

void UserTime::SetAbsolute(double unix_sec) {
  offset_ = unix_sec - (start_unix_ + rate_ * Elapsed());
}

UserTime& UserTime::Instance() {
  static UserTime ut;
  return ut;
}

}  // namespace ntpserver
