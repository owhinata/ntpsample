// Copyright (c) 2025 <Your Name>
// UserTime: adjustable time source (rate + offset)
#pragma once

#include <cstdint>

#include "ntpserver/ntp_server.hpp"

namespace ntpserver {

class UserTime : public TimeSource {
 public:
  UserTime();
  ~UserTime() override = default;

  // TimeSource
  double NowUnix() override;

  // Adjustments
  void SetRate(double rate);          // default 1.0
  void AdjustOffset(double delta);    // adds delta seconds to offset
  void SetAbsolute(double unix_sec);  // set absolute time

  // Singleton helper for convenience.
  static UserTime& Instance();

 private:
  double Elapsed() const;  // seconds from t0 using QPC
  static double CurrentUnix();

  // We avoid Windows types in header to keep it portable.
  int64_t qpc_t0_;     // QueryPerformanceCounter at construction
  double qpc_freq_;    // counts per second
  double start_unix_;  // wall clock at construction (seconds)
  double rate_;
  double offset_;
};

}  // namespace ntpserver
