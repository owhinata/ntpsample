// Copyright (c) 2025
/**
 * @file monotonic_clock.hpp
 * @brief POSIX monotonic clock-based TimeSource implementation.
 *
 * Provides a TimeSource implementation using POSIX
 * clock_gettime(CLOCK_MONOTONIC) for elapsed time measurement and
 * clock_gettime(CLOCK_REALTIME) for absolute time. This is the POSIX equivalent
 * of QpcClock on Windows.
 */
#pragma once

#include "ntpserver/export.hpp"
#include "ntpserver/time_source.hpp"

namespace ntpserver {

/**
 * @brief TimeSource backed by POSIX clock_gettime().
 *
 * Uses CLOCK_MONOTONIC for measuring elapsed time (not affected by system
 * time adjustments) and CLOCK_REALTIME for absolute time reference.
 * Supports rate scaling and absolute time adjustments like QpcClock.
 *
 * Thread-safe: All methods use internal locking.
 */
class NTP_SERVER_API MonotonicClock : public TimeSource {
 public:
  /**
   * @brief Access the singleton instance.
   * @return Reference to the global MonotonicClock instance.
   */
  static MonotonicClock& Instance();

  // TimeSource interface implementation
  TimeSpec NowUnix() override;
  void SetAbsolute(const TimeSpec& time) override;
  void SetRate(double rate) override;
  void SetAbsoluteAndRate(const TimeSpec& time, double rate) override;
  void ResetToRealTime() override;
  double GetRate() const override;

 private:
  MonotonicClock();
  ~MonotonicClock() override = default;

  // Non-copyable, non-movable
  MonotonicClock(const MonotonicClock&) = delete;
  MonotonicClock& operator=(const MonotonicClock&) = delete;

  struct Impl;
  Impl* impl_;
};

}  // namespace ntpserver
