// Copyright (c) 2025 <Your Name>
/**
 * @file time_source.hpp
 * @brief Minimal time source interface (UNIX time provider).
 */
#pragma once

#include "ntpserver/time_spec.hpp"

namespace ntpserver {

/**
 * Interface for time sources.
 * Provides current time as UNIX epoch time with nanosecond precision.
 */
class TimeSource {
 public:
  virtual ~TimeSource() = default;

  /** Returns the current time since the UNIX epoch. */
  virtual TimeSpec NowUnix() = 0;

  /** Sets absolute time (optional; used by clients). */
  virtual void SetAbsolute(const TimeSpec& time) = 0;

  /** Sets progression rate multiplier (1.0 = real time). */
  virtual void SetRate(double /*rate*/) = 0;

  /**
   * @brief Atomically sets both absolute time and rate.
   *
   * Sets both absolute time and progression rate in a single atomic operation,
   * ensuring consistency when both values need to be updated together.
   *
   * @param time Absolute time.
   * @param rate Progression rate multiplier.
   */
  virtual void SetAbsoluteAndRate(const TimeSpec& time, double rate) = 0;

  /**
   * @brief Resets time source to real-time (system clock) with rate 1.0.
   *
   * Resets both absolute time and rate to match the current system time
   * with normal progression (rate = 1.0). Useful for returning to normal
   * operation after testing or manual adjustments.
   */
  virtual void ResetToRealTime() = 0;

  /**
   * @brief Returns current progression rate multiplier.
   *
   * Default implementations may return 1.0 if unknown.
   */
  virtual double GetRate() const { return 1.0; }
};

}  // namespace ntpserver
