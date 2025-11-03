// Copyright (c) 2025 <Your Name>
/**
 * @file time_source.hpp
 * @brief Minimal time source interface (UNIX seconds provider).
 */
#pragma once

namespace ntpserver {

/**
 * Interface for time sources.
 * Provides current time as UNIX seconds (UTC since 1970-01-01).
 */
class TimeSource {
 public:
  virtual ~TimeSource() = default;
  /** Returns the current time in seconds since the UNIX epoch. */
  virtual double NowUnix() = 0;

  /** Sets absolute time in UNIX seconds (optional; used by clients). */
  virtual void SetAbsolute(double /*unix_sec*/) = 0;

  /** Sets progression rate multiplier (1.0 = real time). */
  virtual void SetRate(double /*rate*/) = 0;

  /**
   * @brief Returns current progression rate multiplier.
   *
   * Default implementations may return 1.0 if unknown.
   */
  virtual double GetRate() const { return 1.0; }
};

}  // namespace ntpserver
