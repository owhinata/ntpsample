// Copyright (c) 2025 The NTP Sample Authors
/**
 * @file time_spec.hpp
 * @brief Platform-independent time representation with nanosecond precision.
 *
 * Provides a TimeSpec structure (seconds + nanoseconds) and utility functions
 * for time arithmetic, conversion to/from NTP timestamps, and compatibility
 * with legacy double-based representations.
 */
#pragma once

#include <cmath>
#include <cstdint>

namespace ntpserver {

/**
 * @brief Time specification with nanosecond precision.
 *
 * Represents absolute time as UNIX epoch seconds plus nanosecond fraction.
 * This format is platform-independent and suitable for high-precision
 * time synchronization.
 */
struct TimeSpec {
  int64_t sec;    ///< Seconds since UNIX epoch (1970-01-01 00:00:00 UTC)
  uint32_t nsec;  ///< Nanoseconds (0-999999999)

  /** Default constructor: zero time */
  TimeSpec() : sec(0), nsec(0) {}

  /** Constructor from seconds and nanoseconds */
  TimeSpec(int64_t s, uint32_t ns) : sec(s), nsec(ns) {}

  /**
   * @brief Normalize nanosecond field to [0, 999999999] range.
   *
   * Adjusts sec/nsec so that nsec is always in valid range.
   * Called automatically by arithmetic operations.
   */
  void Normalize();

  /**
   * @brief Convert to double (UNIX seconds).
   *
   * Provided for compatibility with legacy code.
   * May lose precision for times far from epoch.
   */
  double ToDouble() const;

  /**
   * @brief Create TimeSpec from double (UNIX seconds).
   *
   * Provided for compatibility with legacy code.
   */
  static TimeSpec FromDouble(double unix_sec);

  /**
   * @brief Convert to NTP timestamp (64-bit, seconds.fraction).
   *
   * NTP epoch is 1900-01-01, fraction is in units of 2^-32 seconds.
   * @return 64-bit NTP timestamp in host byte order.
   */
  uint64_t ToNtpTimestamp() const;

  /**
   * @brief Create TimeSpec from NTP timestamp.
   *
   * @param ntp_ts 64-bit NTP timestamp in host byte order.
   * @return TimeSpec representing the same instant.
   */
  static TimeSpec FromNtpTimestamp(uint64_t ntp_ts);
};

// Comparison operators
inline bool operator==(const TimeSpec& a, const TimeSpec& b) {
  return a.sec == b.sec && a.nsec == b.nsec;
}

inline bool operator!=(const TimeSpec& a, const TimeSpec& b) {
  return !(a == b);
}

inline bool operator<(const TimeSpec& a, const TimeSpec& b) {
  if (a.sec != b.sec) return a.sec < b.sec;
  return a.nsec < b.nsec;
}

inline bool operator<=(const TimeSpec& a, const TimeSpec& b) {
  return !(b < a);
}

inline bool operator>(const TimeSpec& a, const TimeSpec& b) { return b < a; }

inline bool operator>=(const TimeSpec& a, const TimeSpec& b) {
  return !(a < b);
}

// Arithmetic operators

/**
 * @brief Add two TimeSpec values.
 *
 * @param a First time value.
 * @param b Second time value.
 * @return Normalized result of a + b.
 */
TimeSpec operator+(const TimeSpec& a, const TimeSpec& b);

/**
 * @brief Subtract two TimeSpec values.
 *
 * @param a Minuend.
 * @param b Subtrahend.
 * @return Normalized result of a - b (may have negative sec).
 */
TimeSpec operator-(const TimeSpec& a, const TimeSpec& b);

/**
 * @brief Multiply TimeSpec by a scalar rate.
 *
 * Used for time scaling (e.g., applying progression rate).
 *
 * @param t Time value.
 * @param rate Scalar multiplier.
 * @return Normalized result of t * rate.
 */
TimeSpec operator*(const TimeSpec& t, double rate);

/**
 * @brief Multiply TimeSpec by a scalar rate (commutative).
 *
 * @param rate Scalar multiplier.
 * @param t Time value.
 * @return Normalized result of rate * t.
 */
inline TimeSpec operator*(double rate, const TimeSpec& t) { return t * rate; }

/**
 * @brief Compute absolute difference between two TimeSpec values.
 *
 * @param a First time value.
 * @param b Second time value.
 * @return |a - b| as a non-negative TimeSpec (sec >= 0).
 */
TimeSpec AbsDiff(const TimeSpec& a, const TimeSpec& b);

/**
 * @brief Convert time difference to double (seconds).
 *
 * Useful for displaying/logging time intervals.
 *
 * @param t Time difference.
 * @return Time in seconds as double.
 */
inline double ToSeconds(const TimeSpec& t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nsec) * 1e-9;
}

}  // namespace ntpserver
