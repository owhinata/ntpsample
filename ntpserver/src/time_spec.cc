// Copyright (c) 2025 The NTP Sample Authors
/**
 * @file time_spec.cc
 * @brief Implementation of TimeSpec utilities.
 */
#include "ntpserver/time_spec.hpp"

#include <cmath>

#include "ntpserver/ntp_types.hpp"

namespace ntpserver {

void TimeSpec::Normalize() {
  // Ensure 0 <= nsec < 1e9 while preserving the represented real value.
  constexpr uint32_t kNsecPerSec = 1000000000u;

  if (nsec >= kNsecPerSec) {
    sec += static_cast<int64_t>(nsec / kNsecPerSec);
    nsec %= kNsecPerSec;
  }
}

double TimeSpec::ToDouble() const {
  return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
}

TimeSpec TimeSpec::FromDouble(double unix_sec) {
  // Decompose into integer seconds (floor) and fractional part.
  // This preserves the represented real value up to the rounding
  // error inherent in double, even for negative times.
  constexpr double kNsecPerSecD = 1e9;
  constexpr uint32_t kNsecPerSec = 1000000000u;

  double sec_d = std::floor(unix_sec);
  double frac = unix_sec - sec_d;  // in [0, 1)

  int64_t sec = static_cast<int64_t>(sec_d);
  uint32_t nsec = static_cast<uint32_t>(std::llround(frac * kNsecPerSecD));

  // Handle possible 1.000000000 rounding.
  if (nsec >= kNsecPerSec) {
    nsec -= kNsecPerSec;
    ++sec;
  }

  return TimeSpec(sec, nsec);
}

uint64_t TimeSpec::ToNtpTimestamp() const {
  // Convert UNIX epoch to NTP epoch
  int64_t ntp_sec = sec + kNtpUnixEpochDiff;

  // Convert nanoseconds to NTP fraction (units of 2^-32 seconds)
  // fraction = nsec * 2^32 / 10^9
  uint64_t frac = (static_cast<uint64_t>(nsec) << 32) / 1000000000u;

  uint64_t ntp_ts =
      (static_cast<uint64_t>(ntp_sec) << 32) | (frac & 0xFFFFFFFFu);
  return ntp_ts;
}

TimeSpec TimeSpec::FromNtpTimestamp(uint64_t ntp_ts) {
  // Extract seconds and fraction
  uint32_t ntp_sec = static_cast<uint32_t>(ntp_ts >> 32);
  uint32_t ntp_frac = static_cast<uint32_t>(ntp_ts & 0xFFFFFFFFu);

  // Convert NTP epoch to UNIX epoch
  int64_t unix_sec = static_cast<int64_t>(ntp_sec) - kNtpUnixEpochDiff;

  // Convert NTP fraction to nanoseconds
  // nsec = frac * 10^9 / 2^32
  uint32_t nsec = static_cast<uint32_t>(
      (static_cast<uint64_t>(ntp_frac) * 1000000000u) >> 32);

  return TimeSpec(unix_sec, nsec);
}

TimeSpec operator+(const TimeSpec& a, const TimeSpec& b) {
  TimeSpec result;
  result.sec = a.sec + b.sec;
  result.nsec = a.nsec + b.nsec;
  result.Normalize();
  return result;
}

TimeSpec operator-(const TimeSpec& a, const TimeSpec& b) {
  TimeSpec result;
  result.sec = a.sec - b.sec;

  if (a.nsec >= b.nsec) {
    result.nsec = a.nsec - b.nsec;
  } else {
    // Borrow from seconds
    result.sec -= 1;
    result.nsec = 1000000000u + a.nsec - b.nsec;
  }

  return result;
}

TimeSpec operator*(const TimeSpec& t, double rate) {
  // Multiply sec and nsec separately, then combine
  double total_sec = t.ToDouble() * rate;
  return TimeSpec::FromDouble(total_sec);
}

TimeSpec AbsDiff(const TimeSpec& a, const TimeSpec& b) {
  if (a >= b) {
    return a - b;
  } else {
    TimeSpec diff = b - a;
    // Ensure result is positive
    if (diff.sec < 0) {
      diff.sec = -diff.sec;
    }
    return diff;
  }
}

}  // namespace ntpserver
