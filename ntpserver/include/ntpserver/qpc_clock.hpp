// Copyright (c) 2025 <Your Name>
/**
 * Adjustable time source (rate + offset).
 *
 * Provides a UNIX-seconds clock derived from QPC. You can adjust the
 * progression rate and apply an absolute or relative offset for testing.
 */
#pragma once

#include <cstdint>

#include "ntpserver/ntp_server.hpp"

namespace ntpserver {

/** QpcClock implementation backed by QueryPerformanceCounter. */
class QpcClock : public TimeSource {
 public:
  QpcClock();
  ~QpcClock() override = default;

  /** Returns current time as UNIX seconds. */
  double NowUnix() override;

  // Adjustments
  /** Sets progression rate (default 1.0). */
  void SetRate(double rate);
  /** Adds a relative offset in seconds to the current time. */
  void AdjustOffset(double delta);
  /** Sets absolute time in UNIX seconds. */
  void SetAbsolute(double unix_sec);

  /** Singleton accessor for convenience. */
  static QpcClock& Instance();

 private:
  /** Seconds elapsed since construction measured via QPC. */
  double Elapsed() const;
  /** Current wall-clock UNIX seconds (system_clock). */
  static double CurrentUnix();

  // We avoid Windows types in header to keep it portable.
  /** QPC value captured at construction. */
  int64_t qpc_t0_;
  /** QPC frequency (counts per second). */
  double qpc_freq_;
  /** Wall-clock UNIX seconds at construction. */
  double start_unix_;
  double rate_;
  double offset_;
};

}  // namespace ntpserver
