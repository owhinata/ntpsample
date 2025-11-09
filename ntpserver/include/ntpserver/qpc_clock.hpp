// Copyright (c) 2025 <Your Name>
/**
 * Adjustable time source with rate and absolute time control.
 *
 * Provides a UNIX-seconds clock derived from QPC. You can adjust the
 * progression rate and set absolute time for testing.
 */
#pragma once

#include <cstdint>
#include <mutex>

#include "ntpserver/time_source.hpp"

namespace ntpserver {

/** QpcClock implementation backed by QueryPerformanceCounter. */
class QpcClock : public TimeSource {
 public:
  QpcClock();
  ~QpcClock() override = default;

  /** Returns current time. */
  TimeSpec NowUnix() override;

  // Adjustments
  /** Sets progression rate (default 1.0). */
  void SetRate(double rate) override;
  /** Gets progression rate. */
  double GetRate() const override;
  /** Sets absolute time. */
  void SetAbsolute(const TimeSpec& time) override;
  /** Atomically sets both absolute time and rate. */
  void SetAbsoluteAndRate(const TimeSpec& time, double rate) override;
  /** Resets to real-time (system clock) with rate 1.0. */
  void ResetToRealTime() override;

  /** Singleton accessor for convenience. */
  static QpcClock& Instance();

 private:
  /** Seconds elapsed since last anchor point (qpc_t0_) measured via QPC. */
  double Elapsed() const;
  /** Current wall-clock time (system_clock). */
  static TimeSpec CurrentUnix();

  // We avoid Windows types in header to keep it portable.
  /** QPC value at anchor point (updated by
   * SetAbsolute/SetRate/SetAbsoluteAndRate). */
  int64_t qpc_t0_;
  /** QPC frequency (counts per second). */
  double qpc_freq_;
  /** Time at anchor point (qpc_t0_). */
  TimeSpec start_time_;
  /** Progression rate multiplier. */
  double rate_;
  mutable std::mutex mtx_;
};

}  // namespace ntpserver
