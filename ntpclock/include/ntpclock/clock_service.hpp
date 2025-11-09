// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Server-synchronized clock service (seconds as double).
 *
 * This library provides an application-local clock synchronized to a single
 * NTP-like server over IPv4/UDP. It never changes the OS clock.
 *
 * Monotonicity guarantee:
 * - During slew correction, NowUnix() is monotonic non-decreasing.
 * - When a step correction is applied, time may jump backwards exactly once.
 */
#pragma once

#include <cstdint>
#include <memory>
#include <ostream>
#include <string>

#include "ntpserver/time_source.hpp"

namespace ntpclock {

/**
 * @brief Immutable options for ClockService.
 *
 * Use the Builder to construct instances. All fields are read-only via
 * getters. Time source defaults to QpcClock::Instance().
 */
class Options {
 public:
  /**
   * @brief Fluent builder for Options.
   */
  class Builder {
   public:
    Builder();
    explicit Builder(const Options& base);

    /** Set polling interval in milliseconds (default: 1000). */
    Builder& PollIntervalMs(int v);
    /** Set step threshold in milliseconds (default: 200). */
    Builder& StepThresholdMs(int v);
    /** Set slew rate in ms per second (default: 5.0). */
    Builder& SlewRateMsPerSec(double v);
    /** Set maximum acceptable RTT in ms (default: 100). */
    Builder& MaxRttMs(int v);
    /** Set minimum samples to report synchronized (default: 3). */
    Builder& MinSamplesToLock(int v);
    /** Number of recent offsets for median target (default: 5). */
    Builder& OffsetWindow(int v);
    /** Number of samples for skew OLS window (default: 20). */
    Builder& SkewWindow(int v);

    Options Build() const;

   private:
    int poll_interval_ms_;
    int step_threshold_ms_;
    double slew_rate_ms_per_s_;
    int max_rtt_ms_;
    int min_samples_to_lock_;
    int offset_window_;
    int skew_window_;
  };

  /** @name Getters (immutable) */
  ///@{
  int PollIntervalMs() const { return poll_interval_ms_; }
  int StepThresholdMs() const { return step_threshold_ms_; }
  double SlewRateMsPerSec() const { return slew_rate_ms_per_s_; }
  int MaxRttMs() const { return max_rtt_ms_; }
  int MinSamplesToLock() const { return min_samples_to_lock_; }
  int OffsetWindow() const { return offset_window_; }
  int SkewWindow() const { return skew_window_; }
  ///@}

  /** Stream formatter for logging. */
  friend std::ostream& operator<<(std::ostream& os, const Options& o);

 private:
  // Private ctor for Builder
  Options(int poll_ms, int step_ms, double slew_ms_per_s, int max_rtt_ms,
          int min_samples, int offset_window, int skew_window)
      : poll_interval_ms_(poll_ms),
        step_threshold_ms_(step_ms),
        slew_rate_ms_per_s_(slew_ms_per_s),
        max_rtt_ms_(max_rtt_ms),
        min_samples_to_lock_(min_samples),
        offset_window_(offset_window),
        skew_window_(skew_window) {}

  int poll_interval_ms_;
  int step_threshold_ms_;
  double slew_rate_ms_per_s_;
  int max_rtt_ms_;
  int min_samples_to_lock_;
  int offset_window_;
  int skew_window_;
};

/**
 * @brief Current synchronization status snapshot.
 */
struct Status {
  enum class Correction { None, Slew, Step };

  bool synchronized = false;
  int rtt_ms = 0;
  /** Latest computed round-trip delay (seconds). */
  double last_delay_s = 0.0;
  double offset_s = 0.0;
  double skew_ppm = 0.0;
  double last_update_unix_s = 0.0;
  int samples = 0;
  Correction last_correction = Correction::None;
  double last_correction_amount_s = 0.0;
  std::string last_error;

  // Debug summary of estimator windows
  int offset_window = 0;  // configured window size
  int skew_window = 0;    // configured window size
  int window_count = 0;   // current sample count retained
  double offset_median_s = 0.0;
  double offset_min_s = 0.0;
  double offset_max_s = 0.0;
  double offset_applied_s = 0.0;
  double offset_target_s = 0.0;

  /** Stream formatter for logging. */
  friend std::ostream& operator<<(std::ostream& os, const Status& s);
};

/**
 * @brief NTP-like server synchronized clock service.
 *
 * Control background synchronization with Start/Stop. NowUnix() returns the
 * server-synchronized current time as UNIX seconds (double). Monotonicity is
 * guaranteed during slew, but a single backward jump is allowed at the moment
 * a step correction is applied.
 */
class ClockService {
 public:
  ClockService();
  ~ClockService();

  /**
   * @brief Start background synchronization.
   * @param time_source Local time source (must remain valid until Stop).
   * @param ip  IPv4 address in numeric form (no DNS).
   * @param port UDP port of the server.
   * @param opt  Immutable options snapshot.
   * @return true if worker thread started, false if time_source is null.
   */
  bool Start(ntpserver::TimeSource* time_source, const std::string& ip,
             uint16_t port, const Options& opt);
  void Stop();

  /**
   * @brief Return server-synchronized current time in UNIX seconds.
   * @note Monotonic non-decreasing during slew. If a step was just applied,
   *       one backward jump is allowed.
   */
  double NowUnix() const;

  Status GetStatus() const;
  Options GetOptions() const;
  /**
   * @brief Atomically replace options with a new immutable snapshot.
   * Build the new Options via Options::Builder.
   */
  void SetOptions(const Options& opt);

 private:
  struct Impl;
  std::unique_ptr<Impl> p_;
};

}  // namespace ntpclock
