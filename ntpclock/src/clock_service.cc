// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Implementation of a server-synchronized clock service.
 *
 * The service polls a single IPv4 NTP-like server at a fixed interval,
 * estimates offset, and applies either a bounded-rate slew or a step.
 * Slew keeps NowUnix() monotonic non-decreasing. A step may introduce a
 * single backward jump immediately after application.
 */

#include "ntpclock/clock_service.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "internal/clock_corrector.hpp"
#include "internal/ntp_client.hpp"
#include "internal/offset_estimator.hpp"
#include "internal/skew_estimator.hpp"
#include "internal/sync_estimator_state.hpp"
#include "internal/vendor_hint_processor.hpp"
#include "ntpserver/ntp_extension.hpp"
#include "ntpserver/qpc_clock.hpp"

using std::chrono::milliseconds;

// ---------------- Options ----------------
ntpclock::Options::Builder::Builder()
    : poll_interval_ms_(1000),
      step_threshold_ms_(200),
      slew_rate_ms_per_s_(5.0),
      max_rtt_ms_(100),
      min_samples_to_lock_(3),
      offset_window_(5),
      skew_window_(20) {}

ntpclock::Options::Builder::Builder(const Options& base)
    : poll_interval_ms_(base.PollIntervalMs()),
      step_threshold_ms_(base.StepThresholdMs()),
      slew_rate_ms_per_s_(base.SlewRateMsPerSec()),
      max_rtt_ms_(base.MaxRttMs()),
      min_samples_to_lock_(base.MinSamplesToLock()),
      offset_window_(base.OffsetWindow()),
      skew_window_(base.SkewWindow()) {}

ntpclock::Options::Builder& ntpclock::Options::Builder::PollIntervalMs(int v) {
  poll_interval_ms_ = std::max(1, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::StepThresholdMs(int v) {
  step_threshold_ms_ = std::max(0, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::SlewRateMsPerSec(
    double v) {
  slew_rate_ms_per_s_ = std::max(0.0, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::MaxRttMs(int v) {
  max_rtt_ms_ = std::max(1, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::MinSamplesToLock(
    int v) {
  min_samples_to_lock_ = std::max(1, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::OffsetWindow(int v) {
  offset_window_ = std::max(1, v);
  return *this;
}

ntpclock::Options::Builder& ntpclock::Options::Builder::SkewWindow(int v) {
  skew_window_ = std::max(1, v);
  return *this;
}

ntpclock::Options ntpclock::Options::Builder::Build() const {
  return ntpclock::Options(poll_interval_ms_, step_threshold_ms_,
                           slew_rate_ms_per_s_, max_rtt_ms_,
                           min_samples_to_lock_, offset_window_, skew_window_);
}

namespace ntpclock {
std::ostream& operator<<(std::ostream& os, const Options& o) {
  os << "poll=" << o.PollIntervalMs() << "ms, step>=" << o.StepThresholdMs()
     << "ms, slew=" << o.SlewRateMsPerSec() << "ms/s, max_rtt=" << o.MaxRttMs()
     << "ms, min_lock=" << o.MinSamplesToLock();
  return os;
}

std::ostream& operator<<(std::ostream& os, const Status& s) {
  os << "sync=" << (s.synchronized ? "true" : "false") << ", rtt=" << s.rtt_ms
     << "ms"
     << ", offset=" << s.offset_s << "s"
     << ", skew=" << s.skew_ppm << "ppm"
     << ", last_update=" << s.last_update_unix_s << ", samples=" << s.samples
     << ", last_corr="
     << (s.last_correction == ntpclock::Status::Correction::None
             ? "none"
             : (s.last_correction == ntpclock::Status::Correction::Slew
                    ? "slew"
                    : "step"))
     << ", corr_amount=" << s.last_correction_amount_s
     << ", delay_s=" << s.last_delay_s << ", wcount=" << s.window_count
     << "/ow=" << s.offset_window << "/sw=" << s.skew_window
     << ", med=" << s.offset_median_s << ", min=" << s.offset_min_s
     << ", max=" << s.offset_max_s << ", applied=" << s.offset_applied_s
     << ", target=" << s.offset_target_s;
  if (!s.last_error.empty()) os << ", err='" << s.last_error << "'";
  return os;
}
}  // namespace ntpclock

// ---------------- Impl ----------------
struct ntpclock::ClockService::Impl {
  std::string ip;
  uint16_t port = 0;

  // TimeSource is immutable during execution (set at Start, cleared at Stop)
  ntpserver::TimeSource* time_source = nullptr;

  Options opts{Options::Builder().Build()};
  std::mutex opts_mtx;

  std::atomic<bool> running{false};
  std::thread thread;

  // Offsets (seconds)
  std::atomic<double> offset_applied_s{0.0};

  // Status
  Status status{};
  std::mutex status_mtx;

  // Extracted components (SRP)
  internal::ClockCorrector clock_corrector{&offset_applied_s};
  internal::VendorHintProcessor vendor_hint_processor;
  internal::SyncEstimatorState estimator_state;

  // Networking
  bool UdpExchange(double* out_offset_s, int* out_rtt_ms,
                   std::vector<uint8_t>* out_response, std::string* err);

  /**
   * @brief Result of vendor hint processing.
   */
  struct VendorHintResult {
    bool applied = false;        ///< Whether any hint was applied
    bool abs_applied = false;    ///< Whether SetAbsolute was applied
    double step_amount_s = 0.0;  ///< Step amount (if abs_applied)
    double timestamp = 0.0;      ///< Timestamp when applied
  };

  /**
   * @brief Parse NTP vendor extension from a received datagram and apply.
   *
   * @param rx Raw datagram bytes (NTP header + optional extension fields).
   * @return VendorHintResult indicating what was applied and when.
   * @note Applies SetRate/SetAbsolute when payload differences are meaningful,
   *       and resets estimator windows accordingly. Does NOT update status.
   */
  VendorHintResult ApplyVendorHintFromRx(const std::vector<uint8_t>& rx);

  // Worker and helper methods
  void Loop();

  /**
   * @brief Build status for inhibited state.
   *
   * Constructs Status with current sample info but no estimator updates.
   * Used when step corrections are inhibited (vendor hints settling).
   */
  Status BuildInhibitedStatus(const Options& snapshot, double sample_offset_s,
                              int sample_rtt_ms, double tnow, int good_samples,
                              const VendorHintResult& hint_result);

  /**
   * @brief Update estimators and compute target offset from valid sample.
   *
   * @return Tuple of (median, min, max) offset statistics.
   */
  std::tuple<double, double, double> UpdateEstimatorsAndTarget(
      const Options& snapshot, double sample_offset_s, double tnow);

  /**
   * @brief Build status for successful sample processing.
   */
  Status BuildSuccessStatus(const Options& snapshot, double sample_offset_s,
                            int sample_rtt_ms, double tnow, int good_samples,
                            double median, double omin, double omax,
                            Status::Correction correction,
                            double correction_amount);
};

namespace {
#pragma pack(push, 1)
struct NtpPacket {
  uint8_t li_vn_mode;
  uint8_t stratum;
  uint8_t poll;
  int8_t precision;
  uint32_t root_delay;
  uint32_t root_dispersion;
  uint32_t ref_id;
  uint64_t ref_timestamp;
  uint64_t orig_timestamp;
  uint64_t recv_timestamp;
  uint64_t tx_timestamp;
};
#pragma pack(pop)
}  // namespace

bool ntpclock::ClockService::Impl::UdpExchange(
    double* out_offset_s, int* out_rtt_ms, std::vector<uint8_t>* out_response,
    std::string* err) {
  auto get_timestamp = [this]() {
    return time_source ? time_source->NowUnix() : 0.0;
  };

  auto result = internal::NtpClient::Exchange(ip, port, get_timestamp);

  if (!result.success) {
    if (err) *err = result.error;
    return false;
  }

  *out_offset_s = result.offset_s;
  *out_rtt_ms = result.rtt_ms;
  *out_response = std::move(result.response_bytes);
  return true;
}

ntpclock::ClockService::Impl::VendorHintResult
ntpclock::ClockService::Impl::ApplyVendorHintFromRx(
    const std::vector<uint8_t>& rx) {
  VendorHintResult hint_result;

  auto ts = time_source;
  if (!ts) return hint_result;

  // Get options snapshot
  double step_threshold_s = 0.0;
  double poll_interval_s = 1.0;
  {
    std::lock_guard<std::mutex> lk(opts_mtx);
    step_threshold_s = opts.StepThresholdMs() / 1000.0;
    poll_interval_s = opts.PollIntervalMs() / 1000.0;
  }

  // Process vendor hints (automatically inhibits step corrections)
  auto result = vendor_hint_processor.ProcessAndApply(
      rx, sizeof(NtpPacket), ts, step_threshold_s, poll_interval_s);

  // If reset is needed, clear estimator state
  if (result.reset_needed) {
    estimator_state.Clear();
    offset_applied_s.store(0.0, std::memory_order_relaxed);

    // If absolute time was set, allow backward jump
    if (result.abs_applied) {
      clock_corrector.AllowBackwardOnce();
      hint_result.applied = true;
      hint_result.abs_applied = true;
      hint_result.step_amount_s = result.step_amount_s;
      hint_result.timestamp = ts->NowUnix();
    } else {
      hint_result.applied = true;
    }
  }

  return hint_result;
}

ntpclock::Status ntpclock::ClockService::Impl::BuildInhibitedStatus(
    const Options& snapshot, double sample_offset_s, int sample_rtt_ms,
    double tnow, int good_samples, const VendorHintResult& hint_result) {
  Status st;
  st.synchronized = (good_samples >= snapshot.MinSamplesToLock());
  st.rtt_ms = sample_rtt_ms;
  st.last_delay_s = sample_rtt_ms / 1000.0;
  st.offset_s = sample_offset_s;
  st.last_update_unix_s = tnow;
  st.last_error.clear();

  // Include vendor hint step correction if applied
  if (hint_result.abs_applied) {
    st.last_correction = Status::Correction::Step;
    st.last_correction_amount_s = hint_result.step_amount_s;
    st.window_count = 0;
    st.offset_applied_s = 0.0;
    st.offset_target_s = 0.0;
  }

  return st;
}

std::tuple<double, double, double>
ntpclock::ClockService::Impl::UpdateEstimatorsAndTarget(const Options& snapshot,
                                                        double sample_offset_s,
                                                        double tnow) {
  // Add sample to sliding window
  int maxw = std::max(snapshot.OffsetWindow(), snapshot.SkewWindow());
  estimator_state.AddSample(sample_offset_s, tnow, maxw);

  // Compute robust target: median of last window, and min/max for debug
  auto stats = estimator_state.ComputeOffsetStats(snapshot.OffsetWindow(),
                                                  sample_offset_s);
  double median = stats.median;
  double omin = stats.min;
  double omax = stats.max;

  return std::make_tuple(median, omin, omax);
}

ntpclock::Status ntpclock::ClockService::Impl::BuildSuccessStatus(
    const Options& snapshot, double sample_offset_s, int sample_rtt_ms,
    double tnow, int good_samples, double median, double omin, double omax,
    Status::Correction correction, double correction_amount) {
  Status st;
  st.synchronized = (good_samples >= snapshot.MinSamplesToLock());
  st.rtt_ms = sample_rtt_ms;
  st.last_delay_s = sample_rtt_ms / 1000.0;
  st.offset_s = sample_offset_s;
  st.last_update_unix_s = tnow;
  st.skew_ppm = estimator_state.ComputeSkewPpm(snapshot.SkewWindow());
  st.samples = good_samples;
  st.offset_window = snapshot.OffsetWindow();
  st.skew_window = snapshot.SkewWindow();
  st.window_count = estimator_state.GetSampleCount();
  st.offset_median_s = median;
  st.offset_min_s = omin;
  st.offset_max_s = omax;
  st.offset_applied_s = offset_applied_s.load(std::memory_order_relaxed);
  st.offset_target_s = median;
  st.last_correction = correction;
  st.last_correction_amount_s = correction_amount;
  st.last_error.clear();

  return st;
}

void ntpclock::ClockService::Impl::Loop() {
  // Loop until stopped; snapshot options each iteration for runtime changes.

  int good_samples = 0;
  while (running.load(std::memory_order_acquire)) {
    // 1. Snapshot configuration for this iteration
    auto snapshot = [&]() {
      std::lock_guard<std::mutex> lk(opts_mtx);
      return opts;  // copy
    }();
    const double step_thresh_s = snapshot.StepThresholdMs() / 1000.0;
    const double slew_rate_s_per_s = snapshot.SlewRateMsPerSec() / 1000.0;
    const int poll_interval_ms = snapshot.PollIntervalMs();

    // 2. Acquire NTP sample
    double sample_offset_s = 0.0;
    int sample_rtt_ms = 0;
    std::vector<uint8_t> response;
    std::string err;
    bool ok = UdpExchange(&sample_offset_s, &sample_rtt_ms, &response, &err);

    // 2a. Process vendor hint from response if available
    VendorHintResult hint_result;
    if (ok && response.size() > sizeof(NtpPacket)) {
      hint_result = ApplyVendorHintFromRx(response);
    }

    // 3. Build status based on sample validity and inhibition state
    Status st_local;
    double tnow = time_source ? time_source->NowUnix() : 0.0;
    bool is_inhibited =
        hint_result.applied || vendor_hint_processor.IsStepInhibited(tnow);

    if (ok && sample_rtt_ms <= snapshot.MaxRttMs() && is_inhibited) {
      // Inhibited path: vendor hint applied or settling from previous hint
      st_local = BuildInhibitedStatus(snapshot, sample_offset_s, sample_rtt_ms,
                                      tnow, good_samples, hint_result);
    } else if (ok && sample_rtt_ms <= snapshot.MaxRttMs()) {
      // Normal path: process sample and apply correction
      auto [median, omin, omax] =
          UpdateEstimatorsAndTarget(snapshot, sample_offset_s, tnow);

      double applied = offset_applied_s.load(std::memory_order_relaxed);
      double correction_amount = 0.0;
      Status::Correction correction = clock_corrector.Apply(
          applied, median, step_thresh_s, slew_rate_s_per_s,
          poll_interval_ms / 1000.0, &correction_amount);

      good_samples++;
      st_local = BuildSuccessStatus(snapshot, sample_offset_s, sample_rtt_ms,
                                    tnow, good_samples, median, omin, omax,
                                    correction, correction_amount);
    } else {
      // Error path: sample acquisition failed or RTT exceeded threshold
      st_local.synchronized = (good_samples >= snapshot.MinSamplesToLock());
      st_local.rtt_ms = sample_rtt_ms;
      st_local.last_delay_s = sample_rtt_ms / 1000.0;
      st_local.offset_s = sample_offset_s;
      st_local.last_error = err.empty() ? "sample rejected" : err;
    }

    // 4. Update shared status (single location for all paths)
    {
      std::lock_guard<std::mutex> lk(status_mtx);
      status = st_local;
    }

    // 5. Sleep until next poll
    std::this_thread::sleep_for(milliseconds(poll_interval_ms));
  }
}

// ---------------- ClockService ----------------
ntpclock::ClockService::ClockService() : p_(new Impl()) {}
ntpclock::ClockService::~ClockService() { Stop(); }

bool ntpclock::ClockService::Start(ntpserver::TimeSource* time_source,
                                   const std::string& ip, uint16_t port,
                                   const Options& opt) {
  if (!time_source) return false;

  Stop();
  p_->ip = ip;
  p_->port = port;
  p_->time_source = time_source;

  {
    std::lock_guard<std::mutex> lk(p_->opts_mtx);
    p_->opts = opt;
  }

  p_->running.store(true, std::memory_order_release);
  p_->thread = std::thread([this]() { p_->Loop(); });
  return true;
}

void ntpclock::ClockService::Stop() {
  if (!p_->running.exchange(false)) return;
  if (p_->thread.joinable()) p_->thread.join();
  p_->time_source = nullptr;
}

double ntpclock::ClockService::NowUnix() const {
  auto ts = p_->time_source;
  if (!ts) return 0.0;

  double base = ts->NowUnix();

  // ClockCorrector applies offset and enforces monotonicity
  return p_->clock_corrector.GetMonotonicTime(base);
}

ntpclock::Status ntpclock::ClockService::GetStatus() const {
  std::lock_guard<std::mutex> lk(p_->status_mtx);
  return p_->status;
}

ntpclock::Options ntpclock::ClockService::GetOptions() const {
  std::lock_guard<std::mutex> lk(p_->opts_mtx);
  return p_->opts;
}

void ntpclock::ClockService::SetOptions(const Options& opt) {
  std::lock_guard<std::mutex> lk(p_->opts_mtx);
  p_->opts = opt;
}
