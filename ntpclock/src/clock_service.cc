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

// Platform-specific includes for htonl/ntohl
#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "internal/clock_corrector.hpp"
#include "internal/sync_estimator_state.hpp"
#include "internal/udp_socket.hpp"
#include "internal/vendor_hint_processor.hpp"
#include "ntpserver/ntp_types.hpp"
#include "ntpserver/platform/default_time_source.hpp"

using std::chrono::milliseconds;

// ---------------- Options ----------------
ntpclock::Options::Builder::Builder()
    : poll_interval_ms_(10000),
      step_threshold_ms_(200),
      slew_rate_ms_per_s_(5.0),
      max_rtt_ms_(100),
      min_samples_to_lock_(3),
      offset_window_(5),
      skew_window_(10),
      log_sink_cb_(Options::LogCallback()) {}

ntpclock::Options::Builder::Builder(const Options& base)
    : poll_interval_ms_(base.PollIntervalMs()),
      step_threshold_ms_(base.StepThresholdMs()),
      slew_rate_ms_per_s_(base.SlewRateMsPerSec()),
      max_rtt_ms_(base.MaxRttMs()),
      min_samples_to_lock_(base.MinSamplesToLock()),
      offset_window_(base.OffsetWindow()),
      skew_window_(base.SkewWindow()),
      log_sink_cb_(base.LogSink()) {}

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

ntpclock::Options::Builder& ntpclock::Options::Builder::LogSink(
    LogCallback cb) {
  log_sink_cb_ = std::move(cb);
  return *this;
}

ntpclock::Options ntpclock::Options::Builder::Build() const {
  return ntpclock::Options(
      poll_interval_ms_, step_threshold_ms_, slew_rate_ms_per_s_, max_rtt_ms_,
      min_samples_to_lock_, offset_window_, skew_window_, log_sink_cb_);
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
     << ", last_update=" << s.last_update.ToDouble()
     << ", samples=" << s.samples << ", last_corr="
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
  // TimeSource is immutable during execution (set at Start, cleared at Stop)
  ntpserver::TimeSource* time_source = nullptr;

  Options opts{Options::Builder().Build()};
  std::mutex opts_mtx;

  std::atomic<bool> running{false};
  std::thread thread;

  // Status
  Status status{};
  std::mutex status_mtx;

  // Extracted components (SRP)
  internal::ClockCorrector clock_corrector;
  internal::VendorHintProcessor vendor_hint_processor;
  internal::SyncEstimatorState estimator_state;

  // Networking
  internal::UdpSocket udp_socket;
  std::atomic<bool> exchange_abort{false};
  std::mutex socket_err_mtx;
  std::string socket_last_error;

  // Logging
  Options::LogCallback log_callback_;

  bool UdpExchange(double* out_offset_s, int* out_rtt_ms,
                   std::vector<uint8_t>* out_response, std::string* err);

  /**
   * @brief Build an NTP request packet.
   *
   * @param T1 Transmit timestamp.
   * @return Raw NTP request packet bytes.
   */
  std::vector<uint8_t> BuildNtpRequest(const ntpserver::TimeSpec& T1);

  /**
   * @brief Wait for exchange response with abort checking.
   *
   * @param timeout_ms Total timeout in milliseconds.
   * @param out_msg Output message on success.
   * @param err Output error message on failure.
   * @return true if exchange response received, false on timeout/abort/push.
   */
  bool WaitForExchangeResponse(int timeout_ms,
                               internal::UdpSocket::Message* out_msg,
                               std::string* err);

  /**
   * @brief Process NTP response to compute offset and RTT.
   *
   * @param msg Received message.
   * @param T1 Client transmit timestamp.
   * @param out_offset_s Output computed offset.
   * @param out_rtt_ms Output computed RTT.
   * @param err Output error message on failure.
   * @return true on success, false on parsing error.
   */
  bool ProcessNtpResponse(const internal::UdpSocket::Message& msg,
                          const ntpserver::TimeSpec& T1, double* out_offset_s,
                          int* out_rtt_ms, std::string* err);

  /**
   * @brief Result of vendor hint processing.
   */
  struct VendorHintResult {
    bool applied = false;             ///< Whether any hint was applied
    bool abs_applied = false;         ///< Whether SetAbsolute was applied
    bool epoch_changed = false;       ///< Whether server epoch changed
    ntpserver::TimeSpec step_amount;  ///< Step amount (if abs_applied)
  };

  /**
   * @brief Parse NTP vendor extension from a received datagram and apply.
   *
   * @param rx Raw datagram bytes (NTP header + optional extension fields).
   * @param snapshot Options snapshot for threshold values.
   * @return VendorHintResult indicating what was applied.
   */
  VendorHintResult ApplyVendorHintFromRx(const std::vector<uint8_t>& rx,
                                         const Options& snapshot);

  /**
   * @brief Handle a Push notification message.
   *
   * Processes vendor hints from a server-initiated Push message. If hints
   * are applied (e.g., rate change), sets exchange_abort to interrupt any
   * ongoing Exchange.
   *
   * @param msg Push message from UdpSocket.
   * @param snapshot Options snapshot for threshold values.
   */
  void HandlePushMessage(const internal::UdpSocket::Message& msg,
                         const Options& snapshot);

  /**
   * @brief Apply vendor hints and update internal state.
   *
   * Processes vendor hints, clears estimators if needed, and allows
   * backward jumps for SetAbsolute.
   *
   * @param rx Raw datagram bytes (NTP header + optional extension fields).
   * @param allow_abort If true, sets exchange_abort flag when hints applied.
   * @return VendorHintResult indicating what was applied.
   */
  VendorHintResult ApplyVendorHints(const std::vector<uint8_t>& rx,
                                    bool allow_abort);

  /**
   * @brief Update estimators and compute target offset from valid sample.
   *
   * @return Tuple of (median, min, max) offset statistics.
   */
  std::tuple<double, double, double> UpdateEstimatorsAndTarget(
      const Options& snapshot, double sample_offset_s, double tnow);

  /**
   * @brief Build base status with common fields.
   *
   * Sets common fields that appear in all status objects.
   */
  void SetBaseStatusFields(Status* st, const Options& snapshot,
                           double sample_offset_s, int sample_rtt_ms,
                           double tnow, int good_samples);

  /**
   * @brief Build status for vendor hint applied state.
   *
   * Constructs Status with current sample info but no estimator updates.
   * Used when vendor hint was applied in the current loop iteration.
   */
  Status BuildVendorHintAppliedStatus(const Options& snapshot,
                                      double sample_offset_s, int sample_rtt_ms,
                                      double tnow, int good_samples,
                                      const VendorHintResult& hint_result);

  /**
   * @brief Build status for successful sample processing.
   */
  Status BuildSuccessStatus(const Options& snapshot, double sample_offset_s,
                            int sample_rtt_ms, double tnow, int good_samples,
                            double median, double omin, double omax,
                            Status::Correction correction,
                            double correction_amount,
                            const VendorHintResult& hint_result);

  /**
   * @brief Wait for Push message or poll deadline.
   *
   * @param next_poll_time Poll deadline time point.
   * @param snapshot Options snapshot for HandlePushMessage.
   * @param good_samples Pointer to good_samples counter (reset on Push).
   * @return true if Push was received, false if poll deadline reached.
   */
  bool WaitForPushOrPollDeadline(
      std::chrono::steady_clock::time_point next_poll_time,
      const Options& snapshot, int* good_samples);

  /**
   * @brief Execute Exchange, process vendor hints, and build Status.
   *
   * @param snapshot Options snapshot.
   * @param step_thresh_s Step threshold in seconds.
   * @param slew_rate_s_per_s Slew rate in seconds per second.
   * @param poll_interval_ms Poll interval in milliseconds.
   * @param good_samples Reference to good sample counter (incremented on
   * success).
   * @return Constructed Status object.
   */
  Status ProcessExchangeAndBuildStatus(const Options& snapshot,
                                       double step_thresh_s,
                                       double slew_rate_s_per_s,
                                       int poll_interval_ms, int* good_samples);

  // Worker and helper methods
  void Loop();
  void ReportSocketError(const std::string& msg);
  std::string GetSocketError();
  void ClearSocketError();
};

namespace {

/** Write TimeSpec to NTP timestamp (64-bit big-endian). */
inline void WriteTimestamp(const ntpserver::TimeSpec& ts, uint8_t* dst8) {
  uint64_t ntp_ts = ts.ToNtpTimestamp();
  uint32_t sec_be = htonl(static_cast<uint32_t>(ntp_ts >> 32));
  uint32_t frac_be = htonl(static_cast<uint32_t>(ntp_ts & 0xFFFFFFFFULL));
  std::memcpy(dst8 + 0, &sec_be, 4);
  std::memcpy(dst8 + 4, &frac_be, 4);
}

/** Read NTP timestamp (64-bit big-endian) to TimeSpec. */
inline ntpserver::TimeSpec ReadTimestamp(const uint8_t* src8) {
  uint32_t sec_be = 0, frac_be = 0;
  std::memcpy(&sec_be, src8 + 0, 4);
  std::memcpy(&frac_be, src8 + 4, 4);
  uint32_t sec = ntohl(sec_be);
  uint32_t frac = ntohl(frac_be);
  uint64_t ntp_ts = (static_cast<uint64_t>(sec) << 32) | frac;
  return ntpserver::TimeSpec::FromNtpTimestamp(ntp_ts);
}
}  // namespace

bool ntpclock::ClockService::Impl::UdpExchange(
    double* out_offset_s, int* out_rtt_ms, std::vector<uint8_t>* out_response,
    std::string* err) {
  if (!udp_socket.IsOpen()) {
    if (err) *err = "socket not open";
    return false;
  }

  // Reset abort flag
  exchange_abort.store(false, std::memory_order_relaxed);

  // Get transmit timestamp (T1) and build request
  assert(time_source != nullptr && "time_source must be set during Start()");
  ntpserver::TimeSpec T1 = time_source->NowUnix();
  std::vector<uint8_t> req = BuildNtpRequest(T1);

  // Send request
  if (!udp_socket.Send(req)) {
    if (err) *err = "send failed";
    return false;
  }

  // Wait for response
  internal::UdpSocket::Message msg;
  if (!WaitForExchangeResponse(500, &msg, err)) {
    return false;
  }

  // Process response
  if (!ProcessNtpResponse(msg, T1, out_offset_s, out_rtt_ms, err)) {
    return false;
  }

  *out_response = std::move(msg.data);
  return true;
}

std::vector<uint8_t> ntpclock::ClockService::Impl::BuildNtpRequest(
    const ntpserver::TimeSpec& T1) {
  ntpserver::NtpPacket req{};
  std::memset(&req, 0, sizeof(req));
  req.li_vn_mode = static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);  // v4, client

  WriteTimestamp(T1, reinterpret_cast<uint8_t*>(&req.tx_timestamp));

  std::vector<uint8_t> buf(sizeof(req));
  std::memcpy(buf.data(), &req, sizeof(req));
  return buf;
}

bool ntpclock::ClockService::Impl::WaitForExchangeResponse(
    int timeout_ms, internal::UdpSocket::Message* out_msg, std::string* err) {
  const int kPollIntervalMs = 50;
  int elapsed_ms = 0;

  while (elapsed_ms < timeout_ms) {
    // Check for abort signal
    if (exchange_abort.load(std::memory_order_relaxed)) {
      if (err) *err = "aborted by push";
      return false;
    }

    // Wait for message with short timeout
    if (udp_socket.WaitMessage(kPollIntervalMs, out_msg)) {
      // Check message type
      if (out_msg->type == internal::UdpSocket::MessageType::Push) {
        // Push received during Exchange - abort
        if (err) *err = "push during exchange";
        return false;
      }
      return true;
    }

    elapsed_ms += kPollIntervalMs;
  }

  // Timeout
  if (err) *err = "recvfrom timeout/failure";
  return false;
}

bool ntpclock::ClockService::Impl::ProcessNtpResponse(
    const internal::UdpSocket::Message& msg, const ntpserver::TimeSpec& T1,
    double* out_offset_s, int* out_rtt_ms, std::string* err) {
  if (msg.data.size() < sizeof(ntpserver::NtpPacket)) {
    if (err) *err = "response too small";
    return false;
  }

  // Parse NTP packet
  ntpserver::NtpPacket resp{};
  std::memcpy(&resp, msg.data.data(), sizeof(resp));

  // Extract timestamps as TimeSpec
  ntpserver::TimeSpec T2 =
      ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.recv_timestamp));
  ntpserver::TimeSpec T3 =
      ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.tx_timestamp));
  const ntpserver::TimeSpec& T4 = msg.recv_time;

  // Compute offset and delay using TimeSpec arithmetic
  // delay = (T4 - T1) - (T3 - T2)
  ntpserver::TimeSpec delay = (T4 - T1) - (T3 - T2);
  // offset = ((T2 - T1) + (T3 - T4)) / 2
  ntpserver::TimeSpec offset_2x = (T2 - T1) + (T3 - T4);

  *out_offset_s = offset_2x.ToDouble() / 2.0;
  *out_rtt_ms =
      static_cast<int>(std::max(0.0, delay.ToDouble()) * 1000.0 + 0.5);
  return true;
}

ntpclock::ClockService::Impl::VendorHintResult
ntpclock::ClockService::Impl::ApplyVendorHintFromRx(
    const std::vector<uint8_t>& rx, const Options& snapshot) {
  (void)snapshot;  // Reserved for future use
  return ApplyVendorHints(rx, false);
}

void ntpclock::ClockService::Impl::HandlePushMessage(
    const internal::UdpSocket::Message& msg, const Options& snapshot) {
  (void)snapshot;  // Reserved for future use
  ApplyVendorHints(msg.data, true);
}

ntpclock::ClockService::Impl::VendorHintResult
ntpclock::ClockService::Impl::ApplyVendorHints(const std::vector<uint8_t>& rx,
                                               bool allow_abort) {
  VendorHintResult hint_result;

  auto ts = time_source;
  if (!ts) return hint_result;

  // Process vendor hints with epoch detection
  bool epoch_changed = false;
  auto result = vendor_hint_processor.ProcessWithEpochDetection(
      rx, sizeof(ntpserver::NtpPacket), ts, &epoch_changed);

  hint_result.epoch_changed = epoch_changed;

  // If reset is needed, clear estimator state
  if (result.reset_needed) {
    if (allow_abort) {
      exchange_abort.store(true, std::memory_order_relaxed);
    }
    estimator_state.Clear();
    clock_corrector.ResetOffset();

    // If absolute time was set, allow backward jump
    if (result.abs_applied) {
      clock_corrector.AllowBackwardOnce();
      hint_result.abs_applied = true;
      hint_result.step_amount = result.step_amount;
    }
    hint_result.applied = true;
  }

  return hint_result;
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

void ntpclock::ClockService::Impl::SetBaseStatusFields(
    Status* st, const Options& snapshot, double sample_offset_s,
    int sample_rtt_ms, double tnow, int good_samples) {
  st->synchronized = (good_samples >= snapshot.MinSamplesToLock());
  st->rtt_ms = sample_rtt_ms;
  st->last_delay_s = sample_rtt_ms / 1000.0;
  st->offset_s = sample_offset_s;
  st->last_update = ntpserver::TimeSpec::FromDouble(tnow);
  st->last_error.clear();
  const std::string sock_err = GetSocketError();
  if (!sock_err.empty()) st->last_error = sock_err;
}

ntpclock::Status ntpclock::ClockService::Impl::BuildVendorHintAppliedStatus(
    const Options& snapshot, double sample_offset_s, int sample_rtt_ms,
    double tnow, int good_samples, const VendorHintResult& hint_result) {
  Status st;
  SetBaseStatusFields(&st, snapshot, sample_offset_s, sample_rtt_ms, tnow,
                      good_samples);

  // Include vendor hint step correction if applied
  if (hint_result.abs_applied) {
    st.last_correction = Status::Correction::Step;
    st.last_correction_amount_s = hint_result.step_amount.ToDouble();
    st.window_count = 0;
    st.offset_applied_s = clock_corrector.GetOffsetApplied();
    st.offset_target_s = 0.0;
    // Override offset_s with actual step amount (NTP offset is meaningless
    // after TimeSource update)
    st.offset_s = hint_result.step_amount.ToDouble();
  }

  // Set epoch changed flag and current epoch number
  st.epoch_changed = hint_result.epoch_changed;
  st.epoch = vendor_hint_processor.GetCurrentEpoch();

  return st;
}

ntpclock::Status ntpclock::ClockService::Impl::BuildSuccessStatus(
    const Options& snapshot, double sample_offset_s, int sample_rtt_ms,
    double tnow, int good_samples, double median, double omin, double omax,
    Status::Correction correction, double correction_amount,
    const VendorHintResult& hint_result) {
  Status st;
  SetBaseStatusFields(&st, snapshot, sample_offset_s, sample_rtt_ms, tnow,
                      good_samples);

  st.skew_ppm = estimator_state.ComputeSkewPpm(snapshot.SkewWindow());
  st.samples = good_samples;
  st.offset_window = snapshot.OffsetWindow();
  st.skew_window = snapshot.SkewWindow();
  st.window_count = estimator_state.GetSampleCount();
  st.offset_median_s = median;
  st.offset_min_s = omin;
  st.offset_max_s = omax;
  st.offset_applied_s = clock_corrector.GetOffsetApplied();
  st.offset_target_s = median;
  st.last_correction = correction;
  st.last_correction_amount_s = correction_amount;
  st.epoch_changed = hint_result.epoch_changed;
  st.epoch = vendor_hint_processor.GetCurrentEpoch();

  return st;
}

bool ntpclock::ClockService::Impl::WaitForPushOrPollDeadline(
    std::chrono::steady_clock::time_point next_poll_time,
    const Options& snapshot, int* good_samples) {
  auto now = std::chrono::steady_clock::now();

  while (now < next_poll_time) {
    auto remaining =
        std::chrono::duration_cast<milliseconds>(next_poll_time - now);
    int wait_ms = std::min(static_cast<int>(remaining.count()), 100);

    if (wait_ms <= 0) break;

    internal::UdpSocket::Message msg;
    if (udp_socket.WaitMessage(wait_ms, &msg)) {
      if (msg.type == internal::UdpSocket::MessageType::Push) {
        if (log_callback_) {
          std::ostringstream oss;
          oss << "[ClockService] Push notification received, resetting sync "
                 "state (good_samples="
              << *good_samples << "->0)";
          log_callback_(oss.str());
        }
        HandlePushMessage(msg, snapshot);
        // Reset good_samples when Push is received (epoch change notification)
        *good_samples = 0;
        return true;
      }
      // Ignore non-Push messages (unexpected Exchange responses)
    }

    now = std::chrono::steady_clock::now();
  }

  return false;
}

ntpclock::Status ntpclock::ClockService::Impl::ProcessExchangeAndBuildStatus(
    const Options& snapshot, double step_thresh_s, double slew_rate_s_per_s,
    int poll_interval_ms, int* good_samples) {
  // Acquire NTP sample
  double sample_offset_s = 0.0;
  int sample_rtt_ms = 0;
  std::vector<uint8_t> response;
  std::string err;
  bool ok = UdpExchange(&sample_offset_s, &sample_rtt_ms, &response, &err);

  // Clear socket error on successful exchange
  if (ok) {
    ClearSocketError();
  }

  // Process vendor hint from response if available
  VendorHintResult hint_result;
  if (ok && response.size() > sizeof(ntpserver::NtpPacket)) {
    hint_result = ApplyVendorHintFromRx(response, snapshot);
  }

  // Build status based on sample validity and vendor hint state
  Status st_local;
  assert(time_source != nullptr && "time_source must be set during Start()");
  double tnow = time_source->NowUnix().ToDouble();

  if (ok && sample_rtt_ms <= snapshot.MaxRttMs() && hint_result.applied) {
    // Vendor hint applied: reset sync state and skip normal correction
    int old_samples = *good_samples;
    *good_samples = 0;
    st_local =
        BuildVendorHintAppliedStatus(snapshot, sample_offset_s, sample_rtt_ms,
                                     tnow, *good_samples, hint_result);
    if (log_callback_) {
      std::ostringstream oss;
      oss << "[ClockService] Epoch change detected in exchange, resetting sync "
             "state (good_samples="
          << old_samples << "->0)";
      log_callback_(oss.str());
    }
  } else if (ok && sample_rtt_ms <= snapshot.MaxRttMs()) {
    // Normal path: process sample and apply correction
    auto [median, omin, omax] =
        UpdateEstimatorsAndTarget(snapshot, sample_offset_s, tnow);

    double applied = clock_corrector.GetOffsetApplied();
    double correction_amount = 0.0;
    Status::Correction correction =
        clock_corrector.Apply(applied, median, step_thresh_s, slew_rate_s_per_s,
                              poll_interval_ms / 1000.0, &correction_amount);

    (*good_samples)++;
    st_local = BuildSuccessStatus(snapshot, sample_offset_s, sample_rtt_ms,
                                  tnow, *good_samples, median, omin, omax,
                                  correction, correction_amount, hint_result);
  } else {
    // Error path: sample acquisition failed or RTT exceeded threshold
    st_local.synchronized = (*good_samples >= snapshot.MinSamplesToLock());
    st_local.rtt_ms = sample_rtt_ms;
    st_local.last_delay_s = sample_rtt_ms / 1000.0;
    st_local.offset_s = sample_offset_s;
    st_local.last_error = err.empty() ? "sample rejected" : err;
  }

  return st_local;
}

void ntpclock::ClockService::Impl::Loop() {
  int good_samples = 0;
  auto next_poll_time = std::chrono::steady_clock::now();

  while (running.load(std::memory_order_acquire)) {
    // 1. Snapshot configuration for this iteration
    auto snapshot = [&]() {
      std::lock_guard<std::mutex> lk(opts_mtx);
      return opts;
    }();
    const double step_thresh_s = snapshot.StepThresholdMs() / 1000.0;
    const double slew_rate_s_per_s = snapshot.SlewRateMsPerSec() / 1000.0;
    const int poll_interval_ms = snapshot.PollIntervalMs();

    // 2. Wait for Push or poll deadline
    // If Push is received, returns immediately for instant Exchange execution
    WaitForPushOrPollDeadline(next_poll_time, snapshot, &good_samples);

    // Update next poll deadline
    next_poll_time =
        std::chrono::steady_clock::now() + milliseconds(poll_interval_ms);

    // 3. Execute Exchange and build status
    Status st_local = ProcessExchangeAndBuildStatus(
        snapshot, step_thresh_s, slew_rate_s_per_s, poll_interval_ms,
        &good_samples);

    // 4. Update shared status
    {
      std::lock_guard<std::mutex> lk(status_mtx);
      status = st_local;
    }
  }
}

void ntpclock::ClockService::Impl::ReportSocketError(const std::string& msg) {
  {
    std::lock_guard<std::mutex> lk(socket_err_mtx);
    socket_last_error = msg;
  }
  {
    std::lock_guard<std::mutex> lk(status_mtx);
    status.last_error = msg;
  }
}

std::string ntpclock::ClockService::Impl::GetSocketError() {
  std::lock_guard<std::mutex> lk(socket_err_mtx);
  return socket_last_error;
}

void ntpclock::ClockService::Impl::ClearSocketError() {
  std::lock_guard<std::mutex> lk(socket_err_mtx);
  socket_last_error.clear();
}

// ---------------- ClockService ----------------
ntpclock::ClockService::ClockService() : p_(new Impl()) {}
ntpclock::ClockService::~ClockService() { Stop(); }

bool ntpclock::ClockService::Start(ntpserver::TimeSource* time_source,
                                   const std::string& ip, uint16_t port,
                                   const Options& opt) {
  // Use platform default TimeSource if time_source is nullptr
  if (!time_source) {
    time_source = &ntpserver::platform::GetDefaultTimeSource();
  }

  Stop();
  p_->time_source = time_source;

  {
    std::lock_guard<std::mutex> lk(p_->opts_mtx);
    p_->opts = opt;
  }
  p_->log_callback_ = opt.LogSink();
  p_->vendor_hint_processor.SetLogSink(p_->log_callback_);

  // Open UDP socket for persistent connection
  auto get_time = [time_source]() -> ntpserver::TimeSpec {
    assert(time_source != nullptr && "time_source must be set during Start()");
    return time_source->NowUnix();
  };
  auto log_fn = [impl = p_.get()](const std::string& msg) {
    if (impl) impl->ReportSocketError(msg);
  };
  if (!p_->udp_socket.Open(ip, port, get_time, log_fn)) {
    p_->time_source = nullptr;
    return false;
  }

  p_->running.store(true, std::memory_order_release);
  p_->thread = std::thread([this]() { p_->Loop(); });
  return true;
}

bool ntpclock::ClockService::Start(const std::string& ip, uint16_t port,
                                   const Options& opt) {
  return Start(nullptr, ip, port, opt);
}

void ntpclock::ClockService::Stop() {
  if (!p_->running.exchange(false)) return;
  p_->udp_socket.Close();  // Close socket first to unblock receive thread
  if (p_->thread.joinable()) p_->thread.join();
  p_->time_source = nullptr;
}

ntpserver::TimeSpec ntpclock::ClockService::NowUnix() const {
  auto ts = p_->time_source;
  if (!ts) return ntpserver::TimeSpec{};

  ntpserver::TimeSpec base = ts->NowUnix();

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

double ntpclock::ClockService::GetRate() const {
  auto ts = p_->time_source;
  return ts ? ts->GetRate() : 1.0;
}
