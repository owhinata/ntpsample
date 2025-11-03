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

#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

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
      skew_window_(20),
      time_source_(nullptr) {}

ntpclock::Options::Builder::Builder(const Options& base)
    : poll_interval_ms_(base.PollIntervalMs()),
      step_threshold_ms_(base.StepThresholdMs()),
      slew_rate_ms_per_s_(base.SlewRateMsPerSec()),
      max_rtt_ms_(base.MaxRttMs()),
      min_samples_to_lock_(base.MinSamplesToLock()),
      offset_window_(base.OffsetWindow()),
      skew_window_(base.SkewWindow()),
      time_source_(base.TimeSourcePtr()) {}

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

ntpclock::Options::Builder& ntpclock::Options::Builder::TimeSource(
    ntpserver::TimeSource* ts) {
  time_source_ = ts;
  return *this;
}

ntpclock::Options ntpclock::Options::Builder::Build() const {
  return ntpclock::Options(
      poll_interval_ms_, step_threshold_ms_, slew_rate_ms_per_s_, max_rtt_ms_,
      min_samples_to_lock_, offset_window_, skew_window_, time_source_);
}

namespace ntpclock {
std::ostream& operator<<(std::ostream& os, const Options& o) {
  os << "poll=" << o.PollIntervalMs() << "ms, step>=" << o.StepThresholdMs()
     << "ms, slew=" << o.SlewRateMsPerSec() << "ms/s, max_rtt=" << o.MaxRttMs()
     << "ms, min_lock=" << o.MinSamplesToLock()
     << ", ts=" << (o.TimeSourcePtr() ? "custom" : "<null>");
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
  Options opts{Options::Builder().Build()};
  std::mutex opts_mtx;

  std::atomic<bool> running{false};
  std::thread thread;

  // Offsets (seconds)
  std::atomic<double> offset_applied_s{0.0};
  double offset_target_s = 0.0;
  std::mutex est_mtx;

  // Status
  Status status{};
  std::mutex status_mtx;

  // Monotonicity controls
  std::atomic<double> last_now_returned_s{0.0};
  std::atomic<bool> allow_backward_once{false};

  // Networking
  bool UdpExchange(double* out_offset_s, int* out_rtt_ms, std::string* err);
  uint32_t last_ctrl_seq{0};

  /**
   * @brief Parse NTP vendor extension from a received datagram and apply.
   *
   * @param rx Raw datagram bytes (NTP header + optional extension fields).
   * @note Applies SetRate/SetAbsolute when payload differences are meaningful,
   *       and resets estimator windows accordingly. Deduplicates by sequence.
   */
  void ApplyVendorHintFromRx(const std::vector<uint8_t>& rx);

  // Worker
  void Loop();

  // Simple estimators/storage
  std::vector<double> offsets_;
  std::vector<double> times_;
};

namespace {
constexpr uint32_t kNtpUnixEpochDiff = 2208988800UL;  // seconds

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
  uint64_t recv_timestamp;  // T2
  uint64_t tx_timestamp;    // T3
};
#pragma pack(pop)

inline void WriteTimestamp(double unix_seconds, uint8_t* dst8) {
  uint32_t sec =
      static_cast<uint32_t>(std::floor(unix_seconds + kNtpUnixEpochDiff));
  double frac_d = (unix_seconds + kNtpUnixEpochDiff) - static_cast<double>(sec);
  uint32_t frac =
      static_cast<uint32_t>(frac_d * static_cast<double>(1ULL << 32));
  uint32_t sec_be = htonl(sec);
  uint32_t frac_be = htonl(frac);
  std::memcpy(dst8 + 0, &sec_be, 4);
  std::memcpy(dst8 + 4, &frac_be, 4);
}

inline double ReadTimestamp(const uint8_t* src8) {
  uint32_t sec_be = 0, frac_be = 0;
  std::memcpy(&sec_be, src8 + 0, 4);
  std::memcpy(&frac_be, src8 + 4, 4);
  uint32_t sec = ntohl(sec_be);
  uint32_t frac = ntohl(frac_be);
  return (static_cast<double>(sec) - static_cast<double>(kNtpUnixEpochDiff)) +
         (static_cast<double>(frac) / static_cast<double>(1ULL << 32));
}
}  // namespace

bool ntpclock::ClockService::Impl::UdpExchange(double* out_offset_s,
                                               int* out_rtt_ms,
                                               std::string* err) {
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    if (err) *err = "WSAStartup failed";
    return false;
  }

  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) {
    if (err) *err = "socket() failed";
    WSACleanup();
    return false;
  }

  // Destination address
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1) {
    if (err) *err = "inet_pton failed (IPv4 only)";
    closesocket(s);
    WSACleanup();
    return false;
  }

  // Build request
  NtpPacket req{};
  std::memset(&req, 0, sizeof(req));
  req.li_vn_mode = static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);  // v4, client

  double T1 = 0.0, T4 = 0.0;
  {
    std::lock_guard<std::mutex> lk(opts_mtx);
    T1 = opts.TimeSourcePtr()->NowUnix();
  }
  WriteTimestamp(T1, reinterpret_cast<uint8_t*>(&req.tx_timestamp));

  // Send
  int sent = sendto(s, reinterpret_cast<const char*>(&req), sizeof(req), 0,
                    reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (sent != sizeof(req)) {
    if (err) *err = "sendto failed";
    closesocket(s);
    WSACleanup();
    return false;
  }

  // Timeout 500ms
  DWORD timeout = 500;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO,
             reinterpret_cast<const char*>(&timeout), sizeof(timeout));

  // Receive into a buffer to allow NTP extension fields beyond 48 bytes
  std::vector<uint8_t> rx(1500);
  sockaddr_in from{};
  int fromlen = sizeof(from);
  int recvd = recvfrom(s, reinterpret_cast<char*>(rx.data()),
                       static_cast<int>(rx.size()), 0,
                       reinterpret_cast<sockaddr*>(&from), &fromlen);
  {
    std::lock_guard<std::mutex> lk(opts_mtx);
    T4 = opts.TimeSourcePtr()->NowUnix();
  }

  if (recvd < static_cast<int>(sizeof(NtpPacket))) {
    if (err) *err = "recvfrom timeout/failure";
    closesocket(s);
    WSACleanup();
    return false;
  }

  closesocket(s);
  WSACleanup();

  NtpPacket resp{};
  std::memcpy(&resp, rx.data(), sizeof(resp));

  // Parse optional NTP Extension Field for vendor hints (ABS/RATE)
  if (recvd > static_cast<int>(sizeof(NtpPacket))) {
    ApplyVendorHintFromRx(rx);
  }
  double T2 = ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.recv_timestamp));
  double T3 = ReadTimestamp(reinterpret_cast<uint8_t*>(&resp.tx_timestamp));

  double delay = (T4 - T1) - (T3 - T2);
  double offset = ((T2 - T1) + (T3 - T4)) / 2.0;

  *out_offset_s = offset;
  *out_rtt_ms = static_cast<int>(std::max(0.0, delay) * 1000.0 + 0.5);
  return true;
}

void ntpclock::ClockService::Impl::ApplyVendorHintFromRx(
    const std::vector<uint8_t>& rx) {
  if (rx.size() <= sizeof(NtpPacket)) return;
  const uint8_t* p = rx.data() + sizeof(NtpPacket);
  size_t remain = rx.size() - sizeof(NtpPacket);
  if (remain < 4U) return;
  uint16_t typ = static_cast<uint16_t>((p[0] << 8) | p[1]);
  uint16_t len = static_cast<uint16_t>((p[2] << 8) | p[3]);
  if (typ != ntpserver::NtpVendorExt::kEfTypeVendorHint || len < 4U ||
      len > remain)
    return;
  std::vector<uint8_t> val(p + 4, p + len);
  ntpserver::NtpVendorExt::Payload v{};
  if (!ntpserver::NtpVendorExt::Parse(val, &v)) return;
  if (v.seq <= last_ctrl_seq) return;  // dedupe stale/duplicate
  last_ctrl_seq = v.seq;

  bool reset = false;
  ntpserver::TimeSource* ts = nullptr;
  int step_ms = 0;
  {
    std::lock_guard<std::mutex> lk(opts_mtx);
    ts = opts.TimeSourcePtr();
    step_ms = opts.StepThresholdMs();
  }
  if (ts == nullptr) return;

  const double rate_eps = 1e-12;  // ~1e-6 ppm
  if ((v.flags & ntpserver::NtpVendorExt::kFlagRate) != 0U) {
    double cur_rate = ts->GetRate();
    if (std::abs(v.rate_scale - cur_rate) > rate_eps) {
      ts->SetRate(v.rate_scale);
      reset = true;
    }
  }
  if ((v.flags & ntpserver::NtpVendorExt::kFlagAbs) != 0U) {
    double cur = ts->NowUnix();
    double abs_thresh = std::max(0, step_ms) / 1000.0;
    if (std::abs(v.abs_unix_s - cur) >= abs_thresh) {
      ts->SetAbsolute(v.abs_unix_s);
      reset = true;
      allow_backward_once.store(true, std::memory_order_relaxed);
    }
  }
  if (reset) {
    std::lock_guard<std::mutex> lk(est_mtx);
    offsets_.clear();
    times_.clear();
    offset_applied_s.store(0.0, std::memory_order_relaxed);
    offset_target_s = 0.0;
  }
}

void ntpclock::ClockService::Impl::Loop() {
  // Loop until stopped; snapshot options each iteration for runtime changes.

  int good_samples = 0;
  while (running.load(std::memory_order_acquire)) {
    auto snapshot = [&]() {
      std::lock_guard<std::mutex> lk(opts_mtx);
      return opts;  // copy
    }();
    const double step_thresh_s = snapshot.StepThresholdMs() / 1000.0;
    const double slew_rate_s_per_s = snapshot.SlewRateMsPerSec() / 1000.0;
    const int poll_interval_ms = snapshot.PollIntervalMs();

    double sample_offset_s = 0.0;
    int sample_rtt_ms = 0;
    std::string err;
    bool ok = UdpExchange(&sample_offset_s, &sample_rtt_ms, &err);

    Status st_local;
    st_local.last_error.clear();
    st_local.rtt_ms = sample_rtt_ms;
    st_local.last_delay_s = sample_rtt_ms / 1000.0;
    st_local.offset_s = sample_offset_s;
    st_local.last_update_unix_s = 0.0;
    st_local.samples = 0;
    st_local.skew_ppm = 0.0;
    st_local.last_correction = Status::Correction::None;
    st_local.last_correction_amount_s = 0.0;

    if (ok && sample_rtt_ms <= snapshot.MaxRttMs()) {
      // Record sample time for estimators
      double tnow = 0.0;
      {
        std::lock_guard<std::mutex> lk(opts_mtx);
        tnow = opts.TimeSourcePtr()->NowUnix();
      }
      // Keep sliding windows
      offsets_.push_back(sample_offset_s);
      times_.push_back(tnow);
      int maxw = std::max(snapshot.OffsetWindow(), snapshot.SkewWindow());
      if (maxw < 1) maxw = 1;
      while (offsets_.size() > static_cast<size_t>(maxw))
        offsets_.erase(offsets_.begin());
      while (times_.size() > static_cast<size_t>(maxw))
        times_.erase(times_.begin());

      // Compute robust target: median of last window, and min/max for debug
      double median = sample_offset_s;
      double omin = sample_offset_s, omax = sample_offset_s;
      {
        size_t n = offsets_.size();
        size_t win = static_cast<size_t>(std::max(1, snapshot.OffsetWindow()));
        size_t start = (n > win) ? (n - win) : 0;
        std::vector<double> tmp(offsets_.begin() + start, offsets_.end());
        if (!tmp.empty()) {
          std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2,
                           tmp.end());
          median = tmp[tmp.size() / 2];
          auto mm = std::minmax_element(tmp.begin(), tmp.end());
          omin = *mm.first;
          omax = *mm.second;
        }
        std::lock_guard<std::mutex> lk(est_mtx);
        offset_target_s = median;
      }

      double applied = offset_applied_s.load(std::memory_order_relaxed);
      double delta = 0.0;
      {
        std::lock_guard<std::mutex> lk(est_mtx);
        delta = offset_target_s - applied;
      }

      if (std::abs(delta) >= step_thresh_s) {
        // Step correction (may jump backwards)
        offset_applied_s.store(applied + delta, std::memory_order_relaxed);
        allow_backward_once.store(true, std::memory_order_release);
        st_local.last_correction = Status::Correction::Step;
        st_local.last_correction_amount_s = delta;
      } else {
        // Slew correction (bounded rate)
        double max_change = slew_rate_s_per_s * (poll_interval_ms / 1000.0);
        double change = std::clamp(delta, -max_change, max_change);
        offset_applied_s.store(applied + change, std::memory_order_relaxed);
        if (std::abs(change) > 0.0) {
          st_local.last_correction = Status::Correction::Slew;
          st_local.last_correction_amount_s = change;
        }
      }

      good_samples++;
      st_local.synchronized = (good_samples >= snapshot.MinSamplesToLock());
      st_local.last_update_unix_s = tnow;
      // Estimate skew (ppm) via simple OLS slope of offset over time
      if (times_.size() >= 2) {
        size_t n =
            std::min(times_.size(),
                     static_cast<size_t>(std::max(1, snapshot.SkewWindow())));
        double mean_t = 0.0, mean_o = 0.0;
        for (size_t i = times_.size() - n; i < times_.size(); ++i) {
          mean_t += times_[i];
          mean_o += offsets_[i];
        }
        mean_t /= static_cast<double>(n);
        mean_o /= static_cast<double>(n);
        double num = 0.0, den = 0.0;
        for (size_t i = times_.size() - n; i < times_.size(); ++i) {
          double dt = times_[i] - mean_t;
          double doff = offsets_[i] - mean_o;
          num += dt * doff;
          den += dt * dt;
        }
        double slope = (den > 0.0) ? (num / den) : 0.0;  // sec offset per sec
        st_local.skew_ppm = slope * 1e6;                 // convert to ppm
      }
      st_local.samples = good_samples;
      st_local.offset_window = snapshot.OffsetWindow();
      st_local.skew_window = snapshot.SkewWindow();
      st_local.window_count = static_cast<int>(offsets_.size());
      st_local.offset_median_s = median;
      st_local.offset_min_s = omin;
      st_local.offset_max_s = omax;
      st_local.offset_applied_s =
          offset_applied_s.load(std::memory_order_relaxed);
      {
        std::lock_guard<std::mutex> lk(est_mtx);
        st_local.offset_target_s = offset_target_s;
      }
    } else {
      // Failure; keep previous applied offset, keep trying.
      st_local.synchronized = (good_samples >= snapshot.MinSamplesToLock());
      st_local.last_error = err.empty() ? "sample rejected" : err;
    }

    {
      std::lock_guard<std::mutex> lk(status_mtx);
      status = st_local;
    }

    std::this_thread::sleep_for(milliseconds(poll_interval_ms));
  }
}

// ---------------- ClockService ----------------
ntpclock::ClockService::ClockService() : p_(new Impl()) {}
ntpclock::ClockService::~ClockService() { Stop(); }

bool ntpclock::ClockService::Start(const std::string& ip, uint16_t port,
                                   const Options& opt) {
  Stop();
  p_->ip = ip;
  p_->port = port;
  // Ensure non-null time source (default to QpcClock::Instance()).
  ntpserver::TimeSource* ts = opt.TimeSourcePtr()
                                  ? opt.TimeSourcePtr()
                                  : &ntpserver::QpcClock::Instance();
  Options effective = Options::Builder(opt).TimeSource(ts).Build();
  {
    std::lock_guard<std::mutex> lk(p_->opts_mtx);
    p_->opts = effective;
  }
  p_->running.store(true, std::memory_order_release);
  p_->thread = std::thread([this]() { p_->Loop(); });
  return true;
}

void ntpclock::ClockService::Stop() {
  if (!p_->running.exchange(false)) return;
  if (p_->thread.joinable()) p_->thread.join();
}

double ntpclock::ClockService::NowUnix() const {
  double base = 0.0;
  {
    std::lock_guard<std::mutex> lk(p_->opts_mtx);
    base = p_->opts.TimeSourcePtr()->NowUnix();
  }
  double candidate =
      base + p_->offset_applied_s.load(std::memory_order_relaxed);

  // Monotonic during slew only; allow one backward jump right after a Step.
  if (p_->allow_backward_once.exchange(false)) {
    p_->last_now_returned_s.store(candidate, std::memory_order_relaxed);
    return candidate;
  }
  double last = p_->last_now_returned_s.load(std::memory_order_relaxed);
  const double eps = 1e-9;
  double clamped = std::max(candidate, last + eps);
  p_->last_now_returned_s.store(clamped, std::memory_order_relaxed);
  return clamped;
}

double ntpclock::ClockService::OffsetSeconds() const {
  return p_->offset_applied_s.load(std::memory_order_relaxed);
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
