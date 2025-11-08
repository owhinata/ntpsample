/**
 * @file
 * @test OptionsTest.BuilderAndStream
 * @brief Verify Options builder and stream operator.
 *
 * @steps
 * 1. Build Options via Builder with custom values.
 * 2. Stream to ostringstream.
 *
 * @expected Stream contains key fields like "poll=".
 */
#include <gtest/gtest.h>
#include <chrono>

#include "ntpclock/clock_service.hpp"
#include "ntpserver/ntp_server.hpp"

using ntpclock::ClockService;
using ntpclock::Options;
using ntpclock::Status;

TEST(OptionsTest, BuilderAndStream) {
  auto opts = Options::Builder()
                  .PollIntervalMs(1000)
                  .StepThresholdMs(200)
                  .SlewRateMsPerSec(5.0)
                  .MaxRttMs(100)
                  .MinSamplesToLock(3)
                  .Build();
  std::ostringstream oss;
  oss << opts;
  EXPECT_NE(oss.str().find("poll="), std::string::npos);
}

/**
 * @test ClockServiceTest.StartStopWithoutServer
 * @brief Start/Stop with non-routable address must not crash.
 *
 * @steps
 * 1. Start service against TEST-NET-3 IPv4 address.
 * 2. Call NowUnix() once and Stop().
 *
 * @expected No crash; function calls return normally.
 */
TEST(ClockServiceTest, StartStopWithoutServer) {
  ClockService svc;
  auto opts = Options::Builder().PollIntervalMs(100).Build();
  // Start against a non-routable test net address: no crash expected.
  svc.Start(&ntpserver::QpcClock::Instance(), "203.0.113.1", 9123,
            opts);  // TEST-NET-3 per RFC 5737
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)svc.NowUnix();
  svc.Stop();
}

namespace {
/** Simple fake time source for tests. */
class FakeTimeSource : public ntpserver::TimeSource {
 public:
  explicit FakeTimeSource(double start_unix)
      : start_unix_(start_unix), start_tp_(std::chrono::steady_clock::now()) {}
  double NowUnix() override {
    auto dt = std::chrono::steady_clock::now() - start_tp_;
    return start_unix_ + std::chrono::duration<double>(dt).count();
  }
  void SetAbsolute(double unix_sec) override {
    start_unix_ = unix_sec;
    start_tp_ = std::chrono::steady_clock::now();
  }
  void SetRate(double rate) override { (void)rate; }
  void Adjust(double delta) { SetAbsolute(NowUnix() + delta); }

 private:
  std::atomic<double> start_unix_;
  std::chrono::steady_clock::time_point start_tp_;
};
}  // namespace

/**
 * @test ClockServiceTest.SlewIsMonotonic
 * @brief With small offset (< step threshold), NowUnix() stays non-decreasing.
 *
 * @steps
 * 1. Start a local NtpServer on a test port with server time = now + 50ms.
 * 2. Start ClockService with 100ms polling.
 * 3. Wait for synchronization.
 * 4. Call NowUnix() repeatedly and ensure non-decreasing.
 *
 * @expected No backward jumps are observed during slew-only corrections.
 */
TEST(ClockServiceTest, SlewIsMonotonic) {
  using namespace std::chrono;
  // Server with time ahead by 50 ms
  double now = duration<double>(system_clock::now().time_since_epoch()).count();
  FakeTimeSource server_ts(now + 0.050);
  FakeTimeSource client_ts(now);

  ntpserver::NtpServer server;
  server.SetTimeSource(&server_ts);
  ASSERT_TRUE(server.Start(29333));
  std::this_thread::sleep_for(milliseconds(50));

  ClockService svc;
  auto opts = Options::Builder()
                  .PollIntervalMs(100)
                  .StepThresholdMs(200)
                  .MaxRttMs(200)
                  .Build();
  ASSERT_TRUE(svc.Start(&client_ts, "127.0.0.1", 29333, opts));
  std::this_thread::sleep_for(milliseconds(700));

  auto st = svc.GetStatus();
  EXPECT_TRUE(st.samples >= 3);

  double prev = svc.NowUnix();
  for (int i = 0; i < 50; ++i) {
    std::this_thread::sleep_for(milliseconds(10));
    double cur = svc.NowUnix();
    EXPECT_GE(cur, prev) << "time decreased during slew";
    prev = cur;
  }
  svc.Stop();
  server.Stop();
}

/**
 * @test ClockServiceTest.StepAllowsBackwardOnce
 * @brief A step correction may cause a single backward jump.
 *
 * @steps
 * 1. Start NtpServer and synchronize ClockService.
 * 2. Move server time back by 300 ms (>= step threshold).
 * 3. Wait one polling interval and call NowUnix().
 * 4. Call NowUnix() again.
 *
 * @expected First call after step is < previous time. Next calls are
 *           non-decreasing again.
 */
TEST(ClockServiceTest, StepAllowsBackwardOnce) {
  using namespace std::chrono;
  double now = duration<double>(system_clock::now().time_since_epoch()).count();
  FakeTimeSource server_ts(now);
  FakeTimeSource client_ts(now);

  ntpserver::NtpServer server;
  server.SetTimeSource(&server_ts);
  ASSERT_TRUE(server.Start(29334));
  std::this_thread::sleep_for(milliseconds(50));

  ClockService svc;
  auto opts = Options::Builder()
                  .PollIntervalMs(100)
                  .StepThresholdMs(200)
                  .OffsetWindow(1)  // use latest sample for target
                  .Build();
  ASSERT_TRUE(svc.Start(&client_ts, "127.0.0.1", 29334, opts));
  std::this_thread::sleep_for(milliseconds(500));

  double last_now_before_step = svc.NowUnix();
  // Step server backward by 300 ms
  server_ts.Adjust(-0.300);
  // Poll status; capture NowUnix() while waiting to keep a pre-step baseline
  bool stepped = false;
  for (int i = 0; i < 20; ++i) {
    std::this_thread::sleep_for(milliseconds(50));
    auto s = svc.GetStatus();
    if (s.last_correction == Status::Correction::Step) {
      stepped = true;
      break;
    }
    // still pre-step, update baseline
    last_now_before_step = svc.NowUnix();
  }
  ASSERT_TRUE(stepped) << "did not observe Step within timeout";
  double first_after = svc.NowUnix();
  // Expect a backward jump relative to the immediate pre-step time.
  EXPECT_LT(first_after, last_now_before_step);

  // Next call should not go backward again
  std::this_thread::sleep_for(milliseconds(10));
  double after2 = svc.NowUnix();
  EXPECT_GE(after2, first_after);

  svc.Stop();
  server.Stop();
}
