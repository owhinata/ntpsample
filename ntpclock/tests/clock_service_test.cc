/**
 * @file
 * @test OptionsTest.BuilderAndStream
 * @brief Verify Options builder and stream operator.
 * @steps
 *   1. Build Options via Builder with custom values.
 *   2. Stream to ostringstream.
 * @expected Stream contains key fields like "poll=".
 */
#include <gtest/gtest.h>

#include "ntpclock/clock_service.hpp"

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
 * @steps
 *   1. Start service against TEST-NET-3 IPv4 address.
 *   2. Call NowUnix() once and Stop().
 * @expected No crash; function calls return normally.
 */
TEST(ClockServiceTest, StartStopWithoutServer) {
  ClockService svc;
  auto opts = Options::Builder().PollIntervalMs(100).Build();
  // Start against a non-routable test net address: no crash expected.
  svc.Start("203.0.113.1", 9123, opts);  // TEST-NET-3 per RFC 5737
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  (void)svc.NowUnix();
  svc.Stop();
}
