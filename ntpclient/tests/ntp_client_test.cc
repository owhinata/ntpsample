// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_client_test.cc
 * @brief Unit tests for ntpclient using an injectable transport.
 */
#include "ntpclient/ntp_client.hpp"

#include <gtest/gtest.h>
#include <winsock2.h>

#include <string>

#include "ntpserver/ntp_server.hpp"
#include "ntpserver/qpc_clock.hpp"

namespace ntpclient {

/**
 * @test NtpClientTest.SyncOnce_StepsClockWithRealServer
 * @brief Verify sync against a real in-process NTP server (UDP localhost).
 * @steps
 * 1. Start ntpserver on a test port and set its clock to T.
 * 2. Initialize client clock to T-5 (large offset).
 * 3. Call SyncOnce to the server and check client clock ~ T.
 * @expected
 * - SyncOnce returns true.
 * - Client clock steps within 0.5s of server time.
 */
TEST(NtpClientTest, SyncOnce_StepsClockWithRealServer) {
  const uint16_t kPort = 29125;
  auto& server_clock = ntpserver::QpcClock::Instance();
  const double server_time = 1'700'000'000.0;
  server_clock.SetAbsolute(server_time);

  ntpserver::NtpServer server;
  server.SetTimeSource(&server_clock);
  ASSERT_TRUE(server.Start(kPort));

  ntpserver::QpcClock client_clock;
  client_clock.SetAbsolute(server_time - 5.0);
  NtpClient client(&client_clock);

  ASSERT_TRUE(client.SyncOnce("127.0.0.1", kPort, 1000));
  EXPECT_NEAR(client_clock.NowUnix(), server_time, 0.5);

  server.Stop();
}

}  // namespace ntpclient
