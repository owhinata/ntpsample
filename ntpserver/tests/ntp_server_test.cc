// Copyright (c) 2025 <Your Name>
#include <gtest/gtest.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ntpserver/ntp_server.hpp"

namespace ntpserver {

namespace {
constexpr uint32_t kNtpUnixEpochDiff = 2208988800UL;  // seconds

uint64_t ToNtpTimestamp(double unix_seconds) {
  double sec;
  double frac =
      std::modf(unix_seconds + static_cast<double>(kNtpUnixEpochDiff), &sec);
  uint64_t s = static_cast<uint64_t>(sec);
  uint64_t f = static_cast<uint64_t>(frac * static_cast<double>(1ULL << 32));
  return (s << 32) | f;
}

uint32_t Hton32(uint32_t v) { return htonl(v); }
uint64_t Hton64(uint64_t v) {
  uint32_t hi = htonl(static_cast<uint32_t>(v >> 32));
  uint32_t lo = htonl(static_cast<uint32_t>(v & 0xFFFFFFFFULL));
  return (static_cast<uint64_t>(lo) << 32) | hi;
}

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

class FakeTimeSource : public TimeSource {
 public:
  explicit FakeTimeSource(double t) : value_(t) {}
  double NowUnix() override { return value_.load(); }
  void SetAbsolute(double unix_sec) override { value_.store(unix_sec); }
  void SetRate(double rate) override {
    // No-op for tests; could emulate rate if needed.
    (void)rate;
  }
  void Set(double t) { value_.store(t); }

 private:
  std::atomic<double> value_;
};

class WinsockGuard {
 public:
  WinsockGuard() { WSAStartup(MAKEWORD(2, 2), &wsa_); }
  ~WinsockGuard() { WSACleanup(); }

 private:
  WSADATA wsa_{};
};
}  // namespace

/**
 * @test NtpServerTest.RespondsWithServerModeAndEchoesOrigTimestamp
 * @brief Verify that the server answers in mode 4 and echoes t1.
 *
 * @steps
 * 1. Start NtpServer on a test port with a fixed FakeTimeSource.
 * 2. Send a client-mode NTP request from localhost.
 * 3. Receive the response and parse header/timestamps.
 *
 * @expected
 * - li_vn_mode indicates VN=4 and Mode=4 (server).
 * - stratum equals 1 (default).
 * - orig_timestamp equals request's tx_timestamp (echoed).
 * - recv/tx timestamps match the fixed time source.
 */
TEST(NtpServerTest, RespondsWithServerModeAndEchoesOrigTimestamp) {
  WinsockGuard wsg;

  FakeTimeSource ts(1700000000.25);  // fixed test time
  NtpServer server;
  server.SetTimeSource(&ts);
  server.SetStratum(1);
  server.SetPrecision(-20);
  ASSERT_TRUE(server.Start(29123));

  // Small delay to ensure server thread enters recv loop.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_NE(sock, INVALID_SOCKET);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  addr.sin_port = htons(29123);

  NtpPacket req{};
  req.li_vn_mode =
      static_cast<uint8_t>((0 << 6) | (4 << 3) | 3);  // client mode
  req.tx_timestamp = Hton64(ToNtpTimestamp(ts.NowUnix()));

  int sent = sendto(sock, reinterpret_cast<const char*>(&req), sizeof(req), 0,
                    reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  ASSERT_EQ(sent, static_cast<int>(sizeof(req)));

  // Receive response with timeout
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(sock, &rfds);
  timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  int ready = select(0, &rfds, nullptr, nullptr, &tv);
  ASSERT_GT(ready, 0) << "timeout waiting for NTP response";

  sockaddr_in from{};
  int fromlen = sizeof(from);
  NtpPacket resp{};
  int n = recvfrom(sock, reinterpret_cast<char*>(&resp), sizeof(resp), 0,
                   reinterpret_cast<sockaddr*>(&from), &fromlen);
  ASSERT_EQ(n, static_cast<int>(sizeof(resp)));

  // Basic header checks
  uint8_t mode = resp.li_vn_mode & 0x07;
  uint8_t vn = (resp.li_vn_mode >> 3) & 0x07;
  EXPECT_EQ(mode, 4);
  EXPECT_EQ(vn, 4);
  EXPECT_EQ(resp.stratum, 1);

  // Echoed transmit timestamp should match our request's tx.
  EXPECT_EQ(resp.orig_timestamp, req.tx_timestamp);

  // Reference/recv/tx timestamps are based on our fixed time.
  uint64_t expected = Hton64(ToNtpTimestamp(ts.NowUnix()));
  EXPECT_EQ(resp.recv_timestamp, expected);
  EXPECT_EQ(resp.tx_timestamp, expected);

  closesocket(sock);
  server.Stop();
}

}  // namespace ntpserver
