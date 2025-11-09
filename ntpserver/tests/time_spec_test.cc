// Copyright (c) 2025 <Your Name>
/**
 * @test TimeSpec basic operations
 * @brief Test TimeSpec construction, normalization, and arithmetic.
 */
#include "ntpserver/time_spec.hpp"

#include <gtest/gtest.h>

using ntpserver::TimeSpec;

TEST(TimeSpecTest, DefaultConstruction) {
  TimeSpec t;
  EXPECT_EQ(t.sec, 0);
  EXPECT_EQ(t.nsec, 0u);
}

TEST(TimeSpecTest, Normalization) {
  TimeSpec t(1, 1500000000u);  // 1.5 billion nsec = 1.5 sec overflow
  t.Normalize();
  EXPECT_EQ(t.sec, 2);
  EXPECT_EQ(t.nsec, 500000000u);
}

TEST(TimeSpecTest, Addition) {
  TimeSpec a(10, 500000000u);  // 10.5 sec
  TimeSpec b(5, 700000000u);   // 5.7 sec
  TimeSpec c = a + b;
  EXPECT_EQ(c.sec, 16);
  EXPECT_EQ(c.nsec, 200000000u);  // 10.5 + 5.7 = 16.2
}

TEST(TimeSpecTest, Subtraction) {
  TimeSpec a(10, 500000000u);  // 10.5 sec
  TimeSpec b(5, 300000000u);   // 5.3 sec
  TimeSpec c = a - b;
  EXPECT_EQ(c.sec, 5);
  EXPECT_EQ(c.nsec, 200000000u);  // 10.5 - 5.3 = 5.2
}

TEST(TimeSpecTest, SubtractionWithBorrow) {
  TimeSpec a(10, 300000000u);  // 10.3 sec
  TimeSpec b(5, 500000000u);   // 5.5 sec
  TimeSpec c = a - b;
  EXPECT_EQ(c.sec, 4);
  EXPECT_EQ(c.nsec, 800000000u);  // 10.3 - 5.5 = 4.8
}

TEST(TimeSpecTest, Multiplication) {
  TimeSpec a(10, 0);  // 10.0 sec
  TimeSpec b = a * 2.5;
  EXPECT_EQ(b.sec, 25);
  EXPECT_EQ(b.nsec, 0u);  // 10.0 * 2.5 = 25.0
}

TEST(TimeSpecTest, Comparison) {
  TimeSpec a(10, 500000000u);
  TimeSpec b(10, 500000000u);
  TimeSpec c(10, 600000000u);
  TimeSpec d(11, 0u);

  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  EXPECT_LT(a, c);
  EXPECT_LT(c, d);
  EXPECT_GT(d, a);
}

TEST(TimeSpecTest, ToDoubleAndFromDouble) {
  TimeSpec a(10, 500000000u);  // 10.5 sec
  double d = a.ToDouble();
  EXPECT_NEAR(d, 10.5, 1e-9);

  TimeSpec b = TimeSpec::FromDouble(10.5);
  EXPECT_EQ(b.sec, 10);
  EXPECT_NEAR(b.nsec, 500000000u, 1u);  // Allow 1ns rounding error
}

TEST(TimeSpecTest, NtpTimestampConversion) {
  // Create a known time: 2025-01-01 00:00:00 UTC
  // UNIX timestamp: approximately 1735689600
  TimeSpec a(1735689600, 0);

  uint64_t ntp = a.ToNtpTimestamp();
  TimeSpec b = TimeSpec::FromNtpTimestamp(ntp);

  EXPECT_EQ(a.sec, b.sec);
  EXPECT_NEAR(a.nsec, b.nsec, 1u);  // Allow 1ns rounding error
}

TEST(TimeSpecTest, AbsDiff) {
  TimeSpec a(10, 500000000u);
  TimeSpec b(5, 300000000u);

  TimeSpec diff1 = ntpserver::AbsDiff(a, b);
  TimeSpec diff2 = ntpserver::AbsDiff(b, a);

  EXPECT_EQ(diff1.sec, 5);
  EXPECT_EQ(diff1.nsec, 200000000u);
  EXPECT_EQ(diff1, diff2);
}
