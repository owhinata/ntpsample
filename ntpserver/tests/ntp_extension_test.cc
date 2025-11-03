/**
 * @test NTP vendor extension serialize/parse
 * @brief Roundtrip and validation for ntp_extension (ABS/RATE).
 * @steps
 *  1) Build ABS+RATE payload and serialize to bytes.
 *  2) Parse back and compare all fields.
 *  3) Corrupt magic/version and expect parser failure.
 * @expected
 *  - Roundtrip fields match (bitwise within double representation).
 *  - Invalid magic/version yields Parse()==false.
 */

#include <gtest/gtest.h>

#include "ntpserver/ntp_extension.hpp"

namespace {

using ntpserver::NtpVendorExt;

TEST(NtpExtensionTest, RoundtripAbsRate) {
  NtpVendorExt::Payload p{};
  p.seq = 42;
  p.flags = NtpVendorExt::kFlagAbs | NtpVendorExt::kFlagRate;
  p.server_unix_s = 1762139748.210;
  p.abs_unix_s = 1762139748.999123;
  p.rate_scale = 1.000001;  // +1 ppm

  auto bytes = NtpVendorExt::Serialize(p);

  NtpVendorExt::Payload q{};
  ASSERT_TRUE(NtpVendorExt::Parse(bytes, &q));

  EXPECT_EQ(q.magic, NtpVendorExt::kMagic);
  EXPECT_EQ(q.version, NtpVendorExt::kVersion);
  EXPECT_EQ(q.flags, p.flags);
  EXPECT_EQ(q.seq, p.seq);
  EXPECT_DOUBLE_EQ(q.server_unix_s, p.server_unix_s);
  EXPECT_DOUBLE_EQ(q.abs_unix_s, p.abs_unix_s);
  EXPECT_DOUBLE_EQ(q.rate_scale, p.rate_scale);
}

TEST(NtpExtensionTest, RejectsBadMagicAndVersion) {
  NtpVendorExt::Payload p{};
  p.flags = NtpVendorExt::kFlagAbs;
  p.server_unix_s = 1.0;
  p.abs_unix_s = 2.0;
  auto bytes = NtpVendorExt::Serialize(p);

  // Corrupt magic
  bytes[0] ^= 0xFF;
  NtpVendorExt::Payload out{};
  EXPECT_FALSE(NtpVendorExt::Parse(bytes, &out));

  // Restore then corrupt version
  bytes = NtpVendorExt::Serialize(p);
  bytes[4] ^= 0xFF;  // version
  EXPECT_FALSE(NtpVendorExt::Parse(bytes, &out));
}

}  // namespace

