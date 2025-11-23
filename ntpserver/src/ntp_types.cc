// Copyright (c) 2025 The NTP Sample Authors
/**
 * @file ntp_types.cc
 * @brief Implementation of NTP vendor extension (serialize/parse).
 */
#include "ntpserver/ntp_types.hpp"

#include <array>
#include <cstring>
#include <vector>

namespace ntpserver {

std::vector<uint8_t> NtpVendorExt::Serialize(const Payload& p) {
  std::vector<uint8_t> out;
  const bool has_abs = (p.flags & kFlagAbs) != 0U;
  const bool has_rate = (p.flags & kFlagRate) != 0U;

  // Version 2 format sizes:
  // Header: 4+1+1+2+4 = 12
  // server_time: 8+4 = 12
  // abs_time (optional): 8+4 = 12
  // rate_scale (optional): 8
  out.reserve(12 + 12 + (has_abs ? 12 : 0) + (has_rate ? 8 : 0) + 3);

  auto append_be32 = [&](uint32_t v) {
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xffU));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xffU));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xffU));
    out.push_back(static_cast<uint8_t>(v & 0xffU));
  };
  auto append_u8 = [&](uint8_t v) { out.push_back(v); };
  auto append_zero_u8 = [&]() { out.push_back(0U); };
  auto append_be64i = [&](int64_t v) {
    uint64_t u = static_cast<uint64_t>(v);
    out.push_back(static_cast<uint8_t>((u >> 56) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 48) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 40) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 32) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xffU));
    out.push_back(static_cast<uint8_t>(u & 0xffU));
  };
  auto append_be64f = [&](double d) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    uint64_t u = 0U;
    std::memcpy(&u, &d, sizeof(double));
    out.push_back(static_cast<uint8_t>((u >> 56) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 48) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 40) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 32) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 24) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 16) & 0xffU));
    out.push_back(static_cast<uint8_t>((u >> 8) & 0xffU));
    out.push_back(static_cast<uint8_t>(u & 0xffU));
  };
  auto append_timespec = [&](const TimeSpec& ts) {
    append_be64i(ts.sec);
    append_be32(ts.nsec);
  };

  // Header
  append_be32(p.magic);
  append_u8(p.version);
  append_u8(p.flags);
  append_zero_u8();
  append_zero_u8();
  append_be32(p.seq);

  // Server time (always present in v2)
  append_timespec(p.server_time);

  // Optional fields
  if (has_abs) append_timespec(p.abs_time);
  if (has_rate) append_be64f(p.rate_scale);

  // 4-byte alignment padding
  while ((out.size() % 4U) != 0U) out.push_back(0U);
  return out;
}

bool NtpVendorExt::Parse(const std::vector<uint8_t>& bytes, Payload* out) {
  if (out == nullptr) return false;
  // Minimum size: header (12) + server_time (12) = 24 bytes
  if (bytes.size() < 24U) return false;

  auto rd32 = [&](size_t at) {
    return (static_cast<uint32_t>(bytes[at]) << 24) |
           (static_cast<uint32_t>(bytes[at + 1]) << 16) |
           (static_cast<uint32_t>(bytes[at + 2]) << 8) |
           (static_cast<uint32_t>(bytes[at + 3]));
  };
  auto rd64i = [&](size_t at) {
    uint64_t u = (static_cast<uint64_t>(bytes[at]) << 56) |
                 (static_cast<uint64_t>(bytes[at + 1]) << 48) |
                 (static_cast<uint64_t>(bytes[at + 2]) << 40) |
                 (static_cast<uint64_t>(bytes[at + 3]) << 32) |
                 (static_cast<uint64_t>(bytes[at + 4]) << 24) |
                 (static_cast<uint64_t>(bytes[at + 5]) << 16) |
                 (static_cast<uint64_t>(bytes[at + 6]) << 8) |
                 (static_cast<uint64_t>(bytes[at + 7]));
    return static_cast<int64_t>(u);
  };
  auto rd64f = [&](size_t at, double* d) {
    uint64_t u = (static_cast<uint64_t>(bytes[at]) << 56) |
                 (static_cast<uint64_t>(bytes[at + 1]) << 48) |
                 (static_cast<uint64_t>(bytes[at + 2]) << 40) |
                 (static_cast<uint64_t>(bytes[at + 3]) << 32) |
                 (static_cast<uint64_t>(bytes[at + 4]) << 24) |
                 (static_cast<uint64_t>(bytes[at + 5]) << 16) |
                 (static_cast<uint64_t>(bytes[at + 6]) << 8) |
                 (static_cast<uint64_t>(bytes[at + 7]));
    std::memcpy(d, &u, sizeof(double));
  };
  auto rd_timespec = [&](size_t at) {
    int64_t sec = rd64i(at);
    uint32_t nsec = rd32(at + 8);
    return TimeSpec(sec, nsec);
  };

  size_t off = 0U;
  Payload p{};

  // Header
  p.magic = rd32(off);
  off += 4U;
  p.version = bytes[off++];
  p.flags = bytes[off++];
  off += 2U;  // reserved
  p.seq = rd32(off);
  off += 4U;

  // Validate magic/version early
  if (p.magic != kMagic || p.version != kVersion) return false;

  // Server time (always present in v2)
  p.server_time = rd_timespec(off);
  off += 12U;

  const bool has_abs = (p.flags & kFlagAbs) != 0U;
  const bool has_rate = (p.flags & kFlagRate) != 0U;

  // Calculate required size
  const size_t need = off + (has_abs ? 12U : 0U) + (has_rate ? 8U : 0U);
  if (bytes.size() < need) return false;

  // Optional fields
  if (has_abs) {
    p.abs_time = rd_timespec(off);
    off += 12U;
  }
  if (has_rate) {
    rd64f(off, &p.rate_scale);
    off += 8U;
  }

  *out = p;
  return true;
}

}  // namespace ntpserver
