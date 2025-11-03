// ntp_extension.cc
// Implementation of NTP vendor extension (serialize/parse).

#include "ntpserver/ntp_extension.hpp"

#include <array>
#include <cstring>

namespace ntpserver {

std::vector<uint8_t> NtpVendorExt::Serialize(const Payload& p) {
  std::vector<uint8_t> out;
  const bool has_abs = (p.flags & kFlagAbs) != 0U;
  const bool has_rate = (p.flags & kFlagRate) != 0U;

  // Fixed prefix: magic(4)+ver(1)+flags(1)+res(2)+seq(4)+srv(8)
  out.resize(4 + 1 + 1 + 2 + 4 + 8);
  size_t off = 0U;

  auto be32 = [](uint32_t v) {
    return std::array<uint8_t, 4>{
        static_cast<uint8_t>((v >> 24) & 0xffU),
        static_cast<uint8_t>((v >> 16) & 0xffU),
        static_cast<uint8_t>((v >> 8) & 0xffU),
        static_cast<uint8_t>(v & 0xffU)};
  };

  auto push_be64f = [&](double d) {
    static_assert(sizeof(double) == 8, "double must be 8 bytes");
    uint64_t u = 0U;
    std::memcpy(&u, &d, sizeof(double));
    std::array<uint8_t, 8> b{{
        static_cast<uint8_t>((u >> 56) & 0xffU),
        static_cast<uint8_t>((u >> 48) & 0xffU),
        static_cast<uint8_t>((u >> 40) & 0xffU),
        static_cast<uint8_t>((u >> 32) & 0xffU),
        static_cast<uint8_t>((u >> 24) & 0xffU),
        static_cast<uint8_t>((u >> 16) & 0xffU),
        static_cast<uint8_t>((u >> 8) & 0xffU),
        static_cast<uint8_t>(u & 0xffU),
    }};
    out.insert(out.end(), b.begin(), b.end());
  };

  // magic
  auto m = be32(p.magic);
  std::memcpy(out.data() + off, m.data(), 4);
  off += 4U;
  // version
  out[off++] = p.version;
  // flags
  out[off++] = p.flags;
  // reserved
  out[off++] = 0U;
  out[off++] = 0U;
  // seq
  auto s = be32(p.seq);
  std::memcpy(out.data() + off, s.data(), 4);
  off += 4U;
  // server_unix_s
  push_be64f(p.server_unix_s);

  if (has_abs) push_be64f(p.abs_unix_s);
  if (has_rate) push_be64f(p.rate_scale);

  while ((out.size() % 4U) != 0U) out.push_back(0U);
  return out;
}

bool NtpVendorExt::Parse(const std::vector<uint8_t>& bytes, Payload* out) {
  if (out == nullptr) return false;
  if (bytes.size() < (4U + 1U + 1U + 2U + 4U + 8U)) return false;

  auto rd32 = [&](size_t at) {
    return (static_cast<uint32_t>(bytes[at]) << 24) |
           (static_cast<uint32_t>(bytes[at + 1]) << 16) |
           (static_cast<uint32_t>(bytes[at + 2]) << 8) |
           (static_cast<uint32_t>(bytes[at + 3]));
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

  size_t off = 0U;
  Payload p{};
  p.magic = rd32(off);
  off += 4U;
  p.version = bytes[off++];
  p.flags = bytes[off++];
  off += 2U;  // reserved
  p.seq = rd32(off);
  off += 4U;
  rd64f(off, &p.server_unix_s);
  off += 8U;

  if (p.magic != kMagic || p.version != kVersion) return false;

  const bool has_abs = (p.flags & kFlagAbs) != 0U;
  const bool has_rate = (p.flags & kFlagRate) != 0U;

  const size_t need = off + (has_abs ? 8U : 0U) + (has_rate ? 8U : 0U);
  if (bytes.size() < need) return false;

  if (has_abs) {
    rd64f(off, &p.abs_unix_s);
    off += 8U;
  }
  if (has_rate) {
    rd64f(off, &p.rate_scale);
    off += 8U;
  }

  *out = p;
  return true;
}

}  // namespace ntpserver

