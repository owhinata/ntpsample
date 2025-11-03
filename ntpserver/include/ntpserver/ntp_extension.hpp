/**
 * @file ntp_extension.hpp
 * @brief NTP Extension Field for vendor hints (ABS/RATE propagation).
 *
 * This header defines a compact binary layout for a vendor-specific
 * NTP Extension Field (EF) used to propagate time control hints from an
 * NTP server to clients: absolute time (SetAbsolute) and rate scale
 * (SetRate). The EF is embedded in normal NTP packets (UDP/123), both
 * in regular replies and in server-originated notifications (mode 4
 * form factor) when configuration changes occur.
 *
 * Layout (TLV inside an NTP EF, all big-endian, 4-byte aligned):
 *
 *   - magic:    4 bytes, ASCII "NTPC" to avoid false positives
 *   - version:  1 byte, currently 1
 *   - flags:    1 byte, bit0=ABS, bit1=RATE (RESET is not needed)
 *   - reserve:  2 bytes, set to 0
 *   - seq:      4 bytes, monotonically increasing sequence number
 *   - srv_ts:   8 bytes, f64 server unix time (seconds)
 *   - abs:      8 bytes, f64 absolute unix time (present if ABS=1)
 *   - rate:     8 bytes, f64 rate scale (present if RATE=1, 1.0=realtime)
 *   - padding:  zero bytes to 4-byte boundary
 *
 * Client behavior:
 *   - Validate magic/version, apply newer seq only.
 *   - If ABS and/or RATE differ from current TimeSource values by a
 *     meaningful epsilon, apply changes under a single mutex and reset
 *     estimator windows (offset/skew) unconditionally.
 *   - ABS may violate monotonicity momentarily (intentional step).
 *   - RATE preserves monotonicity.
 *
 * @note Authentication is out of scope for the minimal implementation.
 *       When message authentication (MAC/HMAC) is used, this EF must be
 *       included in the covered region.
 */

#pragma once

#include <cstdint>
#include <vector>

namespace ntpserver {

/** @brief Vendor hint EF constants. */
struct NtpVendorExt {
  /** ASCII magic "NTPC". */
  static constexpr uint32_t kMagic = 0x4e545043u;  // 'N''T''P''C'
  /** Version of payload format. */
  static constexpr uint8_t kVersion = 1;

  /** flags bit positions */
  enum : uint8_t {
    kFlagAbs = 1u << 0,   ///< ABS present (SetAbsolute)
    kFlagRate = 1u << 1,  ///< RATE present (SetRate)
  };

  /**
   * @brief Packed payload view (host order fields before BE encoding).
   */
  struct Payload {
    uint32_t magic = kMagic;
    uint8_t version = kVersion;
    uint8_t flags = 0;
    uint16_t reserved = 0;
    uint32_t seq = 0;
    double server_unix_s = 0.0;
    // Optional fields (included when corresponding flag is set)
    double abs_unix_s = 0.0;
    double rate_scale = 1.0;
  };

  /**
   * @brief Serialize Payload into a byte vector (big-endian, 4B aligned).
   * @param p Input payload (host order).
   * @return Raw bytes ready to be placed in an NTP EF value field.
   * @test
   * @brief Roundtrip serialization keeps values bit-identical.
   * @steps Serialize a payload with ABS+RATE, then parse bytes back.
   * @expected Parsed fields equal the original; extra padding is ignored.
   */
  static std::vector<uint8_t> Serialize(const Payload& p);

  /**
   * @brief Parse bytes into Payload (accepts aligned EF value bytes).
   * @param bytes Big-endian EF value bytes.
   * @param out Parsed payload on success; untouched on failure.
   * @return true on success, false on validation/size failure.
   * @test
   * @brief Invalid magic/version is rejected.
   * @steps Corrupt magic and version separately and parse.
   * @expected Function returns false; output remains unchanged.
   */
  static bool Parse(const std::vector<uint8_t>& bytes, Payload* out);
};

}  // namespace ntpserver
