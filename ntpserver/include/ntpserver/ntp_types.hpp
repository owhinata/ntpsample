// Copyright (c) 2025 <Your Name>
/**
 * @file ntp_types.hpp
 * @brief NTP protocol types and structures.
 *
 * This header defines core NTP protocol types used across the codebase:
 * - Basic NTP packet structure (NTPv4)
 * - NTP Extension Field for vendor hints (ABS/RATE propagation)
 * - Common constants and conversions
 *
 * NTP Extension Field (Vendor Hint) Layout:
 *
 * This compact binary layout is used for a vendor-specific NTP Extension
 * Field (EF) to propagate time control hints from an NTP server to clients:
 * absolute time (SetAbsolute) and rate scale (SetRate). The EF is embedded
 * in normal NTP packets (UDP/123), both in regular replies and in
 * server-originated notifications (mode 4 form factor) when configuration
 * changes occur.
 *
 * Layout (TLV inside an NTP EF, all big-endian, 4-byte aligned):
 *
 *   Version 2 format:
 *   - magic:      4 bytes, ASCII "NTPC" to avoid false positives
 *   - version:    1 byte, version 2
 *   - flags:      1 byte, bit0=ABS, bit1=RATE (RESET is not needed)
 *   - reserve:    2 bytes, set to 0
 *   - seq:        4 bytes, monotonically increasing sequence number
 *   - srv_sec:    8 bytes, int64_t server time seconds (UNIX epoch)
 *   - srv_nsec:   4 bytes, uint32_t server time nanoseconds
 *   - abs_sec:    8 bytes, int64_t absolute time seconds (present if ABS=1)
 *   - abs_nsec:   4 bytes, uint32_t absolute time nanoseconds (present if
 * ABS=1)
 *   - rate:       8 bytes, f64 rate scale (present if RATE=1, 1.0=realtime)
 *   - padding:    zero bytes to 4-byte boundary
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

#include "ntpserver/time_spec.hpp"

namespace ntpserver {

/** @brief NTP epoch offset from UNIX epoch (seconds, 1900-01-01 vs 1970-01-01).
 */
constexpr uint32_t kNtpUnixEpochDiff = 2208988800UL;

/**
 * @brief NTPv4 basic packet structure (48 bytes).
 *
 * All fields are transmitted in network (big-endian) byte order.
 * Extension fields may follow after the basic 48-byte packet.
 */
#pragma pack(push, 1)
struct NtpPacket {
  uint8_t li_vn_mode;        ///< Leap Indicator, Version, Mode
  uint8_t stratum;           ///< Stratum level
  uint8_t poll;              ///< Poll interval (log2 seconds)
  int8_t precision;          ///< Precision (log2 seconds)
  uint32_t root_delay;       ///< Root delay (NTP short format)
  uint32_t root_dispersion;  ///< Root dispersion (NTP short format)
  uint32_t ref_id;           ///< Reference ID
  uint64_t ref_timestamp;    ///< Reference timestamp (NTP timestamp format)
  uint64_t orig_timestamp;   ///< Origin timestamp (NTP timestamp format)
  uint64_t recv_timestamp;   ///< Receive timestamp (NTP timestamp format)
  uint64_t tx_timestamp;     ///< Transmit timestamp (NTP timestamp format)
};
#pragma pack(pop)

/** @brief Vendor hint EF constants. */
struct NtpVendorExt {
  /** ASCII magic "NTPC". */
  static constexpr uint32_t kMagic = 0x4e545043u;  // 'N''T''P''C'
  /** Version of payload format. */
  static constexpr uint8_t kVersion = 2;
  /** NTP Extension Field type code (private/experimental). */
  static constexpr uint16_t kEfTypeVendorHint = 0xFF01;

  /** flags bit positions */
  enum : uint8_t {
    kFlagAbs = 1u << 0,   ///< ABS present (SetAbsolute)
    kFlagRate = 1u << 1,  ///< RATE present (SetRate)
    kFlagPush = 1u << 2,  ///< Push notification (server-initiated)
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
    TimeSpec server_time;
    // Optional fields (included when corresponding flag is set)
    TimeSpec abs_time;
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
