// Copyright (c) 2025 <Your Name>
using System.Buffers.Binary;
using System.Runtime.InteropServices;

namespace NtpServer.Internal;

/// <summary>
/// NTP protocol types and structures.
/// </summary>
public static class NtpTypes
{
    /// <summary>NTP epoch offset from UNIX epoch (seconds, 1900-01-01 vs 1970-01-01).</summary>
    public const uint NtpUnixEpochDiff = 2208988800u;
}

/// <summary>
/// NTPv4 basic packet structure (48 bytes).
/// All fields are transmitted in network (big-endian) byte order.
/// Extension fields may follow after the basic 48-byte packet.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct NtpPacket
{
    /// <summary>Leap Indicator, Version, Mode</summary>
    public byte LiVnMode;

    /// <summary>Stratum level</summary>
    public byte Stratum;

    /// <summary>Poll interval (log2 seconds)</summary>
    public byte Poll;

    /// <summary>Precision (log2 seconds)</summary>
    public sbyte Precision;

    /// <summary>Root delay (NTP short format)</summary>
    public uint RootDelay;

    /// <summary>Root dispersion (NTP short format)</summary>
    public uint RootDispersion;

    /// <summary>Reference ID</summary>
    public uint RefId;

    /// <summary>Reference timestamp (NTP timestamp format)</summary>
    public ulong RefTimestamp;

    /// <summary>Origin timestamp (NTP timestamp format)</summary>
    public ulong OrigTimestamp;

    /// <summary>Receive timestamp (NTP timestamp format)</summary>
    public ulong RecvTimestamp;

    /// <summary>Transmit timestamp (NTP timestamp format)</summary>
    public ulong TxTimestamp;
}

/// <summary>
/// Vendor hint extension field constants and serialization.
/// </summary>
public static class NtpVendorExt
{
    /// <summary>ASCII magic "NTPC".</summary>
    public const uint Magic = 0x4e545043u; // 'N''T''P''C'

    /// <summary>Version of payload format.</summary>
    public const byte Version = 2;

    /// <summary>NTP Extension Field type code (private/experimental).</summary>
    public const ushort EfTypeVendorHint = 0xFF01;

    /// <summary>flags bit positions</summary>
    public const byte FlagAbs = 1 << 0; // ABS present (SetAbsolute)
    public const byte FlagRate = 1 << 1; // RATE present (SetRate)
    public const byte FlagPush = 1 << 2; // Push notification (server-initiated)

    /// <summary>
    /// Packed payload view (host order fields before BE encoding).
    /// </summary>
    public class Payload
    {
        public uint Magic { get; set; } = NtpVendorExt.Magic;
        public byte Version { get; set; } = NtpVendorExt.Version;
        public byte Flags { get; set; } = 0;
        public ushort Reserved { get; set; } = 0;
        public uint Seq { get; set; } = 0;
        public TimeSpec ServerTime { get; set; }
        public TimeSpec AbsTime { get; set; }
        public double RateScale { get; set; } = 1.0;
    }

    /// <summary>
    /// Serialize Payload into a byte array (big-endian, 4-byte aligned).
    /// </summary>
    public static byte[] Serialize(Payload p)
    {
        bool hasAbs = (p.Flags & FlagAbs) != 0;
        bool hasRate = (p.Flags & FlagRate) != 0;

        var stream = new MemoryStream();
        var writer = new BinaryWriter(stream);

        // Header (12 bytes)
        ByteOrderHelper.WriteBE32(writer, p.Magic);
        writer.Write(p.Version);
        writer.Write(p.Flags);
        writer.Write((byte)0); // reserved
        writer.Write((byte)0); // reserved
        ByteOrderHelper.WriteBE32(writer, p.Seq);

        // Server time (12 bytes)
        ByteOrderHelper.WriteTimeSpec(writer, p.ServerTime);

        // Optional fields
        if (hasAbs)
        {
            ByteOrderHelper.WriteTimeSpec(writer, p.AbsTime);
        }
        if (hasRate)
        {
            ByteOrderHelper.WriteBE64Double(writer, p.RateScale);
        }

        // 4-byte alignment padding
        while (stream.Length % 4 != 0)
        {
            writer.Write((byte)0);
        }

        return stream.ToArray();
    }

    /// <summary>
    /// Parse bytes into Payload (accepts aligned EF value bytes).
    /// </summary>
    /// <param name="bytes">Big-endian EF value bytes.</param>
    /// <param name="outPayload">Parsed payload on success; null on failure.</param>
    /// <returns>true on success, false on validation/size failure.</returns>
    public static bool Parse(byte[] bytes, out Payload? outPayload)
    {
        outPayload = null;

        // Minimum size: header (12) + server_time (12) = 24 bytes
        if (bytes.Length < 24)
            return false;

        var stream = new MemoryStream(bytes);
        var reader = new BinaryReader(stream);

        var p = new Payload();

        // Header
        p.Magic = ByteOrderHelper.ReadBE32(reader);
        p.Version = reader.ReadByte();
        p.Flags = reader.ReadByte();
        reader.ReadUInt16(); // reserved
        p.Seq = ByteOrderHelper.ReadBE32(reader);

        // Validate magic/version early
        if (p.Magic != Magic || p.Version != Version)
            return false;

        // Server time (always present in v2)
        p.ServerTime = ByteOrderHelper.ReadTimeSpec(reader);

        bool hasAbs = (p.Flags & FlagAbs) != 0;
        bool hasRate = (p.Flags & FlagRate) != 0;

        // Calculate required size
        int need = 24 + (hasAbs ? 12 : 0) + (hasRate ? 8 : 0);
        if (bytes.Length < need)
            return false;

        // Optional fields
        if (hasAbs)
        {
            p.AbsTime = ByteOrderHelper.ReadTimeSpec(reader);
        }
        if (hasRate)
        {
            p.RateScale = ByteOrderHelper.ReadBE64Double(reader);
        }

        outPayload = p;
        return true;
    }
}

/// <summary>
/// RFC 1982 compliant epoch number comparison.
/// </summary>
public static class EpochCompare
{
    /// <summary>Determine if e1 is older than e2 using serial number arithmetic.</summary>
    public static bool IsEpochOlder(uint e1, uint e2)
    {
        int diff = (int)(e1 - e2);
        return diff < 0;
    }

    /// <summary>Determine if e1 is newer than e2 using serial number arithmetic.</summary>
    public static bool IsEpochNewer(uint e1, uint e2)
    {
        int diff = (int)(e1 - e2);
        return diff > 0;
    }
}

/// <summary>
/// Concatenate NTP packet and extension field bytes.
/// </summary>
public static class NtpPacketHelper
{
    /// <summary>
    /// Combines the basic 48-byte NTP packet with optional extension field(s)
    /// into a single contiguous buffer suitable for sendto().
    /// Converts from host byte order to network byte order (big-endian).
    /// Adds NTP Extension Field header (type + length) before the payload.
    /// </summary>
    /// <param name="packet">NTP packet (host byte order for multi-byte fields)</param>
    /// <param name="extensionFieldValue">Extension field payload (without EF header)</param>
    /// <returns>Complete NTP packet with EF header and payload in network byte order</returns>
    public static byte[] ComposeNtpPacketWithEf(NtpPacket packet, byte[] extensionFieldValue)
    {
        // Extension Field format: Type (2 bytes) + Length (2 bytes) + Value (variable)
        int efHeaderSize = 4;
        int efTotalSize =
            extensionFieldValue.Length > 0 ? efHeaderSize + extensionFieldValue.Length : 0;
        byte[] buf = new byte[48 + efTotalSize];
        var span = buf.AsSpan();

        // Manually serialize to avoid Marshal overhead
        // Convert from host byte order to network byte order (big-endian)
        span[0] = packet.LiVnMode;
        span[1] = packet.Stratum;
        span[2] = packet.Poll;
        span[3] = (byte)packet.Precision;
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(4, 4), packet.RootDelay);
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(8, 4), packet.RootDispersion);
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(12, 4), packet.RefId);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(16, 8), packet.RefTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(24, 8), packet.OrigTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(32, 8), packet.RecvTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(40, 8), packet.TxTimestamp);

        // Add extension field with header
        if (extensionFieldValue.Length > 0)
        {
            int offset = 48;
            // Extension Field header: Type (2 bytes) + Length (2 bytes)
            ushort efType = NtpVendorExt.EfTypeVendorHint;
            ushort efLength = (ushort)(efHeaderSize + extensionFieldValue.Length);
            BinaryPrimitives.WriteUInt16BigEndian(span.Slice(offset, 2), efType);
            BinaryPrimitives.WriteUInt16BigEndian(span.Slice(offset + 2, 2), efLength);
            // Extension Field value (payload)
            Array.Copy(
                extensionFieldValue,
                0,
                buf,
                offset + efHeaderSize,
                extensionFieldValue.Length
            );
        }

        return buf;
    }
}
