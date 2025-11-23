// Copyright (c) 2025 The NTP Sample Authors
namespace NtpServer.Internal;

/// <summary>
/// Immutable configuration options for NtpServer.
/// </summary>
public class Options
{
    /// <summary>Callback for logging messages (thread-safe).</summary>
    public delegate void LogCallback(string message);

    public const byte DefaultStratum = 1;
    public const sbyte DefaultPrecision = -20;
    public const uint DefaultRefId = 0x4C4F434C; // "LOCL"
    public static readonly TimeSpan DefaultClientRetention = TimeSpan.FromMinutes(60);

    /// <summary>Stratum level</summary>
    public byte Stratum { get; }

    /// <summary>Precision (log2 seconds)</summary>
    public sbyte Precision { get; }

    /// <summary>Reference ID</summary>
    public uint RefId { get; }

    /// <summary>Client retention duration</summary>
    public TimeSpan ClientRetention { get; }

    /// <summary>Log sink callback</summary>
    public LogCallback? LogSink { get; }

    private Options(
        byte stratum,
        sbyte precision,
        uint refId,
        TimeSpan retention,
        LogCallback? logSink
    )
    {
        Stratum = stratum;
        Precision = precision;
        RefId = refId;
        ClientRetention = retention;
        LogSink = logSink;
    }

    /// <summary>Fluent builder for Options.</summary>
    public class Builder
    {
        private byte _stratum = DefaultStratum;
        private sbyte _precision = DefaultPrecision;
        private uint _refId = DefaultRefId;
        private TimeSpan _clientRetention = DefaultClientRetention;
        private LogCallback? _logSink;

        public Builder Stratum(byte value)
        {
            _stratum = value;
            return this;
        }

        public Builder Precision(sbyte value)
        {
            _precision = value;
            return this;
        }

        public Builder RefId(uint value)
        {
            _refId = value;
            return this;
        }

        public Builder ClientRetention(TimeSpan value)
        {
            _clientRetention = value;
            return this;
        }

        public Builder LogSink(LogCallback callback)
        {
            _logSink = callback;
            return this;
        }

        public Options Build()
        {
            return new Options(_stratum, _precision, _refId, _clientRetention, _logSink);
        }
    }
}

/// <summary>
/// Helper to create reference ID from 4-character ASCII tag.
/// </summary>
public static class RefIdHelper
{
    public static uint MakeRefId(string tag)
    {
        if (tag.Length != 4)
            throw new ArgumentException("Tag must be exactly 4 characters", nameof(tag));

        return (uint)((byte)tag[0] << 24 | (byte)tag[1] << 16 | (byte)tag[2] << 8 | (byte)tag[3]);
    }
}
