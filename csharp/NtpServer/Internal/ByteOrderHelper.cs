// Copyright (c) 2025 <Your Name>
using System.Net;

namespace NtpServer.Internal;

/// <summary>
/// Helper utilities for network byte order (big-endian) conversion.
/// </summary>
public static class ByteOrderHelper
{
    /// <summary>Convert 64-bit value to big-endian (network) order.</summary>
    public static ulong Hton64(ulong hostValue)
    {
        if (BitConverter.IsLittleEndian)
        {
            uint hi = (uint)(hostValue >> 32);
            uint lo = (uint)(hostValue & 0xFFFFFFFFu);
            hi = (uint)IPAddress.HostToNetworkOrder((int)hi);
            lo = (uint)IPAddress.HostToNetworkOrder((int)lo);
            return ((ulong)lo << 32) | hi;
        }
        return hostValue;
    }

    /// <summary>Convert 64-bit value from big-endian (network) to host order.</summary>
    public static ulong Ntoh64(ulong networkValue)
    {
        // Hton64 and Ntoh64 are symmetric for byte swapping
        return Hton64(networkValue);
    }

    /// <summary>Write a 32-bit value in big-endian format.</summary>
    public static void WriteBE32(BinaryWriter writer, uint value)
    {
        writer.Write((byte)((value >> 24) & 0xFF));
        writer.Write((byte)((value >> 16) & 0xFF));
        writer.Write((byte)((value >> 8) & 0xFF));
        writer.Write((byte)(value & 0xFF));
    }

    /// <summary>Read a 32-bit value in big-endian format.</summary>
    public static uint ReadBE32(BinaryReader reader)
    {
        byte b0 = reader.ReadByte();
        byte b1 = reader.ReadByte();
        byte b2 = reader.ReadByte();
        byte b3 = reader.ReadByte();
        return ((uint)b0 << 24) | ((uint)b1 << 16) | ((uint)b2 << 8) | b3;
    }

    /// <summary>Write a 64-bit signed integer in big-endian format.</summary>
    public static void WriteBE64Int(BinaryWriter writer, long value)
    {
        ulong u = (ulong)value;
        writer.Write((byte)((u >> 56) & 0xFF));
        writer.Write((byte)((u >> 48) & 0xFF));
        writer.Write((byte)((u >> 40) & 0xFF));
        writer.Write((byte)((u >> 32) & 0xFF));
        writer.Write((byte)((u >> 24) & 0xFF));
        writer.Write((byte)((u >> 16) & 0xFF));
        writer.Write((byte)((u >> 8) & 0xFF));
        writer.Write((byte)(u & 0xFF));
    }

    /// <summary>Read a 64-bit signed integer in big-endian format.</summary>
    public static long ReadBE64Int(BinaryReader reader)
    {
        ulong u = 0;
        for (int i = 0; i < 8; i++)
        {
            u = (u << 8) | reader.ReadByte();
        }
        return (long)u;
    }

    /// <summary>Write a 64-bit double in big-endian format.</summary>
    public static void WriteBE64Double(BinaryWriter writer, double value)
    {
        byte[] bytes = BitConverter.GetBytes(value);
        if (BitConverter.IsLittleEndian)
        {
            Array.Reverse(bytes);
        }
        writer.Write(bytes);
    }

    /// <summary>Read a 64-bit double in big-endian format.</summary>
    public static double ReadBE64Double(BinaryReader reader)
    {
        byte[] bytes = reader.ReadBytes(8);
        if (BitConverter.IsLittleEndian)
        {
            Array.Reverse(bytes);
        }
        return BitConverter.ToDouble(bytes, 0);
    }

    /// <summary>Write a TimeSpec in big-endian format (12 bytes: 8 for sec, 4 for nsec).</summary>
    public static void WriteTimeSpec(BinaryWriter writer, TimeSpec ts)
    {
        WriteBE64Int(writer, ts.Sec);
        WriteBE32(writer, ts.Nsec);
    }

    /// <summary>Read a TimeSpec in big-endian format (12 bytes: 8 for sec, 4 for nsec).</summary>
    public static TimeSpec ReadTimeSpec(BinaryReader reader)
    {
        long sec = ReadBE64Int(reader);
        uint nsec = ReadBE32(reader);
        return new TimeSpec(sec, nsec);
    }
}
