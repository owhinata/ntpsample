// Copyright (c) 2025 The NTP Sample Authors
namespace NtpServer.Internal;

/// <summary>
/// Time specification with nanosecond precision.
/// Represents absolute time as UNIX epoch seconds plus nanosecond fraction.
/// This format is platform-independent and suitable for high-precision time synchronization.
/// </summary>
public struct TimeSpec : IEquatable<TimeSpec>, IComparable<TimeSpec>
{
    /// <summary>Seconds since UNIX epoch (1970-01-01 00:00:00 UTC)</summary>
    public long Sec;

    /// <summary>Nanoseconds (0-999999999)</summary>
    public uint Nsec;

    private const uint NsecPerSec = 1000000000u;
    private const double NsecPerSecD = 1e9;

    /// <summary>Default constructor: zero time</summary>
    public TimeSpec()
    {
        Sec = 0;
        Nsec = 0;
    }

    /// <summary>Constructor from seconds and nanoseconds</summary>
    public TimeSpec(long sec, uint nsec)
    {
        Sec = sec;
        Nsec = nsec;
    }

    /// <summary>
    /// Normalize nanosecond field to [0, 999999999] range.
    /// Adjusts Sec/Nsec so that Nsec is always in valid range.
    /// </summary>
    public void Normalize()
    {
        if (Nsec >= NsecPerSec)
        {
            Sec += (long)(Nsec / NsecPerSec);
            Nsec %= NsecPerSec;
        }
    }

    /// <summary>
    /// Convert to double (UNIX seconds).
    /// Provided for compatibility with legacy code.
    /// May lose precision for times far from epoch.
    /// </summary>
    public double ToDouble()
    {
        return (double)Sec + (double)Nsec * 1e-9;
    }

    /// <summary>
    /// Create TimeSpec from double (UNIX seconds).
    /// Provided for compatibility with legacy code.
    /// </summary>
    public static TimeSpec FromDouble(double unixSec)
    {
        double secD = Math.Floor(unixSec);
        double frac = unixSec - secD; // in [0, 1)

        long sec = (long)secD;
        uint nsec = (uint)Math.Round(frac * NsecPerSecD);

        // Handle possible 1.000000000 rounding
        if (nsec >= NsecPerSec)
        {
            nsec -= NsecPerSec;
            ++sec;
        }

        return new TimeSpec(sec, nsec);
    }

    /// <summary>
    /// Convert to NTP timestamp (64-bit, seconds.fraction).
    /// NTP epoch is 1900-01-01, fraction is in units of 2^-32 seconds.
    /// </summary>
    /// <returns>64-bit NTP timestamp in host byte order.</returns>
    public ulong ToNtpTimestamp()
    {
        // Convert UNIX epoch to NTP epoch
        long ntpSec = Sec + NtpTypes.NtpUnixEpochDiff;

        // Convert nanoseconds to NTP fraction (units of 2^-32 seconds)
        // fraction = nsec * 2^32 / 10^9
        ulong frac = ((ulong)Nsec << 32) / NsecPerSec;

        ulong ntpTs = ((ulong)ntpSec << 32) | (frac & 0xFFFFFFFFu);
        return ntpTs;
    }

    /// <summary>
    /// Create TimeSpec from NTP timestamp.
    /// </summary>
    /// <param name="ntpTs">64-bit NTP timestamp in host byte order.</param>
    /// <returns>TimeSpec representing the same instant.</returns>
    public static TimeSpec FromNtpTimestamp(ulong ntpTs)
    {
        // Extract seconds and fraction
        uint ntpSec = (uint)(ntpTs >> 32);
        uint ntpFrac = (uint)(ntpTs & 0xFFFFFFFFu);

        // Convert NTP epoch to UNIX epoch
        long unixSec = (long)ntpSec - NtpTypes.NtpUnixEpochDiff;

        // Convert NTP fraction to nanoseconds
        // nsec = frac * 10^9 / 2^32
        uint nsec = (uint)(((ulong)ntpFrac * NsecPerSec) >> 32);

        return new TimeSpec(unixSec, nsec);
    }

    // Equality
    public bool Equals(TimeSpec other)
    {
        return Sec == other.Sec && Nsec == other.Nsec;
    }

    public override bool Equals(object? obj)
    {
        return obj is TimeSpec other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Sec, Nsec);
    }

    // Comparison
    public int CompareTo(TimeSpec other)
    {
        if (Sec != other.Sec)
            return Sec.CompareTo(other.Sec);
        return Nsec.CompareTo(other.Nsec);
    }

    // Operators
    public static bool operator ==(TimeSpec a, TimeSpec b) => a.Equals(b);

    public static bool operator !=(TimeSpec a, TimeSpec b) => !a.Equals(b);

    public static bool operator <(TimeSpec a, TimeSpec b) => a.CompareTo(b) < 0;

    public static bool operator <=(TimeSpec a, TimeSpec b) => a.CompareTo(b) <= 0;

    public static bool operator >(TimeSpec a, TimeSpec b) => a.CompareTo(b) > 0;

    public static bool operator >=(TimeSpec a, TimeSpec b) => a.CompareTo(b) >= 0;

    /// <summary>Add two TimeSpec values.</summary>
    public static TimeSpec operator +(TimeSpec a, TimeSpec b)
    {
        var result = new TimeSpec { Sec = a.Sec + b.Sec, Nsec = a.Nsec + b.Nsec };
        result.Normalize();
        return result;
    }

    /// <summary>Subtract two TimeSpec values.</summary>
    public static TimeSpec operator -(TimeSpec a, TimeSpec b)
    {
        var result = new TimeSpec { Sec = a.Sec - b.Sec };

        if (a.Nsec >= b.Nsec)
        {
            result.Nsec = a.Nsec - b.Nsec;
        }
        else
        {
            // Borrow from seconds
            result.Sec -= 1;
            result.Nsec = NsecPerSec + a.Nsec - b.Nsec;
        }

        return result;
    }

    /// <summary>Multiply TimeSpec by a scalar rate.</summary>
    public static TimeSpec operator *(TimeSpec t, double rate)
    {
        double totalSec = t.ToDouble() * rate;
        return FromDouble(totalSec);
    }

    /// <summary>Multiply TimeSpec by a scalar rate (commutative).</summary>
    public static TimeSpec operator *(double rate, TimeSpec t) => t * rate;

    /// <summary>Compute absolute difference between two TimeSpec values.</summary>
    public static TimeSpec AbsDiff(TimeSpec a, TimeSpec b)
    {
        if (a >= b)
        {
            return a - b;
        }
        else
        {
            var diff = b - a;
            // Ensure result is positive
            if (diff.Sec < 0)
            {
                diff.Sec = -diff.Sec;
            }
            return diff;
        }
    }

    /// <summary>Convert time difference to double (seconds).</summary>
    public static double ToSeconds(TimeSpec t)
    {
        return (double)t.Sec + (double)t.Nsec * 1e-9;
    }

    public override string ToString()
    {
        return $"{Sec}.{Nsec:D9}";
    }
}
