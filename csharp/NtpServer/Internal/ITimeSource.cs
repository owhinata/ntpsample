// Copyright (c) 2025 <Your Name>
namespace NtpServer.Internal;

/// <summary>
/// Interface for time sources.
/// Provides current time as UNIX epoch time with nanosecond precision.
/// </summary>
public interface ITimeSource
{
    /// <summary>Returns the current time since the UNIX epoch.</summary>
    TimeSpec NowUnix();

    /// <summary>Sets absolute time (optional; used by clients).</summary>
    void SetAbsolute(TimeSpec time);

    /// <summary>Sets progression rate multiplier (1.0 = real time).</summary>
    void SetRate(double rate);

    /// <summary>
    /// Atomically sets both absolute time and rate.
    /// Sets both absolute time and progression rate in a single atomic operation,
    /// ensuring consistency when both values need to be updated together.
    /// </summary>
    void SetAbsoluteAndRate(TimeSpec time, double rate);

    /// <summary>
    /// Resets time source to real-time (system clock) with rate 1.0.
    /// Resets both absolute time and rate to match the current system time
    /// with normal progression (rate = 1.0). Useful for returning to normal
    /// operation after testing or manual adjustments.
    /// </summary>
    void ResetToRealTime();

    /// <summary>
    /// Returns current progression rate multiplier.
    /// Default implementations may return 1.0 if unknown.
    /// </summary>
    double GetRate();
}
