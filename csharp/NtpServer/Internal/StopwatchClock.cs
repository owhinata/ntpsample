// Copyright (c) 2025 The NTP Sample Authors
using System.Diagnostics;

namespace NtpServer.Internal;

/// <summary>
/// Adjustable time source with rate and absolute time control.
/// Provides a UNIX-seconds clock derived from Stopwatch. You can adjust the
/// progression rate and set absolute time for testing.
/// </summary>
public class StopwatchClock : ITimeSource
{
    private readonly Stopwatch _stopwatch;
    private readonly object _lock = new();
    private long _anchorTicks; // Stopwatch ticks at anchor point
    private TimeSpec _startTime; // Time at anchor point
    private double _rate; // Progression rate multiplier

    /// <summary>
    /// Create a new StopwatchClock instance.
    /// </summary>
    public StopwatchClock()
    {
        _stopwatch = Stopwatch.StartNew();
        _anchorTicks = _stopwatch.ElapsedTicks;
        _startTime = CurrentUnix();
        _rate = 1.0;
    }

    /// <summary>Returns current time.</summary>
    public TimeSpec NowUnix()
    {
        lock (_lock)
        {
            double elapsed = Elapsed();
            double adjustedElapsed = elapsed * _rate;
            return _startTime + TimeSpec.FromDouble(adjustedElapsed);
        }
    }

    /// <summary>Sets progression rate (default 1.0).</summary>
    public void SetRate(double rate)
    {
        lock (_lock)
        {
            // Capture current time before changing rate
            TimeSpec currentTime = NowUnixNoLock();
            _startTime = currentTime;
            _anchorTicks = _stopwatch.ElapsedTicks;
            _rate = rate;
        }
    }

    /// <summary>Gets progression rate.</summary>
    public double GetRate()
    {
        lock (_lock)
        {
            return _rate;
        }
    }

    /// <summary>Sets absolute time.</summary>
    public void SetAbsolute(TimeSpec time)
    {
        lock (_lock)
        {
            _startTime = time;
            _anchorTicks = _stopwatch.ElapsedTicks;
        }
    }

    /// <summary>Atomically sets both absolute time and rate.</summary>
    public void SetAbsoluteAndRate(TimeSpec time, double rate)
    {
        lock (_lock)
        {
            _startTime = time;
            _anchorTicks = _stopwatch.ElapsedTicks;
            _rate = rate;
        }
    }

    /// <summary>Resets to real-time (system clock) with rate 1.0.</summary>
    public void ResetToRealTime()
    {
        lock (_lock)
        {
            _startTime = CurrentUnix();
            _anchorTicks = _stopwatch.ElapsedTicks;
            _rate = 1.0;
        }
    }

    /// <summary>Seconds elapsed since last anchor point measured via Stopwatch.</summary>
    private double Elapsed()
    {
        long currentTicks = _stopwatch.ElapsedTicks;
        long deltaTicks = currentTicks - _anchorTicks;
        return (double)deltaTicks / Stopwatch.Frequency;
    }

    /// <summary>Current wall-clock time (system clock).</summary>
    private static TimeSpec CurrentUnix()
    {
        DateTimeOffset now = DateTimeOffset.UtcNow;
        long sec = now.ToUnixTimeSeconds();
        // Calculate nanoseconds from the millisecond component
        uint nsec = (uint)(now.Millisecond * 1_000_000);
        return new TimeSpec(sec, nsec);
    }

    /// <summary>NowUnix without acquiring lock (caller must hold lock).</summary>
    private TimeSpec NowUnixNoLock()
    {
        double elapsed = Elapsed();
        double adjustedElapsed = elapsed * _rate;
        return _startTime + TimeSpec.FromDouble(adjustedElapsed);
    }
}
