// Copyright (c) 2025 The NTP Sample Authors
using NtpServer.Internal;

namespace NtpClock.Internal;

/// <summary>
/// Current synchronization status snapshot.
/// Provides detailed information about the clock service state including
/// synchronization status, timing metrics, and estimator window statistics.
/// </summary>
public class Status
{
    /// <summary>Type of correction applied.</summary>
    public enum CorrectionType
    {
        /// <summary>No correction applied.</summary>
        None,

        /// <summary>Gradual slew correction.</summary>
        Slew,

        /// <summary>Immediate step correction.</summary>
        Step,
    }

    /// <summary>Whether the clock is synchronized to the server.</summary>
    public bool Synchronized { get; set; }

    /// <summary>Latest round-trip time in milliseconds.</summary>
    public int RttMs { get; set; }

    /// <summary>Latest computed round-trip delay in seconds.</summary>
    public double LastDelayS { get; set; }

    /// <summary>Current offset from server in seconds.</summary>
    public double OffsetS { get; set; }

    /// <summary>Estimated clock skew in parts per million.</summary>
    public double SkewPpm { get; set; }

    /// <summary>Time of last successful update.</summary>
    public TimeSpec LastUpdate { get; set; }

    /// <summary>Total number of samples received.</summary>
    public int Samples { get; set; }

    /// <summary>Type of last correction applied.</summary>
    public CorrectionType LastCorrection { get; set; }

    /// <summary>Amount of last correction in seconds.</summary>
    public double LastCorrectionAmountS { get; set; }

    /// <summary>True when upstream server epoch changed.</summary>
    public bool EpochChanged { get; set; }

    /// <summary>Current epoch number from upstream server.</summary>
    public uint Epoch { get; set; }

    /// <summary>Last error message (empty if no error).</summary>
    public string LastError { get; set; } = string.Empty;

    // Debug summary of estimator windows
    /// <summary>Configured offset window size.</summary>
    public int OffsetWindow { get; set; }

    /// <summary>Configured skew window size.</summary>
    public int SkewWindow { get; set; }

    /// <summary>Current sample count retained in windows.</summary>
    public int WindowCount { get; set; }

    /// <summary>Median offset from window in seconds.</summary>
    public double OffsetMedianS { get; set; }

    /// <summary>Minimum offset from window in seconds.</summary>
    public double OffsetMinS { get; set; }

    /// <summary>Maximum offset from window in seconds.</summary>
    public double OffsetMaxS { get; set; }

    /// <summary>Currently applied offset in seconds.</summary>
    public double OffsetAppliedS { get; set; }

    /// <summary>Target offset for next correction in seconds.</summary>
    public double OffsetTargetS { get; set; }

    public Status()
    {
        LastUpdate = new TimeSpec();
    }

    public override string ToString()
    {
        return $"Status(sync={Synchronized}, rtt={RttMs}ms, offset={OffsetS:F9}s, skew={SkewPpm:F3}ppm, samples={Samples})";
    }
}
