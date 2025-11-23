// Copyright (c) 2025 <Your Name>
namespace NtpClock.Internal;

/// <summary>
/// Immutable configuration options for ClockService.
/// Use the Builder to construct instances. All fields are read-only via properties.
/// </summary>
public class Options
{
    /// <summary>Callback for logging messages (thread-safe).</summary>
    public delegate void LogCallback(string message);

    public const int DefaultPollIntervalMs = 10000;
    public const int DefaultStepThresholdMs = 200;
    public const double DefaultSlewRateMsPerSec = 5.0;
    public const int DefaultMaxRttMs = 100;
    public const int DefaultMinSamplesToLock = 3;
    public const int DefaultOffsetWindow = 5;
    public const int DefaultSkewWindow = 10;

    /// <summary>Polling interval in milliseconds.</summary>
    public int PollIntervalMs { get; }

    /// <summary>Step threshold in milliseconds.</summary>
    public int StepThresholdMs { get; }

    /// <summary>Slew rate in milliseconds per second.</summary>
    public double SlewRateMsPerSec { get; }

    /// <summary>Maximum acceptable round-trip time in milliseconds.</summary>
    public int MaxRttMs { get; }

    /// <summary>Minimum samples required to report synchronized.</summary>
    public int MinSamplesToLock { get; }

    /// <summary>Number of recent offsets for median target computation.</summary>
    public int OffsetWindow { get; }

    /// <summary>Number of samples for skew OLS window.</summary>
    public int SkewWindow { get; }

    /// <summary>Log sink callback.</summary>
    public LogCallback? LogSink { get; }

    private Options(
        int pollIntervalMs,
        int stepThresholdMs,
        double slewRateMsPerSec,
        int maxRttMs,
        int minSamplesToLock,
        int offsetWindow,
        int skewWindow,
        LogCallback? logSink
    )
    {
        PollIntervalMs = pollIntervalMs;
        StepThresholdMs = stepThresholdMs;
        SlewRateMsPerSec = slewRateMsPerSec;
        MaxRttMs = maxRttMs;
        MinSamplesToLock = minSamplesToLock;
        OffsetWindow = offsetWindow;
        SkewWindow = skewWindow;
        LogSink = logSink;
    }

    /// <summary>Fluent builder for Options.</summary>
    public class Builder
    {
        private int _pollIntervalMs = DefaultPollIntervalMs;
        private int _stepThresholdMs = DefaultStepThresholdMs;
        private double _slewRateMsPerSec = DefaultSlewRateMsPerSec;
        private int _maxRttMs = DefaultMaxRttMs;
        private int _minSamplesToLock = DefaultMinSamplesToLock;
        private int _offsetWindow = DefaultOffsetWindow;
        private int _skewWindow = DefaultSkewWindow;
        private LogCallback? _logSink;

        public Builder() { }

        public Builder(Options baseOptions)
        {
            _pollIntervalMs = baseOptions.PollIntervalMs;
            _stepThresholdMs = baseOptions.StepThresholdMs;
            _slewRateMsPerSec = baseOptions.SlewRateMsPerSec;
            _maxRttMs = baseOptions.MaxRttMs;
            _minSamplesToLock = baseOptions.MinSamplesToLock;
            _offsetWindow = baseOptions.OffsetWindow;
            _skewWindow = baseOptions.SkewWindow;
            _logSink = baseOptions.LogSink;
        }

        /// <summary>Set polling interval in milliseconds.</summary>
        public Builder PollIntervalMs(int value)
        {
            _pollIntervalMs = value;
            return this;
        }

        /// <summary>Set step threshold in milliseconds.</summary>
        public Builder StepThresholdMs(int value)
        {
            _stepThresholdMs = value;
            return this;
        }

        /// <summary>Set slew rate in milliseconds per second.</summary>
        public Builder SlewRateMsPerSec(double value)
        {
            _slewRateMsPerSec = value;
            return this;
        }

        /// <summary>Set maximum acceptable RTT in milliseconds.</summary>
        public Builder MaxRttMs(int value)
        {
            _maxRttMs = value;
            return this;
        }

        /// <summary>Set minimum samples to report synchronized.</summary>
        public Builder MinSamplesToLock(int value)
        {
            _minSamplesToLock = value;
            return this;
        }

        /// <summary>Number of recent offsets for median target.</summary>
        public Builder OffsetWindow(int value)
        {
            _offsetWindow = value;
            return this;
        }

        /// <summary>Number of samples for skew OLS window.</summary>
        public Builder SkewWindow(int value)
        {
            _skewWindow = value;
            return this;
        }

        /// <summary>Set log sink callback.</summary>
        public Builder LogSink(LogCallback callback)
        {
            _logSink = callback;
            return this;
        }

        public Options Build()
        {
            return new Options(
                _pollIntervalMs,
                _stepThresholdMs,
                _slewRateMsPerSec,
                _maxRttMs,
                _minSamplesToLock,
                _offsetWindow,
                _skewWindow,
                _logSink
            );
        }
    }
}
