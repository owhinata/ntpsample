// Copyright (c) 2025 <Your Name>
using NtpServer.Internal;

namespace NtpClock.Internal;

/// <summary>
/// Unified clock correction and monotonicity management.
/// Combines correction decision logic (slew vs step) with monotonic time
/// enforcement. When a step correction is applied, coordinates the one-time
/// backward jump allowance for NowUnix().
/// </summary>
public class ClockCorrector
{
    private readonly object _lock = new object();
    private TimeSpec _offsetApplied = new TimeSpec();
    private TimeSpec _lastReturned = new TimeSpec();
    private bool _allowBackwardOnce = false;

    /// <summary>
    /// Apply offset correction based on target vs current delta.
    /// Called periodically from the sync loop to update the offset correction
    /// parameter. Does NOT read time; only updates the correction state.
    /// </summary>
    /// <param name="currentOffset">Currently applied offset (seconds).</param>
    /// <param name="targetOffset">Desired target offset (seconds).</param>
    /// <param name="stepThresholdS">Threshold for step correction (seconds).</param>
    /// <param name="slewRateSPerS">Maximum slew rate (seconds per second).</param>
    /// <param name="pollIntervalS">Polling interval (seconds).</param>
    /// <param name="outAmountS">Output parameter for actual correction amount applied.</param>
    /// <returns>Type of correction applied (None, Slew, or Step).</returns>
    public Status.CorrectionType Apply(
        double currentOffset,
        double targetOffset,
        double stepThresholdS,
        double slewRateSPerS,
        double pollIntervalS,
        out double outAmountS
    )
    {
        double delta = targetOffset - currentOffset;

        if (Math.Abs(delta) >= stepThresholdS)
        {
            // Step correction: apply full delta immediately
            TimeSpec newOffset = TimeSpec.FromDouble(currentOffset + delta);
            TimeSpec deltaTs = TimeSpec.FromDouble(delta);

            lock (_lock)
            {
                _offsetApplied = newOffset;
                // Adjust last_returned by the step amount to allow backward jump
                _lastReturned = _lastReturned + deltaTs;
                _allowBackwardOnce = true;
            }

            outAmountS = delta;
            return Status.CorrectionType.Step;
        }
        else
        {
            // Slew correction: apply bounded-rate change
            double maxChange = slewRateSPerS * pollIntervalS;
            double change = Math.Clamp(delta, -maxChange, maxChange);

            if (Math.Abs(change) > 0.0)
            {
                TimeSpec newOffset = TimeSpec.FromDouble(currentOffset + change);

                lock (_lock)
                {
                    _offsetApplied = newOffset;
                }

                outAmountS = change;
                return Status.CorrectionType.Slew;
            }
        }

        outAmountS = 0.0;
        return Status.CorrectionType.None;
    }

    /// <summary>
    /// Allow the next time reading to go backward once.
    /// Typically called after external time adjustments (e.g., vendor hints
    /// applying SetAbsolute) to permit GetMonotonicTime() to reflect the backward
    /// jump.
    /// </summary>
    public void AllowBackwardOnce()
    {
        lock (_lock)
        {
            _allowBackwardOnce = true;
        }
    }

    /// <summary>
    /// Get monotonic corrected time.
    /// Called from NowUnix() at arbitrary times to read the current corrected
    /// time. Applies the offset correction and enforces monotonic progression.
    /// </summary>
    /// <param name="baseTime">The raw base time from TimeSource.</param>
    /// <returns>The monotonic corrected time (base + offset, with monotonicity
    /// enforced, unless backward jump is explicitly allowed).</returns>
    public TimeSpec GetMonotonicTime(TimeSpec baseTime)
    {
        lock (_lock)
        {
            // Apply current offset correction
            TimeSpec candidate = baseTime + _offsetApplied;

            // Allow one backward jump right after a step correction
            if (_allowBackwardOnce)
            {
                _allowBackwardOnce = false;
                _lastReturned = candidate;
                return candidate;
            }

            // Otherwise, enforce monotonicity with a small epsilon
            TimeSpec eps = TimeSpec.FromDouble(1e-9);
            TimeSpec minNext = _lastReturned + eps;

            if (candidate < minNext)
            {
                _lastReturned = minNext;
                return minNext;
            }
            else
            {
                _lastReturned = candidate;
                return candidate;
            }
        }
    }

    /// <summary>
    /// Get current offset correction value.
    /// </summary>
    /// <returns>Current applied offset (in seconds as double for compatibility).</returns>
    public double GetOffsetApplied()
    {
        lock (_lock)
        {
            return _offsetApplied.ToDouble();
        }
    }

    /// <summary>
    /// Reset offset to zero.
    /// </summary>
    public void ResetOffset()
    {
        lock (_lock)
        {
            _offsetApplied = new TimeSpec();
            _lastReturned = new TimeSpec();
            _allowBackwardOnce = false;
        }
    }
}
