using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;
using NtpServer.Internal;

namespace NtpClock.Tests;

[TestClass]
public class ClockCorrectorTests
{
    [TestMethod]
    public void Constructor_InitializesWithZeroOffset()
    {
        var corrector = new ClockCorrector();
        Assert.AreEqual(0.0, corrector.GetOffsetApplied());
    }

    [TestMethod]
    public void Apply_SmallDelta_ReturnsSlew()
    {
        var corrector = new ClockCorrector();

        // Delta is 0.05s, threshold is 0.2s -> should slew
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.05,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreEqual(0.005, amount, 1e-9); // Limited by slew rate
        Assert.AreEqual(0.005, corrector.GetOffsetApplied(), 1e-9);
    }

    [TestMethod]
    public void Apply_LargeDelta_ReturnsStep()
    {
        var corrector = new ClockCorrector();

        // Delta is 0.5s, threshold is 0.2s -> should step
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.5,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Step, result);
        Assert.AreEqual(0.5, amount, 1e-9);
        Assert.AreEqual(0.5, corrector.GetOffsetApplied(), 1e-9);
    }

    [TestMethod]
    public void Apply_DeltaExactlyAtThreshold_ReturnsStep()
    {
        var corrector = new ClockCorrector();

        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.2,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Step, result);
        Assert.AreEqual(0.2, amount, 1e-9);
    }

    [TestMethod]
    public void Apply_NoDelta_ReturnsNone()
    {
        var corrector = new ClockCorrector();

        var result = corrector.Apply(
            currentOffset: 0.1,
            targetOffset: 0.1,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.None, result);
        Assert.AreEqual(0.0, amount);
    }

    [TestMethod]
    public void Apply_VerySmallDelta_AppliesSlew()
    {
        var corrector = new ClockCorrector();

        // Very small delta should still be applied as slew
        var result = corrector.Apply(
            currentOffset: 0.1,
            targetOffset: 0.1 + 1e-15,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreNotEqual(0.0, amount);
    }

    [TestMethod]
    public void Apply_SlewRateLimitsChange()
    {
        var corrector = new ClockCorrector();

        // Target delta is 0.1s, but slew rate limits to 0.005s/s * 1s = 0.005s
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.1,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreEqual(0.005, amount, 1e-9);
    }

    [TestMethod]
    public void Apply_SlewWithLongerPollInterval()
    {
        var corrector = new ClockCorrector();

        // Poll interval is 10s, so max change is 0.005s/s * 10s = 0.05s
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.1,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 10.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreEqual(0.05, amount, 1e-9);
    }

    [TestMethod]
    public void Apply_NegativeDelta_SlewsBackward()
    {
        var corrector = new ClockCorrector();

        // Current offset is 0.1s, target is 0.05s -> delta is -0.05s
        var result = corrector.Apply(
            currentOffset: 0.1,
            targetOffset: 0.05,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreEqual(-0.005, amount, 1e-9); // Limited by slew rate
        Assert.AreEqual(0.095, corrector.GetOffsetApplied(), 1e-9);
    }

    [TestMethod]
    public void Apply_NegativeDelta_Steps()
    {
        var corrector = new ClockCorrector();

        // Delta is -0.5s, magnitude exceeds threshold -> step
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: -0.5,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Step, result);
        Assert.AreEqual(-0.5, amount, 1e-9);
        Assert.AreEqual(-0.5, corrector.GetOffsetApplied(), 1e-9);
    }

    [TestMethod]
    public void Apply_ProgressiveSlew()
    {
        var corrector = new ClockCorrector();

        // First correction
        corrector.Apply(0.0, 0.1, 0.2, 0.005, 1.0, out double amount1);
        Assert.AreEqual(0.005, amount1, 1e-9);
        Assert.AreEqual(0.005, corrector.GetOffsetApplied(), 1e-9);

        // Second correction (continuing from current offset)
        corrector.Apply(0.005, 0.1, 0.2, 0.005, 1.0, out double amount2);
        Assert.AreEqual(0.005, amount2, 1e-9);
        Assert.AreEqual(0.01, corrector.GetOffsetApplied(), 1e-9);

        // Third correction
        corrector.Apply(0.01, 0.1, 0.2, 0.005, 1.0, out double amount3);
        Assert.AreEqual(0.005, amount3, 1e-9);
        Assert.AreEqual(0.015, corrector.GetOffsetApplied(), 1e-9);
    }

    [TestMethod]
    public void GetMonotonicTime_AppliesOffset()
    {
        var corrector = new ClockCorrector();

        // Apply offset of 0.5s
        corrector.Apply(0.0, 0.5, 1.0, 0.5, 1.0, out _);

        var baseTime = new TimeSpec(1000, 0);
        var result = corrector.GetMonotonicTime(baseTime);

        var expected = new TimeSpec(1000, 500_000_000);
        Assert.AreEqual(expected.Sec, result.Sec);
        Assert.AreEqual(expected.Nsec, result.Nsec, 1000u);
    }

    [TestMethod]
    public void GetMonotonicTime_EnforcesMonotonicity()
    {
        var corrector = new ClockCorrector();

        var time1 = new TimeSpec(1000, 0);
        var result1 = corrector.GetMonotonicTime(time1);

        // Second call with earlier time - should return monotonic value
        var time2 = new TimeSpec(999, 500_000_000);
        var result2 = corrector.GetMonotonicTime(time2);

        Assert.IsTrue(result2 > result1);
    }

    [TestMethod]
    public void GetMonotonicTime_AllowsForwardProgress()
    {
        var corrector = new ClockCorrector();

        var time1 = new TimeSpec(1000, 0);
        var result1 = corrector.GetMonotonicTime(time1);

        var time2 = new TimeSpec(1001, 0);
        var result2 = corrector.GetMonotonicTime(time2);

        Assert.IsTrue(result2 > result1);
        Assert.AreEqual(1001, result2.Sec);
    }

    [TestMethod]
    public void GetMonotonicTime_AfterStep_AllowsBackwardOnce()
    {
        var corrector = new ClockCorrector();

        // Initial time
        var time1 = new TimeSpec(1000, 0);
        var result1 = corrector.GetMonotonicTime(time1);
        Assert.AreEqual(1000, result1.Sec);

        // Apply backward step of -10 seconds
        corrector.Apply(0.0, -10.0, 1.0, 0.005, 1.0, out _);

        // Next call should allow the backward jump
        var time2 = new TimeSpec(1001, 0);
        var result2 = corrector.GetMonotonicTime(time2);
        Assert.AreEqual(991, result2.Sec); // 1001 - 10

        // But subsequent call should be monotonic again
        var time3 = new TimeSpec(990, 0);
        var result3 = corrector.GetMonotonicTime(time3);
        Assert.IsTrue(result3 >= result2);
    }

    [TestMethod]
    public void AllowBackwardOnce_PermitsBackwardJump()
    {
        var corrector = new ClockCorrector();

        // Establish baseline
        var time1 = new TimeSpec(1000, 0);
        var result1 = corrector.GetMonotonicTime(time1);

        // Call AllowBackwardOnce
        corrector.AllowBackwardOnce();

        // Next call with earlier time should be allowed
        var time2 = new TimeSpec(900, 0);
        var result2 = corrector.GetMonotonicTime(time2);
        Assert.AreEqual(900, result2.Sec);
    }

    [TestMethod]
    public void AllowBackwardOnce_OnlyWorksOnce()
    {
        var corrector = new ClockCorrector();

        var time1 = new TimeSpec(1000, 0);
        corrector.GetMonotonicTime(time1);

        corrector.AllowBackwardOnce();

        var time2 = new TimeSpec(900, 0);
        var result2 = corrector.GetMonotonicTime(time2);
        Assert.AreEqual(900, result2.Sec);

        // Next call should enforce monotonicity again
        var time3 = new TimeSpec(850, 0);
        var result3 = corrector.GetMonotonicTime(time3);
        Assert.IsTrue(result3 >= result2);
    }

    [TestMethod]
    public void ResetOffset_ClearsAppliedOffset()
    {
        var corrector = new ClockCorrector();

        corrector.Apply(0.0, 0.5, 1.0, 0.5, 1.0, out _);
        Assert.AreEqual(0.5, corrector.GetOffsetApplied(), 1e-9);

        corrector.ResetOffset();
        Assert.AreEqual(0.0, corrector.GetOffsetApplied());
    }

    [TestMethod]
    public void ResetOffset_ClearsMonotonicState()
    {
        var corrector = new ClockCorrector();

        // Advance time
        var time1 = new TimeSpec(1000, 0);
        corrector.GetMonotonicTime(time1);

        corrector.ResetOffset();

        // After reset, should be able to use earlier time
        var time2 = new TimeSpec(500, 0);
        var result = corrector.GetMonotonicTime(time2);
        Assert.AreEqual(500, result.Sec);
    }

    [TestMethod]
    public void ResetOffset_ClearsBackwardAllowance()
    {
        var corrector = new ClockCorrector();

        corrector.AllowBackwardOnce();
        corrector.ResetOffset();

        // Establish baseline
        var time1 = new TimeSpec(1000, 0);
        corrector.GetMonotonicTime(time1);

        // Should now enforce monotonicity (backward allowance was cleared)
        var time2 = new TimeSpec(900, 0);
        var result = corrector.GetMonotonicTime(time2);
        Assert.IsTrue(result >= time1);
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentApplyAndGet()
    {
        var corrector = new ClockCorrector();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 1000; i++)
                {
                    corrector.Apply(i * 0.001, (i + 1) * 0.001, 0.2, 0.005, 1.0, out _);
                }
            }
            catch
            {
                error = true;
            }
        });

        var t2 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 1000; i++)
                {
                    _ = corrector.GetOffsetApplied();
                }
            }
            catch
            {
                error = true;
            }
        });

        t1.Start();
        t2.Start();
        t1.Join();
        t2.Join();

        Assert.IsFalse(error);
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentGetMonotonicTime()
    {
        var corrector = new ClockCorrector();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 1000; i++)
                {
                    var time = new TimeSpec(1000 + i, 0);
                    _ = corrector.GetMonotonicTime(time);
                }
            }
            catch
            {
                error = true;
            }
        });

        var t2 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 1000; i++)
                {
                    var time = new TimeSpec(2000 + i, 0);
                    _ = corrector.GetMonotonicTime(time);
                }
            }
            catch
            {
                error = true;
            }
        });

        t1.Start();
        t2.Start();
        t1.Join();
        t2.Join();

        Assert.IsFalse(error);
    }

    [TestMethod]
    public void Apply_ZeroSlewRate_StillAllowsCorrection()
    {
        var corrector = new ClockCorrector();

        // With zero slew rate, delta should be clamped to 0
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.1,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.0,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.None, result);
        Assert.AreEqual(0.0, amount);
    }

    [TestMethod]
    public void Apply_ExactSlewAmount_AppliesCompletely()
    {
        var corrector = new ClockCorrector();

        // Delta exactly matches max slew amount
        var result = corrector.Apply(
            currentOffset: 0.0,
            targetOffset: 0.005,
            stepThresholdS: 0.2,
            slewRateSPerS: 0.005,
            pollIntervalS: 1.0,
            out double amount
        );

        Assert.AreEqual(Status.CorrectionType.Slew, result);
        Assert.AreEqual(0.005, amount, 1e-9);
    }

    [TestMethod]
    public void GetMonotonicTime_WithSmallEpsilon()
    {
        var corrector = new ClockCorrector();

        var time1 = new TimeSpec(1000, 0);
        corrector.GetMonotonicTime(time1);

        // Very small forward progress should still advance
        var time2 = new TimeSpec(1000, 2);
        var result = corrector.GetMonotonicTime(time2);

        Assert.IsTrue(result > time1);
    }
}
