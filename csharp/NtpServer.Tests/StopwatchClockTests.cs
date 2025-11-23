using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpServer.Internal;

namespace NtpServer.Tests;

[TestClass]
public class StopwatchClockTests
{
    [TestMethod]
    public void Constructor_CreatesNewInstance()
    {
        var clock = new StopwatchClock();
        Assert.IsNotNull(clock);
    }

    [TestMethod]
    public void MultipleInstances_AreIndependent()
    {
        var clock1 = new StopwatchClock();
        var clock2 = new StopwatchClock();
        Assert.AreNotSame(clock1, clock2);
    }

    [TestMethod]
    public void NowUnix_ReturnsReasonableTime()
    {
        var clock = new StopwatchClock();
        var now = clock.NowUnix();

        // Check that time is reasonable (after year 2020, before year 2100)
        Assert.IsGreaterThan(1577836800, now.Sec); // 2020-01-01
        Assert.IsLessThan(4102444800, now.Sec); // 2100-01-01
    }

    [TestMethod]
    public void NowUnix_AdvancesWithTime()
    {
        var clock = new StopwatchClock();
        var time1 = clock.NowUnix();

        Thread.Sleep(100); // Wait 100ms

        var time2 = clock.NowUnix();

        // Time should have advanced
        Assert.IsTrue(time2 > time1);

        // Should be approximately 100ms difference (allow 50ms tolerance)
        var diff = TimeSpec.ToSeconds(time2 - time1);
        Assert.IsGreaterThanOrEqualTo(0.05, diff, $"Expected >= 0.05s, got {diff}s");
        Assert.IsLessThanOrEqualTo(0.15, diff, $"Expected <= 0.15s, got {diff}s");
    }

    [TestMethod]
    public void SetAbsolute_ChangesTime()
    {
        var clock = new StopwatchClock();
        var targetTime = new TimeSpec(1234567890, 0);

        clock.SetAbsolute(targetTime);
        var now = clock.NowUnix();

        // Should be very close to target time (within 10ms)
        var diff = TimeSpec.AbsDiff(now, targetTime);
        Assert.IsLessThan(0.01, TimeSpec.ToSeconds(diff));
    }

    [TestMethod]
    public void SetRate_AffectsTimeProgression()
    {
        var clock = new StopwatchClock();

        // Set rate to 2.0 (double speed)
        clock.SetRate(2.0);
        var time1 = clock.NowUnix();

        Thread.Sleep(100); // Wait 100ms

        var time2 = clock.NowUnix();

        // Should have advanced ~200ms (2x speed)
        var diff = TimeSpec.ToSeconds(time2 - time1);
        Assert.IsGreaterThanOrEqualTo(0.15, diff, $"Expected >= 0.15s, got {diff}s");
        Assert.IsLessThanOrEqualTo(0.25, diff, $"Expected <= 0.25s, got {diff}s");
    }

    [TestMethod]
    public void GetRate_ReturnsCurrentRate()
    {
        var clock = new StopwatchClock();

        // Default rate should be 1.0
        Assert.AreEqual(1.0, clock.GetRate(), 1e-6);

        // Set new rate
        clock.SetRate(1.5);
        Assert.AreEqual(1.5, clock.GetRate(), 1e-6);
    }

    [TestMethod]
    public void SetAbsoluteAndRate_UpdatesBoth()
    {
        var clock = new StopwatchClock();
        var targetTime = new TimeSpec(2000000000, 0);

        clock.SetAbsoluteAndRate(targetTime, 0.5);

        // Check time
        var now = clock.NowUnix();
        var timeDiff = TimeSpec.AbsDiff(now, targetTime);
        Assert.IsLessThan(0.01, TimeSpec.ToSeconds(timeDiff));

        // Check rate
        Assert.AreEqual(0.5, clock.GetRate(), 1e-6);
    }

    [TestMethod]
    public void ResetToRealTime_RestoresNormalOperation()
    {
        var clock = new StopwatchClock();

        // Set abnormal time and rate
        clock.SetAbsoluteAndRate(new TimeSpec(1000000000, 0), 10.0);

        // Reset
        clock.ResetToRealTime();

        // Rate should be 1.0
        Assert.AreEqual(1.0, clock.GetRate(), 1e-6);

        // Time should be reasonable (current system time)
        var now = clock.NowUnix();
        Assert.IsGreaterThan(1577836800, now.Sec); // After 2020-01-01
        Assert.IsLessThan(4102444800, now.Sec); // Before 2100-01-01
    }

    [TestMethod]
    public void SetRate_PreservesCurrentTime()
    {
        var clock = new StopwatchClock();
        var timeBefore = clock.NowUnix();

        // Change rate
        clock.SetRate(2.0);

        var timeAfter = clock.NowUnix();

        // Time should not have jumped significantly (within 10ms)
        var diff = TimeSpec.AbsDiff(timeBefore, timeAfter);
        Assert.IsLessThan(0.01, TimeSpec.ToSeconds(diff));
    }

    [TestMethod]
    public void MultipleInstances_OperateIndependently()
    {
        var clock1 = new StopwatchClock();
        var clock2 = new StopwatchClock();

        clock1.SetRate(2.0);
        clock2.SetRate(0.5);

        Assert.AreEqual(2.0, clock1.GetRate(), 1e-6);
        Assert.AreEqual(0.5, clock2.GetRate(), 1e-6);

        var time1a = clock1.NowUnix();
        var time2a = clock2.NowUnix();

        Thread.Sleep(100);

        var time1b = clock1.NowUnix();
        var time2b = clock2.NowUnix();

        var diff1 = TimeSpec.ToSeconds(time1b - time1a);
        var diff2 = TimeSpec.ToSeconds(time2b - time2a);

        // clock1 should advance ~2x faster than clock2
        Assert.IsGreaterThan(diff2 * 1.5, diff1, $"clock1: {diff1}s, clock2: {diff2}s");
    }
}
