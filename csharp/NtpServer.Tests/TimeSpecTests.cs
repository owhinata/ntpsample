using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpServer.Internal;

namespace NtpServer.Tests;

[TestClass]
public class TimeSpecTests
{
    [TestMethod]
    public void DefaultConstructor_CreatesZeroTime()
    {
        var ts = new TimeSpec();
        Assert.AreEqual(0L, ts.Sec);
        Assert.AreEqual(0u, ts.Nsec);
    }

    [TestMethod]
    public void Constructor_SetsValues()
    {
        var ts = new TimeSpec(100, 500_000_000);
        Assert.AreEqual(100L, ts.Sec);
        Assert.AreEqual(500_000_000u, ts.Nsec);
    }

    [TestMethod]
    public void Normalize_HandlesOverflow()
    {
        var ts = new TimeSpec(10, 1_500_000_000);
        ts.Normalize();
        Assert.AreEqual(11L, ts.Sec);
        Assert.AreEqual(500_000_000u, ts.Nsec);
    }

    [TestMethod]
    public void Normalize_HandlesMultipleSeconds()
    {
        var ts = new TimeSpec(5, 3_000_000_000);
        ts.Normalize();
        Assert.AreEqual(8L, ts.Sec);
        Assert.AreEqual(0u, ts.Nsec);
    }

    [TestMethod]
    public void ToDouble_ConvertsCorrectly()
    {
        var ts = new TimeSpec(10, 500_000_000);
        double result = ts.ToDouble();
        Assert.AreEqual(10.5, result, 1e-9);
    }

    [TestMethod]
    public void FromDouble_ConvertsCorrectly()
    {
        var ts = TimeSpec.FromDouble(10.5);
        Assert.AreEqual(10L, ts.Sec);
        Assert.AreEqual(500_000_000u, ts.Nsec, 1u); // Allow 1ns tolerance for rounding
    }

    [TestMethod]
    public void FromDouble_HandlesNegativeValues()
    {
        var ts = TimeSpec.FromDouble(-5.25);
        Assert.AreEqual(-6L, ts.Sec);
        Assert.AreEqual(750_000_000u, ts.Nsec, 1_000_000u); // Allow tolerance
    }

    [TestMethod]
    public void NtpTimestamp_RoundTrip()
    {
        var original = new TimeSpec(1234567890, 123456789);
        ulong ntpTs = original.ToNtpTimestamp();
        var recovered = TimeSpec.FromNtpTimestamp(ntpTs);

        // Allow small tolerance due to precision loss in NTP format
        Assert.AreEqual(original.Sec, recovered.Sec);
        Assert.AreEqual(original.Nsec, recovered.Nsec, 1000u);
    }

    [TestMethod]
    public void Equality_WorksCorrectly()
    {
        var ts1 = new TimeSpec(100, 200);
        var ts2 = new TimeSpec(100, 200);
        var ts3 = new TimeSpec(100, 300);

        Assert.IsTrue(ts1 == ts2);
        Assert.IsFalse(ts1 == ts3);
        Assert.IsTrue(ts1.Equals(ts2));
        Assert.IsFalse(ts1.Equals(ts3));
    }

    [TestMethod]
    public void Comparison_WorksCorrectly()
    {
        var ts1 = new TimeSpec(100, 200);
        var ts2 = new TimeSpec(100, 300);
        var ts3 = new TimeSpec(101, 100);

        Assert.IsTrue(ts1 < ts2);
        Assert.IsTrue(ts2 < ts3);
        Assert.IsTrue(ts1 <= ts2);
        Assert.IsTrue(ts2 > ts1);
        Assert.IsTrue(ts3 >= ts2);
    }

    [TestMethod]
    public void Addition_WorksCorrectly()
    {
        var ts1 = new TimeSpec(10, 600_000_000);
        var ts2 = new TimeSpec(5, 500_000_000);
        var result = ts1 + ts2;

        Assert.AreEqual(16L, result.Sec);
        Assert.AreEqual(100_000_000u, result.Nsec);
    }

    [TestMethod]
    public void Subtraction_WorksCorrectly()
    {
        var ts1 = new TimeSpec(10, 600_000_000);
        var ts2 = new TimeSpec(5, 200_000_000);
        var result = ts1 - ts2;

        Assert.AreEqual(5L, result.Sec);
        Assert.AreEqual(400_000_000u, result.Nsec);
    }

    [TestMethod]
    public void Subtraction_WithBorrow()
    {
        var ts1 = new TimeSpec(10, 200_000_000);
        var ts2 = new TimeSpec(5, 600_000_000);
        var result = ts1 - ts2;

        Assert.AreEqual(4L, result.Sec);
        Assert.AreEqual(600_000_000u, result.Nsec);
    }

    [TestMethod]
    public void Multiplication_WorksCorrectly()
    {
        var ts = new TimeSpec(10, 0);
        var result = ts * 2.5;

        Assert.AreEqual(25L, result.Sec);
        Assert.AreEqual(0u, result.Nsec, 1_000_000u); // Allow tolerance
    }

    [TestMethod]
    public void AbsDiff_WorksCorrectly()
    {
        var ts1 = new TimeSpec(10, 0);
        var ts2 = new TimeSpec(5, 0);

        var diff1 = TimeSpec.AbsDiff(ts1, ts2);
        var diff2 = TimeSpec.AbsDiff(ts2, ts1);

        Assert.AreEqual(5L, diff1.Sec);
        Assert.AreEqual(5L, diff2.Sec);
    }

    [TestMethod]
    public void ToSeconds_WorksCorrectly()
    {
        var ts = new TimeSpec(10, 500_000_000);
        double seconds = TimeSpec.ToSeconds(ts);
        Assert.AreEqual(10.5, seconds, 1e-9);
    }
}
