using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;
using NtpServer.Internal;

namespace NtpClock.Tests;

[TestClass]
public class StatusTests
{
    [TestMethod]
    public void Constructor_InitializesWithDefaults()
    {
        var status = new Status();

        Assert.IsFalse(status.Synchronized);
        Assert.AreEqual(0, status.RttMs);
        Assert.AreEqual(0.0, status.LastDelayS);
        Assert.AreEqual(0.0, status.OffsetS);
        Assert.AreEqual(0.0, status.SkewPpm);
        Assert.AreEqual(0, status.Samples);
        Assert.AreEqual(Status.CorrectionType.None, status.LastCorrection);
        Assert.AreEqual(0.0, status.LastCorrectionAmountS);
        Assert.IsFalse(status.EpochChanged);
        Assert.AreEqual(0u, status.Epoch);
        Assert.AreEqual(string.Empty, status.LastError);
        Assert.AreEqual(0, status.OffsetWindow);
        Assert.AreEqual(0, status.SkewWindow);
        Assert.AreEqual(0, status.WindowCount);
        Assert.AreEqual(0.0, status.OffsetMedianS);
        Assert.AreEqual(0.0, status.OffsetMinS);
        Assert.AreEqual(0.0, status.OffsetMaxS);
        Assert.AreEqual(0.0, status.OffsetAppliedS);
        Assert.AreEqual(0.0, status.OffsetTargetS);
    }

    [TestMethod]
    public void Constructor_InitializesLastUpdateToZero()
    {
        var status = new Status();

        Assert.AreEqual(0L, status.LastUpdate.Sec);
        Assert.AreEqual(0u, status.LastUpdate.Nsec);
    }

    [TestMethod]
    public void Synchronized_CanBeSet()
    {
        var status = new Status { Synchronized = true };

        Assert.IsTrue(status.Synchronized);
    }

    [TestMethod]
    public void RttMs_CanBeSet()
    {
        var status = new Status { RttMs = 50 };

        Assert.AreEqual(50, status.RttMs);
    }

    [TestMethod]
    public void LastDelayS_CanBeSet()
    {
        var status = new Status { LastDelayS = 0.025 };

        Assert.AreEqual(0.025, status.LastDelayS);
    }

    [TestMethod]
    public void OffsetS_CanBeSet()
    {
        var status = new Status { OffsetS = 0.123 };

        Assert.AreEqual(0.123, status.OffsetS);
    }

    [TestMethod]
    public void SkewPpm_CanBeSet()
    {
        var status = new Status { SkewPpm = 25.5 };

        Assert.AreEqual(25.5, status.SkewPpm);
    }

    [TestMethod]
    public void LastUpdate_CanBeSet()
    {
        var time = new TimeSpec(1234567890, 123456789);
        var status = new Status { LastUpdate = time };

        Assert.AreEqual(1234567890L, status.LastUpdate.Sec);
        Assert.AreEqual(123456789u, status.LastUpdate.Nsec);
    }

    [TestMethod]
    public void Samples_CanBeSet()
    {
        var status = new Status { Samples = 42 };

        Assert.AreEqual(42, status.Samples);
    }

    [TestMethod]
    public void LastCorrection_CanBeSet()
    {
        var status = new Status { LastCorrection = Status.CorrectionType.Slew };

        Assert.AreEqual(Status.CorrectionType.Slew, status.LastCorrection);
    }

    [TestMethod]
    public void LastCorrectionAmountS_CanBeSet()
    {
        var status = new Status { LastCorrectionAmountS = 0.005 };

        Assert.AreEqual(0.005, status.LastCorrectionAmountS);
    }

    [TestMethod]
    public void EpochChanged_CanBeSet()
    {
        var status = new Status { EpochChanged = true };

        Assert.IsTrue(status.EpochChanged);
    }

    [TestMethod]
    public void Epoch_CanBeSet()
    {
        var status = new Status { Epoch = 12345u };

        Assert.AreEqual(12345u, status.Epoch);
    }

    [TestMethod]
    public void LastError_CanBeSet()
    {
        var status = new Status { LastError = "Connection timeout" };

        Assert.AreEqual("Connection timeout", status.LastError);
    }

    [TestMethod]
    public void OffsetWindow_CanBeSet()
    {
        var status = new Status { OffsetWindow = 5 };

        Assert.AreEqual(5, status.OffsetWindow);
    }

    [TestMethod]
    public void SkewWindow_CanBeSet()
    {
        var status = new Status { SkewWindow = 20 };

        Assert.AreEqual(20, status.SkewWindow);
    }

    [TestMethod]
    public void WindowCount_CanBeSet()
    {
        var status = new Status { WindowCount = 10 };

        Assert.AreEqual(10, status.WindowCount);
    }

    [TestMethod]
    public void OffsetMedianS_CanBeSet()
    {
        var status = new Status { OffsetMedianS = 0.05 };

        Assert.AreEqual(0.05, status.OffsetMedianS);
    }

    [TestMethod]
    public void OffsetMinS_CanBeSet()
    {
        var status = new Status { OffsetMinS = 0.01 };

        Assert.AreEqual(0.01, status.OffsetMinS);
    }

    [TestMethod]
    public void OffsetMaxS_CanBeSet()
    {
        var status = new Status { OffsetMaxS = 0.1 };

        Assert.AreEqual(0.1, status.OffsetMaxS);
    }

    [TestMethod]
    public void OffsetAppliedS_CanBeSet()
    {
        var status = new Status { OffsetAppliedS = 0.075 };

        Assert.AreEqual(0.075, status.OffsetAppliedS);
    }

    [TestMethod]
    public void OffsetTargetS_CanBeSet()
    {
        var status = new Status { OffsetTargetS = 0.08 };

        Assert.AreEqual(0.08, status.OffsetTargetS);
    }

    [TestMethod]
    public void ToString_ReturnsFormattedString()
    {
        var status = new Status
        {
            Synchronized = true,
            RttMs = 50,
            OffsetS = 0.123456789,
            SkewPpm = 25.567,
            Samples = 42,
        };

        string result = status.ToString();

        StringAssert.Contains(result, "sync=True");
        StringAssert.Contains(result, "rtt=50ms");
        StringAssert.Contains(result, "offset=0.123456789s");
        StringAssert.Contains(result, "skew=25.567ppm");
        StringAssert.Contains(result, "samples=42");
    }

    [TestMethod]
    public void ToString_WithNotSynchronized()
    {
        var status = new Status { Synchronized = false };

        string result = status.ToString();

        StringAssert.Contains(result, "sync=False");
    }

    [TestMethod]
    public void CorrectionType_HasExpectedValues()
    {
        // Verify enum values are as expected
        var none = Status.CorrectionType.None;
        var slew = Status.CorrectionType.Slew;
        var step = Status.CorrectionType.Step;

        Assert.AreNotEqual(none, slew);
        Assert.AreNotEqual(slew, step);
        Assert.AreNotEqual(none, step);
    }

    [TestMethod]
    public void AllProperties_CanBeSetTogether()
    {
        var status = new Status
        {
            Synchronized = true,
            RttMs = 50,
            LastDelayS = 0.025,
            OffsetS = 0.123,
            SkewPpm = 25.5,
            LastUpdate = new TimeSpec(1234567890, 123456789),
            Samples = 42,
            LastCorrection = Status.CorrectionType.Slew,
            LastCorrectionAmountS = 0.005,
            EpochChanged = true,
            Epoch = 12345,
            LastError = "Test error",
            OffsetWindow = 5,
            SkewWindow = 20,
            WindowCount = 10,
            OffsetMedianS = 0.05,
            OffsetMinS = 0.01,
            OffsetMaxS = 0.1,
            OffsetAppliedS = 0.075,
            OffsetTargetS = 0.08,
        };

        Assert.IsTrue(status.Synchronized);
        Assert.AreEqual(50, status.RttMs);
        Assert.AreEqual(0.025, status.LastDelayS);
        Assert.AreEqual(0.123, status.OffsetS);
        Assert.AreEqual(25.5, status.SkewPpm);
        Assert.AreEqual(1234567890L, status.LastUpdate.Sec);
        Assert.AreEqual(42, status.Samples);
        Assert.AreEqual(Status.CorrectionType.Slew, status.LastCorrection);
        Assert.AreEqual(0.005, status.LastCorrectionAmountS);
        Assert.IsTrue(status.EpochChanged);
        Assert.AreEqual(12345u, status.Epoch);
        Assert.AreEqual("Test error", status.LastError);
        Assert.AreEqual(5, status.OffsetWindow);
        Assert.AreEqual(20, status.SkewWindow);
        Assert.AreEqual(10, status.WindowCount);
        Assert.AreEqual(0.05, status.OffsetMedianS);
        Assert.AreEqual(0.01, status.OffsetMinS);
        Assert.AreEqual(0.1, status.OffsetMaxS);
        Assert.AreEqual(0.075, status.OffsetAppliedS);
        Assert.AreEqual(0.08, status.OffsetTargetS);
    }

    [TestMethod]
    public void NegativeValues_AreAllowed()
    {
        var status = new Status
        {
            RttMs = -10,
            LastDelayS = -0.5,
            OffsetS = -0.123,
            SkewPpm = -25.5,
            Samples = -5,
            LastCorrectionAmountS = -0.1,
            OffsetWindow = -3,
            SkewWindow = -10,
            WindowCount = -7,
            OffsetMedianS = -0.05,
            OffsetMinS = -0.1,
            OffsetMaxS = -0.01,
            OffsetAppliedS = -0.075,
            OffsetTargetS = -0.08,
        };

        Assert.AreEqual(-10, status.RttMs);
        Assert.AreEqual(-0.5, status.LastDelayS);
        Assert.AreEqual(-0.123, status.OffsetS);
        Assert.AreEqual(-25.5, status.SkewPpm);
        Assert.AreEqual(-5, status.Samples);
        Assert.AreEqual(-0.1, status.LastCorrectionAmountS);
        Assert.AreEqual(-3, status.OffsetWindow);
        Assert.AreEqual(-10, status.SkewWindow);
        Assert.AreEqual(-7, status.WindowCount);
        Assert.AreEqual(-0.05, status.OffsetMedianS);
        Assert.AreEqual(-0.1, status.OffsetMinS);
        Assert.AreEqual(-0.01, status.OffsetMaxS);
        Assert.AreEqual(-0.075, status.OffsetAppliedS);
        Assert.AreEqual(-0.08, status.OffsetTargetS);
    }

    [TestMethod]
    public void LastError_EmptyByDefault()
    {
        var status = new Status();

        Assert.AreEqual(string.Empty, status.LastError);
        Assert.AreEqual(0, status.LastError.Length);
    }

    [TestMethod]
    public void LastError_CanBeCleared()
    {
        var status = new Status { LastError = "Some error" };

        status.LastError = string.Empty;

        Assert.AreEqual(string.Empty, status.LastError);
    }
}
