using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;

namespace NtpClock.Tests;

[TestClass]
public class OptionsTests
{
    [TestMethod]
    public void DefaultBuilder_CreatesOptionsWithDefaultValues()
    {
        var builder = new Options.Builder();
        var options = builder.Build();

        Assert.AreEqual(Options.DefaultPollIntervalMs, options.PollIntervalMs);
        Assert.AreEqual(Options.DefaultStepThresholdMs, options.StepThresholdMs);
        Assert.AreEqual(Options.DefaultSlewRateMsPerSec, options.SlewRateMsPerSec);
        Assert.AreEqual(Options.DefaultMaxRttMs, options.MaxRttMs);
        Assert.AreEqual(Options.DefaultMinSamplesToLock, options.MinSamplesToLock);
        Assert.AreEqual(Options.DefaultOffsetWindow, options.OffsetWindow);
        Assert.AreEqual(Options.DefaultSkewWindow, options.SkewWindow);
        Assert.IsNull(options.LogSink);
    }

    [TestMethod]
    public void Builder_SetPollIntervalMs_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.PollIntervalMs(2000).Build();

        Assert.AreEqual(2000, options.PollIntervalMs);
    }

    [TestMethod]
    public void Builder_SetStepThresholdMs_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.StepThresholdMs(500).Build();

        Assert.AreEqual(500, options.StepThresholdMs);
    }

    [TestMethod]
    public void Builder_SetSlewRateMsPerSec_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.SlewRateMsPerSec(10.0).Build();

        Assert.AreEqual(10.0, options.SlewRateMsPerSec);
    }

    [TestMethod]
    public void Builder_SetMaxRttMs_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.MaxRttMs(200).Build();

        Assert.AreEqual(200, options.MaxRttMs);
    }

    [TestMethod]
    public void Builder_SetMinSamplesToLock_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.MinSamplesToLock(5).Build();

        Assert.AreEqual(5, options.MinSamplesToLock);
    }

    [TestMethod]
    public void Builder_SetOffsetWindow_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.OffsetWindow(10).Build();

        Assert.AreEqual(10, options.OffsetWindow);
    }

    [TestMethod]
    public void Builder_SetSkewWindow_SetsValue()
    {
        var builder = new Options.Builder();
        var options = builder.SkewWindow(30).Build();

        Assert.AreEqual(30, options.SkewWindow);
    }

    [TestMethod]
    public void Builder_SetLogSink_SetsValue()
    {
        var builder = new Options.Builder();
        bool called = false;
        Options.LogCallback callback = (msg) =>
        {
            called = true;
        };
        var options = builder.LogSink(callback).Build();

        Assert.IsNotNull(options.LogSink);
        options.LogSink("test");
        Assert.IsTrue(called);
    }

    [TestMethod]
    public void Builder_ChainedCalls_SetsAllValues()
    {
        var builder = new Options.Builder();
        bool logCalled = false;
        Options.LogCallback callback = (msg) =>
        {
            logCalled = true;
        };

        var options = builder
            .PollIntervalMs(3000)
            .StepThresholdMs(400)
            .SlewRateMsPerSec(7.5)
            .MaxRttMs(150)
            .MinSamplesToLock(4)
            .OffsetWindow(8)
            .SkewWindow(25)
            .LogSink(callback)
            .Build();

        Assert.AreEqual(3000, options.PollIntervalMs);
        Assert.AreEqual(400, options.StepThresholdMs);
        Assert.AreEqual(7.5, options.SlewRateMsPerSec);
        Assert.AreEqual(150, options.MaxRttMs);
        Assert.AreEqual(4, options.MinSamplesToLock);
        Assert.AreEqual(8, options.OffsetWindow);
        Assert.AreEqual(25, options.SkewWindow);
        Assert.IsNotNull(options.LogSink);
        options.LogSink("test");
        Assert.IsTrue(logCalled);
    }

    [TestMethod]
    public void Builder_FromExistingOptions_CopiesAllValues()
    {
        bool logCalled = false;
        Options.LogCallback callback = (msg) =>
        {
            logCalled = true;
        };

        var original = new Options.Builder()
            .PollIntervalMs(2500)
            .StepThresholdMs(300)
            .SlewRateMsPerSec(6.0)
            .MaxRttMs(120)
            .MinSamplesToLock(6)
            .OffsetWindow(7)
            .SkewWindow(22)
            .LogSink(callback)
            .Build();

        var builder = new Options.Builder(original);
        var copy = builder.Build();

        Assert.AreEqual(original.PollIntervalMs, copy.PollIntervalMs);
        Assert.AreEqual(original.StepThresholdMs, copy.StepThresholdMs);
        Assert.AreEqual(original.SlewRateMsPerSec, copy.SlewRateMsPerSec);
        Assert.AreEqual(original.MaxRttMs, copy.MaxRttMs);
        Assert.AreEqual(original.MinSamplesToLock, copy.MinSamplesToLock);
        Assert.AreEqual(original.OffsetWindow, copy.OffsetWindow);
        Assert.AreEqual(original.SkewWindow, copy.SkewWindow);
        Assert.IsNotNull(copy.LogSink);
        copy.LogSink("test");
        Assert.IsTrue(logCalled);
    }

    [TestMethod]
    public void Builder_FromExistingOptions_CanOverrideValues()
    {
        var original = new Options.Builder().PollIntervalMs(1500).MaxRttMs(100).Build();

        var modified = new Options.Builder(original).PollIntervalMs(3500).Build();

        Assert.AreEqual(3500, modified.PollIntervalMs);
        Assert.AreEqual(100, modified.MaxRttMs); // Unchanged from original
    }

    [TestMethod]
    public void Options_AreImmutable()
    {
        var builder = new Options.Builder();
        var options1 = builder.PollIntervalMs(1000).Build();
        var options2 = builder.PollIntervalMs(2000).Build();

        // First built options should not be affected by subsequent changes
        Assert.AreEqual(1000, options1.PollIntervalMs);
        Assert.AreEqual(2000, options2.PollIntervalMs);
    }

    [TestMethod]
    public void Builder_MultipleBuilds_ProduceSeparateInstances()
    {
        var builder = new Options.Builder().PollIntervalMs(1500);

        var options1 = builder.Build();
        var options2 = builder.Build();

        Assert.AreNotSame(options1, options2);
        Assert.AreEqual(options1.PollIntervalMs, options2.PollIntervalMs);
    }

    [TestMethod]
    public void LogCallback_CanBeNull()
    {
        var builder = new Options.Builder();
        var options = builder.Build();

        Assert.IsNull(options.LogSink);
    }

    [TestMethod]
    public void Builder_ZeroValues_AreAllowed()
    {
        var builder = new Options.Builder();
        var options = builder
            .PollIntervalMs(0)
            .StepThresholdMs(0)
            .SlewRateMsPerSec(0.0)
            .MaxRttMs(0)
            .MinSamplesToLock(0)
            .OffsetWindow(0)
            .SkewWindow(0)
            .Build();

        Assert.AreEqual(0, options.PollIntervalMs);
        Assert.AreEqual(0, options.StepThresholdMs);
        Assert.AreEqual(0.0, options.SlewRateMsPerSec);
        Assert.AreEqual(0, options.MaxRttMs);
        Assert.AreEqual(0, options.MinSamplesToLock);
        Assert.AreEqual(0, options.OffsetWindow);
        Assert.AreEqual(0, options.SkewWindow);
    }

    [TestMethod]
    public void Builder_NegativeValues_AreAllowed()
    {
        var builder = new Options.Builder();
        var options = builder
            .PollIntervalMs(-100)
            .StepThresholdMs(-50)
            .SlewRateMsPerSec(-1.5)
            .MaxRttMs(-20)
            .MinSamplesToLock(-5)
            .OffsetWindow(-10)
            .SkewWindow(-15)
            .Build();

        Assert.AreEqual(-100, options.PollIntervalMs);
        Assert.AreEqual(-50, options.StepThresholdMs);
        Assert.AreEqual(-1.5, options.SlewRateMsPerSec);
        Assert.AreEqual(-20, options.MaxRttMs);
        Assert.AreEqual(-5, options.MinSamplesToLock);
        Assert.AreEqual(-10, options.OffsetWindow);
        Assert.AreEqual(-15, options.SkewWindow);
    }
}
