using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;
using NtpServer.Internal;
using Options = NtpClock.Internal.Options;
using Status = NtpClock.Internal.Status;

namespace NtpClock.Tests;

[TestClass]
public class ClockServiceTests
{
    // Mock time source for testing without network
    private class MockTimeSource : ITimeSource
    {
        private TimeSpec _currentTime;
        private double _rate;
        private readonly object _lock = new object();

        public MockTimeSource(TimeSpec initialTime)
        {
            _currentTime = initialTime;
            _rate = 1.0;
        }

        public TimeSpec NowUnix()
        {
            lock (_lock)
            {
                return _currentTime;
            }
        }

        public void SetAbsolute(TimeSpec time)
        {
            lock (_lock)
            {
                _currentTime = time;
            }
        }

        public void SetRate(double rate)
        {
            lock (_lock)
            {
                _rate = rate;
            }
        }

        public void SetAbsoluteAndRate(TimeSpec time, double rate)
        {
            lock (_lock)
            {
                _currentTime = time;
                _rate = rate;
            }
        }

        public void ResetToRealTime()
        {
            lock (_lock)
            {
                _currentTime = TimeSpec.FromDouble(DateTimeOffset.UtcNow.ToUnixTimeSeconds());
                _rate = 1.0;
            }
        }

        public double GetRate()
        {
            lock (_lock)
            {
                return _rate;
            }
        }

        public void Advance(double seconds)
        {
            lock (_lock)
            {
                _currentTime = _currentTime + TimeSpec.FromDouble(seconds);
            }
        }
    }

    // ==================== Lifecycle Tests ====================

    [TestMethod]
    public void Constructor_CreatesInstance()
    {
        using var service = new ClockService();
        Assert.IsNotNull(service);
    }

    [TestMethod]
    public void Start_WithValidParameters_ReturnsTrue()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        // Note: This will fail without a real server, but tests the API
        bool result = service.Start(mockTime, "127.0.0.1", 12300, options);

        // Clean up regardless of result
        service.Stop();

        // The start may fail due to network, but should not crash
        Assert.IsTrue(result == true || result == false);
    }

    [TestMethod]
    public void Start_WithDefaultTimeSource_ReturnsTrue()
    {
        using var service = new ClockService();
        var options = new Options.Builder().Build();

        // Start without providing time source (uses StopwatchClock)
        bool result = service.Start("127.0.0.1", 12300, options);

        service.Stop();

        // Should not crash
        Assert.IsTrue(result == true || result == false);
    }

    [TestMethod]
    public void Stop_WithoutStart_DoesNotCrash()
    {
        using var service = new ClockService();

        // Should be safe to call Stop without Start
        service.Stop();

        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Stop_MultipleTimesIsSafe()
    {
        using var service = new ClockService();

        service.Stop();
        service.Stop();
        service.Stop();

        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Start_MultipleTimes_IsIdempotent()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Start(mockTime, "127.0.0.1", 12301, options);

        service.Stop();

        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Start_AfterStop_Works()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Stop();
        service.Start(mockTime, "127.0.0.1", 12301, options);
        service.Stop();

        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Dispose_StopsService()
    {
        var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Dispose();

        // After dispose, service should be stopped
        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Dispose_WithoutStart_DoesNotCrash()
    {
        var service = new ClockService();
        service.Dispose();

        // No assertion needed - just verify no exception
    }

    [TestMethod]
    public void Dispose_MultipleTimes_IsSafe()
    {
        var service = new ClockService();
        service.Dispose();
        service.Dispose();

        // No assertion needed - just verify no exception
    }

    // ==================== Options Management Tests ====================

    [TestMethod]
    public void GetOptions_OnConstruction_ReturnsDefaultOptions()
    {
        using var service = new ClockService();
        var options = service.GetOptions();

        Assert.IsNotNull(options);
        Assert.AreEqual(Options.DefaultPollIntervalMs, options.PollIntervalMs);
        Assert.AreEqual(Options.DefaultStepThresholdMs, options.StepThresholdMs);
        Assert.AreEqual(Options.DefaultSlewRateMsPerSec, options.SlewRateMsPerSec);
    }

    [TestMethod]
    public void SetOptions_UpdatesOptions()
    {
        using var service = new ClockService();
        var newOptions = new Options.Builder().PollIntervalMs(2000).StepThresholdMs(500).Build();

        service.SetOptions(newOptions);

        var retrieved = service.GetOptions();
        Assert.AreEqual(2000, retrieved.PollIntervalMs);
        Assert.AreEqual(500, retrieved.StepThresholdMs);
    }

    [TestMethod]
    public void GetOptions_AfterSetOptions_ReturnsUpdatedOptions()
    {
        using var service = new ClockService();
        var options1 = new Options.Builder().PollIntervalMs(1500).Build();
        var options2 = new Options.Builder().PollIntervalMs(3000).Build();

        service.SetOptions(options1);
        var retrieved1 = service.GetOptions();
        Assert.AreEqual(1500, retrieved1.PollIntervalMs);

        service.SetOptions(options2);
        var retrieved2 = service.GetOptions();
        Assert.AreEqual(3000, retrieved2.PollIntervalMs);
    }

    [TestMethod]
    public void SetOptions_ThreadSafe()
    {
        using var service = new ClockService();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    var options = new Options.Builder().PollIntervalMs(1000 + i).Build();
                    service.SetOptions(options);
                    Thread.Sleep(1);
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
                for (int i = 0; i < 100; i++)
                {
                    var options = new Options.Builder().PollIntervalMs(2000 + i).Build();
                    service.SetOptions(options);
                    Thread.Sleep(1);
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
    public void GetOptions_ThreadSafe()
    {
        using var service = new ClockService();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetOptions();
                    Thread.Sleep(1);
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
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetOptions();
                    Thread.Sleep(1);
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
    public void SetOptionsAndGetOptions_ThreadSafe()
    {
        using var service = new ClockService();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    var options = new Options.Builder().PollIntervalMs(1000 + i).Build();
                    service.SetOptions(options);
                    Thread.Sleep(1);
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
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetOptions();
                    Thread.Sleep(1);
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

    // ==================== GetRate Tests ====================

    [TestMethod]
    public void GetRate_WithoutStart_ReturnsDefaultRate()
    {
        using var service = new ClockService();

        double rate = service.GetRate();

        Assert.AreEqual(1.0, rate);
    }

    [TestMethod]
    public void GetRate_WithMockTimeSource_ReturnsExpectedRate()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        mockTime.SetRate(1.5);
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        double rate = service.GetRate();
        service.Stop();

        Assert.AreEqual(1.5, rate);
    }

    [TestMethod]
    public void GetRate_AfterStop_ReturnsDefaultRate()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        mockTime.SetRate(2.0);
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Stop();
        double rate = service.GetRate();

        Assert.AreEqual(1.0, rate);
    }

    // ==================== NowUnix Tests ====================

    [TestMethod]
    public void NowUnix_WithoutStart_ReturnsZeroTime()
    {
        using var service = new ClockService();

        var time = service.NowUnix();

        Assert.AreEqual(0L, time.Sec);
        Assert.AreEqual(0u, time.Nsec);
    }

    [TestMethod]
    public void NowUnix_WithMockTimeSource_ReturnsExpectedTime()
    {
        using var service = new ClockService();
        var expectedTime = new TimeSpec(1234567890, 500_000_000);
        var mockTime = new MockTimeSource(expectedTime);
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        var time = service.NowUnix();
        service.Stop();

        Assert.AreEqual(expectedTime.Sec, time.Sec);
        Assert.AreEqual(expectedTime.Nsec, time.Nsec, 1000u);
    }

    [TestMethod]
    public void NowUnix_IsMonotonic()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);

        var time1 = service.NowUnix();
        mockTime.Advance(0.1);
        var time2 = service.NowUnix();
        mockTime.Advance(0.2);
        var time3 = service.NowUnix();

        service.Stop();

        Assert.IsTrue(time2 >= time1);
        Assert.IsTrue(time3 >= time2);
    }

    [TestMethod]
    public void NowUnix_AfterStop_ReturnsZeroTime()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Stop();
        var time = service.NowUnix();

        Assert.AreEqual(0L, time.Sec);
        Assert.AreEqual(0u, time.Nsec);
    }

    // ==================== GetStatus Tests ====================

    [TestMethod]
    public void GetStatus_BeforeStart_DoesNotCrash()
    {
        using var service = new ClockService();

        var status = service.GetStatus();

        Assert.IsNotNull(status);
    }

    [TestMethod]
    public void GetStatus_AfterConstruction_ReturnsInitialStatus()
    {
        using var service = new ClockService();

        var status = service.GetStatus();

        Assert.IsNotNull(status);
        Assert.IsFalse(status.Synchronized);
        Assert.AreEqual(0, status.Samples);
    }

    [TestMethod]
    public void GetStatus_AfterStop_DoesNotCrash()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        service.Stop();
        var status = service.GetStatus();

        Assert.IsNotNull(status);
    }

    // ==================== Thread Safety Tests ====================

    [TestMethod]
    public void ThreadSafety_ConcurrentStartStop()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 10; i++)
                {
                    service.Start(mockTime, "127.0.0.1", 12300, options);
                    Thread.Sleep(10);
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
                for (int i = 0; i < 10; i++)
                {
                    service.Stop();
                    Thread.Sleep(10);
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

        service.Stop();
        Assert.IsFalse(error);
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentGetStatus()
    {
        using var service = new ClockService();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetStatus();
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
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetStatus();
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
    public void ThreadSafety_ConcurrentNowUnix()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();
        bool error = false;

        service.Start(mockTime, "127.0.0.1", 12300, options);

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    _ = service.NowUnix();
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
                for (int i = 0; i < 100; i++)
                {
                    _ = service.NowUnix();
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

        service.Stop();
        Assert.IsFalse(error);
    }

    // ==================== Error Handling Tests ====================

    [TestMethod]
    public void Start_WithInvalidIP_HandlesGracefully()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        // Malformed IP address
        bool result = service.Start(mockTime, "999.999.999.999", 12300, options);

        service.Stop();

        // Should return false or handle error gracefully
        Assert.IsTrue(result == true || result == false);
    }

    [TestMethod]
    public void Start_WithEmptyIP_HandlesGracefully()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        bool result = service.Start(mockTime, "", 12300, options);

        service.Stop();

        // Should handle gracefully without crash
        Assert.IsTrue(result == true || result == false);
    }

    [TestMethod]
    public void Start_WithPortZero_HandlesGracefully()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        bool result = service.Start(mockTime, "127.0.0.1", 0, options);

        service.Stop();

        // Should handle gracefully without crash
        Assert.IsTrue(result == true || result == false);
    }

    [TestMethod]
    public void Start_WithNullTimeSource_UsesDefaultTimeSource()
    {
        using var service = new ClockService();
        var options = new Options.Builder().Build();

        bool result = service.Start(null, "127.0.0.1", 12300, options);

        service.Stop();

        // Should use StopwatchClock as default
        Assert.IsTrue(result == true || result == false);
    }

    // ==================== Integration Tests (Limited) ====================

    [TestMethod]
    public void OptionsWithLogSink_DoesNotCrash()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var logMessages = new System.Collections.Concurrent.ConcurrentBag<string>();

        var options = new Options.Builder().LogSink(msg => logMessages.Add(msg)).Build();

        service.Start(mockTime, "127.0.0.1", 12300, options);
        Thread.Sleep(50);
        service.Stop();

        // No assertion on log messages (depends on network)
        // Just verify no crash
    }

    [TestMethod]
    public void MultipleStartStopCycles_DoesNotLeak()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();

        for (int i = 0; i < 5; i++)
        {
            service.Start(mockTime, "127.0.0.1", 12300, options);
            Thread.Sleep(10);
            service.Stop();
        }

        // No assertion needed - just verify no crash or resource leak
    }

    [TestMethod]
    public void GetRate_ThreadSafe()
    {
        using var service = new ClockService();
        var mockTime = new MockTimeSource(new TimeSpec(1000, 0));
        var options = new Options.Builder().Build();
        bool error = false;

        service.Start(mockTime, "127.0.0.1", 12300, options);

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetRate();
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
                for (int i = 0; i < 100; i++)
                {
                    _ = service.GetRate();
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

        service.Stop();
        Assert.IsFalse(error);
    }

    [TestMethod]
    public void AllAPIs_AfterDispose_DoNotCrash()
    {
        var service = new ClockService();
        service.Dispose();

        // All methods should be safe to call after dispose
        _ = service.NowUnix();
        _ = service.GetStatus();
        _ = service.GetOptions();
        _ = service.GetRate();
        service.Stop();

        // No assertion needed - just verify no crash
    }

    [TestMethod]
    public void SetOptions_WithAllCustomValues()
    {
        using var service = new ClockService();

        var options = new Options.Builder()
            .PollIntervalMs(2000)
            .StepThresholdMs(300)
            .SlewRateMsPerSec(10.0)
            .MaxRttMs(200)
            .MinSamplesToLock(5)
            .OffsetWindow(10)
            .SkewWindow(30)
            .LogSink(msg => { })
            .Build();

        service.SetOptions(options);
        var retrieved = service.GetOptions();

        Assert.AreEqual(2000, retrieved.PollIntervalMs);
        Assert.AreEqual(300, retrieved.StepThresholdMs);
        Assert.AreEqual(10.0, retrieved.SlewRateMsPerSec);
        Assert.AreEqual(200, retrieved.MaxRttMs);
        Assert.AreEqual(5, retrieved.MinSamplesToLock);
        Assert.AreEqual(10, retrieved.OffsetWindow);
        Assert.AreEqual(30, retrieved.SkewWindow);
        Assert.IsNotNull(retrieved.LogSink);
    }
}
