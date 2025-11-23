using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;

namespace NtpClock.Tests;

[TestClass]
public class SyncEstimatorStateTests
{
    [TestMethod]
    public void Constructor_CreatesEmptyState()
    {
        var state = new SyncEstimatorState();
        Assert.AreEqual(0, state.GetSampleCount());
    }

    [TestMethod]
    public void AddSample_IncrementsSampleCount()
    {
        var state = new SyncEstimatorState();
        state.AddSample(0.1, 1000.0, 10);
        Assert.AreEqual(1, state.GetSampleCount());
    }

    [TestMethod]
    public void AddSample_MaintainsWindowSize()
    {
        var state = new SyncEstimatorState();

        // Add samples beyond window size
        for (int i = 0; i < 15; i++)
        {
            state.AddSample(i * 0.01, 1000.0 + i, 10);
        }

        Assert.AreEqual(10, state.GetSampleCount());
    }

    [TestMethod]
    public void AddSample_RemovesOldestWhenWindowExceeded()
    {
        var state = new SyncEstimatorState();

        // Add 5 samples with window size 3
        state.AddSample(1.0, 1000.0, 3);
        state.AddSample(2.0, 1001.0, 3);
        state.AddSample(3.0, 1002.0, 3);
        state.AddSample(4.0, 1003.0, 3);
        state.AddSample(5.0, 1004.0, 3);

        Assert.AreEqual(3, state.GetSampleCount());

        // Median should be from most recent 3 samples: 3.0, 4.0, 5.0
        var stats = state.ComputeOffsetStats(3, 0.0);
        Assert.AreEqual(4.0, stats.Median, 1e-9);
    }

    [TestMethod]
    public void AddSample_HandlesZeroWindow()
    {
        var state = new SyncEstimatorState();
        state.AddSample(0.1, 1000.0, 0);

        // Window size 0 is clamped to 1
        Assert.AreEqual(1, state.GetSampleCount());
    }

    [TestMethod]
    public void AddSample_HandlesNegativeWindow()
    {
        var state = new SyncEstimatorState();
        state.AddSample(0.1, 1000.0, -5);

        // Negative window is clamped to 1
        Assert.AreEqual(1, state.GetSampleCount());
    }

    [TestMethod]
    public void ComputeOffsetStats_EmptyState_ReturnsFallback()
    {
        var state = new SyncEstimatorState();
        var stats = state.ComputeOffsetStats(5, 42.0);

        Assert.AreEqual(42.0, stats.Median);
        Assert.AreEqual(42.0, stats.Min);
        Assert.AreEqual(42.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_SingleSample_ReturnsValue()
    {
        var state = new SyncEstimatorState();
        state.AddSample(3.5, 1000.0, 10);

        var stats = state.ComputeOffsetStats(5, 0.0);

        Assert.AreEqual(3.5, stats.Median);
        Assert.AreEqual(3.5, stats.Min);
        Assert.AreEqual(3.5, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_OddCount_ReturnsMiddleMedian()
    {
        var state = new SyncEstimatorState();

        // Add 5 samples: 1, 2, 3, 4, 5
        for (int i = 1; i <= 5; i++)
        {
            state.AddSample(i * 1.0, 1000.0 + i, 10);
        }

        var stats = state.ComputeOffsetStats(5, 0.0);

        // Median of [1, 2, 3, 4, 5] is 3 (index 2)
        Assert.AreEqual(3.0, stats.Median);
        Assert.AreEqual(1.0, stats.Min);
        Assert.AreEqual(5.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_EvenCount_ReturnsUpperMiddleMedian()
    {
        var state = new SyncEstimatorState();

        // Add 4 samples: 1, 2, 3, 4
        for (int i = 1; i <= 4; i++)
        {
            state.AddSample(i * 1.0, 1000.0 + i, 10);
        }

        var stats = state.ComputeOffsetStats(4, 0.0);

        // Median of [1, 2, 3, 4] is arr[2] = 3 (upper middle for even count)
        Assert.AreEqual(3.0, stats.Median);
        Assert.AreEqual(1.0, stats.Min);
        Assert.AreEqual(4.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_UsesOnlyRecentWindow()
    {
        var state = new SyncEstimatorState();

        // Add 10 samples: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
        for (int i = 1; i <= 10; i++)
        {
            state.AddSample(i * 1.0, 1000.0 + i, 20);
        }

        // Request window of 5 - should use only last 5: 6, 7, 8, 9, 10
        var stats = state.ComputeOffsetStats(5, 0.0);

        Assert.AreEqual(8.0, stats.Median); // Middle of [6,7,8,9,10]
        Assert.AreEqual(6.0, stats.Min);
        Assert.AreEqual(10.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_UnsortedData_SortsCorrectly()
    {
        var state = new SyncEstimatorState();

        // Add unsorted samples
        double[] offsets = { 5.0, 2.0, 8.0, 1.0, 9.0 };
        for (int i = 0; i < offsets.Length; i++)
        {
            state.AddSample(offsets[i], 1000.0 + i, 10);
        }

        var stats = state.ComputeOffsetStats(5, 0.0);

        // Sorted: [1, 2, 5, 8, 9], median is 5 (index 2)
        Assert.AreEqual(5.0, stats.Median);
        Assert.AreEqual(1.0, stats.Min);
        Assert.AreEqual(9.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_NegativeValues_HandledCorrectly()
    {
        var state = new SyncEstimatorState();

        double[] offsets = { -3.0, -1.0, 0.0, 1.0, 3.0 };
        for (int i = 0; i < offsets.Length; i++)
        {
            state.AddSample(offsets[i], 1000.0 + i, 10);
        }

        var stats = state.ComputeOffsetStats(5, 0.0);

        Assert.AreEqual(0.0, stats.Median);
        Assert.AreEqual(-3.0, stats.Min);
        Assert.AreEqual(3.0, stats.Max);
    }

    [TestMethod]
    public void ComputeOffsetStats_ZeroWindow_ClampsToOne()
    {
        var state = new SyncEstimatorState();
        state.AddSample(5.0, 1000.0, 10);
        state.AddSample(10.0, 1001.0, 10);

        var stats = state.ComputeOffsetStats(0, 0.0);

        // Window 0 is clamped to 1, should return last sample
        Assert.AreEqual(10.0, stats.Median);
    }

    [TestMethod]
    public void ComputeSkewPpm_EmptyState_ReturnsZero()
    {
        var state = new SyncEstimatorState();
        double skew = state.ComputeSkewPpm(20);

        Assert.AreEqual(0.0, skew);
    }

    [TestMethod]
    public void ComputeSkewPpm_OneSample_ReturnsZero()
    {
        var state = new SyncEstimatorState();
        state.AddSample(0.1, 1000.0, 20);

        double skew = state.ComputeSkewPpm(20);

        Assert.AreEqual(0.0, skew);
    }

    [TestMethod]
    public void ComputeSkewPpm_NoVariance_ReturnsZero()
    {
        var state = new SyncEstimatorState();

        // Add samples at same time (no time variance)
        state.AddSample(1.0, 1000.0, 20);
        state.AddSample(2.0, 1000.0, 20);
        state.AddSample(3.0, 1000.0, 20);

        double skew = state.ComputeSkewPpm(20);

        Assert.AreEqual(0.0, skew);
    }

    [TestMethod]
    public void ComputeSkewPpm_PositiveSlope_ReturnsPositivePpm()
    {
        var state = new SyncEstimatorState();

        // Add samples with positive slope: offset increases over time
        // offset = 0.001 * time (1ms per second = 1000 ppm)
        for (int i = 0; i < 10; i++)
        {
            double time = 1000.0 + i;
            double offset = 0.001 * i;
            state.AddSample(offset, time, 20);
        }

        double skew = state.ComputeSkewPpm(20);

        // Expected: 0.001 s/s = 1000 ppm
        Assert.AreEqual(1000.0, skew, 1.0);
    }

    [TestMethod]
    public void ComputeSkewPpm_NegativeSlope_ReturnsNegativePpm()
    {
        var state = new SyncEstimatorState();

        // Add samples with negative slope: offset decreases over time
        // offset = -0.0005 * time (-0.5ms per second = -500 ppm)
        for (int i = 0; i < 10; i++)
        {
            double time = 1000.0 + i;
            double offset = -0.0005 * i;
            state.AddSample(offset, time, 20);
        }

        double skew = state.ComputeSkewPpm(20);

        // Expected: -0.0005 s/s = -500 ppm
        Assert.AreEqual(-500.0, skew, 1.0);
    }

    [TestMethod]
    public void ComputeSkewPpm_UsesOnlyRecentWindow()
    {
        var state = new SyncEstimatorState();

        // Add 20 samples with one slope
        for (int i = 0; i < 20; i++)
        {
            state.AddSample(0.001 * i, 1000.0 + i, 30);
        }

        // Add 10 more samples with different slope
        for (int i = 0; i < 10; i++)
        {
            state.AddSample(0.02 + 0.0001 * i, 1020.0 + i, 30);
        }

        // Request window of 5 - should use only last 5 samples
        double skew = state.ComputeSkewPpm(5);

        // Last 5 samples have slope 0.0001 s/s = 100 ppm
        Assert.AreEqual(100.0, skew, 10.0);
    }

    [TestMethod]
    public void ComputeSkewPpm_ZeroWindow_ClampsToOne()
    {
        var state = new SyncEstimatorState();

        state.AddSample(0.0, 1000.0, 20);
        state.AddSample(0.001, 1001.0, 20);

        double skew = state.ComputeSkewPpm(0);

        // Window 0 clamped to 1, uses last 1 sample - needs at least 2 for skew
        Assert.AreEqual(0.0, skew);
    }

    [TestMethod]
    public void ComputeSkewPpm_AccurateRegression()
    {
        var state = new SyncEstimatorState();

        // Generate precise linear relationship: offset = 0.0002 * (time - 1000)
        // Slope = 0.0002 s/s = 200 ppm
        for (int i = 0; i < 20; i++)
        {
            double time = 1000.0 + i * 10.0;
            double offset = 0.0002 * (time - 1000.0);
            state.AddSample(offset, time, 25);
        }

        double skew = state.ComputeSkewPpm(20);

        Assert.AreEqual(200.0, skew, 0.1);
    }

    [TestMethod]
    public void Clear_RemovesAllSamples()
    {
        var state = new SyncEstimatorState();

        // Add samples
        for (int i = 0; i < 10; i++)
        {
            state.AddSample(i * 0.01, 1000.0 + i, 20);
        }

        Assert.AreEqual(10, state.GetSampleCount());

        state.Clear();

        Assert.AreEqual(0, state.GetSampleCount());
    }

    [TestMethod]
    public void Clear_ResetsToInitialState()
    {
        var state = new SyncEstimatorState();

        state.AddSample(5.0, 1000.0, 20);
        state.AddSample(10.0, 1001.0, 20);
        state.Clear();

        var stats = state.ComputeOffsetStats(5, 99.0);

        Assert.AreEqual(99.0, stats.Median); // Should use fallback
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentAddAndRead()
    {
        var state = new SyncEstimatorState();
        const int iterations = 1000;
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < iterations; i++)
                {
                    state.AddSample(i * 0.001, 1000.0 + i, 100);
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
                for (int i = 0; i < iterations; i++)
                {
                    _ = state.GetSampleCount();
                    _ = state.ComputeOffsetStats(50, 0.0);
                    _ = state.ComputeSkewPpm(20);
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
    public void ThreadSafety_ConcurrentClear()
    {
        var state = new SyncEstimatorState();
        bool error = false;

        var t1 = new Thread(() =>
        {
            try
            {
                for (int i = 0; i < 100; i++)
                {
                    state.AddSample(i * 0.001, 1000.0 + i, 50);
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
                for (int i = 0; i < 10; i++)
                {
                    Thread.Sleep(5);
                    state.Clear();
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
}
