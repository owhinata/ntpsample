// Copyright (c) 2025 The NTP Sample Authors
namespace NtpClock.Internal;

/// <summary>
/// Thread-safe storage for offset/time samples with sliding window.
/// Manages vectors of offset and time samples and provides robust statistics
/// computation (median, min, max) and skew estimation (OLS regression).
/// Automatically maintains window size and provides synchronized access for
/// multi-threaded environments.
/// </summary>
public class SyncEstimatorState
{
    /// <summary>Computed offset statistics.</summary>
    public struct Stats
    {
        /// <summary>Median offset in seconds.</summary>
        public double Median;

        /// <summary>Minimum offset in window (seconds).</summary>
        public double Min;

        /// <summary>Maximum offset in window (seconds).</summary>
        public double Max;
    }

    private readonly object _lock = new object();
    private readonly List<double> _offsets = new List<double>();
    private readonly List<double> _times = new List<double>();

    /// <summary>
    /// Add a new sample pair to the sliding windows.
    /// Appends samples and removes oldest if window size is exceeded.
    /// Thread-safe.
    /// </summary>
    /// <param name="offsetS">Offset measurement in seconds.</param>
    /// <param name="timeS">Time of measurement (UNIX seconds).</param>
    /// <param name="maxWindow">Maximum window size (keeps max of offset/skew windows).</param>
    public void AddSample(double offsetS, double timeS, int maxWindow)
    {
        lock (_lock)
        {
            _offsets.Add(offsetS);
            _times.Add(timeS);

            int maxW = Math.Max(1, maxWindow);

            // Trim oldest samples if window exceeded
            while (_offsets.Count > maxW)
            {
                _offsets.RemoveAt(0);
            }
            while (_times.Count > maxW)
            {
                _times.RemoveAt(0);
            }
        }
    }

    /// <summary>
    /// Compute offset statistics from current samples.
    /// Selects the most recent samples up to the window size, computes
    /// the median via sorting and nth element selection, and finds min/max.
    /// Thread-safe.
    /// </summary>
    /// <param name="window">Window size for offset computation.</param>
    /// <param name="fallback">Fallback value when no samples available.</param>
    /// <returns>Stats with median, min, max.</returns>
    public Stats ComputeOffsetStats(int window, double fallback)
    {
        lock (_lock)
        {
            Stats result = new Stats
            {
                Median = fallback,
                Min = fallback,
                Max = fallback,
            };

            int n = _offsets.Count;
            if (n == 0)
                return result;

            int win = Math.Max(1, window);
            int start = Math.Max(0, n - win);

            // Copy relevant window
            var tmp = new double[n - start];
            for (int i = start; i < n; i++)
            {
                tmp[i - start] = _offsets[i];
            }

            if (tmp.Length == 0)
                return result;

            // Compute median
            Array.Sort(tmp);
            result.Median = tmp[tmp.Length / 2];

            // Compute min/max
            result.Min = tmp[0];
            result.Max = tmp[tmp.Length - 1];

            return result;
        }
    }

    /// <summary>
    /// Compute clock skew from current samples.
    /// Applies OLS regression: slope = Cov(time, offset) / Var(time).
    /// Uses only the most recent samples within the specified window size.
    /// Returns 0.0 if fewer than 2 samples or if variance is zero.
    /// Thread-safe.
    /// </summary>
    /// <param name="window">Window size for skew computation.</param>
    /// <returns>Estimated skew in parts per million (PPM), or 0.0 on failure.</returns>
    public double ComputeSkewPpm(int window)
    {
        lock (_lock)
        {
            if (_times.Count < 2 || _times.Count != _offsets.Count)
            {
                return 0.0;
            }

            int n = Math.Min(_times.Count, Math.Max(1, window));

            // Compute means
            double meanT = 0.0;
            double meanO = 0.0;
            int start = _times.Count - n;
            for (int i = start; i < _times.Count; i++)
            {
                meanT += _times[i];
                meanO += _offsets[i];
            }
            meanT /= n;
            meanO /= n;

            // OLS regression: slope = Cov(t, o) / Var(t)
            double num = 0.0;
            double den = 0.0;
            for (int i = start; i < _times.Count; i++)
            {
                double dt = _times[i] - meanT;
                double doff = _offsets[i] - meanO;
                num += dt * doff;
                den += dt * dt;
            }

            double slope = (den > 0.0) ? (num / den) : 0.0; // sec offset per sec
            return slope * 1e6; // convert to ppm
        }
    }

    /// <summary>
    /// Get current number of samples in the window.
    /// Thread-safe.
    /// </summary>
    /// <returns>Number of samples stored.</returns>
    public int GetSampleCount()
    {
        lock (_lock)
        {
            return _offsets.Count;
        }
    }

    /// <summary>
    /// Clear all stored samples.
    /// Typically called when estimator reset is needed (e.g., after
    /// vendor hint application or configuration change).
    /// Thread-safe.
    /// </summary>
    public void Clear()
    {
        lock (_lock)
        {
            _offsets.Clear();
            _times.Clear();
        }
    }
}
