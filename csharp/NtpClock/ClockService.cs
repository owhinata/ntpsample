// Copyright (c) 2025 <Your Name>
using System.Buffers.Binary;
using NtpClock.Internal;
using NtpServer.Internal;
using Options = NtpClock.Internal.Options;
using Status = NtpClock.Internal.Status;

namespace NtpClock;

/// <summary>
/// NTP-like server synchronized clock service.
///
/// This library provides an application-local clock synchronized to a single
/// NTP-like server over IPv4/UDP. It never changes the OS clock.
///
/// Monotonicity guarantee:
/// - During slew correction, NowUnix() is monotonic non-decreasing.
/// - When a step correction is applied, time may jump backwards exactly once.
///
/// Control background synchronization with Start/Stop. NowUnix() returns the
/// server-synchronized current time as UNIX seconds. Monotonicity is
/// guaranteed during slew, but a single backward jump is allowed at the moment
/// a step correction is applied.
/// </summary>
public class ClockService : IDisposable
{
    private const int NtpPacketSize = 48;

    private readonly SemaphoreSlim _startStopLock = new SemaphoreSlim(1, 1);
    private readonly object _optionsLock = new object();
    private readonly object _statusLock = new object();

    private ITimeSource? _timeSource;
    private Options _options;
    private Status _status = new Status();

    private volatile bool _running;
    private Thread? _workerThread;

    private readonly ClockCorrector _clockCorrector = new ClockCorrector();
    private readonly VendorHintProcessor _vendorHintProcessor = new VendorHintProcessor();
    private readonly SyncEstimatorState _estimatorState = new SyncEstimatorState();
    private readonly UdpSocket _udpSocket = new UdpSocket();

    private volatile bool _exchangeAbort;

    private readonly object _socketErrorLock = new object();
    private string _socketLastError = string.Empty;

    public ClockService()
    {
        _options = new Options.Builder().Build();
    }

    /// <summary>
    /// Start background synchronization.
    /// </summary>
    /// <param name="timeSource">Local time source (must remain valid until Stop).
    /// If null, uses StopwatchClock as default.</param>
    /// <param name="serverIp">IPv4 address in numeric form (no DNS).</param>
    /// <param name="serverPort">UDP port of the server.</param>
    /// <param name="options">Immutable options snapshot.</param>
    /// <returns>true if worker thread started.</returns>
    public bool Start(ITimeSource? timeSource, string serverIp, ushort serverPort, Options options)
    {
        _startStopLock.Wait();
        try
        {
            // Use default time source if null
            if (timeSource == null)
            {
                timeSource = new StopwatchClock();
            }

            Stop();
            _timeSource = timeSource;

            lock (_optionsLock)
            {
                _options = options;
            }

            // Wrap the callback to match VendorHintProcessor.LogCallback signature
            if (options.LogSink != null)
            {
                _vendorHintProcessor.SetLogSink(msg => options.LogSink(msg));
            }

            // Open UDP socket for persistent connection
            TimeSpec GetTime() => _timeSource?.NowUnix() ?? new TimeSpec();
            void LogError(string msg) => ReportSocketError(msg);

            if (!_udpSocket.Open(serverIp, serverPort, GetTime, LogError))
            {
                _timeSource = null;
                return false;
            }

            _running = true;
            _workerThread = new Thread(Loop) { IsBackground = false, Name = "ClockService.Loop" };
            _workerThread.Start();

            return true;
        }
        finally
        {
            _startStopLock.Release();
        }
    }

    /// <summary>
    /// Start background synchronization with default StopwatchClock.
    /// </summary>
    /// <param name="serverIp">IPv4 address in numeric form (no DNS).</param>
    /// <param name="serverPort">UDP port of the server.</param>
    /// <param name="options">Immutable options snapshot.</param>
    /// <returns>true if worker thread started.</returns>
    public bool Start(string serverIp, ushort serverPort, Options options)
    {
        return Start(null, serverIp, serverPort, options);
    }

    /// <summary>
    /// Stop background synchronization.
    /// </summary>
    public void Stop()
    {
        if (!_running)
            return;

        _running = false;
        _udpSocket.Close(); // Close socket first to unblock receive thread

        if (_workerThread != null && _workerThread.IsAlive)
        {
            _workerThread.Join();
        }

        _timeSource = null;
    }

    /// <summary>
    /// Return server-synchronized current time.
    /// </summary>
    /// <remarks>
    /// Monotonic non-decreasing during slew. If a step was just applied,
    /// one backward jump is allowed.
    /// </remarks>
    public TimeSpec NowUnix()
    {
        var ts = _timeSource;
        if (ts == null)
            return new TimeSpec();

        TimeSpec baseTime = ts.NowUnix();

        // ClockCorrector applies offset and enforces monotonicity
        return _clockCorrector.GetMonotonicTime(baseTime);
    }

    /// <summary>
    /// Get current status snapshot.
    /// </summary>
    public Status GetStatus()
    {
        lock (_statusLock)
        {
            return _status;
        }
    }

    /// <summary>
    /// Get current options snapshot.
    /// </summary>
    public Options GetOptions()
    {
        lock (_optionsLock)
        {
            return _options;
        }
    }

    /// <summary>
    /// Atomically replace options with a new immutable snapshot.
    /// Build the new Options via Options.Builder.
    /// </summary>
    /// <param name="options">New options to apply.</param>
    public void SetOptions(Options options)
    {
        lock (_optionsLock)
        {
            _options = options;
        }
    }

    /// <summary>
    /// Get current clock rate from the underlying TimeSource.
    /// </summary>
    /// <returns>Clock rate (1.0 = nominal speed).</returns>
    public double GetRate()
    {
        var ts = _timeSource;
        return ts?.GetRate() ?? 1.0;
    }

    public void Dispose()
    {
        Stop();
        _udpSocket.Dispose();
        _startStopLock.Dispose();
    }

    // ======================== Private Methods ========================

    private void Loop()
    {
        int goodSamples = 0;
        var nextPollTime = DateTime.UtcNow;

        while (_running)
        {
            // 1. Snapshot configuration for this iteration
            Options snapshot;
            lock (_optionsLock)
            {
                snapshot = _options;
            }

            double stepThreshS = snapshot.StepThresholdMs / 1000.0;
            double slewRateSPerS = snapshot.SlewRateMsPerSec / 1000.0;
            int pollIntervalMs = snapshot.PollIntervalMs;

            // 2. Wait for Push or poll deadline
            // If Push is received, returns immediately for instant Exchange execution
            WaitForPushOrPollDeadline(nextPollTime, snapshot, ref goodSamples);

            // Update next poll deadline
            nextPollTime = DateTime.UtcNow.AddMilliseconds(pollIntervalMs);

            // 3. Execute Exchange and build status
            Status stLocal = ProcessExchangeAndBuildStatus(
                snapshot,
                stepThreshS,
                slewRateSPerS,
                pollIntervalMs,
                ref goodSamples
            );

            // 4. Update shared status
            lock (_statusLock)
            {
                _status = stLocal;
            }
        }
    }

    private void WaitForPushOrPollDeadline(
        DateTime nextPollTime,
        Options snapshot,
        ref int goodSamples
    )
    {
        var now = DateTime.UtcNow;

        while (now < nextPollTime)
        {
            var remaining = nextPollTime - now;
            int waitMs = Math.Min((int)remaining.TotalMilliseconds, 100);

            if (waitMs <= 0)
                break;

            if (_udpSocket.WaitMessage(waitMs, out var msg))
            {
                if (msg.Type == UdpSocket.MessageType.Push)
                {
                    if (snapshot.LogSink != null)
                    {
                        snapshot.LogSink(
                            $"[ClockService] Push notification received, resetting sync state (good_samples={goodSamples}->0)"
                        );
                    }
                    HandlePushMessage(msg, snapshot);
                    // Reset good_samples when Push is received (epoch change notification)
                    goodSamples = 0;
                    return;
                }
                // Ignore non-Push messages (unexpected Exchange responses)
            }

            now = DateTime.UtcNow;
        }
    }

    private Status ProcessExchangeAndBuildStatus(
        Options snapshot,
        double stepThreshS,
        double slewRateSPerS,
        int pollIntervalMs,
        ref int goodSamples
    )
    {
        // Acquire NTP sample
        bool ok = UdpExchange(
            out double sampleOffsetS,
            out int sampleRttMs,
            out byte[] response,
            out string err
        );

        // Clear socket error on successful exchange
        if (ok)
        {
            ClearSocketError();
        }

        // Process vendor hint from response if available
        VendorHintResult hintResult = new VendorHintResult();
        if (ok && response.Length > NtpPacketSize)
        {
            hintResult = ApplyVendorHintFromRx(response, snapshot);
        }

        // Build status based on sample validity and vendor hint state
        Status stLocal;
        var ts = _timeSource;
        if (ts == null)
        {
            return new Status { LastError = "time source not set" };
        }

        double tnow = ts.NowUnix().ToDouble();

        if (ok && sampleRttMs <= snapshot.MaxRttMs && hintResult.Applied)
        {
            // Vendor hint applied: reset sync state and skip normal correction
            int oldSamples = goodSamples;
            goodSamples = 0;
            stLocal = BuildVendorHintAppliedStatus(
                snapshot,
                sampleOffsetS,
                sampleRttMs,
                tnow,
                goodSamples,
                hintResult
            );
            if (snapshot.LogSink != null)
            {
                snapshot.LogSink(
                    $"[ClockService] Epoch change detected in exchange, resetting sync state (good_samples={oldSamples}->0)"
                );
            }
        }
        else if (ok && sampleRttMs <= snapshot.MaxRttMs)
        {
            // Normal path: process sample and apply correction
            var (median, omin, omax) = UpdateEstimatorsAndTarget(snapshot, sampleOffsetS, tnow);

            double applied = _clockCorrector.GetOffsetApplied();
            Status.CorrectionType correction = _clockCorrector.Apply(
                applied,
                median,
                stepThreshS,
                slewRateSPerS,
                pollIntervalMs / 1000.0,
                out double correctionAmount
            );

            goodSamples++;
            stLocal = BuildSuccessStatus(
                snapshot,
                sampleOffsetS,
                sampleRttMs,
                tnow,
                goodSamples,
                median,
                omin,
                omax,
                correction,
                correctionAmount,
                hintResult
            );
        }
        else
        {
            // Error path: sample acquisition failed or RTT exceeded threshold
            stLocal = new Status
            {
                Synchronized = (goodSamples >= snapshot.MinSamplesToLock),
                RttMs = sampleRttMs,
                LastDelayS = sampleRttMs / 1000.0,
                OffsetS = sampleOffsetS,
                LastError = string.IsNullOrEmpty(err) ? "sample rejected" : err,
            };
        }

        return stLocal;
    }

    private bool UdpExchange(
        out double outOffsetS,
        out int outRttMs,
        out byte[] outResponse,
        out string err
    )
    {
        outOffsetS = 0.0;
        outRttMs = 0;
        outResponse = Array.Empty<byte>();
        err = string.Empty;

        if (!_udpSocket.IsOpen())
        {
            err = "socket not open";
            return false;
        }

        // Reset abort flag
        _exchangeAbort = false;

        // Get transmit timestamp (T1) and build request
        var ts = _timeSource;
        if (ts == null)
        {
            err = "time source not set";
            return false;
        }

        TimeSpec T1 = ts.NowUnix();
        byte[] req = BuildNtpRequest(T1);

        // Send request
        if (!_udpSocket.Send(req))
        {
            err = "send failed";
            return false;
        }

        // Wait for response
        if (!WaitForExchangeResponse(500, out var msg, out err))
        {
            return false;
        }

        // Process response
        if (!ProcessNtpResponse(msg, T1, out outOffsetS, out outRttMs, out err))
        {
            return false;
        }

        outResponse = msg.Data;
        return true;
    }

    private byte[] BuildNtpRequest(TimeSpec T1)
    {
        NtpPacket req = new NtpPacket();
        req.LiVnMode = (byte)((0 << 6) | (4 << 3) | 3); // v4, client mode

        // Write transmit timestamp
        ulong ntpTs = T1.ToNtpTimestamp();
        req.TxTimestamp = ntpTs;

        // Serialize to bytes
        byte[] buf = new byte[NtpPacketSize];
        var span = buf.AsSpan();

        span[0] = req.LiVnMode;
        span[1] = req.Stratum;
        span[2] = req.Poll;
        span[3] = (byte)req.Precision;
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(4, 4), req.RootDelay);
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(8, 4), req.RootDispersion);
        BinaryPrimitives.WriteUInt32BigEndian(span.Slice(12, 4), req.RefId);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(16, 8), req.RefTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(24, 8), req.OrigTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(32, 8), req.RecvTimestamp);
        BinaryPrimitives.WriteUInt64BigEndian(span.Slice(40, 8), req.TxTimestamp);

        return buf;
    }

    private bool WaitForExchangeResponse(
        int timeoutMs,
        out UdpSocket.Message outMsg,
        out string err
    )
    {
        outMsg = default;
        err = string.Empty;

        const int kPollIntervalMs = 50;
        int elapsedMs = 0;

        while (elapsedMs < timeoutMs)
        {
            // Check for abort signal
            if (_exchangeAbort)
            {
                err = "aborted by push";
                return false;
            }

            // Wait for message with short timeout
            if (_udpSocket.WaitMessage(kPollIntervalMs, out outMsg))
            {
                // Check message type
                if (outMsg.Type == UdpSocket.MessageType.Push)
                {
                    // Push received during Exchange - abort
                    err = "push during exchange";
                    return false;
                }
                return true;
            }

            elapsedMs += kPollIntervalMs;
        }

        // Timeout
        err = "recvfrom timeout/failure";
        return false;
    }

    private bool ProcessNtpResponse(
        UdpSocket.Message msg,
        TimeSpec T1,
        out double outOffsetS,
        out int outRttMs,
        out string err
    )
    {
        outOffsetS = 0.0;
        outRttMs = 0;
        err = string.Empty;

        if (msg.Data.Length < NtpPacketSize)
        {
            err = "response too small";
            return false;
        }

        // Parse NTP packet
        var span = new ReadOnlySpan<byte>(msg.Data, 0, NtpPacketSize);
        NtpPacket resp = new NtpPacket();
        resp.LiVnMode = span[0];
        resp.Stratum = span[1];
        resp.Poll = span[2];
        resp.Precision = (sbyte)span[3];
        resp.RootDelay = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(4, 4));
        resp.RootDispersion = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(8, 4));
        resp.RefId = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(12, 4));
        resp.RefTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(16, 8));
        resp.OrigTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(24, 8));
        resp.RecvTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(32, 8));
        resp.TxTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(40, 8));

        // Extract timestamps as TimeSpec
        TimeSpec T2 = TimeSpec.FromNtpTimestamp(resp.RecvTimestamp);
        TimeSpec T3 = TimeSpec.FromNtpTimestamp(resp.TxTimestamp);
        TimeSpec T4 = msg.RecvTime;

        // Compute offset and delay using TimeSpec arithmetic
        // delay = (T4 - T1) - (T3 - T2)
        TimeSpec delay = (T4 - T1) - (T3 - T2);
        // offset = ((T2 - T1) + (T3 - T4)) / 2
        TimeSpec offset2x = (T2 - T1) + (T3 - T4);

        outOffsetS = offset2x.ToDouble() / 2.0;
        outRttMs = (int)(Math.Max(0.0, delay.ToDouble()) * 1000.0 + 0.5);
        return true;
    }

    private struct VendorHintResult
    {
        public bool Applied;
        public bool AbsApplied;
        public bool EpochChanged;
        public TimeSpec StepAmount;
    }

    private VendorHintResult ApplyVendorHintFromRx(byte[] rx, Options snapshot)
    {
        return ApplyVendorHints(rx, false, snapshot);
    }

    private void HandlePushMessage(UdpSocket.Message msg, Options snapshot)
    {
        ApplyVendorHints(msg.Data, true, snapshot);
    }

    private VendorHintResult ApplyVendorHints(byte[] rx, bool allowAbort, Options snapshot)
    {
        VendorHintResult hintResult = new VendorHintResult();

        var ts = _timeSource;
        if (ts == null)
            return hintResult;

        // Process vendor hints with epoch detection
        var result = _vendorHintProcessor.ProcessWithEpochDetection(
            rx,
            NtpPacketSize,
            ts,
            out bool epochChanged
        );

        hintResult.EpochChanged = epochChanged;

        // If reset is needed, clear estimator state
        if (result.ResetNeeded)
        {
            if (allowAbort)
            {
                _exchangeAbort = true;
            }
            _estimatorState.Clear();
            _clockCorrector.ResetOffset();

            // If absolute time was set, allow backward jump
            if (result.AbsApplied)
            {
                _clockCorrector.AllowBackwardOnce();
                hintResult.AbsApplied = true;
                hintResult.StepAmount = result.StepAmount;
            }
            hintResult.Applied = true;
        }

        return hintResult;
    }

    private (double median, double omin, double omax) UpdateEstimatorsAndTarget(
        Options snapshot,
        double sampleOffsetS,
        double tnow
    )
    {
        // Add sample to sliding window
        int maxw = Math.Max(snapshot.OffsetWindow, snapshot.SkewWindow);
        _estimatorState.AddSample(sampleOffsetS, tnow, maxw);

        // Compute robust target: median of last window, and min/max for debug
        var stats = _estimatorState.ComputeOffsetStats(snapshot.OffsetWindow, sampleOffsetS);
        double median = stats.Median;
        double omin = stats.Min;
        double omax = stats.Max;

        return (median, omin, omax);
    }

    private void SetBaseStatusFields(
        Status st,
        Options snapshot,
        double sampleOffsetS,
        int sampleRttMs,
        double tnow,
        int goodSamples
    )
    {
        st.Synchronized = (goodSamples >= snapshot.MinSamplesToLock);
        st.RttMs = sampleRttMs;
        st.LastDelayS = sampleRttMs / 1000.0;
        st.OffsetS = sampleOffsetS;
        st.LastUpdate = TimeSpec.FromDouble(tnow);
        st.LastError = string.Empty;

        string sockErr = GetSocketError();
        if (!string.IsNullOrEmpty(sockErr))
        {
            st.LastError = sockErr;
        }
    }

    private Status BuildVendorHintAppliedStatus(
        Options snapshot,
        double sampleOffsetS,
        int sampleRttMs,
        double tnow,
        int goodSamples,
        VendorHintResult hintResult
    )
    {
        Status st = new Status();
        SetBaseStatusFields(st, snapshot, sampleOffsetS, sampleRttMs, tnow, goodSamples);

        // Include vendor hint step correction if applied
        if (hintResult.AbsApplied)
        {
            st.LastCorrection = Status.CorrectionType.Step;
            st.LastCorrectionAmountS = hintResult.StepAmount.ToDouble();
            st.WindowCount = 0;
            st.OffsetAppliedS = _clockCorrector.GetOffsetApplied();
            st.OffsetTargetS = 0.0;
            // Override offset_s with actual step amount (NTP offset is meaningless
            // after TimeSource update)
            st.OffsetS = hintResult.StepAmount.ToDouble();
        }

        // Set epoch changed flag and current epoch number
        st.EpochChanged = hintResult.EpochChanged;
        st.Epoch = _vendorHintProcessor.GetCurrentEpoch();

        return st;
    }

    private Status BuildSuccessStatus(
        Options snapshot,
        double sampleOffsetS,
        int sampleRttMs,
        double tnow,
        int goodSamples,
        double median,
        double omin,
        double omax,
        Status.CorrectionType correction,
        double correctionAmount,
        VendorHintResult hintResult
    )
    {
        Status st = new Status();
        SetBaseStatusFields(st, snapshot, sampleOffsetS, sampleRttMs, tnow, goodSamples);

        st.SkewPpm = _estimatorState.ComputeSkewPpm(snapshot.SkewWindow);
        st.Samples = goodSamples;
        st.OffsetWindow = snapshot.OffsetWindow;
        st.SkewWindow = snapshot.SkewWindow;
        st.WindowCount = _estimatorState.GetSampleCount();
        st.OffsetMedianS = median;
        st.OffsetMinS = omin;
        st.OffsetMaxS = omax;
        st.OffsetAppliedS = _clockCorrector.GetOffsetApplied();
        st.OffsetTargetS = median;
        st.LastCorrection = correction;
        st.LastCorrectionAmountS = correctionAmount;
        st.EpochChanged = hintResult.EpochChanged;
        st.Epoch = _vendorHintProcessor.GetCurrentEpoch();

        return st;
    }

    private void ReportSocketError(string msg)
    {
        lock (_socketErrorLock)
        {
            _socketLastError = msg;
        }

        lock (_statusLock)
        {
            _status.LastError = msg;
        }
    }

    private string GetSocketError()
    {
        lock (_socketErrorLock)
        {
            return _socketLastError;
        }
    }

    private void ClearSocketError()
    {
        lock (_socketErrorLock)
        {
            _socketLastError = string.Empty;
        }
    }
}
