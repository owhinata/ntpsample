// Copyright (c) 2025 The NTP Sample Authors
using System.Buffers.Binary;
using NtpServer.Internal;

namespace NtpClock.Internal;

/// <summary>
/// Processes NTP vendor extension hints.
/// Parses vendor extension fields from received NTP packets and applies
/// rate/absolute time changes to a TimeSource. Provides deduplication
/// based on sequence numbers.
/// </summary>
public class VendorHintProcessor
{
    /// <summary>Callback for logging messages (thread-safe).</summary>
    public delegate void LogCallback(string message);

    /// <summary>Result of processing a vendor hint.</summary>
    public struct HintResult
    {
        /// <summary>Whether estimator reset is needed.</summary>
        public bool ResetNeeded;

        /// <summary>Whether SetAbsolute was applied.</summary>
        public bool AbsApplied;

        /// <summary>Amount of step (if AbsApplied).</summary>
        public TimeSpec StepAmount;
    }

    private bool _haveSeq = false;
    private uint _lastSeq = 0;
    private uint _currentEpoch = 0;
    private LogCallback? _logCallback;

    /// <summary>
    /// Set log sink callback for debug logging.
    /// </summary>
    /// <param name="callback">Callback function for logging messages.</param>
    public void SetLogSink(LogCallback callback)
    {
        _logCallback = callback;
    }

    /// <summary>
    /// Process and apply vendor hints from an NTP response.
    /// Parses vendor extension fields, deduplicates by sequence number,
    /// and applies SetRate/SetAbsolute if values differ meaningfully.
    /// </summary>
    /// <param name="rxData">Raw NTP packet bytes (header + optional extensions).</param>
    /// <param name="ntpPacketSize">Size of the basic NTP packet structure.</param>
    /// <param name="timeSource">Target TimeSource to apply hints to.</param>
    /// <param name="stepThresholdS">Threshold for applying absolute time changes (seconds).</param>
    /// <returns>HintResult indicating what was applied and whether reset is needed.</returns>
    public HintResult ProcessAndApply(
        byte[] rxData,
        int ntpPacketSize,
        ITimeSource? timeSource,
        double stepThresholdS
    )
    {
        HintResult result = new HintResult();

        if (!ParseVendorPayload(rxData, ntpPacketSize, out var payload) || payload == null)
        {
            return result;
        }

        if (timeSource == null)
        {
            return result;
        }

        bool hasAbs = (payload.Flags & NtpVendorExt.FlagAbs) != 0;
        bool hasRate = (payload.Flags & NtpVendorExt.FlagRate) != 0;

        // Atomic update when both abs and rate are present
        if (hasAbs && hasRate)
        {
            const double rateEps = 1e-12; // ~1e-6 ppm
            double curRate = timeSource.GetRate();
            TimeSpec curAbs = timeSource.NowUnix();

            bool rateChanged = Math.Abs(payload.RateScale - curRate) > rateEps;
            TimeSpec diff = TimeSpec.AbsDiff(payload.AbsTime, curAbs);
            TimeSpec threshold = TimeSpec.FromDouble(stepThresholdS);
            bool absChanged = diff >= threshold;

            if (rateChanged || absChanged)
            {
                timeSource.SetAbsoluteAndRate(payload.AbsTime, payload.RateScale);
                result.ResetNeeded = true;
                if (absChanged)
                {
                    result.AbsApplied = true;
                    result.StepAmount = payload.AbsTime - curAbs;
                }
            }
        }
        else
        {
            // Apply individually if only one is present
            if (ApplyRateHint(payload, timeSource))
            {
                result.ResetNeeded = true;
            }

            if (ApplyAbsoluteHint(payload, timeSource, stepThresholdS, out var stepAmount))
            {
                result.ResetNeeded = true;
                result.AbsApplied = true;
                result.StepAmount = stepAmount;
            }
        }

        return result;
    }

    /// <summary>
    /// Process packet based on epoch and mode.
    /// Implements epoch-based synchronization.
    /// </summary>
    /// <param name="pkt">NTP packet header.</param>
    /// <param name="vendor">Parsed vendor extension payload.</param>
    /// <param name="timeSource">Target TimeSource to update on new epoch.</param>
    /// <returns>true if packet should be used for NTP sync, false otherwise.</returns>
    public bool ProcessPacket(NtpPacket pkt, NtpVendorExt.Payload vendor, ITimeSource? timeSource)
    {
        byte mode = (byte)(pkt.LiVnMode & 0x07);
        uint packetEpoch = vendor.Seq;

        // Old epoch - ignore
        if (EpochCompare.IsEpochOlder(packetEpoch, _currentEpoch))
        {
            return false;
        }

        // New epoch - update
        if (EpochCompare.IsEpochNewer(packetEpoch, _currentEpoch))
        {
            uint oldEpoch = _currentEpoch;
            _currentEpoch = packetEpoch;

            // Apply TimeSource update using server's current time
            if (timeSource != null)
            {
                bool hasRate = (vendor.Flags & NtpVendorExt.FlagRate) != 0;
                if (hasRate)
                {
                    timeSource.SetAbsoluteAndRate(vendor.ServerTime, vendor.RateScale);
                    _logCallback?.Invoke(
                        $"[DEBUG VendorHint] ProcessPacket: epoch {oldEpoch}->{_currentEpoch} "
                            + $"ABS/RATE applied server_time={vendor.ServerTime} rate={vendor.RateScale}"
                    );
                }
            }

            // New epoch detected - caller should send new Exchange
            return true;
        }

        // Same epoch
        if (mode == 5)
        {
            // Push notification - don't use for NTP sync
            return false;
        }

        // mode=4, same epoch - process normally
        return true;
    }

    /// <summary>
    /// Process packet with epoch detection from raw data.
    /// Combines epoch detection with hint processing. Parses NTP packet and
    /// vendor extension, detects epoch changes, and updates TimeSource.
    /// </summary>
    /// <param name="rxData">Raw NTP packet bytes (header + optional extensions).</param>
    /// <param name="ntpPacketSize">Size of the basic NTP packet structure.</param>
    /// <param name="timeSource">Target TimeSource to update on new epoch.</param>
    /// <param name="outEpochChanged">Output parameter set to true if epoch changed.</param>
    /// <returns>HintResult indicating what was applied and whether reset is needed.</returns>
    public HintResult ProcessWithEpochDetection(
        byte[] rxData,
        int ntpPacketSize,
        ITimeSource? timeSource,
        out bool outEpochChanged
    )
    {
        HintResult result = new HintResult();
        outEpochChanged = false;

        // Parse NTP packet header
        if (rxData.Length < ntpPacketSize)
        {
            return result;
        }

        NtpPacket pkt = new NtpPacket();
        var span = new ReadOnlySpan<byte>(rxData, 0, Math.Min(ntpPacketSize, rxData.Length));
        pkt.LiVnMode = span[0];
        pkt.Stratum = span[1];
        pkt.Poll = span[2];
        pkt.Precision = (sbyte)span[3];
        pkt.RootDelay = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(4, 4));
        pkt.RootDispersion = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(8, 4));
        pkt.RefId = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(12, 4));
        pkt.RefTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(16, 8));
        pkt.OrigTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(24, 8));
        pkt.RecvTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(32, 8));
        pkt.TxTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(40, 8));

        // Parse vendor extension payload
        if (!ParseVendorPayload(rxData, ntpPacketSize, out var payload) || payload == null)
        {
            return result;
        }

        if (timeSource == null)
        {
            return result;
        }

        // Track old epoch and time before ProcessPacket for step calculation
        uint oldEpoch = _currentEpoch;
        TimeSpec timeBefore = timeSource.NowUnix();

        _logCallback?.Invoke(
            $"[DEBUG VendorHint] ProcessWithEpochDetection BEFORE: "
                + $"old_epoch={oldEpoch} packet_epoch={payload.Seq} time_before={timeBefore}"
        );

        bool shouldUse = ProcessPacket(pkt, payload, timeSource);
        bool epochChanged = (_currentEpoch != oldEpoch);

        outEpochChanged = epochChanged;

        // If epoch changed, ProcessPacket already applied ABS/RATE via
        // SetAbsoluteAndRate, so we need to mark reset_needed and abs_applied
        if (epochChanged)
        {
            result.ResetNeeded = true;
            result.AbsApplied = true;
            // Calculate actual step amount
            TimeSpec timeAfter = timeSource.NowUnix();
            result.StepAmount = timeAfter - timeBefore;

            _logCallback?.Invoke(
                $"[DEBUG VendorHint] ProcessWithEpochDetection AFTER: epoch {oldEpoch}->{_currentEpoch} "
                    + $"time_before={timeBefore} time_after={timeAfter} step={result.StepAmount}"
            );
        }

        // ProcessPacket returns false for old epochs and mode=5 packets
        // We still return the result for consistency
        _ = shouldUse;

        return result;
    }

    /// <summary>
    /// Get current epoch number.
    /// </summary>
    /// <returns>Current epoch number being tracked.</returns>
    public uint GetCurrentEpoch()
    {
        return _currentEpoch;
    }

    private bool ParseVendorPayload(
        byte[] rxData,
        int ntpPacketSize,
        out NtpVendorExt.Payload? outPayload
    )
    {
        outPayload = null;

        // Check if there's extension data beyond the NTP packet
        if (rxData.Length <= ntpPacketSize)
        {
            return false;
        }

        // Parse extension field header
        int offset = ntpPacketSize;
        int remain = rxData.Length - ntpPacketSize;
        if (remain < 4)
        {
            return false;
        }

        ushort typ = (ushort)((rxData[offset] << 8) | rxData[offset + 1]);
        ushort len = (ushort)((rxData[offset + 2] << 8) | rxData[offset + 3]);

        // Validate extension field type and length
        if (typ != NtpVendorExt.EfTypeVendorHint || len < 4 || len > remain)
        {
            return false;
        }

        // Extract and parse vendor payload
        byte[] val = new byte[len - 4];
        Array.Copy(rxData, offset + 4, val, 0, len - 4);

        if (!NtpVendorExt.Parse(val, out outPayload) || outPayload == null)
        {
            return false;
        }

        // Deduplicate by sequence number using wrap-safe comparison
        if (!IsSeqNewer(outPayload.Seq))
        {
            return false;
        }
        _lastSeq = outPayload.Seq;
        _haveSeq = true;

        return true;
    }

    private bool ApplyRateHint(NtpVendorExt.Payload payload, ITimeSource? timeSource)
    {
        if (timeSource == null || (payload.Flags & NtpVendorExt.FlagRate) == 0)
        {
            return false;
        }

        const double rateEps = 1e-12; // ~1e-6 ppm
        double curRate = timeSource.GetRate();
        if (Math.Abs(payload.RateScale - curRate) <= rateEps)
        {
            return false;
        }

        timeSource.SetRate(payload.RateScale);
        return true;
    }

    private bool ApplyAbsoluteHint(
        NtpVendorExt.Payload payload,
        ITimeSource? timeSource,
        double stepThresholdS,
        out TimeSpec outStepAmount
    )
    {
        outStepAmount = new TimeSpec();

        if (timeSource == null || (payload.Flags & NtpVendorExt.FlagAbs) == 0)
        {
            return false;
        }

        TimeSpec cur = timeSource.NowUnix();
        TimeSpec diff = TimeSpec.AbsDiff(payload.AbsTime, cur);
        TimeSpec threshold = TimeSpec.FromDouble(stepThresholdS);

        if (diff < threshold)
        {
            return false;
        }

        timeSource.SetAbsolute(payload.AbsTime);
        outStepAmount = payload.AbsTime - cur;
        return true;
    }

    private bool IsSeqNewer(uint seq)
    {
        if (!_haveSeq)
            return true;
        if (seq == _lastSeq)
            return false;

        // Signed diff captures wrap-around: positive diff => newer
        int diff = (int)(seq - _lastSeq);
        return diff > 0;
    }
}
