using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpClock.Internal;
using NtpServer.Internal;

namespace NtpClock.Tests;

[TestClass]
public class VendorHintProcessorTests
{
    private class MockTimeSource : ITimeSource
    {
        private TimeSpec _time;
        private double _rate;

        public MockTimeSource(TimeSpec initialTime, double rate = 1.0)
        {
            _time = initialTime;
            _rate = rate;
        }

        public TimeSpec NowUnix() => _time;

        public void SetAbsolute(TimeSpec time)
        {
            _time = time;
        }

        public void SetRate(double rate)
        {
            _rate = rate;
        }

        public void SetAbsoluteAndRate(TimeSpec time, double rate)
        {
            _time = time;
            _rate = rate;
        }

        public void ResetToRealTime()
        {
            _time = new TimeSpec();
            _rate = 1.0;
        }

        public double GetRate() => _rate;
    }

    private byte[] CreatePacketWithVendorExt(NtpVendorExt.Payload payload)
    {
        // Create basic NTP packet
        var packet = new NtpPacket
        {
            LiVnMode = 0x24, // Mode 4 (server)
            Stratum = 1,
            Poll = 6,
            Precision = -20,
            RefTimestamp = new TimeSpec(1000, 0).ToNtpTimestamp(),
            OrigTimestamp = 0,
            RecvTimestamp = new TimeSpec(1000, 100_000_000).ToNtpTimestamp(),
            TxTimestamp = new TimeSpec(1000, 200_000_000).ToNtpTimestamp(),
        };

        // Serialize vendor extension payload (without EF header)
        byte[] vendorBytes = NtpVendorExt.Serialize(payload);

        // ComposeNtpPacketWithEf will add the EF header (type + length)
        return NtpPacketHelper.ComposeNtpPacketWithEf(packet, vendorBytes);
    }

    [TestMethod]
    public void Constructor_InitializesWithZeroEpoch()
    {
        var processor = new VendorHintProcessor();
        Assert.AreEqual(0u, processor.GetCurrentEpoch());
    }

    [TestMethod]
    public void ProcessAndApply_NoExtension_ReturnsDefault()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        // Packet with no extension (just 48 bytes)
        byte[] data = new byte[48];

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
    }

    [TestMethod]
    public void ProcessAndApply_NullTimeSource_ReturnsDefault()
    {
        var processor = new VendorHintProcessor();

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1100, 0),
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, null, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
    }

    [TestMethod]
    public void ProcessAndApply_RateHint_AppliesRate()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0001,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
        Assert.AreEqual(1.0001, timeSource.GetRate(), 1e-9);
    }

    [TestMethod]
    public void ProcessAndApply_RateHint_NoChangeIfSame()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
    }

    [TestMethod]
    public void ProcessAndApply_RateHint_AppliesIfAboveThreshold()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        // Rate difference of 2e-12 is above epsilon (1e-12)
        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0 + 2e-12,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
    }

    [TestMethod]
    public void ProcessAndApply_AbsoluteHint_AppliesIfAboveThreshold()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1000, 500_000_000), // 0.5s difference
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2); // threshold 0.2s

        Assert.IsTrue(result.ResetNeeded);
        Assert.IsTrue(result.AbsApplied);
        // Step amount = AbsTime - currentTime = 1000.5 - 1000.0 = 0.5
        Assert.AreEqual(0, result.StepAmount.Sec);
        Assert.AreEqual(500_000_000u, result.StepAmount.Nsec, 1000u);
        Assert.AreEqual(1000, timeSource.NowUnix().Sec);
        Assert.AreEqual(500_000_000u, timeSource.NowUnix().Nsec, 1000u);
    }

    [TestMethod]
    public void ProcessAndApply_AbsoluteHint_NoChangeIfBelowThreshold()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1000, 100_000_000), // 0.1s difference
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2); // threshold 0.2s

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
        Assert.AreEqual(1000, timeSource.NowUnix().Sec);
        Assert.AreEqual(0u, timeSource.NowUnix().Nsec);
    }

    [TestMethod]
    public void ProcessAndApply_BothAbsAndRate_AppliesAtomically()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1000, 500_000_000),
            RateScale = 1.0001,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
        Assert.IsTrue(result.AbsApplied);
        Assert.AreEqual(1000, timeSource.NowUnix().Sec);
        Assert.AreEqual(500_000_000u, timeSource.NowUnix().Nsec, 1000u);
        Assert.AreEqual(1.0001, timeSource.GetRate(), 1e-9);
    }

    [TestMethod]
    public void ProcessAndApply_BothAbsAndRate_NoChangeIfBothBelowThreshold()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1000, 100_000_000), // Below threshold
            RateScale = 1.0, // Same rate
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
    }

    [TestMethod]
    public void ProcessAndApply_BothAbsAndRate_AppliesIfRateChanged()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 1,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            AbsTime = new TimeSpec(1000, 0), // No absolute change
            RateScale = 1.0001, // Rate changed
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied); // Abs didn't change enough
    }

    [TestMethod]
    public void ProcessAndApply_DuplicateSequence_Ignored()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload = new NtpVendorExt.Payload
        {
            Seq = 100,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0001,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        // First packet should be processed
        var result1 = processor.ProcessAndApply(data, 48, timeSource, 0.2);
        Assert.IsTrue(result1.ResetNeeded);

        // Second packet with same sequence should be ignored
        timeSource.SetRate(1.0); // Reset rate
        var result2 = processor.ProcessAndApply(data, 48, timeSource, 0.2);
        Assert.IsFalse(result2.ResetNeeded);
        Assert.AreEqual(1.0, timeSource.GetRate()); // Rate unchanged
    }

    [TestMethod]
    public void ProcessAndApply_NewerSequence_Processed()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload1 = new NtpVendorExt.Payload
        {
            Seq = 100,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0001,
        };

        byte[] data1 = CreatePacketWithVendorExt(payload1);
        processor.ProcessAndApply(data1, 48, timeSource, 0.2);

        var payload2 = new NtpVendorExt.Payload
        {
            Seq = 101,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0002,
        };

        byte[] data2 = CreatePacketWithVendorExt(payload2);
        var result = processor.ProcessAndApply(data2, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
        Assert.AreEqual(1.0002, timeSource.GetRate(), 1e-9);
    }

    [TestMethod]
    public void ProcessAndApply_OlderSequence_Ignored()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload1 = new NtpVendorExt.Payload
        {
            Seq = 100,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0001,
        };

        byte[] data1 = CreatePacketWithVendorExt(payload1);
        processor.ProcessAndApply(data1, 48, timeSource, 0.2);

        var payload2 = new NtpVendorExt.Payload
        {
            Seq = 99,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0002,
        };

        byte[] data2 = CreatePacketWithVendorExt(payload2);
        var result = processor.ProcessAndApply(data2, 48, timeSource, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.AreEqual(1.0001, timeSource.GetRate(), 1e-9); // Unchanged
    }

    [TestMethod]
    public void ProcessAndApply_SequenceWrapAround_HandledCorrectly()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var payload1 = new NtpVendorExt.Payload
        {
            Seq = uint.MaxValue - 10,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0001,
        };

        byte[] data1 = CreatePacketWithVendorExt(payload1);
        processor.ProcessAndApply(data1, 48, timeSource, 0.2);

        // Sequence wraps around
        var payload2 = new NtpVendorExt.Payload
        {
            Seq = 10,
            Flags = NtpVendorExt.FlagRate,
            ServerTime = new TimeSpec(1000, 0),
            RateScale = 1.0002,
        };

        byte[] data2 = CreatePacketWithVendorExt(payload2);
        var result = processor.ProcessAndApply(data2, 48, timeSource, 0.2);

        Assert.IsTrue(result.ResetNeeded);
        Assert.AreEqual(1.0002, timeSource.GetRate(), 1e-9);
    }

    [TestMethod]
    public void ProcessPacket_NewEpoch_ReturnsTrue()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var packet = new NtpPacket { LiVnMode = 0x24 }; // Mode 4
        var vendor = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(1000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0001,
        };

        bool shouldUse = processor.ProcessPacket(packet, vendor, timeSource);

        Assert.IsTrue(shouldUse);
        Assert.AreEqual(100u, processor.GetCurrentEpoch());
    }

    [TestMethod]
    public void ProcessPacket_OldEpoch_ReturnsFalse()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var packet1 = new NtpPacket { LiVnMode = 0x24 };
        var vendor1 = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(1000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0,
        };

        processor.ProcessPacket(packet1, vendor1, timeSource);

        var packet2 = new NtpPacket { LiVnMode = 0x24 };
        var vendor2 = new NtpVendorExt.Payload
        {
            Seq = 99,
            ServerTime = new TimeSpec(1001, 0),
            Flags = 0,
        };

        bool shouldUse = processor.ProcessPacket(packet2, vendor2, timeSource);

        Assert.IsFalse(shouldUse);
        Assert.AreEqual(100u, processor.GetCurrentEpoch());
    }

    [TestMethod]
    public void ProcessPacket_SameEpochMode4_ReturnsTrue()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var packet1 = new NtpPacket { LiVnMode = 0x24 };
        var vendor1 = new NtpVendorExt.Payload { Seq = 100, ServerTime = new TimeSpec(1000, 0) };
        processor.ProcessPacket(packet1, vendor1, timeSource);

        var packet2 = new NtpPacket { LiVnMode = 0x24 }; // Mode 4
        var vendor2 = new NtpVendorExt.Payload { Seq = 100, ServerTime = new TimeSpec(1001, 0) };
        bool shouldUse = processor.ProcessPacket(packet2, vendor2, timeSource);

        Assert.IsTrue(shouldUse);
    }

    [TestMethod]
    public void ProcessPacket_SameEpochMode5_ReturnsFalse()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var packet1 = new NtpPacket { LiVnMode = 0x24 };
        var vendor1 = new NtpVendorExt.Payload { Seq = 100, ServerTime = new TimeSpec(1000, 0) };
        processor.ProcessPacket(packet1, vendor1, timeSource);

        var packet2 = new NtpPacket { LiVnMode = 0x25 }; // Mode 5 (broadcast)
        var vendor2 = new NtpVendorExt.Payload { Seq = 100, ServerTime = new TimeSpec(1001, 0) };
        bool shouldUse = processor.ProcessPacket(packet2, vendor2, timeSource);

        Assert.IsFalse(shouldUse);
    }

    [TestMethod]
    public void ProcessPacket_NewEpoch_UpdatesTimeSource()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0), 1.0);

        var packet = new NtpPacket { LiVnMode = 0x24 };
        var vendor = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0001,
        };

        processor.ProcessPacket(packet, vendor, timeSource);

        Assert.AreEqual(2000, timeSource.NowUnix().Sec);
        Assert.AreEqual(1.0001, timeSource.GetRate(), 1e-9);
    }

    [TestMethod]
    public void ProcessWithEpochDetection_NewEpoch_SetsFlag()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var payload = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0001,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessWithEpochDetection(
            data,
            48,
            timeSource,
            out bool epochChanged
        );

        Assert.IsTrue(epochChanged);
        Assert.IsTrue(result.ResetNeeded);
        Assert.IsTrue(result.AbsApplied);
        Assert.AreEqual(100u, processor.GetCurrentEpoch());
    }

    [TestMethod]
    public void ProcessWithEpochDetection_SameEpoch_NoFlag()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var payload1 = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0001,
        };

        byte[] data1 = CreatePacketWithVendorExt(payload1);
        processor.ProcessWithEpochDetection(data1, 48, timeSource, out _);

        var payload2 = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2001, 0),
            Flags = 0,
        };

        byte[] data2 = CreatePacketWithVendorExt(payload2);
        var result = processor.ProcessWithEpochDetection(
            data2,
            48,
            timeSource,
            out bool epochChanged
        );

        Assert.IsFalse(epochChanged);
    }

    [TestMethod]
    public void ProcessWithEpochDetection_CalculatesStepAmount()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        var payload = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0,
        };

        byte[] data = CreatePacketWithVendorExt(payload);

        var result = processor.ProcessWithEpochDetection(
            data,
            48,
            timeSource,
            out bool epochChanged
        );

        Assert.IsTrue(epochChanged);
        Assert.IsTrue(result.AbsApplied);
        // Step amount should be approximately 1000 seconds
        Assert.AreEqual(1000, result.StepAmount.Sec);
    }

    [TestMethod]
    public void SetLogSink_EnablesLogging()
    {
        var processor = new VendorHintProcessor();
        string? lastLog = null;
        processor.SetLogSink(msg => lastLog = msg);

        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));
        var payload = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(2000, 0),
            Flags = NtpVendorExt.FlagRate,
            RateScale = 1.0,
        };

        byte[] data = CreatePacketWithVendorExt(payload);
        processor.ProcessWithEpochDetection(data, 48, timeSource, out _);

        Assert.IsNotNull(lastLog);
        StringAssert.Contains(lastLog, "epoch");
    }

    [TestMethod]
    public void ProcessAndApply_InvalidExtension_ReturnsDefault()
    {
        var processor = new VendorHintProcessor();
        var timeSource = new MockTimeSource(new TimeSpec(1000, 0));

        // Create packet with malformed extension
        byte[] data = new byte[52]; // 48 + 4 (incomplete extension)
        Array.Copy(new byte[48], data, 48);

        var result = processor.ProcessAndApply(data, 48, timeSource, 0.2);

        Assert.IsFalse(result.ResetNeeded);
        Assert.IsFalse(result.AbsApplied);
    }
}
