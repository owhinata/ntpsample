using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpServer.Internal;

namespace NtpServer.Tests;

[TestClass]
public class NtpTypesTests
{
    [TestMethod]
    public void NtpVendorExt_Serialize_WithNoFlags()
    {
        var payload = new NtpVendorExt.Payload
        {
            Seq = 42,
            ServerTime = new TimeSpec(1000, 500_000_000),
            Flags = 0,
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);

        // Header (12) + ServerTime (12) = 24 bytes (aligned to 4)
        Assert.IsGreaterThanOrEqualTo(bytes.Length, 24);
        Assert.AreEqual(0, bytes.Length % 4); // 4-byte aligned
    }

    [TestMethod]
    public void NtpVendorExt_Serialize_WithAbsFlag()
    {
        var payload = new NtpVendorExt.Payload
        {
            Seq = 42,
            ServerTime = new TimeSpec(1000, 500_000_000),
            AbsTime = new TimeSpec(2000, 250_000_000),
            Flags = NtpVendorExt.FlagAbs,
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);

        // Header (12) + ServerTime (12) + AbsTime (12) = 36 bytes
        Assert.IsGreaterThanOrEqualTo(bytes.Length, 36);
        Assert.AreEqual(0, bytes.Length % 4);
    }

    [TestMethod]
    public void NtpVendorExt_Serialize_WithRateFlag()
    {
        var payload = new NtpVendorExt.Payload
        {
            Seq = 42,
            ServerTime = new TimeSpec(1000, 500_000_000),
            RateScale = 1.5,
            Flags = NtpVendorExt.FlagRate,
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);

        // Header (12) + ServerTime (12) + Rate (8) = 32 bytes
        Assert.IsGreaterThanOrEqualTo(bytes.Length, 32);
        Assert.AreEqual(0, bytes.Length % 4);
    }

    [TestMethod]
    public void NtpVendorExt_Serialize_WithAbsAndRate()
    {
        var payload = new NtpVendorExt.Payload
        {
            Seq = 42,
            ServerTime = new TimeSpec(1000, 500_000_000),
            AbsTime = new TimeSpec(2000, 250_000_000),
            RateScale = 2.0,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);

        // Header (12) + ServerTime (12) + AbsTime (12) + Rate (8) = 44 bytes
        Assert.IsGreaterThanOrEqualTo(bytes.Length, 44);
        Assert.AreEqual(0, bytes.Length % 4);
    }

    [TestMethod]
    public void NtpVendorExt_RoundTrip_NoFlags()
    {
        var original = new NtpVendorExt.Payload
        {
            Seq = 100,
            ServerTime = new TimeSpec(1234567890, 123456789),
            Flags = 0,
        };

        byte[] bytes = NtpVendorExt.Serialize(original);
        bool success = NtpVendorExt.Parse(bytes, out var recovered);

        Assert.IsTrue(success);
        Assert.IsNotNull(recovered);
        Assert.AreEqual(NtpVendorExt.Magic, recovered.Magic);
        Assert.AreEqual(NtpVendorExt.Version, recovered.Version);
        Assert.AreEqual(original.Seq, recovered.Seq);
        Assert.AreEqual(original.ServerTime.Sec, recovered.ServerTime.Sec);
        Assert.AreEqual(original.ServerTime.Nsec, recovered.ServerTime.Nsec);
    }

    [TestMethod]
    public void NtpVendorExt_RoundTrip_WithAbsAndRate()
    {
        var original = new NtpVendorExt.Payload
        {
            Seq = 200,
            ServerTime = new TimeSpec(1000, 500_000_000),
            AbsTime = new TimeSpec(2000, 750_000_000),
            RateScale = 1.25,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
        };

        byte[] bytes = NtpVendorExt.Serialize(original);
        bool success = NtpVendorExt.Parse(bytes, out var recovered);

        Assert.IsTrue(success);
        Assert.IsNotNull(recovered);
        Assert.AreEqual(original.Seq, recovered.Seq);
        Assert.AreEqual(original.Flags, recovered.Flags);
        Assert.AreEqual(original.ServerTime.Sec, recovered.ServerTime.Sec);
        Assert.AreEqual(original.AbsTime.Sec, recovered.AbsTime.Sec);
        Assert.AreEqual(original.RateScale, recovered.RateScale, 1e-10);
    }

    [TestMethod]
    public void NtpVendorExt_Parse_InvalidMagic()
    {
        var payload = new NtpVendorExt.Payload
        {
            Magic = 0x12345678, // Invalid magic
            Seq = 42,
            ServerTime = new TimeSpec(1000, 0),
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);
        bool success = NtpVendorExt.Parse(bytes, out var recovered);

        Assert.IsFalse(success);
        Assert.IsNull(recovered);
    }

    [TestMethod]
    public void NtpVendorExt_Parse_InvalidVersion()
    {
        var payload = new NtpVendorExt.Payload
        {
            Version = 99, // Invalid version
            Seq = 42,
            ServerTime = new TimeSpec(1000, 0),
        };

        byte[] bytes = NtpVendorExt.Serialize(payload);
        bool success = NtpVendorExt.Parse(bytes, out var recovered);

        Assert.IsFalse(success);
        Assert.IsNull(recovered);
    }

    [TestMethod]
    public void NtpVendorExt_Parse_TooShort()
    {
        byte[] shortBytes = new byte[10];
        bool success = NtpVendorExt.Parse(shortBytes, out var recovered);

        Assert.IsFalse(success);
        Assert.IsNull(recovered);
    }

    [TestMethod]
    public void EpochCompare_IsEpochOlder()
    {
        // Simple case
        Assert.IsTrue(EpochCompare.IsEpochOlder(10, 20));
        Assert.IsFalse(EpochCompare.IsEpochOlder(20, 10));
        Assert.IsFalse(EpochCompare.IsEpochOlder(15, 15));

        // Wrap-around case
        uint old = uint.MaxValue - 10;
        uint newer = 10;
        Assert.IsTrue(EpochCompare.IsEpochOlder(old, newer));
        Assert.IsFalse(EpochCompare.IsEpochOlder(newer, old));
    }

    [TestMethod]
    public void EpochCompare_IsEpochNewer()
    {
        // Simple case
        Assert.IsTrue(EpochCompare.IsEpochNewer(20, 10));
        Assert.IsFalse(EpochCompare.IsEpochNewer(10, 20));
        Assert.IsFalse(EpochCompare.IsEpochNewer(15, 15));

        // Wrap-around case
        uint old = uint.MaxValue - 10;
        uint newer = 10;
        Assert.IsTrue(EpochCompare.IsEpochNewer(newer, old));
        Assert.IsFalse(EpochCompare.IsEpochNewer(old, newer));
    }
}
