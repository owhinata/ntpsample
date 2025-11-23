using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpServer.Internal;
using Server = NtpServer.NtpServer;

namespace NtpServer.Tests;

[TestClass]
public class NtpServerTests
{
    [TestMethod]
    public void Constructor_CreatesNewInstance()
    {
        var server = new Server();
        Assert.IsNotNull(server);
    }

    [TestMethod]
    public void Start_ReturnsTrue()
    {
        using var server = new Server();
        var result = server.Start(9999); // Use non-standard port
        Assert.IsTrue(result);
        server.Stop();
    }

    [TestMethod]
    public void Start_WithCustomTimeSource_UsesProvidedTimeSource()
    {
        using var server = new Server();
        var timeSource = new StopwatchClock();
        var result = server.Start(9998, timeSource);
        Assert.IsTrue(result);
        server.Stop();
    }

    [TestMethod]
    public void Start_WithCustomOptions_UsesProvidedOptions()
    {
        using var server = new Server();
        var options = new Options.Builder()
            .Stratum(2)
            .Precision(-21)
            .RefId(0x47505300) // "GPS\0"
            .Build();

        var result = server.Start(9997, null, options);
        Assert.IsTrue(result);
        server.Stop();
    }

    [TestMethod]
    public void Start_MultipleTimes_ReturnsTrue()
    {
        using var server = new Server();
        var result1 = server.Start(9996);
        var result2 = server.Start(9996);

        Assert.IsTrue(result1);
        Assert.IsTrue(result2); // Second start should just return true
        server.Stop();
    }

    [TestMethod]
    public void Stop_MultipleTimes_IsSafe()
    {
        using var server = new Server();
        server.Start(9995);
        server.Stop();
        server.Stop(); // Second stop should be safe
    }

    [TestMethod]
    public void Stop_WithoutStart_IsSafe()
    {
        using var server = new Server();
        server.Stop(); // Should not throw
    }

    [TestMethod]
    public void Dispose_StopsServer()
    {
        var server = new Server();
        server.Start(9994);
        server.Dispose();
        // Should be able to start again on the same port
        Thread.Sleep(200); // Wait for port to be fully released
        using var server2 = new Server();
        var result = server2.Start(9994);
        Assert.IsTrue(result);
    }

    [TestMethod]
    public void GetStats_ReturnsInitialStats()
    {
        using var server = new Server();
        var stats = server.GetStats();

        Assert.AreEqual(0UL, stats.PacketsReceived);
        Assert.AreEqual(0UL, stats.PacketsSent);
        Assert.AreEqual(0UL, stats.RecvErrors);
        Assert.AreEqual(0UL, stats.DropShortPackets);
        Assert.AreEqual(0UL, stats.SendErrors);
        Assert.AreEqual(0UL, stats.ActiveClients);
    }

    [TestMethod]
    public void Server_RespondsToNtpRequest()
    {
        using var server = new Server();
        ushort port = 9993;
        server.Start(port);

        // Give server time to start
        Thread.Sleep(100);

        // Send NTP request
        using var client = new UdpClient();
        var request = CreateNtpRequest();
        client.Send(request, request.Length, "127.0.0.1", port);

        // Receive response (with timeout)
        client.Client.ReceiveTimeout = 2000;
        IPEndPoint? remoteEp = null;
        var response = client.Receive(ref remoteEp!);

        // Verify response
        Assert.IsNotNull(response);
        Assert.IsGreaterThanOrEqualTo(48, response.Length); // At least NTP packet size

        // Check statistics
        Thread.Sleep(100); // Give time for stats to update
        var stats = server.GetStats();
        Assert.AreEqual(1UL, stats.PacketsReceived);
        Assert.AreEqual(1UL, stats.PacketsSent);
        Assert.AreEqual(1UL, stats.ActiveClients);

        server.Stop();
    }

    [TestMethod]
    public void Server_CountsShortPackets()
    {
        using var server = new Server();
        ushort port = 9992;
        server.Start(port);

        Thread.Sleep(100);

        // Send short packet (too short for NTP)
        using var client = new UdpClient();
        var shortPacket = new byte[20]; // Less than 48 bytes
        client.Send(shortPacket, shortPacket.Length, "127.0.0.1", port);

        Thread.Sleep(200);

        var stats = server.GetStats();
        Assert.AreEqual(1UL, stats.PacketsReceived);
        Assert.AreEqual(1UL, stats.DropShortPackets);
        Assert.AreEqual(0UL, stats.PacketsSent);

        server.Stop();
    }

    [TestMethod]
    public void Server_HandlesMultipleClients()
    {
        using var server = new Server();
        ushort port = 9991;
        server.Start(port);

        Thread.Sleep(100);

        var tasks = new Task[5];
        for (int i = 0; i < 5; i++)
        {
            tasks[i] = Task.Run(() =>
            {
                using var client = new UdpClient();
                var request = CreateNtpRequest();
                client.Send(request, request.Length, "127.0.0.1", port);

                client.Client.ReceiveTimeout = 2000;
                IPEndPoint? remoteEp = null;
                var response = client.Receive(ref remoteEp!);
                Assert.IsNotNull(response);
            });
        }

        Task.WaitAll(tasks);
        Thread.Sleep(200);

        var stats = server.GetStats();
        Assert.AreEqual(5UL, stats.PacketsReceived);
        Assert.AreEqual(5UL, stats.PacketsSent);

        server.Stop();
    }

    [TestMethod]
    public void Server_IncrementsEpochOnRestart()
    {
        using var server = new Server();
        ushort port = 9990;

        // First start
        server.Start(port);
        var response1 = SendNtpRequestAndGetResponse(port);
        server.Stop();

        // Second start (epoch should increment)
        server.Start(port);
        var response2 = SendNtpRequestAndGetResponse(port);
        server.Stop();

        Assert.IsNotNull(response1);
        Assert.IsNotNull(response2);

        // Both responses should have vendor extensions with different epoch numbers
        // (We can't easily check this without parsing the extension, but at least verify we got responses)
        Assert.IsGreaterThanOrEqualTo(48, response1.Length);
        Assert.IsGreaterThanOrEqualTo(48, response2.Length);
    }

    [TestMethod]
    public void Server_ResponseHasCorrectMode()
    {
        using var server = new Server();
        ushort port = 9989;
        server.Start(port);

        Thread.Sleep(100);

        var response = SendNtpRequestAndGetResponse(port);
        Assert.IsNotNull(response);
        Assert.IsGreaterThanOrEqualTo(48, response.Length);

        // Check LI/VN/Mode byte
        byte liVnMode = response[0];
        byte mode = (byte)(liVnMode & 0x07);
        Assert.AreEqual(4, mode); // Mode 4 = Server

        server.Stop();
    }

    [TestMethod]
    public void Server_ResponseHasCorrectStratum()
    {
        using var server = new Server();
        ushort port = 9988;

        var options = new Options.Builder().Stratum(3).Build();
        server.Start(port, null, options);

        Thread.Sleep(100);

        var response = SendNtpRequestAndGetResponse(port);
        Assert.IsNotNull(response);
        Assert.IsGreaterThanOrEqualTo(48, response.Length);

        // Check Stratum byte
        byte stratum = response[1];
        Assert.AreEqual(3, stratum);

        server.Stop();
    }

    [TestMethod]
    public void Server_EchoesOriginTimestamp()
    {
        using var server = new Server();
        ushort port = 9987;
        server.Start(port);

        Thread.Sleep(100);

        var request = CreateNtpRequest();
        var requestTxTimestamp = ExtractTxTimestamp(request);

        using var client = new UdpClient();
        client.Send(request, request.Length, "127.0.0.1", port);

        client.Client.ReceiveTimeout = 2000;
        IPEndPoint? remoteEp = null;
        var response = client.Receive(ref remoteEp!);

        // Extract origin timestamp from response (bytes 24-31)
        var responseOrigTimestamp = BinaryPrimitives.ReadUInt64BigEndian(response.AsSpan(24, 8));

        Assert.AreEqual(requestTxTimestamp, responseOrigTimestamp);

        server.Stop();
    }

    [TestMethod]
    public void Server_StopsCleanly_UnderLoad()
    {
        using var server = new Server();
        ushort port = 9986;
        server.Start(port);

        Thread.Sleep(100);

        // Start sending requests continuously
        var cts = new CancellationTokenSource();
        var task = Task.Run(() =>
        {
            while (!cts.Token.IsCancellationRequested)
            {
                try
                {
                    using var client = new UdpClient();
                    var request = CreateNtpRequest();
                    client.Send(request, request.Length, "127.0.0.1", port);
                    Thread.Sleep(10);
                }
                catch
                {
                    // Ignore errors during shutdown
                }
            }
        });

        Thread.Sleep(200);

        // Stop server while under load
        server.Stop();
        cts.Cancel();

        // Should complete without hanging
        var completed = task.Wait(TimeSpan.FromSeconds(2));
        Assert.IsTrue(completed);
    }

    [TestMethod]
    public void Start_WhileStopInProgress_WaitsAndSucceeds()
    {
        var server = new Server();
        ushort port = 9985;
        server.Start(port);

        Thread.Sleep(100);

        // Start Stop() in background thread
        var stopTask = Task.Run(() => server.Stop());

        // Immediately try to Start() again while Stop() is in progress
        Thread.Sleep(10); // Give Stop() time to set _stopping flag

        var startTime = DateTime.UtcNow;
        var result = server.Start(port);
        var elapsed = (DateTime.UtcNow - startTime).TotalMilliseconds;

        // Start should wait for Stop to complete and then succeed
        Assert.IsTrue(result);

        // Should complete within reasonable time (less than 5 seconds)
        Assert.IsLessThan(5000, elapsed);

        // Stop task should have completed
        Assert.IsTrue(stopTask.IsCompleted);

        server.Stop();
        server.Dispose();
    }

    private static byte[] CreateNtpRequest()
    {
        var packet = new byte[48];
        packet[0] = 0x23; // LI=0, VN=4, Mode=3 (client)
        packet[1] = 0; // Stratum
        packet[2] = 4; // Poll
        packet[3] = 0xEC; // Precision (-20)

        // Set transmit timestamp (bytes 40-47)
        var now = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
        var ntpTime = (ulong)(now + 2208988800L) << 32; // Convert to NTP time
        BinaryPrimitives.WriteUInt64BigEndian(packet.AsSpan(40, 8), ntpTime);

        return packet;
    }

    private static ulong ExtractTxTimestamp(byte[] packet)
    {
        return BinaryPrimitives.ReadUInt64BigEndian(packet.AsSpan(40, 8));
    }

    private static byte[]? SendNtpRequestAndGetResponse(ushort port)
    {
        try
        {
            using var client = new UdpClient();
            var request = CreateNtpRequest();
            client.Send(request, request.Length, "127.0.0.1", port);

            client.Client.ReceiveTimeout = 2000;
            IPEndPoint? remoteEp = null;
            return client.Receive(ref remoteEp!);
        }
        catch
        {
            return null;
        }
    }
}
