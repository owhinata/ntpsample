using System.Net;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using NtpServer.Internal;

namespace NtpServer.Tests;

[TestClass]
public class ClientTrackerTests
{
    [TestMethod]
    public void Constructor_CreatesNewInstance()
    {
        var tracker = new ClientTracker();
        Assert.IsNotNull(tracker);
        Assert.AreEqual(0, tracker.Count);
    }

    [TestMethod]
    public void Remember_AddsNewClient()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.Remember(endpoint, now);

        Assert.AreEqual(1, tracker.Count);
    }

    [TestMethod]
    public void Remember_UpdatesExistingClient()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var time1 = new TimeSpec(1000000000, 0);
        var time2 = new TimeSpec(1000000100, 0);

        tracker.Remember(endpoint, time1);
        tracker.Remember(endpoint, time2);

        // Should still have only 1 client
        Assert.AreEqual(1, tracker.Count);

        var clients = tracker.GetAll();
        Assert.HasCount(1, clients);
        Assert.AreEqual(time2.Sec, clients[0].LastSeen.Sec);
    }

    [TestMethod]
    public void Remember_DistinguishesDifferentEndpoints()
    {
        var tracker = new ClientTracker();
        var endpoint1 = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var endpoint2 = new IPEndPoint(IPAddress.Parse("192.168.1.101"), 12345);
        var endpoint3 = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12346);
        var now = new TimeSpec(1000000000, 0);

        tracker.Remember(endpoint1, now);
        tracker.Remember(endpoint2, now);
        tracker.Remember(endpoint3, now);

        // Should have 3 different clients (different IP or different port)
        Assert.AreEqual(3, tracker.Count);
    }

    [TestMethod]
    public void GetAll_ReturnsAllClients()
    {
        var tracker = new ClientTracker();
        var endpoint1 = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var endpoint2 = new IPEndPoint(IPAddress.Parse("192.168.1.101"), 12346);
        var now = new TimeSpec(1000000000, 0);

        tracker.Remember(endpoint1, now);
        tracker.Remember(endpoint2, now);

        var clients = tracker.GetAll();
        Assert.HasCount(2, clients);
    }

    [TestMethod]
    public void GetAll_ReturnsCopyOfList()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.Remember(endpoint, now);
        var clients1 = tracker.GetAll();

        // Modify the returned list
        clients1.Clear();

        // Original should still have 1 client
        Assert.AreEqual(1, tracker.Count);
        var clients2 = tracker.GetAll();
        Assert.HasCount(1, clients2);
    }

    [TestMethod]
    public void PruneStale_RemovesOldClients()
    {
        var tracker = new ClientTracker();
        var endpoint1 = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var endpoint2 = new IPEndPoint(IPAddress.Parse("192.168.1.101"), 12346);
        var now = new TimeSpec(1000000000, 0);

        tracker.SetRetention(TimeSpan.FromSeconds(30));

        tracker.Remember(endpoint1, now);
        Thread.Sleep(100); // Wait a bit
        tracker.Remember(endpoint2, now);

        // Prune with a time 35 seconds in the future
        tracker.PruneStale(DateTime.UtcNow.AddSeconds(35));

        // Both clients should be removed (> 30 seconds old)
        Assert.AreEqual(0, tracker.Count);
    }

    [TestMethod]
    public void PruneStale_KeepsRecentClients()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.SetRetention(TimeSpan.FromMinutes(60));

        tracker.Remember(endpoint, now);

        // Prune with a time 10 seconds in the future
        tracker.PruneStale(DateTime.UtcNow.AddSeconds(10));

        // Client should still be there (< 60 minutes old)
        Assert.AreEqual(1, tracker.Count);
    }

    [TestMethod]
    public void SetRetention_UpdatesRetentionPeriod()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.SetRetention(TimeSpan.FromSeconds(1));
        tracker.Remember(endpoint, now);

        Thread.Sleep(1500); // Wait 1.5 seconds

        tracker.PruneStale(DateTime.UtcNow);

        // Client should be removed (> 1 second old)
        Assert.AreEqual(0, tracker.Count);
    }

    [TestMethod]
    public void PruneStale_WithZeroRetention_UsesDefault()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.SetRetention(TimeSpan.Zero);
        tracker.Remember(endpoint, now);

        // Prune with a time 30 minutes in the future
        tracker.PruneStale(DateTime.UtcNow.AddMinutes(30));

        // Client should still be there (default is 60 minutes)
        Assert.AreEqual(1, tracker.Count);
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentRemember()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        var tasks = new Task[10];
        for (int i = 0; i < 10; i++)
        {
            tasks[i] = Task.Run(() =>
            {
                for (int j = 0; j < 100; j++)
                {
                    tracker.Remember(endpoint, now);
                }
            });
        }

        Task.WaitAll(tasks);

        // Should have only 1 client despite concurrent adds
        Assert.AreEqual(1, tracker.Count);
    }

    [TestMethod]
    public void ThreadSafety_ConcurrentReadAndWrite()
    {
        var tracker = new ClientTracker();
        var endpoint = new IPEndPoint(IPAddress.Parse("192.168.1.100"), 12345);
        var now = new TimeSpec(1000000000, 0);

        tracker.Remember(endpoint, now);

        var tasks = new Task[10];
        for (int i = 0; i < 10; i++)
        {
            int index = i;
            tasks[i] = Task.Run(() =>
            {
                if (index % 2 == 0)
                {
                    // Read
                    var clients = tracker.GetAll();
                    Assert.IsNotNull(clients);
                }
                else
                {
                    // Write
                    tracker.Remember(endpoint, now);
                }
            });
        }

        Task.WaitAll(tasks);

        Assert.AreEqual(1, tracker.Count);
    }
}
