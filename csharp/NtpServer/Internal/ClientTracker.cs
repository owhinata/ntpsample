// Copyright (c) 2025 The NTP Sample Authors
using System.Net;
using System.Net.Sockets;

namespace NtpServer.Internal;

/// <summary>
/// Tracks active NTP client endpoints.
/// Records client socket addresses from received requests and updates
/// last-seen timestamps. Supports pruning of stale entries and iteration
/// over active clients for broadcasting control snapshots.
/// Thread-safe: All public methods are protected by an internal lock.
/// </summary>
public class ClientTracker
{
    /// <summary>Client endpoint record.</summary>
    public class Client
    {
        /// <summary>IPv4 endpoint (IP + port).</summary>
        public IPEndPoint EndPoint { get; set; } = null!;

        /// <summary>Last request absolute time (for diagnostics).</summary>
        public TimeSpec LastSeen { get; set; }

        /// <summary>Monotonic timestamp.</summary>
        public DateTime LastSeenMono { get; set; }
    }

    private readonly object _lock = new();
    private readonly List<Client> _clients = new();
    private TimeSpan _retention = TimeSpan.FromMinutes(60);

    /// <summary>
    /// Record or update a client endpoint.
    /// If the client (identified by IP address and port) already exists,
    /// updates its last_seen timestamp. Otherwise, adds a new entry to
    /// the tracked client list.
    /// </summary>
    public void Remember(IPEndPoint endPoint, TimeSpec now)
    {
        lock (_lock)
        {
            var nowMono = DateTime.UtcNow;
            var client = _clients.FirstOrDefault(c => c.EndPoint.Equals(endPoint));
            if (client != null)
            {
                client.LastSeen = now;
                client.LastSeenMono = nowMono;
            }
            else
            {
                _clients.Add(
                    new Client
                    {
                        EndPoint = endPoint,
                        LastSeen = now,
                        LastSeenMono = nowMono,
                    }
                );
            }
        }
    }

    /// <summary>
    /// Get a snapshot of all tracked clients.
    /// </summary>
    /// <returns>Copy of the client list (thread-safe).</returns>
    public List<Client> GetAll()
    {
        lock (_lock)
        {
            return new List<Client>(_clients);
        }
    }

    /// <summary>
    /// Remove clients that have not been seen within max_age.
    /// Uses monotonic timestamps instead of absolute time so that
    /// pruning remains correct even if the time source jumps.
    /// </summary>
    public void PruneStale(DateTime nowMono)
    {
        lock (_lock)
        {
            var effective = _retention;
            if (effective <= TimeSpan.Zero)
            {
                effective = TimeSpan.FromMinutes(60);
            }
            _clients.RemoveAll(c => (nowMono - c.LastSeenMono) > effective);
        }
    }

    /// <summary>
    /// Configure retention duration for clients (default 60 minutes).
    /// </summary>
    public void SetRetention(TimeSpan retention)
    {
        lock (_lock)
        {
            _retention = retention;
        }
    }

    /// <summary>
    /// Get current count of tracked clients.
    /// </summary>
    public int Count
    {
        get
        {
            lock (_lock)
            {
                return _clients.Count;
            }
        }
    }
}
