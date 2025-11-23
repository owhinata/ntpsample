// Copyright (c) 2025 The NTP Sample Authors
using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using NtpServer.Internal;

namespace NtpServer;

/// <summary>
/// Minimal NTPv4 server (UDP/IPv4).
/// Server operates in mode 4 (server). Default stratum is 1 (local reference).
/// </summary>
public class NtpServer : IDisposable
{
    private enum State : byte
    {
        Stopped,
        Starting,
        Running,
        Stopping,
    }

    private int _state = (int)State.Stopped; // Atomic state machine
    private Thread? _workerThread;
    private UdpClient? _socket;
    private ITimeSource? _timeSource;
    private Options? _options;
    private uint _epoch;
    private TimeSpec _epochAbs;
    private double _epochRate;
    private readonly ClientTracker _clientTracker = new();
    private readonly StatsTracker _stats = new();
    private CancellationTokenSource? _cts;

    public NtpServer()
    {
        _epoch = 0;
    }

    /// <summary>
    /// Starts serving on the given UDP port.
    /// </summary>
    /// <param name="port">UDP port to bind (default: 9123).</param>
    /// <param name="timeSource">Time source for timestamps (default: new StopwatchClock).</param>
    /// <param name="options">Immutable configuration snapshot (defaults applied).</param>
    /// <returns>true on success, false on failure.</returns>
    public bool Start(ushort port = 9123, ITimeSource? timeSource = null, Options? options = null)
    {
        // CAS to prevent concurrent Start calls
        var previous = Interlocked.CompareExchange(
            ref _state,
            (int)State.Starting,
            (int)State.Stopped
        );
        if ((State)previous != State.Stopped)
        {
            // Already running, starting, or stopping
            return (State)previous == State.Running;
        }

        // Increment epoch number
        _epoch++;

        // Set time source (use StopwatchClock if null)
        _timeSource = timeSource ?? new StopwatchClock();
        _options = options ?? new Options.Builder().Build();

        // Capture ABS/RATE values for this epoch
        _epochAbs = _timeSource.NowUnix();
        _epochRate = _timeSource.GetRate();

        _options.LogSink?.Invoke(
            $"[DEBUG Server] Start: epoch={_epoch} epoch_abs={_epochAbs} epoch_rate={_epochRate}"
        );

        // Configure client retention
        _clientTracker.SetRetention(_options.ClientRetention);

        // Create and bind UDP socket
        try
        {
            _socket = new UdpClient(port);
            _socket.Client.ReceiveTimeout = 100; // 100ms timeout for graceful shutdown
        }
        catch (Exception ex)
        {
            _stats.SetLastError($"Failed to bind port {port}: {ex.Message}");
            _options.LogSink?.Invoke($"[ERROR] {_stats.GetLastError()}");
            Interlocked.Exchange(ref _state, (int)State.Stopped);
            return false;
        }

        // Check if Stop() was called during initialization (Starting -> Stopping)
        if (
            (State)Interlocked.CompareExchange(ref _state, (int)State.Stopping, (int)State.Stopping)
            == State.Stopping
        )
        {
            _socket?.Close();
            _socket = null;
            _timeSource = null;
            Interlocked.Exchange(ref _state, (int)State.Stopped);
            return false;
        }

        // Start worker thread
        _cts = new CancellationTokenSource();

        // Set Running BEFORE thread creation to avoid race
        Interlocked.Exchange(ref _state, (int)State.Running);
        _workerThread = new Thread(WorkerLoop) { IsBackground = false, Name = "NtpServer" };
        _workerThread.Start();

        // Push notification if ClientTracker not empty
        if (_clientTracker.Count > 0)
        {
            NotifyControlSnapshot();
        }

        _options.LogSink?.Invoke($"[INFO] NTP server started on port {port}");
        return true;
    }

    /// <summary>Stops the server. Safe to call multiple times.</summary>
    public void Stop()
    {
        // Try to transition from Running or Starting to Stopping
        int expected = (int)State.Running;
        if (Interlocked.CompareExchange(ref _state, (int)State.Stopping, expected) != expected)
        {
            // If not Running, try Starting
            expected = (int)State.Starting;
            if (Interlocked.CompareExchange(ref _state, (int)State.Stopping, expected) != expected)
            {
                // Already Stopped or Stopping
                return;
            }
        }

        _options?.LogSink?.Invoke("[INFO] Stopping NTP server...");

        // Close socket first to unblock receive thread
        _socket?.Close();

        // Wait for worker thread to finish
        _workerThread?.Join();

        // Clean up resources
        _cts?.Dispose();
        _socket = null;
        _workerThread = null;
        _cts = null;

        Interlocked.Exchange(ref _state, (int)State.Stopped);
        _options?.LogSink?.Invoke("[INFO] NTP server stopped");
    }

    /// <summary>Returns latest statistics snapshot (thread-safe).</summary>
    public ServerStats GetStats()
    {
        return _stats.Snapshot((ulong)_clientTracker.Count);
    }

    public void Dispose()
    {
        Stop();
    }

    private void WorkerLoop()
    {
        var pruneTimer = DateTime.UtcNow;
        var token = _cts?.Token ?? CancellationToken.None;

        while (
            (State)Interlocked.CompareExchange(ref _state, (int)State.Running, (int)State.Running)
                == State.Running
            && !token.IsCancellationRequested
        )
        {
            try
            {
                // Prune stale clients periodically (every 60 seconds)
                var now = DateTime.UtcNow;
                if ((now - pruneTimer).TotalSeconds >= 60)
                {
                    _clientTracker.PruneStale(now);
                    pruneTimer = now;
                }

                // Receive NTP request
                IPEndPoint? remoteEp = null;
                byte[] rxData;
                try
                {
                    rxData = _socket!.Receive(ref remoteEp!);
                }
                catch (SocketException ex) when (ex.SocketErrorCode == SocketError.TimedOut)
                {
                    continue; // Timeout, check _running and retry
                }
                catch (Exception ex)
                {
                    if (
                        (State)
                            Interlocked.CompareExchange(
                                ref _state,
                                (int)State.Running,
                                (int)State.Running
                            ) != State.Running
                    )
                        break; // Socket closed during shutdown
                    _stats.IncRecvErrors();
                    _stats.SetLastError($"Receive error: {ex.Message}");
                    continue;
                }

                _stats.IncPacketsReceived();

                // Validate packet size
                if (rxData.Length < 48) // NTP packet is 48 bytes
                {
                    _stats.IncShortPackets();
                    continue;
                }

                // Parse NTP request
                var reqPacket = BytesToNtpPacket(rxData);
                var tRecv = _timeSource!.NowUnix();

                // Remember client for push notifications
                _clientTracker.Remember(remoteEp!, tRecv);

                // Build response
                var tTx = _timeSource.NowUnix();
                var tRef = _epochAbs; // Reference timestamp is epoch start time

                var respPacket = BuildResponse(
                    reqPacket,
                    _options!.Stratum,
                    _options.Precision,
                    _options.RefId,
                    tRef,
                    tRecv,
                    tTx
                );

                // Build vendor extension field
                var vendorExt = BuildVendorExtension(tTx);

                // Compose full response
                var txData = NtpPacketHelper.ComposeNtpPacketWithEf(respPacket, vendorExt);

                // Send response
                try
                {
                    _socket.Send(txData, txData.Length, remoteEp);
                    _stats.IncPacketsSent();
                }
                catch (Exception ex)
                {
                    _stats.IncSendErrors();
                    _stats.SetLastError($"Send error: {ex.Message}");
                }
            }
            catch (Exception ex)
            {
                _options?.LogSink?.Invoke($"[ERROR] Worker loop exception: {ex}");
            }
        }
    }

    private static NtpPacket BytesToNtpPacket(byte[] data)
    {
        if (data.Length < 48)
            throw new ArgumentException("Data too short for NTP packet", nameof(data));

        // Parse from network byte order (big-endian) to host byte order
        var span = data.AsSpan();
        return new NtpPacket
        {
            LiVnMode = span[0],
            Stratum = span[1],
            Poll = span[2],
            Precision = (sbyte)span[3],
            RootDelay = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(4, 4)),
            RootDispersion = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(8, 4)),
            RefId = BinaryPrimitives.ReadUInt32BigEndian(span.Slice(12, 4)),
            RefTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(16, 8)),
            OrigTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(24, 8)),
            RecvTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(32, 8)),
            TxTimestamp = BinaryPrimitives.ReadUInt64BigEndian(span.Slice(40, 8)),
        };
    }

    private static NtpPacket BuildResponse(
        NtpPacket req,
        byte stratum,
        sbyte precision,
        uint refIdHost,
        TimeSpec tRef,
        TimeSpec tRecv,
        TimeSpec tTx
    )
    {
        var resp = new NtpPacket
        {
            LiVnMode = (byte)((0 << 6) | (4 << 3) | 4), // LI=0, VN=4, Mode=4 (server)
            Stratum = stratum,
            Poll = 4,
            Precision = precision,
            RootDelay = 0,
            RootDispersion = 0,
            RefId = refIdHost, // Host byte order (ComposeNtpPacketWithEf will convert)
            RefTimestamp = tRef.ToNtpTimestamp(), // Host byte order
            OrigTimestamp = req.TxTimestamp, // Echo client's transmit timestamp (already network order)
            RecvTimestamp = tRecv.ToNtpTimestamp(), // Host byte order
            TxTimestamp = tTx.ToNtpTimestamp(), // Host byte order
        };
        return resp;
    }

    private void NotifyControlSnapshot()
    {
        var ts = _timeSource;
        if (ts == null || _socket == null)
            return;

        TimeSpec now = ts.NowUnix();

        // Build mode=5 (broadcast) packet for Push notifications
        var pushPacket = new NtpPacket
        {
            LiVnMode = (byte)((0 << 6) | (4 << 3) | 5), // LI=0, VN=4, Mode=5 (broadcast)
            Stratum = _options!.Stratum,
            Poll = 4,
            Precision = _options.Precision,
            RootDelay = 0,
            RootDispersion = 0,
            RefId = _options.RefId,
            RefTimestamp = now.ToNtpTimestamp(),
            OrigTimestamp = 0,
            RecvTimestamp = now.ToNtpTimestamp(),
            TxTimestamp = now.ToNtpTimestamp(),
        };

        // Build vendor extension with Push flag
        var payload = new NtpVendorExt.Payload
        {
            Seq = _epoch,
            ServerTime = now,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate | NtpVendorExt.FlagPush,
            AbsTime = _epochAbs,
            RateScale = _epochRate,
        };
        byte[] vendorExt = NtpVendorExt.Serialize(payload);

        // Compose full packet
        byte[] buf = NtpPacketHelper.ComposeNtpPacketWithEf(pushPacket, vendorExt);

        // Prune stale clients
        _clientTracker.PruneStale(DateTime.UtcNow);

        // Send to all clients
        var clients = _clientTracker.GetAll();
        foreach (var client in clients)
        {
            try
            {
                _socket.Send(buf, buf.Length, client.EndPoint);
                _stats.IncPushNotifications();
            }
            catch
            {
                // Ignore send errors for push notifications
            }
        }

        _options.LogSink?.Invoke($"[INFO] Sent Push notifications to {clients.Count} clients");
    }

    private byte[] BuildVendorExtension(TimeSpec serverTime)
    {
        var payload = new NtpVendorExt.Payload
        {
            Seq = _epoch,
            ServerTime = serverTime,
            Flags = NtpVendorExt.FlagAbs | NtpVendorExt.FlagRate,
            AbsTime = _epochAbs,
            RateScale = _epochRate,
        };
        return NtpVendorExt.Serialize(payload);
    }

    private class StatsTracker
    {
        private long _packetsReceived;
        private long _packetsSent;
        private long _recvErrors;
        private long _shortPackets;
        private long _sendErrors;
        private long _pushNotifications;
        private readonly object _lock = new();
        private string _lastError = string.Empty;

        public void Reset()
        {
            Interlocked.Exchange(ref _packetsReceived, 0);
            Interlocked.Exchange(ref _packetsSent, 0);
            Interlocked.Exchange(ref _recvErrors, 0);
            Interlocked.Exchange(ref _shortPackets, 0);
            Interlocked.Exchange(ref _sendErrors, 0);
            Interlocked.Exchange(ref _pushNotifications, 0);
            lock (_lock)
            {
                _lastError = string.Empty;
            }
        }

        public void IncPacketsReceived() => Interlocked.Increment(ref _packetsReceived);

        public void IncPacketsSent() => Interlocked.Increment(ref _packetsSent);

        public void IncRecvErrors() => Interlocked.Increment(ref _recvErrors);

        public void IncShortPackets() => Interlocked.Increment(ref _shortPackets);

        public void IncSendErrors() => Interlocked.Increment(ref _sendErrors);

        public void IncPushNotifications() => Interlocked.Increment(ref _pushNotifications);

        public void SetLastError(string text)
        {
            lock (_lock)
            {
                _lastError = text;
            }
        }

        public string GetLastError()
        {
            lock (_lock)
            {
                return _lastError;
            }
        }

        public ServerStats Snapshot(ulong activeClients)
        {
            return new ServerStats
            {
                PacketsReceived = (ulong)Interlocked.Read(ref _packetsReceived),
                PacketsSent = (ulong)Interlocked.Read(ref _packetsSent),
                RecvErrors = (ulong)Interlocked.Read(ref _recvErrors),
                DropShortPackets = (ulong)Interlocked.Read(ref _shortPackets),
                SendErrors = (ulong)Interlocked.Read(ref _sendErrors),
                PushNotifications = (ulong)Interlocked.Read(ref _pushNotifications),
                ActiveClients = activeClients,
                LastError = GetLastError(),
            };
        }
    }
}
