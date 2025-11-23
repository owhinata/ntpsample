// Copyright (c) 2025 The NTP Sample Authors
using System.Net;
using System.Net.Sockets;
using NtpServer.Internal;

namespace NtpClock.Internal;

/// <summary>
/// UDP socket manager with background receive thread.
/// Manages a single UDP socket for NTP communication, keeping it open to
/// receive both Exchange responses (request/response) and Push notifications
/// (server-initiated). A background receive thread continuously reads from
/// the socket, classifies messages based on vendor extension flags, and
/// queues them for consumption by the main synchronization loop.
/// </summary>
public class UdpSocket : IDisposable
{
    /// <summary>Callback for logging messages (thread-safe).</summary>
    public delegate void LogCallback(string message);

    /// <summary>Message type classification.</summary>
    public enum MessageType
    {
        /// <summary>Response to a client request.</summary>
        ExchangeResponse,

        /// <summary>Server-initiated notification.</summary>
        Push,
    }

    /// <summary>Received message with metadata.</summary>
    public struct Message
    {
        /// <summary>Classified message type.</summary>
        public MessageType Type;

        /// <summary>Raw packet bytes.</summary>
        public byte[] Data;

        /// <summary>Reception timestamp.</summary>
        public TimeSpec RecvTime;
    }

    private const int NtpPacketSize = 48;

    private Socket? _socket;
    private IPEndPoint? _serverEndpoint;
    private Func<TimeSpec>? _getTime;
    private LogCallback? _logCallback;
    private volatile bool _running;
    private Thread? _recvThread;

    private readonly object _queueLock = new object();
    private readonly Queue<Message> _msgQueue = new Queue<Message>();
    private readonly ManualResetEventSlim _queueEvent = new ManualResetEventSlim(false);

    /// <summary>
    /// Open socket and start background receive thread.
    /// Creates a UDP socket, binds to an ephemeral port, connects to the
    /// specified server address, and starts a background thread to receive
    /// and classify incoming messages.
    /// </summary>
    /// <param name="serverIp">Server IPv4 address (numeric string, no DNS).</param>
    /// <param name="serverPort">Server UDP port.</param>
    /// <param name="getTime">Callback to get current time.</param>
    /// <param name="logCallback">Optional logging callback.</param>
    /// <returns>true on success, false on failure.</returns>
    public bool Open(
        string serverIp,
        ushort serverPort,
        Func<TimeSpec> getTime,
        LogCallback? logCallback = null
    )
    {
        if (_socket != null)
        {
            return false; // Already open
        }

        _getTime = getTime;
        _logCallback = logCallback;

        try
        {
            // Create UDP socket
            _socket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

            // Bind to any local address/ephemeral port (port 0 = OS chooses)
            _socket.Bind(new IPEndPoint(IPAddress.Any, 0));

            // Store server endpoint for Send()
            _serverEndpoint = new IPEndPoint(IPAddress.Parse(serverIp), serverPort);

            // Start receive thread
            _running = true;
            _recvThread = new Thread(ReceiveLoop)
            {
                IsBackground = false,
                Name = "UdpSocket.ReceiveLoop",
            };
            _recvThread.Start();

            return true;
        }
        catch (Exception ex)
        {
            LogError($"Socket initialization failed: {ex.Message}");
            _socket?.Close();
            _socket = null;
            return false;
        }
    }

    /// <summary>
    /// Close socket and stop background thread.
    /// Signals the receive thread to stop, closes the socket (which unblocks
    /// recvfrom), and waits for the thread to terminate. Safe to call
    /// multiple times.
    /// </summary>
    public void Close()
    {
        if (_socket == null)
        {
            return;
        }

        // Signal thread to stop
        _running = false;

        // Close socket to unblock Receive()
        _socket?.Close();
        _socket = null;

        // Wait for thread to finish
        _recvThread?.Join();
        _recvThread = null;

        // Clear message queue
        lock (_queueLock)
        {
            _msgQueue.Clear();
        }
    }

    /// <summary>
    /// Send data to the connected server.
    /// </summary>
    /// <param name="data">Bytes to send (typically an NTP request packet).</param>
    /// <returns>true if send succeeded, false otherwise.</returns>
    public bool Send(byte[] data)
    {
        if (_socket == null || _serverEndpoint == null)
        {
            return false;
        }

        try
        {
            _socket.SendTo(data, _serverEndpoint);
            return true;
        }
        catch (Exception ex)
        {
            LogError($"Send failed: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// Wait for a message with timeout.
    /// Blocks until a message is available in the queue or the timeout expires.
    /// Messages are classified by the background receive thread.
    /// </summary>
    /// <param name="timeoutMs">Maximum time to wait in milliseconds.</param>
    /// <param name="outMsg">Pointer to store the received message.</param>
    /// <returns>true if a message was received, false on timeout.</returns>
    public bool WaitMessage(int timeoutMs, out Message outMsg)
    {
        outMsg = default;

        lock (_queueLock)
        {
            // Check if message is already available
            if (_msgQueue.Count > 0)
            {
                outMsg = _msgQueue.Dequeue();
                if (_msgQueue.Count == 0)
                {
                    _queueEvent.Reset();
                }
                return true;
            }
        }

        // Wait with timeout for message availability
        if (timeoutMs <= 0)
        {
            return false;
        }

        if (!_queueEvent.Wait(timeoutMs))
        {
            return false;
        }

        lock (_queueLock)
        {
            if (_msgQueue.Count == 0)
            {
                return false;
            }

            outMsg = _msgQueue.Dequeue();
            if (_msgQueue.Count == 0)
            {
                _queueEvent.Reset();
            }
            return true;
        }
    }

    /// <summary>
    /// Check if socket is currently open.
    /// </summary>
    /// <returns>true if socket is open and receive thread is running.</returns>
    public bool IsOpen()
    {
        return _socket != null && _socket.Connected == false && _running;
    }

    public void Dispose()
    {
        Close();
        _queueEvent.Dispose();
    }

    private void ReceiveLoop()
    {
        byte[] buffer = new byte[1500]; // Max UDP payload
        EndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        while (_running)
        {
            try
            {
                // Check if socket is available for reading (with timeout)
                if (_socket == null || !_socket.Poll(200000, SelectMode.SelectRead)) // 200ms timeout
                {
                    if (!_running)
                    {
                        break; // Shutdown requested
                    }
                    continue; // Timeout, check running flag again
                }

                // Receive packet
                int bytesRead = _socket.ReceiveFrom(buffer, ref remoteEndPoint);

                // Ignore packets smaller than minimum NTP packet size
                if (bytesRead < NtpPacketSize)
                {
                    if (bytesRead > 0)
                    {
                        LogError($"Received short packet ({bytesRead} bytes)");
                    }
                    continue;
                }

                // Get receive timestamp
                TimeSpec recvTime = _getTime?.Invoke() ?? new TimeSpec();

                // Copy data
                byte[] data = new byte[bytesRead];
                Array.Copy(buffer, data, bytesRead);

                // Classify message type
                MessageType type = ClassifyMessage(data);

                // Add to queue
                lock (_queueLock)
                {
                    _msgQueue.Enqueue(
                        new Message
                        {
                            Type = type,
                            Data = data,
                            RecvTime = recvTime,
                        }
                    );
                    _queueEvent.Set();
                }
            }
            catch (SocketException)
            {
                if (_running)
                {
                    // Unexpected error while still running
                    continue;
                }
                else
                {
                    // Socket closed intentionally during shutdown
                    break;
                }
            }
            catch (Exception ex)
            {
                if (_running)
                {
                    LogError($"Receive failed: {ex.Message}");
                }
                break;
            }
        }
    }

    private MessageType ClassifyMessage(byte[] data)
    {
        // Default to ExchangeResponse
        if (data.Length <= NtpPacketSize)
        {
            return MessageType.ExchangeResponse;
        }

        // Check for vendor extension field
        int offset = NtpPacketSize;
        int remain = data.Length - NtpPacketSize;

        // Need at least 4 bytes for EF header (type + length)
        if (remain < 4)
        {
            return MessageType.ExchangeResponse;
        }

        // Parse EF header (big-endian)
        ushort efType = (ushort)((data[offset] << 8) | data[offset + 1]);
        ushort efLen = (ushort)((data[offset + 2] << 8) | data[offset + 3]);

        // Check if this is our vendor hint EF
        if (efType != NtpVendorExt.EfTypeVendorHint || efLen < 4 || efLen > remain)
        {
            return MessageType.ExchangeResponse;
        }

        // Extract vendor payload (skip 4-byte EF header)
        byte[] payload = new byte[efLen - 4];
        Array.Copy(data, offset + 4, payload, 0, efLen - 4);

        // Parse vendor extension
        if (!NtpVendorExt.Parse(payload, out var vendor) || vendor == null)
        {
            return MessageType.ExchangeResponse;
        }

        // Check for Push flag
        if ((vendor.Flags & NtpVendorExt.FlagPush) != 0)
        {
            return MessageType.Push;
        }

        return MessageType.ExchangeResponse;
    }

    private void LogError(string text)
    {
        _logCallback?.Invoke(text);
    }
}
