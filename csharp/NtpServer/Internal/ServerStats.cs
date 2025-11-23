// Copyright (c) 2025 <Your Name>
namespace NtpServer.Internal;

/// <summary>
/// Server statistics snapshot.
/// </summary>
public class ServerStats
{
    /// <summary>Valid NTP datagrams processed</summary>
    public ulong PacketsReceived { get; set; }

    /// <summary>Responses sent successfully</summary>
    public ulong PacketsSent { get; set; }

    /// <summary>recvfrom() failures</summary>
    public ulong RecvErrors { get; set; }

    /// <summary>Datagrams shorter than NTP header</summary>
    public ulong DropShortPackets { get; set; }

    /// <summary>sendto() failures or partial sends</summary>
    public ulong SendErrors { get; set; }

    /// <summary>NotifyControlSnapshot sends</summary>
    public ulong PushNotifications { get; set; }

    /// <summary>Currently tracked client endpoints</summary>
    public ulong ActiveClients { get; set; }

    /// <summary>Latest error message (with code)</summary>
    public string LastError { get; set; } = string.Empty;
}
