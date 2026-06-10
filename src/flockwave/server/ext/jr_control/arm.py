"""JRPT ARM sync-packet construction and UDP broadcast.

1:1 port of ``JR_precise_timing/tools/sync_broadcast.py`` and
``shared/sync_protocol.h``. The 52-byte little-endian packet is::

    magic       u32   0x4A525054 ('JRPT')
    version     u16   1
    cmd         u16   ARM=1, ABORT=2, PING=3, LOAD_FILE=4
    seq         u32
    show_id     u32
    abs_time_us u64   controller TX time (us since epoch)
    start_time_us u64 playback start time (us since epoch)
    fps_num     u16
    fps_den     u16
    frame_count u32
    file_id     u32   DR loads file<id>.bin
    reserved    u32   0
    crc32       u32   CRC-32/ISO-HDLC over the first 48 bytes

The JR firmware snaps ``start_time_us`` to the nearest GPS-PPS second, so a
controller clock error of up to +/-500 ms is absorbed.
"""

from __future__ import annotations

import socket
import struct
import time

__all__ = (
    "ARM_UDP_PORT",
    "CMD",
    "MAGIC",
    "VERSION",
    "PACKET_SIZE",
    "build_packet",
    "broadcast_arm",
    "crc32_iso_hdlc",
    "local_ipv4_addresses",
    "now_us",
    "resolve_broadcast_targets",
)

MAGIC = 0x4A525054  # 'JRPT'
VERSION = 1
ARM_UDP_PORT = 8765
PACKET_SIZE = 52

CMD = {
    "NONE": 0,
    "ARM": 1,
    "ABORT": 2,
    "PING": 3,
    "LOAD_FILE": 4,
}

_FMT_NO_CRC = "<IHHIIQQHHIII"  # 48 bytes


def crc32_iso_hdlc(buf: bytes) -> int:
    """CRC-32/ISO-HDLC (poly 0xEDB88320) — matches firmware ``sync_crc32``."""
    c = 0xFFFFFFFF
    for b in buf:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 & -(c & 1))
    return c ^ 0xFFFFFFFF


def now_us() -> int:
    """Current wall-clock time in microseconds since the Unix epoch."""
    return int(time.time() * 1_000_000)


def build_packet(
    *,
    cmd: int,
    seq: int,
    show_id: int,
    abs_time_us: int,
    start_time_us: int,
    fps_num: int,
    fps_den: int,
    frame_count: int,
    file_id: int,
) -> bytes:
    """Build a complete 52-byte JRPT packet (header + CRC)."""
    head = struct.pack(
        _FMT_NO_CRC,
        MAGIC,
        VERSION,
        cmd,
        seq,
        show_id,
        abs_time_us,
        start_time_us,
        fps_num,
        fps_den,
        frame_count,
        file_id,
        0,  # reserved
    )
    assert len(head) == 48, f"head={len(head)}"
    crc = crc32_iso_hdlc(head)
    packet = head + struct.pack("<I", crc)
    assert len(packet) == PACKET_SIZE
    return packet


def local_ipv4_addresses() -> list[str]:
    """Best-effort enumeration of this host's non-loopback IPv4 addresses.

    Used so the ARM broadcast can be emitted out of *every* interface the
    control PC is attached to — a limited broadcast to 255.255.255.255 only
    egresses a single (OS-chosen) interface, which can silently miss boards on
    another NIC's subnet.
    """
    ips: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                ips.add(ip)
    except OSError:
        pass

    # Fallback: the UDP "connect to a public IP" trick reveals the primary
    # outbound interface address even when gethostname() resolves poorly.
    if not ips:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("8.8.8.8", 80))
            ips.add(probe.getsockname()[0])
        except OSError:
            pass
        finally:
            probe.close()

    return sorted(ips)


def resolve_broadcast_targets(
    target: str | None,
) -> list[tuple[str | None, str]]:
    """Resolve the destinations a broadcast should be sent to.

    Returns a list of ``(bind_source_ip_or_None, destination_ip)`` pairs.

    * An explicit unicast/directed-broadcast ``target`` (e.g. ``192.168.255.255``
      or a single board IP) yields a single, un-bound destination.
    * Otherwise the packet is fanned out as a limited broadcast
      (``255.255.255.255``) bound to *each* local interface, so the whole
      subnet on every attached NIC receives it.
    """
    if target and target not in ("", "0.0.0.0", "255.255.255.255"):
        return [(None, target)]

    interfaces = local_ipv4_addresses()
    if interfaces:
        return [(ip, "255.255.255.255") for ip in interfaces]
    return [(None, "255.255.255.255")]


def broadcast_arm(
    *,
    start_in: float = 5.0,
    fps_num: int = 30,
    fps_den: int = 1,
    frame_count: int = 300,
    show_id: int = 1,
    file_id: int = 99,
    seq: int = 1,
    repeat: int = 5,
    interval: float = 0.05,
    target: str | None = None,
    port: int = ARM_UDP_PORT,
    cmd: str = "ARM",
) -> dict:
    """Broadcast an ARM (or other) command over UDP.

    When ``target`` is ``None`` (the default) the packet is emitted as a limited
    broadcast out of *every* local interface so all boards on the subnet receive
    it simultaneously; pass an explicit address to unicast or send a directed
    subnet broadcast (e.g. ``192.168.255.255``).

    The packet is retransmitted ``repeat`` times ``interval`` seconds apart to
    survive UDP loss. ``interval`` must stay below 0.1 s so all retransmits land
    inside the same PPS-second budget. All boards still start at the absolute
    ``start_time_us`` carried in the packet (PPS-snapped), so exact send timing
    only affects delivery, not synchronisation.

    Returns a small summary dict describing what was sent.

    Note: this performs blocking ``socket`` sends and short ``time.sleep`` calls;
    call it from a worker thread (e.g. ``trio.to_thread.run_sync``) so it does
    not block the async event loop.
    """
    if interval >= 0.1:
        raise ValueError("interval must be < 0.1s to protect the PPS budget")
    if cmd not in CMD:
        raise ValueError(f"unknown cmd {cmd!r}; one of {sorted(CMD)}")

    cmd_id = CMD[cmd]
    abs_t = now_us()
    start_t = abs_t + int(start_in * 1_000_000) if cmd_id == CMD["ARM"] else 0

    destinations = resolve_broadcast_targets(target)

    # Open one socket per (bound) interface up front so every retransmit reaches
    # all interfaces with minimal jitter.
    sockets: list[tuple[socket.socket, str]] = []
    for bind_ip, dest in destinations:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if bind_ip is not None:
            try:
                sock.bind((bind_ip, 0))
            except OSError:
                # Skip interfaces we cannot bind to rather than aborting.
                sock.close()
                continue
        sockets.append((sock, dest))

    sent = 0
    try:
        current_seq = seq
        for i in range(repeat):
            packet = build_packet(
                cmd=cmd_id,
                seq=current_seq,
                show_id=show_id,
                abs_time_us=abs_t if i == 0 else now_us(),
                start_time_us=start_t,
                fps_num=fps_num,
                fps_den=fps_den,
                frame_count=frame_count,
                file_id=file_id,
            )
            for sock, dest in sockets:
                try:
                    sock.sendto(packet, (dest, port))
                    sent += 1
                except OSError:
                    # One interface failing should not stop the others.
                    continue
            current_seq += 1
            if i + 1 < repeat:
                time.sleep(interval)
    finally:
        for sock, _dest in sockets:
            sock.close()

    return {
        "cmd": cmd,
        "targets": [f"{dest}:{port}" for _bind, dest in destinations],
        "interfaces": [bind for bind, _dest in destinations if bind],
        "sent": sent,
        "absTimeUs": abs_t,
        "startTimeUs": start_t,
        "startIn": start_in if cmd_id == CMD["ARM"] else 0,
        "fps": f"{fps_num}/{fps_den}",
        "frameCount": frame_count,
        "showId": show_id,
        "fileId": file_id,
    }
