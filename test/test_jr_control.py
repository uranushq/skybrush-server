"""Tests for the JR-control extension ARM packet construction."""

import struct

from flockwave.server.ext.jr_control.arm import (
    ARM_UDP_PORT,
    CMD,
    MAGIC,
    PACKET_SIZE,
    VERSION,
    build_packet,
    crc32_iso_hdlc,
    local_ipv4_addresses,
    resolve_broadcast_targets,
)


def test_arm_port_is_8765():
    assert ARM_UDP_PORT == 8765


def test_packet_size_and_magic():
    packet = build_packet(
        cmd=CMD["ARM"],
        seq=1,
        show_id=1,
        abs_time_us=1_000_000,
        start_time_us=6_000_000,
        fps_num=30,
        fps_den=1,
        frame_count=300,
        file_id=99,
    )
    assert len(packet) == PACKET_SIZE == 52

    magic, version, cmd = struct.unpack_from("<IHH", packet, 0)
    assert magic == MAGIC == 0x4A525054
    assert version == VERSION == 1
    assert cmd == CMD["ARM"] == 1


def test_packet_crc_matches_payload():
    packet = build_packet(
        cmd=CMD["ARM"],
        seq=7,
        show_id=2,
        abs_time_us=123_456_789,
        start_time_us=128_456_789,
        fps_num=25,
        fps_den=1,
        frame_count=500,
        file_id=12,
    )
    head, crc = packet[:48], struct.unpack_from("<I", packet, 48)[0]
    assert crc == crc32_iso_hdlc(head)


def test_crc32_known_vector():
    # CRC-32/ISO-HDLC of b"123456789" is the standard check value 0xCBF43926.
    assert crc32_iso_hdlc(b"123456789") == 0xCBF43926


def test_full_field_layout_roundtrip():
    fields = dict(
        cmd=CMD["LOAD_FILE"],
        seq=42,
        show_id=3,
        abs_time_us=10_000_000,
        start_time_us=15_000_000,
        fps_num=24,
        fps_den=2,
        frame_count=720,
        file_id=5,
    )
    packet = build_packet(**fields)
    (
        magic, version, cmd, seq, show_id,
        abs_time_us, start_time_us,
        fps_num, fps_den, frame_count,
        file_id, reserved,
    ) = struct.unpack_from("<IHHIIQQHHIII", packet, 0)
    assert magic == MAGIC
    assert version == VERSION
    assert cmd == fields["cmd"]
    assert seq == fields["seq"]
    assert show_id == fields["show_id"]
    assert abs_time_us == fields["abs_time_us"]
    assert start_time_us == fields["start_time_us"]
    assert (fps_num, fps_den) == (fields["fps_num"], fields["fps_den"])
    assert frame_count == fields["frame_count"]
    assert file_id == fields["file_id"]
    assert reserved == 0


# --------------------------------------------------------------------------- #
# broadcast target resolution (whole-subnet fan-out)
# --------------------------------------------------------------------------- #


def test_explicit_directed_broadcast_is_single_target():
    assert resolve_broadcast_targets("192.168.255.255") == [
        (None, "192.168.255.255")
    ]


def test_explicit_unicast_is_single_target():
    assert resolve_broadcast_targets("192.168.11.3") == [(None, "192.168.11.3")]


def test_no_target_fans_out_limited_broadcast():
    # None / "" / 255.255.255.255 all mean "broadcast across every interface".
    for target in (None, "", "255.255.255.255"):
        targets = resolve_broadcast_targets(target)
        assert len(targets) >= 1
        # Every destination is the limited broadcast address...
        assert all(dest == "255.255.255.255" for _bind, dest in targets)
        # ...and each is bound to a concrete interface when any were found.
        binds = [bind for bind, _dest in targets]
        assert binds == [None] or all(b is not None for b in binds)


def test_local_ipv4_addresses_are_strings():
    addresses = local_ipv4_addresses()
    assert isinstance(addresses, list)
    assert all(isinstance(ip, str) and not ip.startswith("127.") for ip in addresses)
