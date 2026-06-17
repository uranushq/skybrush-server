"""Tests for YAW_CONTROL binary encoding."""

import struct

import pytest

from flockwave.server.show.yaw_control import (
    encode_yaw_control_block,
    encode_yaw_control_from_show,
    yaw_at_time_from_payload,
)


def test_constant_yaw_zero():
    payload = encode_yaw_control_block(
        {"autoYaw": False, "setpoints": [[0, 0], [10, 0], [22, 0]]}
    )
    assert payload == b"\x00\x00\x00"


def test_constant_yaw_negative():
    payload = encode_yaw_control_block(
        {"autoYaw": False, "setpoints": [[0, -44], [5, -44]]}
    )
    assert payload == struct.pack("<Bh", 0, -440)


def test_user_example_two_second_ramp():
    """Match documented 11-byte fixture: 30° + two 1s segments of +3.6°."""
    payload = encode_yaw_control_block(
        {
            "autoYaw": False,
            "setpoints": [[0, 30.0], [1.0, 33.6], [2.0, 37.2]],
        }
    )
    assert payload == bytes.fromhex("002c01e8032400e8032400")
    assert len(payload) == 11


def test_ramp_with_subdegree_steps():
    """Two 1s segments with +0.2° per second."""
    payload = encode_yaw_control_block(
        {
            "autoYaw": False,
            "setpoints": [[0, 4.0], [1.0, 4.2], [2.0, 4.4]],
        }
    )
    assert payload == bytes.fromhex("002800e8030200e8030200")
    assert yaw_at_time_from_payload(payload, 1.5) == pytest.approx(4.3)


def test_auto_yaw_flag():
    payload = encode_yaw_control_block(
        {"autoYaw": True, "autoYawOffset": 52.0, "setpoints": [[0, 52]]}
    )
    assert payload == bytes.fromhex("010802")


def test_encode_from_show_dict():
    show = {
        "yawControl": {
            "autoYaw": False,
            "setpoints": [[0, 0], [10, 0]],
        }
    }
    assert encode_yaw_control_from_show(show) == b"\x00\x00\x00"


def test_yaw_interpolation_at_time():
    payload = encode_yaw_control_block(
        {
            "autoYaw": False,
            "setpoints": [[0, 30.0], [1.0, 33.6], [2.0, 37.2]],
        }
    )
    assert yaw_at_time_from_payload(payload, 0.0) == pytest.approx(30.0)
    assert yaw_at_time_from_payload(payload, 0.5) == pytest.approx(31.8)
    assert yaw_at_time_from_payload(payload, 1.5) == pytest.approx(35.4)
    assert yaw_at_time_from_payload(payload, 2.0) == pytest.approx(37.2)


def test_shortest_path_wrap():
    payload = encode_yaw_control_block(
        {
            "autoYaw": False,
            "setpoints": [[0, 170.0], [1.0, -170.0]],
        }
    )
    _, change = struct.unpack_from("<Hh", payload, 3)
    assert change == 200  # +20° shortest path in deci-degrees


def test_splits_long_duration():
    payload = encode_yaw_control_block(
        {
            "autoYaw": False,
            "setpoints": [[0, 0.0], [70.0, 70.0]],
        }
    )
    assert len(payload) == 3 + 4 + 4
    d1, c1 = struct.unpack_from("<Hh", payload, 3)
    d2, c2 = struct.unpack_from("<Hh", payload, 7)
    assert d1 == 65535
    assert d2 == 4465
    assert c1 + c2 == 700
