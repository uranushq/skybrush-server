"""Tests for the LED-generator extension: bin format + show compilation."""

import struct

import pytest

from flockwave.server.ext.led_generator.binformat import (
    END_MARKER,
    build_bin,
    parse_bin,
)
from flockwave.server.ext.led_generator.compile import CompileError, compile_show


# --------------------------------------------------------------------------- #
# binformat
# --------------------------------------------------------------------------- #


def test_build_bin_roundtrip():
    width, height, fps = 4, 4, 15
    frame0 = bytes(range(width * height * 3))  # 0..47
    frame1 = bytes((255 - v for v in range(width * height * 3)))

    data = build_bin([frame0, frame1], width=width, height=height, fps=fps, save_time=123)

    # Size = header + 2 frames + trailer
    assert len(data) == 16 + 2 * (width * height * 3) + 16

    parsed = parse_bin(data)
    assert parsed["total_frames"] == 2
    assert parsed["width"] == width
    assert parsed["height"] == height
    assert parsed["fps"] == fps
    assert parsed["save_time"] == 123
    assert parsed["end_marker"] == END_MARKER
    assert parsed["frames"] == [frame0, frame1]


def test_build_bin_header_and_trailer_layout():
    data = build_bin([bytes(48)], width=4, height=4, fps=30, save_time=999)
    total_frames, height, width, fps = struct.unpack_from("<IIII", data, 0)
    assert (total_frames, height, width, fps) == (1, 4, 4, 30)

    t_frames, save_time, marker = struct.unpack_from("<IQI", data, len(data) - 16)
    assert t_frames == 1
    assert save_time == 999
    assert marker == 0xDEADBEEF


def test_build_bin_rejects_wrong_frame_size():
    with pytest.raises(ValueError):
        build_bin([bytes(10)], width=4, height=4, fps=15)


# --------------------------------------------------------------------------- #
# compile_show (per-drone model)
# --------------------------------------------------------------------------- #


def _make_drones(count, k, fn):
    """Build per-drone colour sets; fn(drone, local) -> [r, g, b]."""
    return [[fn(d, l) for l in range(k * k)] for d in range(count)]


def _base_model(**overrides):
    k = overrides.pop("ledsPerDrone", 4)
    count = overrides.pop("droneCount", 21)
    fn = overrides.pop("fn", lambda d, l: [d % 256, l, 0])
    model = {
        "ledsPerDrone": k,
        "droneCount": count,
        "fps": overrides.pop("fps", 10),
        "boards": overrides.pop(
            "boards",
            [
                {
                    "startSec": 0.0,
                    "durationSec": 1.0,
                    "drones": _make_drones(count, k, fn),
                }
            ],
        ),
    }
    model.update(overrides)
    return model


def test_compile_basic():
    result = compile_show(_base_model())  # 21 drones, 4x4, 10fps, 1s board
    assert len(result.bins) == 21
    assert result.total_frames == 10
    assert result.tile_width == 4 and result.tile_height == 4
    for per_drone in result.bins:
        parsed = parse_bin(per_drone.data)
        assert parsed["total_frames"] == 10
        assert parsed["width"] == 4 and parsed["height"] == 4


def test_compile_preserves_per_drone_content_and_order():
    # Each drone's local pixel l encodes (drone, local) so we can verify that a
    # drone's LED set stays intact and is mapped to the right .bin (reading order).
    k = 4
    result = compile_show(_base_model(fn=lambda d, l: [d % 256, l, 0]))
    for drone_index in (0, 5, 9, 20):
        frame0 = parse_bin(result.bins[drone_index].data)["frames"][0]
        for local in range(k * k):
            base = local * 3
            assert (frame0[base], frame0[base + 1], frame0[base + 2]) == (
                drone_index % 256,
                local,
                0,
            )


def test_compile_drone_count():
    result = compile_show(_base_model(droneCount=18))
    assert len(result.bins) == 18
    assert [b.drone_index for b in result.bins] == list(range(18))


def test_compile_black_gap_before_board():
    k, fps, count = 4, 10, 21
    model = _base_model(
        fps=fps,
        droneCount=count,
        boards=[
            {
                "startSec": 1.0,
                "durationSec": 1.0,
                "drones": _make_drones(count, k, lambda d, l: [255, 255, 255]),
            }
        ],
    )
    result = compile_show(model)
    assert result.total_frames == 20  # 2s * 10fps

    frames = parse_bin(result.bins[0].data)["frames"]
    black = bytes(k * k * 3)
    assert all(f == black for f in frames[:10])
    assert all(f != black for f in frames[10:])


def test_compile_rejects_overlapping_boards():
    k, count = 4, 21
    drones = _make_drones(count, k, lambda d, l: [1, 2, 3])
    model = _base_model(
        boards=[
            {"startSec": 0.0, "durationSec": 1.0, "drones": drones},
            {"startSec": 0.5, "durationSec": 1.0, "drones": drones},
        ]
    )
    with pytest.raises(CompileError):
        compile_show(model)


def test_compile_rejects_wrong_drones_length():
    model = _base_model(
        boards=[
            {
                "startSec": 0.0,
                "durationSec": 1.0,
                "drones": _make_drones(5, 4, lambda d, l: [0, 0, 0]),  # not 21
            }
        ]
    )
    with pytest.raises(CompileError):
        compile_show(model)


def test_compile_rejects_bad_drone_pixel_count():
    model = _base_model(
        boards=[
            {
                "startSec": 0.0,
                "durationSec": 1.0,
                "drones": [[[0, 0, 0]] for _ in range(21)],  # each drone too short
            }
        ]
    )
    with pytest.raises(CompileError):
        compile_show(model)


def test_compile_clamps_channel_values():
    result = compile_show(_base_model(fn=lambda d, l: [300, -5, 128]))
    frame0 = parse_bin(result.bins[0].data)["frames"][0]
    assert frame0[0:3] == bytes([255, 0, 128])
