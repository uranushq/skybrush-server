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
# compile_show
# --------------------------------------------------------------------------- #


def _gradient_grid(grid_w: int, grid_h: int):
    """Build a board whose pixel at (x, y) encodes its position as [x, y, 0]."""
    return [[x % 256, y % 256, 0] for y in range(grid_h) for x in range(grid_w)]


def _base_model(**overrides):
    k = overrides.pop("ledsPerDrone", 4)
    rows = overrides.pop("droneRows", 3)
    cols = overrides.pop("droneCols", 7)
    grid_w, grid_h = cols * k, rows * k
    model = {
        "ledsPerDrone": k,
        "droneRows": rows,
        "droneCols": cols,
        "droneCount": overrides.pop("droneCount", rows * cols),
        "fps": overrides.pop("fps", 10),
        "boards": overrides.pop(
            "boards",
            [
                {
                    "id": "b0",
                    "name": "board 0",
                    "pixels": _gradient_grid(grid_w, grid_h),
                    "startSec": 0.0,
                    "durationSec": 1.0,
                }
            ],
        ),
    }
    model.update(overrides)
    return model


def test_compile_basic_3x7_4x4():
    model = _base_model()  # 21 drones, 4x4, 10 fps, 1s board
    result = compile_show(model)

    assert len(result.bins) == 21
    assert result.total_frames == 10  # 1s * 10fps
    assert result.tile_width == 4 and result.tile_height == 4

    for per_drone in result.bins:
        parsed = parse_bin(per_drone.data)
        assert parsed["total_frames"] == 10
        assert parsed["width"] == 4 and parsed["height"] == 4


def test_compile_tile_extraction_reading_order():
    k, cols = 4, 7
    model = _base_model()
    result = compile_show(model)

    # Drone 9 -> row 1, col 2 -> grid origin (x=8, y=4)
    drone_index = 9
    drone_row, drone_col = drone_index // cols, drone_index % cols
    origin_x, origin_y = drone_col * k, drone_row * k

    frame0 = parse_bin(result.bins[drone_index].data)["frames"][0]
    # Check tile pixel (ty=1, tx=2): grid (y=origin_y+1, x=origin_x+2) -> [x, y, 0]
    ty, tx = 1, 2
    base = (ty * k + tx) * 3
    r, g, b = frame0[base], frame0[base + 1], frame0[base + 2]
    assert (r, g, b) == (origin_x + tx, origin_y + ty, 0)


def test_compile_missing_drones_removed_from_bottom_right():
    model = _base_model(droneCount=18)  # 21 slots, only 18 drones
    result = compile_show(model)

    assert len(result.bins) == 18
    indices = [b.drone_index for b in result.bins]
    assert indices == list(range(18))  # 18, 19, 20 (bottom-right) dropped


def test_compile_black_gap_before_board():
    k, cols, rows, fps = 4, 7, 3, 10
    grid_w, grid_h = cols * k, rows * k
    model = _base_model(
        fps=fps,
        boards=[
            {
                "id": "b0",
                "name": "delayed",
                "pixels": _gradient_grid(grid_w, grid_h),
                "startSec": 1.0,  # starts at 1s
                "durationSec": 1.0,
            }
        ],
    )
    result = compile_show(model)
    assert result.total_frames == 20  # total duration 2s * 10fps

    frames = parse_bin(result.bins[0].data)["frames"]
    black = bytes(k * k * 3)
    assert all(f == black for f in frames[:10])  # first second is black
    assert all(f != black for f in frames[10:])  # board active afterwards


def test_compile_rejects_overlapping_boards():
    k, cols, rows = 4, 7, 3
    grid_w, grid_h = cols * k, rows * k
    grid = _gradient_grid(grid_w, grid_h)
    model = _base_model(
        boards=[
            {"id": "a", "name": "a", "pixels": grid, "startSec": 0.0, "durationSec": 1.0},
            {"id": "b", "name": "b", "pixels": grid, "startSec": 0.5, "durationSec": 1.0},
        ]
    )
    with pytest.raises(CompileError):
        compile_show(model)


def test_compile_rejects_dronecount_over_grid():
    with pytest.raises(CompileError):
        compile_show(_base_model(droneCount=22))  # > 3*7


def test_compile_rejects_bad_pixel_count():
    model = _base_model(boards=[
        {"id": "a", "name": "a", "pixels": [[0, 0, 0]], "startSec": 0.0, "durationSec": 1.0}
    ])
    with pytest.raises(CompileError):
        compile_show(model)


def test_compile_clamps_channel_values():
    k, cols, rows = 4, 7, 3
    grid_w, grid_h = cols * k, rows * k
    pixels = [[300, -5, 128] for _ in range(grid_w * grid_h)]
    model = _base_model(boards=[
        {"id": "a", "name": "a", "pixels": pixels, "startSec": 0.0, "durationSec": 1.0}
    ])
    result = compile_show(model)
    frame0 = parse_bin(result.bins[0].data)["frames"][0]
    assert frame0[0:3] == bytes([255, 0, 128])
