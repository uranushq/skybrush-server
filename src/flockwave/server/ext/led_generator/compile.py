"""Compile a Skybrush-Live "LED show" model into per-drone ``.bin`` files.

The model mirrors the TypeScript ``LedShow`` type authored in the browser::

    {
      "ledsPerDrone": 4,          # k -> each drone has a k x k LED panel
      "droneRows": 3,             # drone arrangement (rows)
      "droneCols": 7,             # drone arrangement (cols)
      "droneCount": 21,           # <= droneRows * droneCols; extras are removed
                                  # from the bottom-right (highest tile indices)
      "fps": 15,                  # global export frame rate
      "boards": [
        {
          "id": "...",
          "name": "...",
          "pixels": [[r, g, b], ...],   # length gridH*gridW, row-major
          "startSec": 0.0,
          "durationSec": 2.0
        },
        ...
      ]
    }

with ``gridW = droneCols * k`` and ``gridH = droneRows * k``.

A board is a *static* still frame held for ``durationSec``. On export the
timeline is sampled at ``fps``; a board fills every frame inside its span and
gaps between boards render black ``(0, 0, 0)``.

Drones are numbered in reading order (left->right, top->bottom)::

    tile_id = drone_row * droneCols + drone_col

so dropping drones ``>= droneCount`` naturally removes them from the
bottom-right of the grid.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .binformat import build_bin

__all__ = ("CompileError", "CompiledShow", "PerDroneBin", "compile_show")


class CompileError(ValueError):
    """Raised when the incoming LED-show model is invalid."""


@dataclass(frozen=True)
class PerDroneBin:
    """A single compiled per-drone tile file."""

    drone_index: int  # 0-based, reading order
    data: bytes  # complete .bin byte string


@dataclass(frozen=True)
class CompiledShow:
    """Result of :func:`compile_show`."""

    total_frames: int
    fps: int
    tile_width: int
    tile_height: int
    bins: list[PerDroneBin]


def _require_int(model: Mapping[str, Any], key: str, *, minimum: int) -> int:
    value = model.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise CompileError(f"'{key}' must be an integer")
    if value < minimum:
        raise CompileError(f"'{key}' must be >= {minimum}")
    return value


def _clamp_channel(value: Any) -> int:
    try:
        ivalue = int(value)
    except (TypeError, ValueError) as exc:
        raise CompileError(f"invalid color channel value: {value!r}") from exc
    return 0 if ivalue < 0 else 255 if ivalue > 255 else ivalue


def _grid_bytes(pixels: Sequence[Any], *, grid_w: int, grid_h: int) -> bytes:
    """Flatten a row-major ``[[r,g,b], ...]`` board into raw RGB bytes."""
    expected = grid_w * grid_h
    if not isinstance(pixels, (list, tuple)) or len(pixels) != expected:
        raise CompileError(
            f"'pixels' must have exactly {expected} entries "
            f"({grid_w}x{grid_h}), got "
            f"{len(pixels) if isinstance(pixels, (list, tuple)) else type(pixels)}"
        )
    out = bytearray(expected * 3)
    for i, pixel in enumerate(pixels):
        if not isinstance(pixel, (list, tuple)) or len(pixel) != 3:
            raise CompileError(f"'pixels[{i}]' must be [r, g, b]")
        base = i * 3
        out[base] = _clamp_channel(pixel[0])
        out[base + 1] = _clamp_channel(pixel[1])
        out[base + 2] = _clamp_channel(pixel[2])
    return bytes(out)


def _extract_tile(
    grid: bytes, *, drone_index: int, cols: int, k: int, grid_w: int
) -> bytes:
    """Extract the k x k RGB sub-block for a drone from a full-grid RGB buffer."""
    drone_row = drone_index // cols
    drone_col = drone_index % cols
    start_y = drone_row * k
    start_x = drone_col * k
    row_stride = grid_w * 3
    tile = bytearray(k * k * 3)
    for y in range(k):
        src = ((start_y + y) * grid_w + start_x) * 3
        dst = y * k * 3
        tile[dst : dst + k * 3] = grid[src : src + k * 3]
    return bytes(tile)


def compile_show(model: Mapping[str, Any]) -> CompiledShow:
    """Compile an LED-show model into per-drone ``.bin`` byte strings."""
    k = _require_int(model, "ledsPerDrone", minimum=1)
    if k not in (3, 4):
        raise CompileError("'ledsPerDrone' must be 3 or 4")
    rows = _require_int(model, "droneRows", minimum=1)
    cols = _require_int(model, "droneCols", minimum=1)
    fps = _require_int(model, "fps", minimum=1)
    drone_count = _require_int(model, "droneCount", minimum=1)
    if drone_count > rows * cols:
        raise CompileError(
            f"'droneCount' ({drone_count}) exceeds droneRows*droneCols "
            f"({rows * cols})"
        )

    grid_w = cols * k
    grid_h = rows * k

    boards = model.get("boards")
    if not isinstance(boards, list) or len(boards) == 0:
        raise CompileError("'boards' must be a non-empty array")

    # Validate / normalize boards and pre-compute their full-grid RGB bytes.
    spans: list[tuple[float, float, bytes]] = []  # (start, end, grid_bytes)
    for index, board in enumerate(boards):
        if not isinstance(board, Mapping):
            raise CompileError(f"'boards[{index}]' must be an object")
        try:
            start = float(board.get("startSec", 0.0))
            duration = float(board.get("durationSec", 0.0))
        except (TypeError, ValueError) as exc:
            raise CompileError(
                f"'boards[{index}]' has invalid startSec/durationSec"
            ) from exc
        if duration <= 0:
            raise CompileError(f"'boards[{index}].durationSec' must be > 0")
        if start < 0:
            raise CompileError(f"'boards[{index}].startSec' must be >= 0")
        grid = _grid_bytes(board.get("pixels"), grid_w=grid_w, grid_h=grid_h)
        spans.append((start, start + duration, grid))

    # Reject overlapping boards (the timeline UI enforces this, but be safe).
    ordered = sorted(spans, key=lambda s: s[0])
    for (s0, e0, _), (s1, _e1, _g) in zip(ordered, ordered[1:]):
        if s1 < e0 - 1e-9:
            raise CompileError("boards on the timeline must not overlap")

    total_duration = max(end for _s, end, _g in spans)
    total_frames = max(1, round(total_duration * fps))

    black_tile = bytes(k * k * 3)

    # For each board pre-compute its per-drone tile bytes so the per-frame loop
    # is just a list of references.
    board_tiles: list[list[bytes]] = []
    for _s, _e, grid in spans:
        tiles = [
            _extract_tile(grid, drone_index=d, cols=cols, k=k, grid_w=grid_w)
            for d in range(drone_count)
        ]
        board_tiles.append(tiles)

    # Assign a board (or None) to every frame.
    frame_board: list[int | None] = []
    for i in range(total_frames):
        t = i / fps
        active: int | None = None
        for bindex, (start, end, _g) in enumerate(spans):
            if start <= t < end:
                active = bindex
                break
        frame_board.append(active)

    bins: list[PerDroneBin] = []
    for drone_index in range(drone_count):
        frames = [
            board_tiles[b][drone_index] if b is not None else black_tile
            for b in frame_board
        ]
        data = build_bin(frames, width=k, height=k, fps=fps)
        bins.append(PerDroneBin(drone_index=drone_index, data=data))

    return CompiledShow(
        total_frames=total_frames,
        fps=fps,
        tile_width=k,
        tile_height=k,
        bins=bins,
    )
