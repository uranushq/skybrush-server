"""Compile a Skybrush-Live "LED show" model into per-drone ``.bin`` files.

The model mirrors the TypeScript ``LedShow`` type authored in the browser.
Drone count, LEDs-per-drone and FPS are global; colours are stored **per drone**
(each drone's ``k*k`` set), so the per-board formation does not affect the
generated ``.bin`` files — a drone's file only depends on its own LED sequence::

    {
      "ledsPerDrone": 4,          # k -> each drone has a k x k LED panel
      "droneCount": 21,           # number of drones (reading order)
      "fps": 15,                  # global export frame rate
      "boards": [
        {
          "startSec": 0.0,
          "durationSec": 2.0,
          # one entry per drone (length droneCount); each is k*k row-major RGB
          "drones": [ [[r, g, b], ... (k*k) ], ... (droneCount) ],
          "rows": 3, "cols": 7    # formation (optional, ignored for .bin)
        },
        ...
      ]
    }

A board is a *static* still frame held for ``durationSec``. On export the
timeline is sampled at ``fps``; a board fills every frame inside its span and
gaps between boards render black ``(0, 0, 0)``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .binformat import build_bin

__all__ = ("CompileError", "CompiledShow", "PerDroneBin", "compile_show")


class CompileError(ValueError):
    """Raised when the incoming LED-show model is invalid."""


@dataclass(frozen=True)
class PerDroneBin:
    """A single compiled per-drone file."""

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


def _drone_bytes(pixels: Any, *, leds: int, board_index: int, drone_index: int) -> bytes:
    """Flatten one drone's ``k*k`` row-major ``[[r,g,b], ...]`` into RGB bytes."""
    expected = leds * leds
    if not isinstance(pixels, (list, tuple)) or len(pixels) != expected:
        raise CompileError(
            f"'boards[{board_index}].drones[{drone_index}]' must have "
            f"{expected} pixels ({leds}x{leds})"
        )
    out = bytearray(expected * 3)
    for i, pixel in enumerate(pixels):
        if not isinstance(pixel, (list, tuple)) or len(pixel) != 3:
            raise CompileError(
                f"'boards[{board_index}].drones[{drone_index}][{i}]' must be [r, g, b]"
            )
        base = i * 3
        out[base] = _clamp_channel(pixel[0])
        out[base + 1] = _clamp_channel(pixel[1])
        out[base + 2] = _clamp_channel(pixel[2])
    return bytes(out)


def compile_show(model: Mapping[str, Any]) -> CompiledShow:
    """Compile an LED-show model into per-drone ``.bin`` byte strings."""
    leds = _require_int(model, "ledsPerDrone", minimum=1)
    if leds not in (3, 4):
        raise CompileError("'ledsPerDrone' must be 3 or 4")
    drone_count = _require_int(model, "droneCount", minimum=1)
    fps = _require_int(model, "fps", minimum=1)

    boards = model.get("boards")
    if not isinstance(boards, list) or len(boards) == 0:
        raise CompileError("'boards' must be a non-empty array")

    # Validate boards and pre-compute each board's per-drone byte buffers.
    spans: list[tuple[float, float, list[bytes]]] = []  # (start, end, drone_bytes)
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

        drones = board.get("drones")
        if not isinstance(drones, list) or len(drones) != drone_count:
            raise CompileError(
                f"'boards[{index}].drones' must have exactly {drone_count} entries"
            )
        drone_bytes = [
            _drone_bytes(drones[d], leds=leds, board_index=index, drone_index=d)
            for d in range(drone_count)
        ]
        spans.append((start, start + duration, drone_bytes))

    ordered = sorted(spans, key=lambda s: s[0])
    for (s0, e0, _), (s1, _e1, _d) in zip(ordered, ordered[1:]):
        if s1 < e0 - 1e-9:
            raise CompileError("boards on the timeline must not overlap")

    total_duration = max(end for _s, end, _d in spans)
    total_frames = max(1, round(total_duration * fps))

    black = bytes(leds * leds * 3)

    # Which board (or None) is active for every frame.
    frame_board: list[int | None] = []
    for i in range(total_frames):
        t = i / fps
        active: int | None = None
        for bindex, (start, end, _d) in enumerate(spans):
            if start <= t < end:
                active = bindex
                break
        frame_board.append(active)

    bins: list[PerDroneBin] = []
    for drone_index in range(drone_count):
        frames = [
            spans[b][2][drone_index] if b is not None else black
            for b in frame_board
        ]
        data = build_bin(frames, width=leds, height=leds, fps=fps)
        bins.append(PerDroneBin(drone_index=drone_index, data=data))

    return CompiledShow(
        total_frames=total_frames,
        fps=fps,
        tile_width=leds,
        tile_height=leds,
        bins=bins,
    )
