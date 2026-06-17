"""Binary ``.bin`` LED-frame format.

Direct port of the format produced by the standalone ``pixelart-to-bin`` tool
so that files generated here are byte-compatible with what the JR LED-board
firmware expects.

Layout (all integers little-endian, tightly packed, no alignment padding)::

    HEADER  (16 bytes)  struct '<IIII'  total_frames, height, width, fps
    BODY    (total_frames * height * width * 3 bytes)
            raw RGB, row-major (y top->bottom, x left->right), 3 bytes/pixel
    TRAILER (16 bytes)  struct '<IQI'   total_frames, save_time_unix, 0xDEADBEEF

A valid file therefore satisfies::

    len(file) == 16 + total_frames * height * width * 3 + 16
    header.total_frames == trailer.total_frames
    trailer.end_marker == 0xDEADBEEF
"""

from __future__ import annotations

import struct
import time
from typing import Sequence

__all__ = ("END_MARKER", "HEADER_SIZE", "TRAILER_SIZE", "build_bin", "parse_bin")

END_MARKER = 0xDEADBEEF
HEADER_SIZE = 16
TRAILER_SIZE = 16

_HEADER = struct.Struct("<IIII")
_TRAILER = struct.Struct("<IQI")


def build_bin(
    frames: Sequence[bytes],
    *,
    width: int,
    height: int,
    fps: int,
    save_time: int | None = None,
) -> bytes:
    """Assemble a complete ``.bin`` byte string.

    Args:
        frames: one entry per frame, each a raw row-major RGB buffer of exactly
            ``width * height * 3`` bytes.
        width: frame width in pixels.
        height: frame height in pixels.
        fps: playback speed hint stored in the header.
        save_time: Unix timestamp written into the trailer; defaults to "now".

    Raises:
        ValueError: if any frame has the wrong length.
    """
    expected = width * height * 3
    for index, frame in enumerate(frames):
        if len(frame) != expected:
            raise ValueError(
                f"frame {index} has {len(frame)} bytes, expected {expected} "
                f"({width}x{height} RGB)"
            )

    total_frames = len(frames)
    if save_time is None:
        save_time = int(time.time())

    header = _HEADER.pack(total_frames, height, width, fps)
    body = b"".join(frames)
    trailer = _TRAILER.pack(total_frames, save_time, END_MARKER)
    return header + body + trailer


def parse_bin(data: bytes) -> dict:
    """Parse a ``.bin`` buffer back into its fields (used by tests/tools).

    Returns a dict with ``total_frames``, ``height``, ``width``, ``fps``,
    ``save_time``, ``end_marker`` and the list of raw RGB ``frames``.

    Raises:
        ValueError: if the buffer is structurally invalid.
    """
    if len(data) < HEADER_SIZE + TRAILER_SIZE:
        raise ValueError("buffer too small to contain header and trailer")

    total_frames, height, width, fps = _HEADER.unpack_from(data, 0)
    frame_size = width * height * 3
    expected_len = HEADER_SIZE + total_frames * frame_size + TRAILER_SIZE
    if len(data) != expected_len:
        raise ValueError(
            f"file size {len(data)} does not match expected {expected_len}"
        )

    t_frames, save_time, end_marker = _TRAILER.unpack_from(
        data, HEADER_SIZE + total_frames * frame_size
    )
    if end_marker != END_MARKER:
        raise ValueError(f"bad end marker: {end_marker:#x}")
    if t_frames != total_frames:
        raise ValueError(
            f"trailer frame count {t_frames} != header frame count {total_frames}"
        )

    frames = [
        data[HEADER_SIZE + i * frame_size : HEADER_SIZE + (i + 1) * frame_size]
        for i in range(total_frames)
    ]
    return {
        "total_frames": total_frames,
        "height": height,
        "width": width,
        "fps": fps,
        "save_time": save_time,
        "end_marker": end_marker,
        "frames": frames,
    }
