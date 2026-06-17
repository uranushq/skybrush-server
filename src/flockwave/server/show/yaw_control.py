"""Encoder for Skybrush ``YAW_CONTROL`` (``.skyb`` block type 5) payloads."""

from __future__ import annotations

import struct
from typing import Any

__all__ = (
    "encode_yaw_control_block",
    "encode_yaw_control_from_show",
    "yaw_at_time_from_payload",
)

_MAX_DURATION_MS = 65535


def _normalize_yaw_deg(yaw: float) -> float:
    yaw = yaw % 360.0
    if yaw >= 180.0:
        yaw -= 360.0
    return yaw


def _yaw_delta_deg(start: float, end: float) -> float:
    """Shortest signed yaw change from *start* to *end* in degrees."""
    return (end - start + 180.0) % 360.0 - 180.0


def _deg_to_ddeg(value: float) -> int:
    ddeg = int(round(value * 10.0))
    if ddeg > 32767:
        return 32767
    if ddeg < -32768:
        return -32768
    return ddeg


def _merge_setpoints(setpoints: list[Any]) -> list[tuple[float, float]]:
    merged: list[tuple[float, float]] = []
    for item in sorted(setpoints, key=lambda entry: float(entry[0])):
        if not isinstance(item, (list, tuple)) or len(item) < 2:
            raise ValueError("yawControl setpoints must be [time, yaw] pairs")
        t = round(float(item[0]), 4)
        yaw = _normalize_yaw_deg(float(item[1]))
        if merged and merged[-1][0] == t:
            merged[-1] = (t, yaw)
        else:
            merged.append((t, yaw))
    return merged


def _append_duration_delta(
    payload: bytearray,
    *,
    duration_ms: int,
    change_ddeg: int,
) -> None:
    if duration_ms <= 0:
        return
    remaining_ms = duration_ms
    remaining_change = change_ddeg
    while remaining_ms > 0:
        chunk_ms = min(remaining_ms, _MAX_DURATION_MS)
        if remaining_ms == chunk_ms:
            chunk_change = remaining_change
        elif duration_ms > 0:
            chunk_change = int(round(change_ddeg * (chunk_ms / duration_ms)))
            remaining_change -= chunk_change
        else:
            chunk_change = 0
        payload.extend(struct.pack("<Hh", chunk_ms, chunk_change))
        remaining_ms -= chunk_ms


def encode_yaw_control_block(yaw_control: dict[str, Any]) -> bytes | None:
    """Encode a ``yawControl`` JSON object into a YAW_CONTROL block payload.

    The payload layout matches libskybrush ``sb_yaw_control``:

    - 1 byte flags (bit0 = auto_yaw)
    - int16 initial yaw offset in deci-degrees
    - repeated (uint16 duration_ms, int16 yaw_change_ddeg) deltas
    """
    setpoints = yaw_control.get("setpoints")
    if not setpoints:
        return None

    points = _merge_setpoints(list(setpoints))
    if not points:
        return None

    if points[0][0] > 0:
        points.insert(0, (0.0, points[0][1]))

    auto_yaw = bool(yaw_control.get("autoYaw", False))
    offset_yaw = float(yaw_control.get("autoYawOffset", points[0][1]))
    offset_ddeg = _deg_to_ddeg(offset_yaw)

    flags = 0x01 if auto_yaw else 0x00
    payload = bytearray(struct.pack("<Bh", flags, offset_ddeg))

    constant_yaw = all(abs(_yaw_delta_deg(yaw, offset_yaw)) < 1e-9 for _, yaw in points)
    if constant_yaw:
        return bytes(payload)

    for (t0, y0), (t1, y1) in zip(points, points[1:]):
        duration_ms = int(round((t1 - t0) * 1000.0))
        change_ddeg = _deg_to_ddeg(_yaw_delta_deg(y0, y1))
        _append_duration_delta(
            payload, duration_ms=duration_ms, change_ddeg=change_ddeg
        )

    if len(payload) == 3:
        return bytes(payload)
    return bytes(payload)


def encode_yaw_control_from_show(show: dict[str, Any]) -> bytes | None:
    """Return encoded YAW_CONTROL bytes from a per-drone show dict, if any."""
    yaw_control = show.get("yawControl")
    if not isinstance(yaw_control, dict):
        return None
    return encode_yaw_control_block(yaw_control)


def yaw_at_time_from_payload(payload: bytes, t_sec: float) -> float | None:
    """Decode yaw at *t_sec* from a YAW_CONTROL payload (for tests)."""
    if len(payload) < 3:
        return None

    flags = payload[0]
    if flags & 0xFE:
        return None

    offset_ddeg = struct.unpack_from("<h", payload, 1)[0]
    start_ddeg = offset_ddeg
    start_ms = 0
    cursor = 3

    while cursor + 4 <= len(payload):
        duration_ms, change_ddeg = struct.unpack_from("<Hh", payload, cursor)
        end_ms = start_ms + duration_ms
        if t_sec * 1000.0 <= end_ms or cursor + 4 == len(payload):
            if duration_ms == 0:
                return start_ddeg / 10.0
            rel = max(0.0, min(1.0, (t_sec * 1000.0 - start_ms) / duration_ms))
            return (start_ddeg + change_ddeg * rel) / 10.0
        start_ms = end_ms
        start_ddeg += change_ddeg
        cursor += 4

    return start_ddeg / 10.0
