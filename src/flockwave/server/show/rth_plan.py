"""Encoder for Skybrush ``RTH_PLAN`` (``.skyb`` block type 4) payloads."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from itertools import count
from math import ceil
from struct import Struct
from typing import Any, ClassVar, Sequence

from .utils import BoundingBoxCalculator, encode_variable_length_integer

__all__ = (
    "RTHAction",
    "RTHPlan",
    "RTHPlanEntry",
    "encode_rth_plan_block",
    "encode_rth_plan_from_show",
)


class RTHAction(Enum):
    LAND = "land"
    GO_TO_KEEPING_ALTITUDE_AND_LAND = "goTo"


@dataclass(frozen=True)
class RTHPlanEntry:
    """Single entry in a return-to-home plan of a single drone."""

    time: int
    action: RTHAction
    target: tuple[float, ...] = ()
    duration: int = 0
    pre_delay: int = 0
    post_delay: int = 0

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> RTHPlanEntry:
        time = data.get("time")
        if time is None:
            raise ValueError("RTH plan entries must have timestamps")

        if isinstance(time, float) and time.is_integer():
            time = int(time)
        if not isinstance(time, int):
            raise ValueError("RTH plan entry timestamps must be integers")

        try:
            action = RTHAction(data.get("action"))
        except Exception as exc:
            raise ValueError("invalid action found in RTH plan entry") from exc

        if action is RTHAction.LAND:
            return cls(time=int(time), action=action)

        target = tuple(data.get("target") or ())
        if len(target) != 2 or not all(isinstance(item, (int, float)) for item in target):
            raise ValueError("targets in RTH plan entry must be pairs of numbers")

        duration = data.get("duration")
        if duration is None:
            raise ValueError("RTH plan entries with targets must have durations")

        if isinstance(duration, float) and duration.is_integer():
            duration = int(duration)
        if not isinstance(duration, int):
            raise ValueError("RTH plan entry durations must be integers")

        pre_delay = data.get("preDelay") or 0
        if isinstance(pre_delay, float) and pre_delay.is_integer():
            pre_delay = int(pre_delay)
        if not isinstance(pre_delay, int):
            raise ValueError("RTH plan entry pre-delays must be integers")

        post_delay = data.get("postDelay") or 0
        if isinstance(post_delay, float) and post_delay.is_integer():
            post_delay = int(post_delay)
        if not isinstance(post_delay, int):
            raise ValueError("RTH plan entry post-delays must be integers")

        return cls(
            time=int(time),
            action=action,
            target=target,
            duration=duration,
            pre_delay=pre_delay if pre_delay > 0 else 0,
            post_delay=post_delay if post_delay > 0 else 0,
        )

    @property
    def has_pre_delay(self) -> bool:
        return self.pre_delay > 0

    @property
    def has_post_delay(self) -> bool:
        return self.post_delay > 0

    @property
    def has_target(self) -> bool:
        return self.action is RTHAction.GO_TO_KEEPING_ALTITUDE_AND_LAND

    def is_same_as_except_timestamp(self, other: RTHPlanEntry) -> bool:
        return (
            self.action == other.action
            and self.target == other.target
            and self.duration == other.duration
            and self.pre_delay == other.pre_delay
            and self.post_delay == other.post_delay
        )

    def to_json(self) -> dict[str, Any]:
        result: dict[str, Any] = {"time": self.time, "action": self.action.value}
        if self.has_target:
            result["target"] = list(self.target)
            result["duration"] = self.duration
            if self.has_pre_delay:
                result["preDelay"] = self.pre_delay
            if self.has_post_delay:
                result["postDelay"] = self.post_delay
        return result


class RTHPlan(Sequence[RTHPlanEntry]):
    """Return-to-home plan for a single drone."""

    def __init__(self) -> None:
        self._entries: list[RTHPlanEntry] = []

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> RTHPlan:
        if data.get("version") != 1:
            raise RuntimeError("only version 1 RTH plans are supported")

        entries = data.get("entries")
        if entries is None or not hasattr(entries, "__iter__"):
            raise RuntimeError("entries not found in RTH plan")

        plan = cls()
        for entry in entries:
            plan.add_entry(RTHPlanEntry.from_json(entry))
        return plan

    @property
    def bounding_box(self) -> tuple[Sequence[float], Sequence[float]]:
        return self.get_padded_bounding_box()

    @property
    def is_empty(self) -> bool:
        return not self._entries

    @property
    def last_timestamp(self) -> int | None:
        return self._entries[-1].time if self._entries else None

    def add_entry(self, entry: RTHPlanEntry) -> None:
        last = self.last_timestamp
        if last is not None and entry.time <= last:
            raise RuntimeError(
                "Cannot add entry to RTH plan; timestamp must be larger than "
                "the last entry of the RTH plan"
            )
        self._entries.append(entry)

    def get_padded_bounding_box(
        self, margin: float = 0
    ) -> tuple[Sequence[float], Sequence[float]]:
        bbox = BoundingBoxCalculator(dim=2)
        for entry in self._entries:
            if entry.has_target:
                bbox.add(entry.target)

        if margin > 0:
            bbox.pad(margin)

        return bbox.get_corners()

    def propose_scaling_factor(self) -> int:
        try:
            mins, maxs = self.bounding_box
        except ValueError:
            return 1

        coords = [abs(x) for x in mins]
        coords.extend(abs(x) for x in maxs)
        extremum = ceil(max(coords) * 1000)
        return ceil((extremum + 1) / 32768)

    def to_json(self) -> dict[str, Any]:
        return {"version": 1, "entries": [entry.to_json() for entry in self._entries]}

    def __getitem__(self, index: int) -> RTHPlanEntry:
        return self._entries[index]

    def __len__(self) -> int:
        return len(self._entries)


class RTHPlanEncoder:
    """Encoder for RTH plans in the Skybrush binary show file format."""

    _point_struct: ClassVar[Struct] = Struct("<hh")

    def __init__(self, scale: int = 1) -> None:
        self._scale_orig = scale
        self._scale = 1000 / scale

    def encode(self, plan: RTHPlan) -> bytes:
        chunks: list[bytes] = [bytes([self._scale_orig])]

        points = [self._scale_point(entry.target) for entry in plan if entry.target]
        point_index = self._encode_points(points, chunks)
        self._encode_plan_entries(plan, point_index, chunks)
        return b"".join(chunks)

    def _encode_plan_entries(
        self,
        entries: Sequence[RTHPlanEntry],
        point_index: dict[tuple[int, ...], int],
        chunks: list[bytes],
    ) -> None:
        chunks.append(len(entries).to_bytes(2, "little"))
        previous_entry: RTHPlanEntry | None = None
        for entry in entries:
            self._encode_plan_entry(entry, point_index, previous_entry, chunks)
            previous_entry = entry

    def _encode_plan_entry(
        self,
        entry: RTHPlanEntry,
        point_index: dict[tuple[int, ...], int],
        previous_entry: RTHPlanEntry | None,
        chunks: list[bytes],
    ) -> None:
        encode_int = encode_variable_length_integer

        timestamp_diff = entry.time - (previous_entry.time if previous_entry else 0)
        if timestamp_diff < 0:
            raise RuntimeError("timestamps in RTH plan must not go back in time")

        has_pre_delay = entry.has_pre_delay
        has_post_delay = entry.has_post_delay
        has_target = entry.has_target

        if previous_entry and entry.is_same_as_except_timestamp(previous_entry):
            action_bits = 0
            has_post_delay = has_pre_delay = has_target = False
        elif entry.action is RTHAction.LAND:
            action_bits = 1
        elif entry.action is RTHAction.GO_TO_KEEPING_ALTITUDE_AND_LAND:
            action_bits = 2
        else:
            raise ValueError(f"unknown RTH action: {entry.action}")

        flags = (
            (action_bits << 4)
            | (2 if has_pre_delay else 0)
            | (1 if has_post_delay else 0)
        )
        chunks.append(flags.to_bytes(1, "little"))
        chunks.append(encode_int(timestamp_diff))

        if has_target:
            if entry.duration < 0:
                raise ValueError(f"RTH action has negative duration: {entry.duration}")
            scaled_target = self._scale_point(entry.target)
            chunks.append(encode_int(point_index[scaled_target]))
            chunks.append(encode_int(int(entry.duration)))

        if has_pre_delay:
            chunks.append(encode_int(int(entry.pre_delay)))
        if has_post_delay:
            chunks.append(encode_int(int(entry.post_delay)))

    def _encode_points(
        self, points: list[tuple[int, ...]], chunks: list[bytes]
    ) -> dict[tuple[int, ...], int]:
        result: dict[tuple[int, ...], int] = {}
        point_chunks: list[bytes] = []
        id_generator = count()

        if len(points) > 65535:
            raise ValueError("too many points in RTH plan")

        for point in points:
            index = result.get(point)
            if index is None:
                result[point] = index = next(id_generator)
                point_chunks.append(self._point_struct.pack(*point))

        chunks.append(len(point_chunks).to_bytes(2, "little"))
        chunks.extend(point_chunks)
        return result

    def _scale_point(self, point: tuple[float, ...]) -> tuple[int, ...]:
        if len(point) != 2:
            raise ValueError("each point must be two-dimensional")
        return (
            round(point[0] * self._scale),
            round(point[1] * self._scale),
        )


def encode_rth_plan_block(rth_plan: dict[str, Any]) -> bytes | None:
    """Encode an ``rthPlan`` JSON object into an RTH_PLAN block payload."""
    try:
        plan = RTHPlan.from_json(rth_plan)
    except (RuntimeError, ValueError):
        return None

    if plan.is_empty:
        return None

    scaling_factor = plan.propose_scaling_factor()
    if scaling_factor >= 128:
        raise RuntimeError(
            "RTH plan covers too large an area for a Skybrush binary show file"
        )

    return RTHPlanEncoder(scaling_factor).encode(plan)


def encode_rth_plan_from_show(show: dict[str, Any]) -> bytes | None:
    """Return encoded RTH_PLAN bytes from a per-drone show dict, if any."""
    rth_plan = show.get("rthPlan")
    if not isinstance(rth_plan, dict):
        return None
    return encode_rth_plan_block(rth_plan)
