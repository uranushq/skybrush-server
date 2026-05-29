"""Per-drone collision volumes for path planning.

Each drone is modelled as:

- **Body AABB** — axis-aligned box centered on the drone reference point.
- **Wake AABBs** — four tilted cylindrical wakes (one per motor), each
  conservatively approximated by an axis-aligned bounding box.

Collision is detected when **any** body/wake component of one drone overlaps
**any** component of another.
"""

from __future__ import annotations

import math
from typing import Sequence

# Body size (meters): X, Y, Z
BODY_SIZE_X = 0.5
BODY_SIZE_Y = 0.35
BODY_SIZE_Z = 0.5

# Motor offsets from drone reference (meters): FR, FL, RR, RL
MOTOR_POSITIONS = (
    (0.200, 0.183, 0.0),
    (-0.200, 0.183, 0.0),
    (0.200, -0.183, 0.0),
    (-0.200, -0.183, 0.0),
)

# Wake geometry (meters)
WAKE_ANGLE_DEG = 30.0
WAKE_LENGTH = 0.5
WAKE_RADIUS = 0.08

# Legacy names used by the REST API / formation validator
COLLISION_X = BODY_SIZE_X
COLLISION_Y = BODY_SIZE_Y
COLLISION_Z = BODY_SIZE_Z

AABB = tuple[tuple[float, float, float], tuple[float, float, float]]

_HALF_BODY_X = BODY_SIZE_X * 0.5
_HALF_BODY_Y = BODY_SIZE_Y * 0.5
_HALF_BODY_Z = BODY_SIZE_Z * 0.5

_WAKE_THETA = math.radians(WAKE_ANGLE_DEG)


def _axis_overlap(
    lo_a: float, hi_a: float, lo_b: float, hi_b: float, *, inclusive: bool = False
) -> bool:
    if inclusive:
        return lo_a <= hi_b and lo_b <= hi_a
    return lo_a < hi_b and lo_b < hi_a


def _body_aabb_relative() -> AABB:
    return (
        (-_HALF_BODY_X, -_HALF_BODY_Y, -_HALF_BODY_Z),
        (_HALF_BODY_X, _HALF_BODY_Y, _HALF_BODY_Z),
    )


def _wake_aabb_relative(
    start: tuple[float, float, float], direction: tuple[float, float, float]
) -> AABB:
    end = (
        start[0] + direction[0] * WAKE_LENGTH,
        start[1] + direction[1] * WAKE_LENGTH,
        start[2] + direction[2] * WAKE_LENGTH,
    )
    return (
        (
            min(start[0], end[0]) - WAKE_RADIUS,
            min(start[1], end[1]) - WAKE_RADIUS,
            min(start[2], end[2]) - WAKE_RADIUS,
        ),
        (
            max(start[0], end[0]) + WAKE_RADIUS,
            max(start[1], end[1]) + WAKE_RADIUS,
            max(start[2], end[2]) + WAKE_RADIUS,
        ),
    )


def _component_aabbs_relative() -> tuple[AABB, ...]:
    boxes: list[AABB] = [_body_aabb_relative()]

    # Front motors: +Y and -Z; rear motors: -Y and -Z.
    front_direction = (0.0, math.sin(_WAKE_THETA), -math.cos(_WAKE_THETA))
    rear_direction = (0.0, -math.sin(_WAKE_THETA), -math.cos(_WAKE_THETA))

    for start in MOTOR_POSITIONS:
        direction = front_direction if start[1] > 0 else rear_direction
        boxes.append(_wake_aabb_relative(start, direction))

    return tuple(boxes)


def _combined_aabb_relative(components: tuple[AABB, ...]) -> AABB:
    mins = [box[0] for box in components]
    maxs = [box[1] for box in components]
    return (
        (
            min(value[0] for value in mins),
            min(value[1] for value in mins),
            min(value[2] for value in mins),
        ),
        (
            max(value[0] for value in maxs),
            max(value[1] for value in maxs),
            max(value[2] for value in maxs),
        ),
    )


def _aabb_overlap(a_min: Sequence[float], a_max: Sequence[float], b_min: Sequence[float], b_max: Sequence[float]) -> bool:
    return (
        _axis_overlap(a_min[0], a_max[0], b_min[0], b_max[0])
        and _axis_overlap(a_min[1], a_max[1], b_min[1], b_max[1])
        and _axis_overlap(a_min[2], a_max[2], b_min[2], b_max[2])
    )


def _world_aabb(center: Sequence[float], rel: AABB) -> AABB:
    rel_min, rel_max = rel
    return (
        (
            center[0] + rel_min[0],
            center[1] + rel_min[1],
            center[2] + rel_min[2],
        ),
        (
            center[0] + rel_max[0],
            center[1] + rel_max[1],
            center[2] + rel_max[2],
        ),
    )


_COMPONENT_AABBS = _component_aabbs_relative()
_COMBINED_MIN_REL, _COMBINED_MAX_REL = _combined_aabb_relative(_COMPONENT_AABBS)


def volumes_overlap(a: Sequence[float], b: Sequence[float]) -> bool:
    """Returns whether any body/wake component of two drones overlaps."""
    for a_rel in _COMPONENT_AABBS:
        a_min, a_max = _world_aabb(a, a_rel)
        for b_rel in _COMPONENT_AABBS:
            b_min, b_max = _world_aabb(b, b_rel)
            if _aabb_overlap(a_min, a_max, b_min, b_max):
                return True
    return False


def volumes_overlap_at_times(
    a0: Sequence[float],
    a1: Sequence[float],
    b0: Sequence[float],
    b1: Sequence[float],
    *,
    samples: int = 5,
) -> bool:
    """Conservative check for overlap while both drones move linearly."""
    if volumes_overlap(a0, b0) or volumes_overlap(a1, b1):
        return True
    if samples < 2:
        return False
    for i in range(1, samples - 1):
        t = i / (samples - 1)
        a = [a0[k] + t * (a1[k] - a0[k]) for k in range(3)]
        b = [b0[k] + t * (b1[k] - b0[k]) for k in range(3)]
        if volumes_overlap(a, b):
            return True
    return False


def describe_collision_envelope() -> dict[str, float | dict[str, float] | list]:
    """JSON-serialisable description for API error responses."""
    return {
        "body_box": {
            "size_x": BODY_SIZE_X,
            "size_y": BODY_SIZE_Y,
            "size_z": BODY_SIZE_Z,
        },
        "wake": {
            "angle_deg": WAKE_ANGLE_DEG,
            "length": WAKE_LENGTH,
            "radius": WAKE_RADIUS,
            "motors": [
                {"x": m[0], "y": m[1], "z": m[2]} for m in MOTOR_POSITIONS
            ],
        },
        "components": [
            {
                "min": {"x": rel_min[0], "y": rel_min[1], "z": rel_min[2]},
                "max": {"x": rel_max[0], "y": rel_max[1], "z": rel_max[2]},
            }
            for rel_min, rel_max in _COMPONENT_AABBS
        ],
        "combined_aabb": {
            "min": {
                "x": _COMBINED_MIN_REL[0],
                "y": _COMBINED_MIN_REL[1],
                "z": _COMBINED_MIN_REL[2],
            },
            "max": {
                "x": _COMBINED_MAX_REL[0],
                "y": _COMBINED_MAX_REL[1],
                "z": _COMBINED_MAX_REL[2],
            },
        },
        # Kept for backward compatibility with older clients
        "x": COLLISION_X,
        "y": COLLISION_Y,
        "z": COLLISION_Z,
    }
