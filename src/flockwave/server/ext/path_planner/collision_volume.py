"""Per-drone collision volumes for path planning.

Physical model
--------------
Each drone is modelled as a **body AABB** plus four **motor-wake AABBs**
(one tilted cylindrical wake per motor, conservatively approximated by an
axis-aligned box). This detailed model is what gets reported in API error
payloads via :func:`describe_collision_envelope`.

Checking envelope
-----------------
All collision *checks* use a single, **yaw-invariant bounding envelope**
derived from the detailed model: a box whose horizontal half-side is the
horizontal circumradius of every component corner and whose vertical extent
covers every component. Because the horizontal cross-section is a square that
circumscribes the model under any rotation, the check result does not depend
on the commanded yaw of either drone — important now that shows carry yaw
setpoints.

Planning margin
---------------
The solver checks the envelope inflated by :data:`PLANNING_MARGIN` on every
side. Velocity smoothing (see ``converter``) re-times each drone along its
own path with a bounded schedule deviation of at most ~9.7% of one solver
step; the margin absorbs that deviation (with a lot of slack), so clearances
proven at plan time still hold for the smoothed trajectories. The final
verification gate (see ``verify``) re-checks the smoothed trajectories with
``margin=0`` as a defense in depth.

The swept-motion check (:func:`envelope_overlap_swept`) is **exact** for two
drones moving linearly and simultaneously: per axis, the relative offset is
linear in time, so the time interval during which each axis overlaps is
solved in closed form and the three intervals are intersected. There is no
sampling and therefore no tunneling.
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
WAKE_LENGTH = 0.15
WAKE_RADIUS = 0.03

# Margin (meters, per side of each drone's envelope) used for all plan-time
# collision checks. Must stay well above the velocity-smoothing schedule
# deviation bound (~0.097 * step_size per drone).
PLANNING_MARGIN = 0.25

# Legacy names used by the REST API / formation validator
COLLISION_X = BODY_SIZE_X
COLLISION_Y = BODY_SIZE_Y
COLLISION_Z = BODY_SIZE_Z

AABB = tuple[tuple[float, float, float], tuple[float, float, float]]

_HALF_BODY_X = BODY_SIZE_X * 0.5
_HALF_BODY_Y = BODY_SIZE_Y * 0.5
_HALF_BODY_Z = BODY_SIZE_Z * 0.5

_WAKE_THETA = math.radians(WAKE_ANGLE_DEG)


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


def _horizontal_circumradius(components: tuple[AABB, ...]) -> float:
    """Largest horizontal distance from the reference point to any corner."""
    radius = 0.0
    for rel_min, rel_max in components:
        x = max(abs(rel_min[0]), abs(rel_max[0]))
        y = max(abs(rel_min[1]), abs(rel_max[1]))
        radius = max(radius, math.hypot(x, y))
    return radius


_COMPONENT_AABBS = _component_aabbs_relative()
_COMBINED_MIN_REL, _COMBINED_MAX_REL = _combined_aabb_relative(_COMPONENT_AABBS)

# Yaw-invariant bounding envelope (see module docstring).
ENVELOPE_XY_HALF = _horizontal_circumradius(_COMPONENT_AABBS)
ENVELOPE_Z_MIN = _COMBINED_MIN_REL[2]
ENVELOPE_Z_MAX = _COMBINED_MAX_REL[2]
ENVELOPE_Z_HEIGHT = ENVELOPE_Z_MAX - ENVELOPE_Z_MIN

# Center-to-center clearance guaranteed between any two drones whose
# (margin-inflated) envelopes do not overlap. Axis-wise, hence also a lower
# bound on the Euclidean distance.
GUARANTEED_XY_CLEARANCE = 2.0 * ENVELOPE_XY_HALF
PLANNED_XY_CLEARANCE = 2.0 * (ENVELOPE_XY_HALF + PLANNING_MARGIN)

# Conservative "minimum distance" figure for show validation blocks
# (e.g. the .skyc validation settings), floored to a 0.1 m grid.
MIN_DISTANCE_FOR_VALIDATION = math.floor(GUARANTEED_XY_CLEARANCE * 10.0) / 10.0


def envelope_overlap(
    a: Sequence[float], b: Sequence[float], *, margin: float = 0.0
) -> bool:
    """Yaw-invariant envelope overlap check for two drones at *a* and *b*.

    ``margin`` inflates each drone's envelope on every side; pass
    :data:`PLANNING_MARGIN` for plan-time checks and 0 for final verification.
    """
    if abs(a[0] - b[0]) >= 2.0 * (ENVELOPE_XY_HALF + margin):
        return False
    if abs(a[1] - b[1]) >= 2.0 * (ENVELOPE_XY_HALF + margin):
        return False
    return abs(a[2] - b[2]) < ENVELOPE_Z_HEIGHT + 2.0 * margin


def _axis_overlap_interval(
    c: float, d: float, half_width: float
) -> tuple[float, float]:
    """Time interval within [0, 1] where ``|c + t*d| < half_width``."""
    if abs(d) < 1e-12:
        return (0.0, 1.0) if abs(c) < half_width else (1.0, 0.0)
    t_enter = (-half_width - c) / d
    t_exit = (half_width - c) / d
    lo, hi = (t_enter, t_exit) if t_enter <= t_exit else (t_exit, t_enter)
    return max(lo, 0.0), min(hi, 1.0)


def envelope_overlap_swept(
    a0: Sequence[float],
    a1: Sequence[float],
    b0: Sequence[float],
    b1: Sequence[float],
    *,
    margin: float = 0.0,
) -> bool:
    """Exact overlap check while both drones move linearly from t=0 to t=1.

    The relative offset on each axis is linear in time, so the overlap window
    per axis is solved in closed form; a collision exists iff the three
    windows intersect. No sampling, no tunneling.
    """
    lo = 0.0
    hi = 1.0
    for axis, half_width in (
        (0, 2.0 * (ENVELOPE_XY_HALF + margin)),
        (1, 2.0 * (ENVELOPE_XY_HALF + margin)),
        (2, ENVELOPE_Z_HEIGHT + 2.0 * margin),
    ):
        c = a0[axis] - b0[axis]
        d = (a1[axis] - a0[axis]) - (b1[axis] - b0[axis])
        axis_lo, axis_hi = _axis_overlap_interval(c, d, half_width)
        lo = max(lo, axis_lo)
        hi = min(hi, axis_hi)
        if lo >= hi:
            return False
    return True


def volumes_overlap(a: Sequence[float], b: Sequence[float]) -> bool:
    """Legacy alias: yaw-invariant envelope check with no margin."""
    return envelope_overlap(a, b)


def volumes_overlap_at_times(
    a0: Sequence[float],
    a1: Sequence[float],
    b0: Sequence[float],
    b1: Sequence[float],
    *,
    samples: int = 5,
) -> bool:
    """Legacy alias: exact swept check (the *samples* argument is ignored)."""
    return envelope_overlap_swept(a0, a1, b0, b1)


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
            "motors": [{"x": m[0], "y": m[1], "z": m[2]} for m in MOTOR_POSITIONS],
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
        "bounding_envelope": {
            "xy_half": ENVELOPE_XY_HALF,
            "z_min": ENVELOPE_Z_MIN,
            "z_max": ENVELOPE_Z_MAX,
            "planning_margin": PLANNING_MARGIN,
            "guaranteed_xy_clearance": GUARANTEED_XY_CLEARANCE,
            "planned_xy_clearance": PLANNED_XY_CLEARANCE,
        },
        # Kept for backward compatibility with older clients
        "x": COLLISION_X,
        "y": COLLISION_Y,
        "z": COLLISION_Z,
    }
