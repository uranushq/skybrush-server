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

Separation policy
-----------------
All conflict checks use a per-axis (Chebyshev) separation window: two
drones conflict unless they are at least the *separation* apart along at
least one axis. The separation defaults to — and can never drop below —
:data:`HARD_MIN_SEPARATION` (1.45 m); requests may raise it. Velocity
smoothing (see ``converter``) re-times each drone along its own path with a
bounded schedule deviation of at most ~15% of one solver step (natural-log
profile); the final verification gate (see ``verify``) re-checks the
smoothed trajectories with the same separation as a defense in depth.

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
# collision checks. Kept at 0 so the policy minimum separation
# (:data:`HARD_MIN_SEPARATION`) is the effective plan-time clearance;
# velocity-smoothing schedule error is small relative to that floor
# (~0.15 * step_size per drone for the log profile).
PLANNING_MARGIN = 0.0

# ── minimum inter-drone separation ───────────────────────────────────────
# Two drones must be at least this far apart (meters) along AT LEAST one
# axis — i.e. the Chebyshev distance between any two drone centers must be
# >= the separation, which also lower-bounds the Euclidean distance by the
# same value. This is a HARD operational floor: a request may RAISE the
# separation via `min_separation` but nothing may ever lower it below this
# constant. Enforced at request validation (initial/staging/phase points),
# throughout the solver and by the final verification gate.
HARD_MIN_SEPARATION = 1.45

# Boundary tolerance (meters) for the separation checks. 1.45 is not
# exactly representable in binary floating point, so spacings that are
# *nominally* exactly at the floor can land a hair under it after
# arithmetic (e.g. 10 + 1.45 == 11.449999...). One micrometer of slack is
# physically meaningless but makes boundary-exact formations deterministic.
SEPARATION_EPS = 1e-6


def clamp_separation(value) -> float:
    """Coerce a requested separation to a float no smaller than the floor."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return HARD_MIN_SEPARATION
    if parsed != parsed:  # NaN
        return HARD_MIN_SEPARATION
    return max(HARD_MIN_SEPARATION, parsed)


# Backwards-compatible alias: the formation spacing floor now equals the
# hard minimum separation on every axis.
MIN_FORMATION_XY_CLEARANCE = HARD_MIN_SEPARATION

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

# Yaw-invariant physical bounding envelope of one drone (body + wakes).
# Purely descriptive since the separation floor took over the collision
# windows — the floor (1.5 m) is far larger than the physical envelope.
_GEOMETRIC_XY_HALF = _horizontal_circumradius(_COMPONENT_AABBS)
ENVELOPE_XY_HALF = _GEOMETRIC_XY_HALF
ENVELOPE_Z_MIN = _COMBINED_MIN_REL[2]
ENVELOPE_Z_MAX = _COMBINED_MAX_REL[2]
ENVELOPE_Z_HEIGHT = ENVELOPE_Z_MAX - ENVELOPE_Z_MIN

# Center-to-center clearance guaranteed between any two drones that pass
# the overlap check at the default separation. Axis-wise (Chebyshev),
# hence also a lower bound on the Euclidean distance.
GUARANTEED_XY_CLEARANCE = HARD_MIN_SEPARATION
PLANNED_XY_CLEARANCE = HARD_MIN_SEPARATION

# Conservative "minimum distance" figure for show validation blocks
# (e.g. the .skyc validation settings), floored to a 0.1 m grid.
MIN_DISTANCE_FOR_VALIDATION = math.floor(GUARANTEED_XY_CLEARANCE * 10.0) / 10.0


def envelope_overlap(
    a: Sequence[float],
    b: Sequence[float],
    *,
    margin: float = 0.0,
    separation: float = HARD_MIN_SEPARATION,
    b_extends_below: float = 0.0,
    b_extends_above: float = 0.0,
) -> bool:
    """Separation check for two drones at *a* and *b*.

    The drones conflict unless they are at least ``separation`` meters
    apart along at least one axis (``separation`` is clamped to the hard
    floor :data:`HARD_MIN_SEPARATION`; boundary-exact spacings pass).
    ``margin`` inflates the window on every side; ``b_extends_below`` /
    ``b_extends_above`` additionally extend *b*'s conflict zone that many
    meters downward / upward — used by route planning to treat a parked
    drone's downwash column (and the wash-inflicting zone right above it)
    as blocked.
    """
    window = (
        max(separation, HARD_MIN_SEPARATION) + 2.0 * margin - SEPARATION_EPS
    )
    if abs(a[0] - b[0]) >= window:
        return False
    if abs(a[1] - b[1]) >= window:
        return False
    dz = a[2] - b[2]
    return -(window + b_extends_below) < dz < window + b_extends_above


def _axis_overlap_interval(
    c: float, d: float, lo_bound: float, hi_bound: float
) -> tuple[float, float]:
    """Time interval within [0, 1] where ``lo_bound < c + t*d < hi_bound``."""
    if abs(d) < 1e-12:
        return (0.0, 1.0) if lo_bound < c < hi_bound else (1.0, 0.0)
    t_enter = (lo_bound - c) / d
    t_exit = (hi_bound - c) / d
    lo, hi = (t_enter, t_exit) if t_enter <= t_exit else (t_exit, t_enter)
    return max(lo, 0.0), min(hi, 1.0)


def envelope_overlap_swept(
    a0: Sequence[float],
    a1: Sequence[float],
    b0: Sequence[float],
    b1: Sequence[float],
    *,
    margin: float = 0.0,
    separation: float = HARD_MIN_SEPARATION,
    b_extends_below: float = 0.0,
    b_extends_above: float = 0.0,
) -> bool:
    """Exact separation check while both drones move linearly from t=0 to 1.

    The relative offset on each axis is linear in time, so the conflict
    window per axis is solved in closed form; a conflict exists iff the
    three windows intersect. No sampling, no tunneling. ``b_extends_below``
    / ``b_extends_above`` extend *b*'s conflict zone downward / upward (see
    :func:`envelope_overlap`), making the z window asymmetric.
    """
    window = (
        max(separation, HARD_MIN_SEPARATION) + 2.0 * margin - SEPARATION_EPS
    )
    lo = 0.0
    hi = 1.0
    for axis, lo_bound, hi_bound in (
        (0, -window, window),
        (1, -window, window),
        (2, -(window + b_extends_below), window + b_extends_above),
    ):
        c = a0[axis] - b0[axis]
        d = (a1[axis] - a0[axis]) - (b1[axis] - b0[axis])
        axis_lo, axis_hi = _axis_overlap_interval(c, d, lo_bound, hi_bound)
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
        "separation": {
            "hard_min": HARD_MIN_SEPARATION,
            "semantics": "per-axis (Chebyshev): pairs must differ by at "
            "least the separation on x, y or z",
        },
        # Kept for backward compatibility with older clients
        "x": COLLISION_X,
        "y": COLLISION_Y,
        "z": COLLISION_Z,
    }
