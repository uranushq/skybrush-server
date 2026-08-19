"""Path-planner extension — REST API for automatic 3D drone path generation.

Endpoint
--------
POST ``/api/v1/path-planner/plan``

Formation-phase flow (``phases`` present)::

    ground (per-drone [x, y, z_ground])
      │  vertical takeoff
      ▼
    staging hover  (z_ground + staging_altitude, default 5 m)
      │  solver: collision-avoided move (a no-op when already spread out)
      ▼
    staging layout (take-off shape, spread about drone-1 until every pair
      │             clears grid_spacing, default 2 m — the algorithm's start)
      │  solver: phase 1, phase 2, ... (+ per-phase holds and yaw changes)
      ▼
    return to staging hover  (when return_to_initial, default true)
      │  vertical landing
      ▼
    ground

Inside every solver segment all drones depart **simultaneously** and are
coordinated per step through collision clusters (see ``solver``): drones
whose motions interact are sequenced within their cluster, everyone else
flies unimpeded — a formation translating as a unit moves in lockstep with
its relative separations preserved. One downwash-aware operational rule
remains:

* **Staged stack entry** (:func:`_stack_entry_plan`): a drone whose target
  sits within 2.5 m under another drone approaches 2.5 m below its target
  and climbs the final stretch vertically at a constant 0.5 m/s, after the
  drones above it have settled.

Per-phase **clusters** move as rigid groups: drones listed in a phase's
``clusters`` field (``[["drone-1", "drone-2"], ...]``) are pinned to their
straight lines for that transition *and* depart as one — the block waits
until every member can move, so it holds its shape in time as well as in
geometry. Listing a drone that also has a ``fixedPaths`` entry is allowed
and is how a hand-drawn path joins a block: the drawn path wins, the
cluster only synchronises the departure.

A block that *contracts* rather than translates has straight lines that
converge on nearly one point and cannot be flown that way. Whether a block
is schedulable alongside the rest of the fleet is the solver's own
per-step decision, which geometry cannot predict, so a cluster that will
not solve is released and the segment replanned normally — with a warning
on the phase summary — instead of failing a show whose formations are all
legal. Groups whose members all share the *same
displacement vector* (a formation block translated as a unit between
phases) are detected and clustered **automatically** when their straight
corridors clear everything that cannot get out of the way — parked drones,
user-pinned paths and the corridors of the blocks already clustered.
Clashing blocks stay unpinned and are planned normally, since two pinned
corridors that cross would leave the solver no legal move. Explicit
``clusters`` entries always win.

A phase may pin selected drones to **user-defined fixed paths** via
``fixedPaths`` (``[{"droneId": "drone-3", "path": [{x, y, z}, ...]}]``):
during the transition *into* that phase the drone follows the waypoints
verbatim (the last waypoint must equal its phase target) while the solver
routes everyone else around it. Fixed-path drones skip the staged stack
entry — the drawn path is trusted as the user's intent — but the final
verification gate still checks the result.

Safety contract (fail-loudly)
-----------------------------
Planning failures never produce partial output: if any solver segment fails,
if a trajectory violates the velocity/yaw-rate limits, or if the final
spatio-temporal verification gate finds an envelope overlap, the request is
answered with an error response and **nothing is saved or uploaded**.

All CPU-heavy work (solving, show building, verification) runs in a worker
thread so the server event loop keeps serving MAVLink traffic while a plan
is computed.
"""

from __future__ import annotations

import re
from contextlib import ExitStack
from copy import deepcopy
from itertools import combinations, permutations
from json import dumps
from logging import Logger
from math import ceil, sqrt
from pathlib import Path
from time import perf_counter
from time import time as wall_time
from typing import TYPE_CHECKING, Optional, Sequence

from quart import Blueprint, Response, jsonify, request
from trio import sleep_forever, to_thread

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .collision_volume import (
    GUARANTEED_XY_CLEARANCE,
    HARD_MIN_SEPARATION,
    PLANNING_MARGIN,
    clamp_separation,
    describe_collision_envelope,
    envelope_overlap,
    envelope_overlap_swept,
)
from .converter import (
    _THICKNESS_MAX,
    _THICKNESS_MIN,
    _ramp_from_smoothing,
    DEFAULT_ACCEL_SHAPE,
    DEFAULT_CRUISE_SPEED_M_S,
    DEFAULT_DECEL_SHAPE,
    DEFAULT_LANDING_SPEED_M_S,
    DEFAULT_MAX_YAW_RATE_DEG_S,
    DEFAULT_PROFILE_EXP,
    DEFAULT_PROFILE_LOG,
    DEFAULT_TAKEOFF_SPEED_M_S,
    DEFAULT_VELOCITY_SMOOTHING,
    RAMP_SHAPES,
    TrajectoryLimitError,
    VelocityProfile,
    build_delivery_show_dicts,
    build_show_dicts,
    duration_ms_for_cruise_speed,
    lerp_yaw_deg,
    save_skyb_files,
    vertical_transit_duration_sec,
    yaw_delta_deg,
)
from .drone import Drone
from .output import (
    build_output,
    build_show_specifications,
    skyc_bytes_from_show_dicts,
)
from .solver import (
    DOWNWASH_EXPOSURE_MS,
    DOWNWASH_ROUTE_CLEARANCE,
    PathSolver,
    SolverResult,
    StepRecord,
)
from .validators import (
    SEVERITY_ERROR,
    ValidationContext,
    collect_required_params,
    fetch_required_params,
    resolve_min_alt,
    run_validators,
)
from .verify import verify_show_dicts

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("path_planner", __name__)

# Module-level globals injected at runtime via `overridden(globals(), ...)`.
# Available only while the extension is loaded.
app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None

# Global default velocity-smoothing strength (0..1) applied to *every* planned
# path. Set from the extension configuration in `run()` so it can be adjusted
# from the server config UI; used as the default in `plan()`.
velocity_smoothing: float = DEFAULT_VELOCITY_SMOOTHING


def _parse_profile_knobs(body: dict, smoothing: float = DEFAULT_VELOCITY_SMOOTHING):
    """Parse the velocity-profile knobs of a request.

    Returns ``(profile_exp, profile_log, profile, error_response)`` where the
    error response is ``None`` on success.

    The legacy pair still describes the historical symmetric exp-in / log-out
    shape, and is what the returned ``profile`` falls back to. The per-side
    fields override it independently::

        profile_accel_shape   exp | log | linear | none   (default exp)
        profile_accel_curve   _THICKNESS_MIN .. _MAX      (default profile_exp)
        profile_accel_width   0 .. 1                      (default from smoothing)
        profile_decel_shape   exp | log | linear | none   (default log)
        profile_decel_curve   _THICKNESS_MIN .. _MAX      (default profile_log)
        profile_decel_width   0 .. 1                      (default from smoothing)

    The plateau (constant-speed) region takes whatever the two ramps leave, so
    the widths may sum to at most 1. Both ramps disabled — widths 0, or shapes
    set to ``none`` — means a single constant-speed run, exactly what
    ``velocity_smoothing = 0`` already means.

    Note a ``decel`` ramp evaluates its basis backwards in time, so ``exp`` on
    the deceleration side is the gentle-at-arrival choice and ``log`` brakes
    hardest at arrival.
    """

    def bad(message: str):
        return None, None, None, (jsonify({"error": message}), 400)

    def number(key: str, default: float, low: float, high: float):
        """Parse one optional numeric field, or return an error tuple."""
        raw = body.get(key)
        if raw is None:
            return default, None
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return None, bad(f"'{key}' must be a number")
        if not (low <= value <= high):
            return None, bad(f"'{key}' must be between {low} and {high}")
        return value, None

    try:
        k_exp = float(body.get("profile_exp", DEFAULT_PROFILE_EXP))
        k_log = float(body.get("profile_log", DEFAULT_PROFILE_LOG))
    except (TypeError, ValueError):
        return bad("'profile_exp'/'profile_log' must be numbers")
    for name, value in (("profile_exp", k_exp), ("profile_log", k_log)):
        if not (_THICKNESS_MIN <= value <= _THICKNESS_MAX):
            return bad(
                f"'{name}' must be between {_THICKNESS_MIN} and {_THICKNESS_MAX}"
            )

    default_width = _ramp_from_smoothing(smoothing)
    shapes: dict[str, str] = {}
    curves: dict[str, float] = {}
    widths: dict[str, float] = {}
    for side, default_shape, legacy_curve in (
        ("accel", DEFAULT_ACCEL_SHAPE, k_exp),
        ("decel", DEFAULT_DECEL_SHAPE, k_log),
    ):
        raw_shape = body.get(f"profile_{side}_shape")
        if raw_shape is None:
            shapes[side] = default_shape
        else:
            name = str(raw_shape).strip().lower()
            if name not in RAMP_SHAPES:
                return bad(
                    f"'profile_{side}_shape' must be one of {', '.join(RAMP_SHAPES)}"
                )
            shapes[side] = name

        curve, err = number(
            f"profile_{side}_curve", legacy_curve, _THICKNESS_MIN, _THICKNESS_MAX
        )
        if err is not None:
            return err
        curves[side] = curve

        width, err = number(f"profile_{side}_width", default_width, 0.0, 1.0)
        if err is not None:
            return err
        widths[side] = width

    # A "none" ramp contributes no width, so it can never push the plateau
    # negative — only explicitly requested widths are checked against the sum.
    used = [
        0.0 if shapes[side] == "none" else widths[side] for side in ("accel", "decel")
    ]
    if sum(used) > 1.0 + 1e-9:
        return bad(
            "'profile_accel_width' + 'profile_decel_width' must be at most 1 "
            f"(got {used[0]:g} + {used[1]:g}); the plateau takes the rest"
        )

    profile = VelocityProfile(
        accel_shape=shapes["accel"],
        accel_curve=curves["accel"],
        accel_width=widths["accel"],
        decel_shape=shapes["decel"],
        decel_curve=curves["decel"],
        decel_width=widths["decel"],
    )
    return k_exp, k_log, profile, None


def _profile_report(profile: VelocityProfile) -> dict:
    """Serializable echo of the profile actually applied, for API responses."""
    return {
        "accel_shape": profile.accel_shape,
        "accel_curve": profile.accel_curve,
        "accel_width": profile.accel_width,
        "plateau_width": profile.plateau_width,
        "decel_shape": profile.decel_shape,
        "decel_curve": profile.decel_curve,
        "decel_width": profile.decel_width,
    }

# Base directory for generated files; requests may only choose subdirectories
# of this. Empty string means "parent of the server's working directory".
output_base_dir: str = ""

# Skybrush firmware tends to silently reject very short shows or shows with a
# zero takeoff time; every mode enforces this same minimum ground wait.
MIN_TAKEOFF_TIME = 5.0

# Staging defaults: hover this high above each drone's ground position, then
# spread the take-off layout out to at least this spacing before the
# requested formation phases start.
DEFAULT_STAGING_ALTITUDE = 5.0
DEFAULT_GRID_SPACING = 2.0

# Landing default: when 'landing_grid' is requested, spread the final return
# leg out to at least this spacing (same spread solver as take-off staging)
# instead of returning to the original take-off hover spots.
DEFAULT_LANDING_SPACING = 4.0

# Staging coordinates are rounded to STAGING_ROUND_DIGITS decimals for a
# readable API response; rounding can shave up to 1e-4 m off a pair, so the
# spread aims this far past the requested spacing to stay clear of the
# separation check afterwards.
STAGING_ROUND_DIGITS = 4
STAGING_SPREAD_GUARD = 1e-3

# Vertical-stack entry (downwash mitigation). When a target formation places
# one drone within STACK_VERTICAL_GAP above another with horizontal offset
# below STACK_XY_TOLERANCE (i.e. inside the upper drone's downwash column),
# the lower drone first flies to an approach point STACK_APPROACH_OFFSET
# below its target and enters the last stretch as a pure vertical climb at a
# constant STACK_CLIMB_SPEED — only after every stacked drone above it has
# settled at its own target.
STACK_VERTICAL_GAP = 2.5
STACK_APPROACH_OFFSET = 2.5
STACK_XY_TOLERANCE = GUARANTEED_XY_CLEARANCE
STACK_CLIMB_SPEED = 0.5


# 진행 중인 계획 작업의 실시간 상태. 워커 스레드가 갱신하고
# GET /path-planner/progress 가 읽는다 (단일 작업 가정, 원자적 dict 갱신).
_progress_state: dict = {"active": False}


def _progress_update(**updates) -> None:
    """Merge a progress update; bumps segment bookkeeping on label change."""
    segment = updates.get("segment")
    if segment is not None and segment != _progress_state.get("segment"):
        _progress_state["segment_index"] = (
            _progress_state.get("segment_index", 0) + 1
        )
        _progress_state["segment_started_at"] = wall_time()
    _progress_state.update(updates)


class PlanningError(Exception):
    """A solver segment could not produce a collision-free plan."""

    def __init__(self, message: str, details: Optional[dict] = None) -> None:
        super().__init__(message)
        self.details = details or {}


# ── request parsing helpers ──────────────────────────────────────────────


def _is_vec3(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(v, (int, float)) for v in value)
    ) or (
        isinstance(value, dict)
        and all(isinstance(value.get(key), (int, float)) for key in ("x", "y", "z"))
    )


def _validate_vec3_array(name: str, value):
    if not isinstance(value, list):
        return jsonify({"error": f"'{name}' must be an array of [x,y,z]"}), 400
    if len(value) == 0:
        return jsonify({"error": f"'{name}' must not be empty"}), 400

    for i, pt in enumerate(value):
        if not _is_vec3(pt):
            return (
                jsonify({"error": f"'{name}[{i}]' must be [x, y, z] (3 numbers)"}),
                400,
            )

    return None


def _normalize_vec3_array(name: str, value: list) -> list[list[float]]:
    """Normalize ``[x, y, z]`` or ``{droneId, x, y, z}`` arrays.

    When objects include ``droneId``, the return value is ordered by
    ``drone-1``, ``drone-2`` ... so later phase targets line up with drones.
    """
    has_drone_ids = any(
        isinstance(point, dict) and "droneId" in point for point in value
    )
    if not has_drone_ids:
        return [
            (
                [float(point["x"]), float(point["y"]), float(point["z"])]
                if isinstance(point, dict)
                else [float(point[0]), float(point[1]), float(point[2])]
            )
            for point in value
        ]

    normalized: list[list[float] | None] = [None] * len(value)
    for point_index, point in enumerate(value):
        if not isinstance(point, dict):
            raise ValueError(
                f"'{name}[{point_index}]' must be an object when droneId is used"
            )
        drone_index = _phase_point_drone_index(point, point_index, len(value))
        if drone_index is None:
            raise ValueError(
                f"'{name}[{point_index}].droneId' must match one of "
                "drone-1..drone-N or show-drone-1..show-drone-N"
            )
        if normalized[drone_index] is not None:
            raise ValueError(
                f"'{name}' contains duplicate entry for drone-{drone_index + 1}"
            )
        normalized[drone_index] = [
            float(point["x"]),
            float(point["y"]),
            float(point["z"]),
        ]

    if any(point is None for point in normalized):
        raise ValueError(f"'{name}' is missing one or more drone entries")
    return [point for point in normalized if point is not None]


def _validate_phases(phases, *, num_drones: int):
    if not isinstance(phases, list) or len(phases) == 0:
        return jsonify({"error": "'phases' must be a non-empty array"}), 400

    for phase_index, phase in enumerate(phases):
        if not isinstance(phase, dict):
            return (
                jsonify({"error": f"'phases[{phase_index}]' must be an object"}),
                400,
            )

        points = phase.get("points")
        if not isinstance(points, list) or len(points) != num_drones:
            return (
                jsonify(
                    {
                        "error": (
                            f"'phases[{phase_index}].points' must contain exactly "
                            f"{num_drones} waypoint(s)"
                        )
                    }
                ),
                400,
            )

        try:
            hold_ms = int(phase.get("holdMs", 0))
        except (TypeError, ValueError):
            return (
                jsonify(
                    {"error": f"'phases[{phase_index}].holdMs' must be an integer"}
                ),
                400,
            )
        if hold_ms < 0:
            return (
                jsonify({"error": f"'phases[{phase_index}].holdMs' must be >= 0"}),
                400,
            )

        seen: set[int] = set()
        targets_by_index: dict[int, tuple[float, float, float]] = {}
        for point_index, point in enumerate(points):
            if not isinstance(point, dict):
                return (
                    jsonify(
                        {
                            "error": (
                                f"'phases[{phase_index}].points[{point_index}]' "
                                "must be an object"
                            )
                        }
                    ),
                    400,
                )

            for key in ("x", "y", "z"):
                if not isinstance(point.get(key), (int, float)):
                    return (
                        jsonify(
                            {
                                "error": (
                                    f"'phases[{phase_index}].points[{point_index}].{key}' "
                                    "must be a number"
                                )
                            }
                        ),
                        400,
                    )

            yaw = point.get("yaw")
            if yaw is not None and not isinstance(yaw, (int, float)):
                return (
                    jsonify(
                        {
                            "error": (
                                f"'phases[{phase_index}].points[{point_index}].yaw' "
                                "must be a number"
                            )
                        }
                    ),
                    400,
                )

            drone_index = _phase_point_drone_index(point, point_index, num_drones)
            if drone_index is None:
                return (
                    jsonify(
                        {
                            "error": (
                                f"'phases[{phase_index}].points[{point_index}].droneId' "
                                "must match one of drone-1..drone-N, "
                                "show-drone-1..show-drone-N, or be omitted "
                                "when points are already in drone order"
                            )
                        }
                    ),
                    400,
                )
            if drone_index in seen:
                return (
                    jsonify(
                        {
                            "error": (
                                f"'phases[{phase_index}].points' contains duplicate "
                                f"target for drone-{drone_index + 1}"
                            )
                        }
                    ),
                    400,
                )
            seen.add(drone_index)
            targets_by_index[drone_index] = (
                float(point["x"]),
                float(point["y"]),
                float(point["z"]),
            )

        error = _validate_phase_fixed_paths(
            phase, phase_index, num_drones, targets_by_index
        )
        if error is not None:
            return error

        error = _validate_phase_clusters(phase, phase_index, num_drones)
        if error is not None:
            return error

    return None


# A fixed path's last waypoint must land on the drone's phase target within
# this tolerance (meters) — the path *defines* the transition into the phase.
FIXED_PATH_TARGET_TOLERANCE = 0.01


def _validate_phase_fixed_paths(
    phase: dict,
    phase_index: int,
    num_drones: int,
    targets_by_index: dict[int, tuple[float, float, float]],
):
    """Validate the optional ``fixedPaths`` block of one phase.

    Shape: ``[{"droneId": "drone-3", "path": [{"x":..,"y":..,"z":..}, ...]}]``.
    The path is the exact route the drone must fly while transitioning
    *into* this phase; its last waypoint must match the drone's target in
    the phase's ``points``.
    """
    fixed = phase.get("fixedPaths", phase.get("fixed_paths"))
    if fixed is None:
        return None
    prefix = f"'phases[{phase_index}].fixedPaths"
    if not isinstance(fixed, list):
        return jsonify({"error": f"{prefix}' must be an array"}), 400

    seen: set[int] = set()
    for entry_index, entry in enumerate(fixed):
        if not isinstance(entry, dict):
            return (
                jsonify({"error": f"{prefix}[{entry_index}]' must be an object"}),
                400,
            )
        drone_id = entry.get("droneId", entry.get("id"))
        drone_index = _drone_index_from_id(drone_id) if drone_id is not None else None
        if drone_index is None or not (0 <= drone_index < num_drones):
            return (
                jsonify(
                    {
                        "error": (
                            f"{prefix}[{entry_index}].droneId' must match one "
                            "of drone-1..drone-N"
                        )
                    }
                ),
                400,
            )
        if drone_index in seen:
            return (
                jsonify(
                    {
                        "error": (
                            f"{prefix}' contains duplicate entry for "
                            f"drone-{drone_index + 1}"
                        )
                    }
                ),
                400,
            )
        seen.add(drone_index)

        path = entry.get("path")
        if not isinstance(path, list) or not path:
            return (
                jsonify(
                    {
                        "error": (
                            f"{prefix}[{entry_index}].path' must be a "
                            "non-empty array of waypoints"
                        )
                    }
                ),
                400,
            )
        for wp_index, waypoint in enumerate(path):
            if not isinstance(waypoint, dict) or any(
                not isinstance(waypoint.get(key), (int, float))
                for key in ("x", "y", "z")
            ):
                return (
                    jsonify(
                        {
                            "error": (
                                f"{prefix}[{entry_index}].path[{wp_index}]' "
                                "must be an object with numeric x, y, z"
                            )
                        }
                    ),
                    400,
                )

        target = targets_by_index[drone_index]
        last = path[-1]
        deviation = sqrt(
            (float(last["x"]) - target[0]) ** 2
            + (float(last["y"]) - target[1]) ** 2
            + (float(last["z"]) - target[2]) ** 2
        )
        if deviation > FIXED_PATH_TARGET_TOLERANCE:
            return (
                jsonify(
                    {
                        "error": (
                            f"{prefix}[{entry_index}].path' must end at "
                            f"drone-{drone_index + 1}'s target for this phase "
                            f"(off by {deviation:.3f} m)"
                        )
                    }
                ),
                400,
            )
    return None


def _validate_phase_clusters(phase: dict, phase_index: int, num_drones: int):
    """Validate the optional ``clusters`` block of one phase.

    Shape: ``[["drone-1", "drone-2"], ...]`` — each inner list is a rigid
    group flown in lockstep on straight lines during the transition into
    this phase. A drone may belong to at most one cluster.

    Overlapping with ``fixedPaths`` is allowed and useful: the pin fixes a
    drone's *geometry*, the cluster fixes the block's *timing*, and only
    together do they make a group leave as one. A pinned member keeps the
    path the user drew; the cluster merely holds it back until the rest of
    the block can go too.
    """
    clusters = phase.get("clusters")
    if clusters is None:
        return None
    prefix = f"'phases[{phase_index}].clusters"
    if not isinstance(clusters, list):
        return jsonify({"error": f"{prefix}' must be an array of arrays"}), 400

    seen: set[int] = set()
    for cluster_index, cluster in enumerate(clusters):
        if not isinstance(cluster, list) or not cluster:
            return (
                jsonify(
                    {
                        "error": (
                            f"{prefix}[{cluster_index}]' must be a non-empty "
                            "array of drone ids"
                        )
                    }
                ),
                400,
            )
        for drone_id in cluster:
            index = _drone_index_from_id(drone_id)
            if index is None or not (0 <= index < num_drones):
                return (
                    jsonify(
                        {
                            "error": (
                                f"{prefix}[{cluster_index}]' contains an "
                                f"unknown drone id {drone_id!r}"
                            )
                        }
                    ),
                    400,
                )
            if index in seen:
                return (
                    jsonify(
                        {
                            "error": (
                                f"{prefix}' lists drone-{index + 1} in more "
                                "than one cluster"
                            )
                        }
                    ),
                    400,
                )
            seen.add(index)
    return None


def _phase_cluster_indices(phase: dict, num_drones: int) -> list[set[int]]:
    """Parse a validated phase's ``clusters`` into index sets."""
    clusters = phase.get("clusters") or []
    result: list[set[int]] = []
    for cluster in clusters:
        indices = {
            index
            for index in (_drone_index_from_id(d) for d in cluster)
            if index is not None and 0 <= index < num_drones
        }
        if indices:
            result.append(indices)
    return result


def _corridors_conflict(
    a_start: Sequence[float],
    a_end: Sequence[float],
    b_start: Sequence[float],
    b_end: Sequence[float],
    *,
    separation: float,
) -> bool:
    """Do two straight corridors flown side by side ever come too close?

    Both drones leave on the same step and cover ``step_size`` per step, so
    they are compared by *distance travelled* rather than by normalised
    time: first the stretch both are still flying, then the tail where the
    one that arrived sits parked while the other finishes. Feeding the two
    corridors to a single swept check instead would silently rescale the
    shorter one to the longer one's duration and compare positions the
    drones are never at together.
    """
    length_a = Drone.distance(a_start, a_end)
    length_b = Drone.distance(b_start, b_end)
    shared = min(length_a, length_b)

    def advance(start, end, length: float) -> list[float]:
        if length <= 1e-9:
            return [float(v) for v in start]
        fraction = shared / length
        return [start[axis] + (end[axis] - start[axis]) * fraction for axis in range(3)]

    a_mid = advance(a_start, a_end, length_a)
    b_mid = advance(b_start, b_end, length_b)
    # Leg 1: both moving. Leg 2: one of the two is already parked, so its
    # segment is degenerate and this reduces to a swept-vs-static check.
    return envelope_overlap_swept(
        [float(v) for v in a_start],
        a_mid,
        [float(v) for v in b_start],
        b_mid,
        margin=PLANNING_MARGIN,
        separation=separation,
    ) or envelope_overlap_swept(
        a_mid,
        [float(v) for v in a_end],
        b_mid,
        [float(v) for v in b_end],
        margin=PLANNING_MARGIN,
        separation=separation,
    )


def _pinned_blocks(
    pinned: dict[int, list[tuple[float, float, float]]],
    groups: Sequence[set[int]],
) -> list[set[int]]:
    """The pinned formation blocks that can be given a running order.

    Only whole blocks — user clusters and automatically detected rigid
    groups — qualify. Individual pinned drones are deliberately left out:
    ordering them one by one would serialise the fleet drone by drone and,
    worse, park half of it mid-flight where its held position may not clear
    the other half's targets. A block is included only where every member
    is pinned; a drone free to detour needs no place in the order.
    """
    blocks: list[set[int]] = []
    for group in groups:
        members = {index for index in group if index in pinned}
        if len(members) > 1 and not any(members & block for block in blocks):
            blocks.append(members)
    return blocks


def _dispatch_waves(
    blocks: Sequence[set[int]],
    *,
    current_positions: Sequence[Sequence[float]],
    targets: Sequence[Sequence[float]],
    separation: float,
) -> list[list[set[int]]]:
    """Order pinned blocks so the ones that must vacate a spot fly first.

    A pinned drone cannot detour, so a block heading for spots that another
    block is still standing on — or still has to fly through — can only be
    sent once that block has cleared out. Flying them together deadlocks:
    whoever arrives first parks and becomes an obstacle the other can never
    get past.

    Block *A* must precede *B* when some member of *B* is bound for a spot
    inside a member of *A*'s straight lane. Those edges are sorted
    topologically into waves; blocks in one wave are mutually independent
    and fly together. Returns ``[]`` when the ordering is cyclic — the
    blocks then genuinely cannot all be flown as drawn and the caller is
    better off letting the solver report which ones clash.
    """
    if len(blocks) < 2:
        return [list(blocks)] if blocks else []

    def lane(index: int):
        return (
            tuple(current_positions[index]),
            tuple(targets[index]),
        )

    def needs_clearing(after: set[int], before: set[int]) -> bool:
        """Does any member of ``after`` want a spot in ``before``'s lanes?"""
        return any(
            envelope_overlap_swept(
                list(lane(a)[0]),
                list(lane(a)[1]),
                list(targets[b]),
                list(targets[b]),
                margin=PLANNING_MARGIN,
                separation=separation,
            )
            for a in before
            for b in after
        )

    blocked_by: list[set[int]] = [set() for _ in blocks]
    for later, earlier in permutations(range(len(blocks)), 2):
        if needs_clearing(blocks[later], blocks[earlier]):
            blocked_by[later].add(earlier)

    waves: list[list[set[int]]] = []
    done: set[int] = set()
    while len(done) < len(blocks):
        ready = [
            index
            for index in range(len(blocks))
            if index not in done and blocked_by[index] <= done
        ]
        if not ready:
            return []  # cyclic: no order can satisfy every block
        waves.append([blocks[index] for index in ready])
        done.update(ready)
    return waves


def _pinned_corridors_deadlock(
    a_start: Sequence[float],
    a_end: Sequence[float],
    b_start: Sequence[float],
    b_end: Sequence[float],
    *,
    separation: float,
) -> bool:
    """Would pinning both of these corridors leave the solver no move at all?

    Pinned drones never detour, but they *can* wait, and the solver staggers
    two conflicting pinned lanes by sending whichever drone stands in the
    other's way first. A crossing is therefore only fatal when *no* order
    works. Three schedules are tried: both flying together, A then B, and B
    then A — where "A then B" means A flies its whole line while B waits on
    its start, then B flies while A sits parked on its target.

    Rejecting on the simultaneous check alone would throw away perfectly
    flyable moves, such as a drone descending onto the spot another drone is
    just now vacating.
    """

    def clear(start, end, point) -> bool:
        return not envelope_overlap_swept(
            [float(v) for v in start],
            [float(v) for v in end],
            [float(v) for v in point],
            [float(v) for v in point],
            margin=PLANNING_MARGIN,
            separation=separation,
        )

    if not _corridors_conflict(
        a_start, a_end, b_start, b_end, separation=separation
    ):
        return False
    a_first = clear(a_start, a_end, b_start) and clear(b_start, b_end, a_end)
    b_first = clear(b_start, b_end, a_start) and clear(a_start, a_end, b_end)
    return not (a_first or b_first)


def _pin_flyable_clusters(
    routes: dict[int, list[tuple[float, float, float]]],
    clusters: Sequence[set[int]],
    *,
    current_positions: Sequence[Sequence[float]],
    targets: Sequence[Sequence[float]],
    separation: float,
    label: str,
) -> tuple[list[set[int]], list[str]]:
    """Pin the members of every ``clusters`` block that can fly straight.

    A cluster asks a block to hold its shape by putting each member on a
    straight line to its own spot. That only works while those lines stay
    clear of one another and of the paths the user fixed by hand — which
    holds for a block that *translates*, and fails for one that contracts:
    members fanning in from a wide line onto a tight formation have lines
    that converge on nearly the same point. Pinning such a block strips
    every member of its right to detour and deadlocks the segment, so it is
    left unpinned and planned normally instead. It still reaches exactly
    the same formation, just not in lockstep.

    Mutates ``routes`` and returns the accepted blocks — to be handed to the
    solver as lockstep groups, so they hold their shape in *time* as well as
    in geometry — together with one human-readable note per dropped cluster.
    A block whose members' lanes clear at different moments would otherwise
    trickle away one drone at a time even though each flies a perfect
    straight line.
    """
    def segments(index: int) -> list[tuple[Sequence[float], Sequence[float]]]:
        """The corridor a drone will actually fly, as straight legs."""
        route = routes.get(index) or [tuple(targets[index])]
        points = [tuple(current_positions[index]), *(tuple(p) for p in route)]
        return list(zip(points, points[1:]))

    # Corridors already committed, by drone. User fixedPaths are in from the
    # start; every cluster accepted below joins them.
    reserved: dict[int, list[tuple[Sequence[float], Sequence[float]]]] = {
        index: segments(index) for index in routes
    }

    groups: list[set[int]] = []
    notes: list[str] = []
    for cluster in clusters:
        # The whole cluster is the lockstep unit, including members the user
        # also gave an explicit fixedPath: those keep their drawn path (only
        # unpinned members get a straight line) but still depart with the
        # block. Dropping them here is what used to reduce a three-drone
        # group to nothing.
        members = sorted(cluster)
        if len(members) < 2:
            continue
        corridors = {index: segments(index) for index in members}

        def clash_between(
            left: int, right: int, corridors=corridors, reserved=reserved
        ) -> bool:
            for a_start, a_end in corridors[left]:
                for b_start, b_end in (reserved.get(right) or corridors[right]):
                    if _pinned_corridors_deadlock(
                        a_start, a_end, b_start, b_end, separation=separation
                    ):
                        return True
            return False

        clash: tuple[str, str] | None = None
        for left, right in combinations(members, 2):
            if clash_between(left, right):
                clash = (f"drone-{left + 1}", f"drone-{right + 1}")
                break
        if clash is None:
            for index in members:
                for other in reserved:
                    if other in cluster or not clash_between(index, other):
                        continue
                    clash = (f"drone-{index + 1}", f"drone-{other + 1}")
                    break
                if clash is not None:
                    break

        if clash is not None:
            note = (
                f"cluster {[f'drone-{i + 1}' for i in members]} was not flown "
                f"in lockstep: its straight lines cannot all be flown "
                f"({clash[0]} and {clash[1]} block each other whichever goes "
                "first), so the block was planned normally and still reaches "
                "the same formation"
            )
            notes.append(note)
            if log:
                log.warning(f"segment '{label}': {note}")
            continue

        for index in members:
            routes.setdefault(index, [tuple(targets[index])])
            reserved[index] = corridors[index]
        groups.append(set(members))
    return groups, notes


def _detect_rigid_groups(
    current_positions: Sequence[tuple[float, float, float]],
    targets: Sequence[tuple[float, float, float]],
    *,
    excluded: set[int],
    static_positions: Sequence[Sequence[float]],
    pinned_routes: Sequence[Sequence[Sequence[float]]] = (),
    separation: float = HARD_MIN_SEPARATION,
) -> dict[int, list[tuple[float, float, float]]]:
    """Find formation blocks translated as a unit and pin them to lockstep.

    Groups drones by identical displacement vector (mm resolution). A group
    of two or more flies as a rigid cluster — every member pinned to its
    straight line, so the block translates in lockstep with its internal
    geometry frozen and no member ever weaves through the group.

    A group only earns its pins when its corridors clear everything that
    cannot get out of the way: the parked drones (including their vertical
    downwash pads), the ``pinned_routes`` the user fixed by hand, and the
    corridors of groups already pinned here. Pinning is what takes away a
    drone's right to detour, so two pinned corridors that cross leave the
    solver no move to make and deadlock the whole segment. Groups that
    clash fall back to normal planning, where the solver is free to route
    them around whatever they clashed with; bigger blocks are offered the
    pins first, since they gain the most from flying in lockstep.
    """
    groups: dict[tuple[int, int, int], list[int]] = {}
    for i, (pos, tgt) in enumerate(zip(current_positions, targets)):
        if i in excluded:
            continue
        d = (tgt[0] - pos[0], tgt[1] - pos[1], tgt[2] - pos[2])
        if abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9 and abs(d[2]) < 1e-9:
            continue
        key = (round(d[0] * 1000), round(d[1] * 1000), round(d[2] * 1000))
        groups.setdefault(key, []).append(i)

    # Corridors that will not yield. User-pinned paths are in from the
    # start; every group accepted below joins them.
    reserved: list[tuple[Sequence[float], Sequence[float]]] = [
        (route[i], route[i + 1])
        for route in pinned_routes
        for i in range(len(route) - 1)
    ]

    pinned: dict[int, list[tuple[float, float, float]]] = {}
    for members in sorted(groups.values(), key=lambda m: (-len(m), m[0])):
        if len(members) < 2:
            continue
        corridors = [(current_positions[i], targets[i]) for i in members]
        blocked = any(
            envelope_overlap_swept(
                list(start),
                list(end),
                list(obs),
                list(obs),
                margin=PLANNING_MARGIN,
                separation=separation,
                b_extends_below=DOWNWASH_ROUTE_CLEARANCE,
                b_extends_above=DOWNWASH_ROUTE_CLEARANCE,
            )
            for start, end in corridors
            for obs in static_positions
        ) or any(
            _pinned_corridors_deadlock(
                start, end, other_start, other_end, separation=separation
            )
            for start, end in corridors
            for other_start, other_end in reserved
        )
        if blocked:
            continue
        # Members share one displacement, so their corridors run parallel
        # and can never close on each other — only cross-group clashes
        # matter, hence reserving them after the group is accepted.
        reserved.extend(corridors)
        for i in members:
            pinned[i] = [tuple(targets[i])]
    return pinned


def _phase_fixed_routes(
    phase: dict, num_drones: int
) -> dict[int, list[tuple[float, float, float]]]:
    """Parse a validated phase's ``fixedPaths`` into solver fixed routes."""
    fixed = phase.get("fixedPaths", phase.get("fixed_paths")) or []
    routes: dict[int, list[tuple[float, float, float]]] = {}
    for entry in fixed:
        drone_id = entry.get("droneId", entry.get("id"))
        drone_index = _drone_index_from_id(drone_id)
        if drone_index is None:
            continue
        routes[drone_index] = [
            (float(wp["x"]), float(wp["y"]), float(wp["z"]))
            for wp in entry["path"]
        ]
    return routes


_DRONE_ID_STRING_PREFIXES = ("show-drone-", "drone-")


def _drone_index_from_id(drone_id: int | str) -> int | None:
    """Map a 1-based drone identifier to a 0-based index."""
    if isinstance(drone_id, int):
        return drone_id - 1
    if not isinstance(drone_id, str):
        return None
    for prefix in _DRONE_ID_STRING_PREFIXES:
        if drone_id.startswith(prefix):
            try:
                return int(drone_id.removeprefix(prefix)) - 1
            except ValueError:
                return None
    return None


def _phase_point_drone_index(
    point: dict, fallback_index: int, num_drones: int
) -> int | None:
    drone_id = point.get("droneId", point.get("id"))
    if drone_id is None:
        return fallback_index if fallback_index < num_drones else None
    index = _drone_index_from_id(drone_id)
    if index is None or not (0 <= index < num_drones):
        return None
    return index


def _yaw_from_point(point: dict, default: float = 0.0) -> float:
    yaw = point.get("yaw")
    if yaw is None:
        return default
    return float(yaw)


def _normalize_yaw_array(name: str, value: list, num_drones: int) -> list[float]:
    """Return per-drone yaw angles ordered like ``_normalize_vec3_array``."""
    has_drone_ids = any(
        isinstance(point, dict) and "droneId" in point for point in value
    )
    if not has_drone_ids:
        yaws = [0.0] * len(value)
        for point_index, point in enumerate(value):
            if isinstance(point, dict):
                yaws[point_index] = _yaw_from_point(point)
        return yaws

    normalized: list[float | None] = [None] * num_drones
    for point_index, point in enumerate(value):
        if not isinstance(point, dict):
            raise ValueError(
                f"'{name}[{point_index}]' must be an object when droneId is used"
            )
        drone_index = _phase_point_drone_index(point, point_index, num_drones)
        if drone_index is None:
            raise ValueError(
                f"'{name}[{point_index}].droneId' must match one of "
                "drone-1..drone-N or show-drone-1..show-drone-N"
            )
        if normalized[drone_index] is not None:
            raise ValueError(
                f"'{name}' contains duplicate yaw entry for drone-{drone_index + 1}"
            )
        normalized[drone_index] = _yaw_from_point(point)

    if any(yaw is None for yaw in normalized):
        raise ValueError(f"'{name}' is missing one or more drone yaw entries")
    return [float(yaw) for yaw in normalized if yaw is not None]


def _phase_target_yaws(phase: dict, num_drones: int) -> list[float]:
    yaws: list[float | None] = [None] * num_drones
    for point_index, point in enumerate(phase["points"]):
        drone_index = _phase_point_drone_index(point, point_index, num_drones)
        if drone_index is None:
            continue
        yaws[drone_index] = _yaw_from_point(point)

    if any(yaw is None for yaw in yaws):
        raise ValueError("phase is missing one or more drone yaw entries")
    return [float(yaw) for yaw in yaws if yaw is not None]


def _phase_targets(phase: dict, num_drones: int) -> list[tuple[float, float, float]]:
    targets: list[tuple[float, float, float] | None] = [None] * num_drones
    for point_index, point in enumerate(phase["points"]):
        drone_index = _phase_point_drone_index(point, point_index, num_drones)
        if drone_index is None:
            continue
        targets[drone_index] = (
            float(point["x"]),
            float(point["y"]),
            float(point["z"]),
        )

    if any(target is None for target in targets):
        raise ValueError("phase is missing one or more drone targets")
    return [target for target in targets if target is not None]


def _positions_match(
    positions: Sequence[Sequence[float]],
    targets: Sequence[Sequence[float]],
) -> bool:
    return all(
        abs(pos[axis] - target[axis]) < 1e-9
        for pos, target in zip(positions, targets)
        for axis in range(3)
    )


def _yaw_lists_match(current_yaws: list[float], target_yaws: list[float]) -> bool:
    """Return whether two per-drone yaw arrays are effectively equal."""
    return all(
        abs(yaw_delta_deg(current, target)) < 1e-9
        for current, target in zip(current_yaws, target_yaws)
    )


# ── spacing validation ───────────────────────────────────────────────────


def _find_clearance_violations(
    label: str,
    points: Sequence[Sequence[float]],
    separation: float = HARD_MIN_SEPARATION,
) -> list[dict]:
    """Pairs of points closer than the planner's inflated envelope allows."""
    pairs: list[dict] = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            a = points[i]
            b = points[j]
            if envelope_overlap(
                a, b, margin=PLANNING_MARGIN, separation=separation
            ):
                pairs.append(
                    {
                        "label": label,
                        "first": f"drone-{i + 1}",
                        "second": f"drone-{j + 1}",
                        "delta": [
                            round(abs(a[0] - b[0]), 4),
                            round(abs(a[1] - b[1]), 4),
                            round(abs(a[2] - b[2]), 4),
                        ],
                    }
                )
    return pairs


def _spacing_error_response(
    violations: list[dict], separation: float = HARD_MIN_SEPARATION
):
    if log:
        summary = "; ".join(
            f"{v['label']}: {v['first']}~{v['second']} delta={v['delta']}"
            for v in violations[:10]
        )
        log.warning(
            f"path-planner request rejected -- formation points too close "
            f"({len(violations)} violation(s)): {summary}"
        )
    return (
        jsonify(
            {
                "error": "Formation points are too close to each other",
                "code": "FORMATION_SPACING_TOO_CLOSE",
                "details": {
                    "collision_envelope": describe_collision_envelope(),
                    "required_separation": separation,
                    "required_xy_clearance": separation,
                    "hard_min_separation": HARD_MIN_SEPARATION,
                    "violations": violations,
                },
            }
        ),
        422,
    )


def _validate_point_group_spacing(
    groups: list[tuple[str, Sequence]],
    separation: float = HARD_MIN_SEPARATION,
):
    """422 response when any labelled point set violates the separation."""
    violations: list[dict] = []
    for label, points in groups:
        violations.extend(_find_clearance_violations(label, points, separation))
    if violations:
        return _spacing_error_response(violations, separation)
    return None


def _altitude_floor_error(groups: list[tuple[str, Sequence]], min_z: float):
    """422 response when any planned waypoint sits below the altitude floor.

    The solver clamps its motion to ``min_z``; a target below the floor can
    never be reached and would otherwise surface as a confusing deadlock.
    """
    EPS = 1e-3
    violations: list[dict] = []
    for label, points in groups:
        for i, pt in enumerate(points):
            z = float(pt[2])
            if z + EPS < min_z:
                violations.append({"label": label, "index": i, "z": z})
    if not violations:
        return None
    if log:
        summary = "; ".join(
            f"{v['label']}[{v['index']}] z={v['z']}" for v in violations[:10]
        )
        log.warning(
            f"path-planner request rejected -- waypoints below altitude floor "
            f"{min_z} m ({len(violations)} violation(s)): {summary}"
        )
    return (
        jsonify(
            {
                "error": (
                    f"One or more waypoints are below the altitude floor of "
                    f"{min_z} m used for planning"
                ),
                "code": "BELOW_ALTITUDE_FLOOR",
                "details": {"min_z": min_z, "violations": violations},
            }
        ),
        422,
    )


# ── staging layout ───────────────────────────────────────────────────────


def _staging_spread_scale(
    hover_positions: Sequence[Sequence[float]], spacing: float
) -> float:
    """Smallest uniform horizontal scale that spreads the fleet to ``spacing``.

    Separation is Chebyshev (see :data:`HARD_MIN_SEPARATION`), so a pair is
    already clear when *any* axis reaches ``spacing``. Pairs separated by
    altitude alone are therefore ignored, and the remaining pairs each need
    ``spacing / max(|dx|, |dy|)``. Returns ``1.0`` when the take-off layout
    is already spread out — nothing to do.

    A pair sharing one horizontal spot cannot be spread apart at all; it is
    skipped here and left to the ``staging-hover`` spacing validation, which
    reports it with a proper message.
    """
    scale = 1.0
    for a, b in combinations(hover_positions, 2):
        if abs(a[2] - b[2]) >= spacing:
            continue
        horizontal = max(abs(a[0] - b[0]), abs(a[1] - b[1]))
        if horizontal <= 0.0 or horizontal >= spacing:
            continue
        scale = max(scale, (spacing + STAGING_SPREAD_GUARD) / horizontal)
    return scale


def _staging_spread_targets(
    hover_positions: Sequence[Sequence[float]], spacing: float
) -> list[tuple[float, float, float]]:
    """Spread the take-off layout out about drone-1 until it clears ``spacing``.

    The fleet keeps the shape it took off in: every drone is pushed
    horizontally away from drone-1 (the anchor) by one uniform scale factor,
    the smallest that brings the tightest pair up to ``spacing``. Altitudes
    are left alone.

    Scaling about a fixed point only ever *increases* pairwise distances,
    and does so monotonically along each drone's straight line, so the
    staging move is collision-free by construction — no drone ever has to
    weave past another. When the take-off layout already clears ``spacing``
    the scale is 1 and the hover positions are returned unchanged, which
    makes the staging segment a no-op.
    """
    scale = _staging_spread_scale(hover_positions, spacing)
    if scale <= 1.0:
        return [tuple(float(v) for v in point) for point in hover_positions]

    anchor = hover_positions[0]
    return [
        (
            round(anchor[0] + (point[0] - anchor[0]) * scale, STAGING_ROUND_DIGITS),
            round(anchor[1] + (point[1] - anchor[1]) * scale, STAGING_ROUND_DIGITS),
            round(float(point[2]), STAGING_ROUND_DIGITS),
        )
        for point in hover_positions
    ]


# ── formation planning (runs in a worker thread) ─────────────────────────


def _segment_seed(seed: Optional[int], index: int) -> Optional[int]:
    """Distinct-but-reproducible seed per solver segment."""
    return None if seed is None else seed + index


# ── dense formation entry ────────────────────────────────────────────────
# Passing BETWEEN two parked drones needs them ENTRY_CORRIDOR_FACTOR x the
# separation apart. An image formation is never that loose, so once part of
# it is parked the rest is walled out and whoever arrives last deadlocks.
# That is an arrival ORDER problem: no amount of local detour logic fixes it,
# because the drone is not badly routed, it is late. When the target
# formation is that tight, entry is staged instead of flown in one go.
ENTRY_CORRIDOR_FACTOR = 2.0

# A formation counts as planar when its spread along one horizontal axis is
# this small (meters). An image wall is exactly flat; the tolerance only
# absorbs rounding.
PLANE_FLATNESS_TOLERANCE = 0.5

# How far off the plane the fleet assembles, in multiples of the separation.
# Two separations leaves a full corridor between the standoff copy and the
# real formation.
PLANE_STANDOFF_FACTOR = 2.0


def _min_pairwise_gap(points: Sequence[Sequence[float]]) -> float:
    """Smallest Chebyshev distance between any two of *points*."""
    best = float("inf")
    for a, b in combinations(points, 2):
        gap = max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))
        if gap < best:
            best = gap
    return best


def _plane_normal_axis(
    targets: Sequence[Sequence[float]],
    *,
    tolerance: float = PLANE_FLATNESS_TOLERANCE,
) -> Optional[int]:
    """Index of the horizontal axis the formation is flat along, else None.

    Only x and y qualify: a formation flat in z is a floor pattern, which
    the staged stack entry already owns.
    """
    if len(targets) < 3:
        return None
    spread = [
        max(t[axis] for t in targets) - min(t[axis] for t in targets)
        for axis in range(3)
    ]
    for axis in (0, 1):
        other = 1 - axis
        if (
            spread[axis] <= tolerance
            and spread[other] > tolerance
            and spread[2] > tolerance
        ):
            return axis
    return None


def _plane_entry_targets(
    targets: Sequence[Sequence[float]],
    current_positions: Sequence[Sequence[float]],
    *,
    min_separation: float,
    exempt: Optional[set[int]] = None,
) -> Optional[tuple[list[tuple[float, float, float]], int]]:
    """A copy of the formation standing off the plane, on the fleet's side.

    Returns ``(approach_targets, normal_axis)``, or ``None`` when the target
    formation is not a vertical plane.

    The point of the standoff copy is the leg that follows it: going from
    these points to *targets* is ONE shared displacement for every drone, so
    every pairwise distance is invariant along it. If the formation itself
    respects the separation then so does the whole entry leg, and no drone
    ever has to pass between two parked ones — which is the manoeuvre a
    tight formation makes impossible.
    """
    axis = _plane_normal_axis(targets)
    if axis is None:
        return None
    plane = sum(t[axis] for t in targets) / len(targets)
    fleet = sum(p[axis] for p in current_positions) / len(current_positions)
    standoff = PLANE_STANDOFF_FACTOR * min_separation
    # Stand off towards wherever the fleet already is; when it is sitting on
    # the plane (a wall-to-wall transition) either side is open, so fall back
    # to the near side deterministically.
    side = 1.0 if fleet > plane else -1.0
    exempt = exempt or set()
    approach: list[tuple[float, float, float]] = []
    for index, target in enumerate(targets):
        if index in exempt:
            approach.append(tuple(float(v) for v in target))
            continue
        point = [float(v) for v in target]
        point[axis] = point[axis] + side * standoff
        approach.append(tuple(point))
    return approach, axis


def _stack_entry_plan(
    targets: Sequence[tuple[float, float, float]],
    *,
    min_z: float,
    exempt: Optional[set[int]] = None,
    xy_tolerance: float = STACK_XY_TOLERANCE,
) -> tuple[list[tuple[float, float, float]], list[list[int]]]:
    """Split entry into a stacked formation into approach + climb waves.

    A drone needs the staged entry iff another drone's target sits
    *directly above* its own target: horizontal offsets below the tolerance
    on BOTH axes and a vertical gap of at most :data:`STACK_VERTICAL_GAP`.
    Only these true above/below pairs form the stacking relation — large
    flat formations (e.g. an image wall where every drone shares one x
    plane) must NOT be chained sideways into one giant "column" through
    lateral neighbours, which is what the previous connected-component
    analysis did.

    Drones in *exempt* (user-pinned fixed paths, drones already parked on
    their target) never receive a staged entry: their approach point stays
    their real target and they join no climb wave. They still count as
    "above" for the wave ordering, so a non-exempt drone below them stages
    normally.

    Returns ``(approach_targets, climb_waves)``. ``approach_targets`` equals
    *targets* except for stacked lower drones, which stop
    :data:`STACK_APPROACH_OFFSET` below their real target (never below
    *min_z*). ``climb_waves[k]`` lists the drone indices whose longest
    stacked-above chain has length ``k`` — drones higher in a stack always
    settle before anyone climbs underneath them.

    Raises :class:`PlanningError` when altitude clamping squashes the
    approach points of a true above/below pair together (no safe staged
    entry exists for that vertical line).
    """
    n = len(targets)
    exempt = exempt or set()
    approach: list[tuple[float, float, float]] = [tuple(t) for t in targets]

    # True vertical stacking pairs only: aboves[j] = drones directly above j.
    aboves: dict[int, list[int]] = {j: [] for j in range(n)}
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            if abs(targets[i][0] - targets[j][0]) >= xy_tolerance:
                continue
            if abs(targets[i][1] - targets[j][1]) >= xy_tolerance:
                continue
            dz = targets[i][2] - targets[j][2]
            if 0 < dz <= STACK_VERTICAL_GAP:
                aboves[j].append(i)

    # Longest stacked-above chain per drone (strict z ordering -> acyclic).
    depth_memo: dict[int, int] = {}

    def depth_of(j: int) -> int:
        cached = depth_memo.get(j)
        if cached is not None:
            return cached
        depth = 0
        for i in aboves[j]:
            depth = max(depth, depth_of(i) + 1)
        depth_memo[j] = depth
        return depth

    waves: dict[int, list[int]] = {}
    for j in range(n):
        if j in exempt or not aboves[j]:
            continue
        x, y, z = targets[j]
        approach[j] = (x, y, max(min_z, z - STACK_APPROACH_OFFSET))
        waves.setdefault(depth_of(j), []).append(j)

    # Lift cascade (bottom-up): min_z clamping can compress the approach
    # gaps of a vertical chain below the separation. Raise each staged
    # approach so it stays at least ``xy_tolerance`` (the separation) above
    # the approach of the drone directly below it — capped at the drone's
    # own target, which always suffices when targets respect the altitude
    # floor and the separation.
    for j in sorted(range(n), key=lambda k: targets[k][2]):
        for i in aboves[j]:
            if approach[i] == targets[i]:
                continue  # not staged: its target already clears by >= sep
            needed = approach[j][2] + xy_tolerance
            if approach[i][2] < needed:
                x, y, z = targets[i]
                approach[i] = (x, y, min(z, needed))

    # Collapse check per true pair: the upper drone's approach must stay
    # above the lower drone's approach on their shared vertical line even
    # after the lift cascade (only possible when a target itself violates
    # the altitude floor), otherwise the staged entry cannot be flown.
    for j in range(n):
        for i in aboves[j]:
            if approach[i][2] - approach[j][2] < xy_tolerance - 1e-6 and (
                approach[i] != targets[i] or approach[j] != targets[j]
            ):
                pair = [
                    {
                        "drone": f"drone-{k + 1}",
                        "target": [round(v, 3) for v in targets[k]],
                        "approach": [round(v, 3) for v in approach[k]],
                    }
                    for k in (i, j)
                ]
                if log:
                    log.warning(
                        "staged stack entry failed: drone-%d sits directly "
                        "above drone-%d but min_z=%s m clamps their "
                        "approach points together. pair=%s"
                        % (i + 1, j + 1, min_z, pair)
                    )
                raise PlanningError(
                    "staged stack entry is impossible: approach points for "
                    f"drones {i + 1} and {j + 1} collapse at the "
                    "minimum altitude; raise the formation or min_alt",
                    details={
                        "drones": [f"drone-{i + 1}", f"drone-{j + 1}"],
                        "pair": pair,
                        "min_z": min_z,
                        "stack_xy_tolerance": xy_tolerance,
                        "stack_approach_offset": STACK_APPROACH_OFFSET,
                    },
                )

    return approach, [waves[k] for k in sorted(waves)]


def _yaw_steps_needed(
    current_yaws: list[float],
    target_yaws: list[float],
    duration_sec: float,
    max_yaw_rate_deg_s: float,
) -> int:
    max_delta = max(
        (
            abs(yaw_delta_deg(current, target))
            for current, target in zip(current_yaws, target_yaws)
        ),
        default=0.0,
    )
    if max_delta < 1e-9:
        return 0
    return max(1, ceil((max_delta / max_yaw_rate_deg_s) / duration_sec))


def _append_yaw_transition(
    steps: list[StepRecord],
    positions: Sequence[Sequence[float]],
    current_yaws: list[float],
    target_yaws: list[float],
    *,
    duration_sec: float,
    max_yaw_rate_deg_s: float,
    min_steps: int = 0,
) -> list[float]:
    """Append in-place steps rotating to *target_yaws* within the rate limit.

    Enough steps are inserted for the largest yaw change to stay below
    ``max_yaw_rate_deg_s`` (so the converter's ramp always fits its budget);
    ``min_steps`` extends the tail as a hold at the target yaw. Returns the
    new per-drone yaw list.
    """
    yaw_steps = _yaw_steps_needed(
        current_yaws, target_yaws, duration_sec, max_yaw_rate_deg_s
    )
    total_steps = max(yaw_steps, min_steps)
    for k in range(1, total_steps + 1):
        fraction = 1.0 if yaw_steps == 0 else min(1.0, k / yaw_steps)
        yaws = [
            lerp_yaw_deg(current, target, fraction)
            for current, target in zip(current_yaws, target_yaws)
        ]
        steps.append(
            StepRecord(
                step=steps[-1].step + 1,
                positions={idx: list(pos) for idx, pos in enumerate(positions)},
                collisions=[],
                reverted_drones=[],
                verified=True,
                yaws={idx: yaws[idx] for idx in range(len(positions))},
            )
        )
    return list(target_yaws) if total_steps > 0 else list(current_yaws)


def _extend_with_solver_run(
    combined_steps: list[StepRecord],
    current_positions: list[tuple[float, float, float]],
    targets: Sequence[tuple[float, float, float]],
    *,
    step_size: float,
    seed: Optional[int],
    min_z: float,
    current_yaws: list[float],
    label: str,
    constant_speed: bool = False,
    fixed_routes: Optional[dict[int, list[tuple[float, float, float]]]] = None,
    lockstep_groups: Optional[list[set[int]]] = None,
    min_separation: float = HARD_MIN_SEPARATION,
    report_progress: bool = True,
    tentative: bool = False,
    step_duration_ms: int = 0,
) -> list[tuple[float, float, float]]:
    """Run one solver segment and append its steps to the combined timeline.

    ``constant_speed`` marks the appended steps as smoothing-exempt (used
    for staged stack-entry climbs whose speed must stay exactly constant).
    ``fixed_routes`` pins the listed drones to user-defined waypoint routes
    for this segment and ``lockstep_groups`` makes the listed blocks advance
    together or wait together (see :class:`PathSolver`).

    ``tentative`` marks a run the caller is prepared to retry with looser
    pins: a failure here is an internal step of the fallback ladder, not a
    problem the operator has to act on, so it is logged at debug level. The
    ladder's own warning (or the final re-raise) is what surfaces.

    Raises :class:`PlanningError` when the segment cannot be solved -- the
    caller never sees a partial path.
    """
    num_drones = len(current_positions)

    # 실시간 진행률: 5스텝마다 잔여 총거리로 세그먼트 내 진행도를 계산해
    # 전역 진행 상태에 반영한다 (그리디 계산이 어디까지 왔는지 노출).
    on_step = None
    if report_progress:
        target_list = [tuple(t) for t in targets]
        initial_remaining = sum(
            Drone.distance(p, t)
            for p, t in zip(current_positions, target_list)
        )
        _progress_update(
            segment=label,
            step=0,
            percent=0.0,
            remaining_m=round(initial_remaining, 1),
            initial_remaining_m=round(initial_remaining, 1),
        )
        step_counter = {"count": 0}

        def on_step(record):
            step_counter["count"] += 1
            if step_counter["count"] % 5:
                return
            remaining = sum(
                Drone.distance(record.positions[i], target_list[i])
                for i in range(num_drones)
            )
            fraction = (
                1.0 - remaining / initial_remaining
                if initial_remaining > 1e-9
                else 1.0
            )
            _progress_update(
                segment=label,
                step=record.step,
                remaining_m=round(remaining, 1),
                percent=round(max(0.0, min(1.0, fraction)) * 100.0, 1),
            )

    solver = PathSolver(
        initials=current_positions,
        targets=list(targets),
        step_size=step_size,
        seed=seed,
        on_step=on_step,
        min_z=min_z,
        fixed_routes=fixed_routes,
        lockstep_groups=lockstep_groups,
        min_separation=min_separation,
        step_duration_ms=step_duration_ms,
    )
    started_at = perf_counter()
    result = solver.solve()
    if log:
        emit = log.debug if (tentative and not result.success) else log.info
        emit(
            f"segment '{label}': {'solved' if result.success else 'FAILED'} "
            f"in {perf_counter() - started_at:.1f}s "
            f"({result.total_steps} steps, {num_drones} drones)"
        )
    if not result.success:
        details = {
            "segment": label,
            "reason": result.failure_reason,
            "stuck_drones": [f"drone-{i + 1}" for i in result.stuck_drones],
            "steps_completed": result.total_steps,
        }
        # A solver step lasts step_size / cruise_speed seconds. When that is
        # longer than the whole downwash budget the graded band buys nothing
        # -- transits that a faster show would fly are simply forbidden. The
        # operator can only fix that by changing the speed, so say so instead
        # of leaving them to read a deadlock.
        if step_duration_ms > DOWNWASH_EXPOSURE_MS:
            details["code"] = "CRUISE_SPEED_TOO_SLOW_FOR_DOWNWASH"
            details["step_duration_ms"] = step_duration_ms
            details["downwash_exposure_ms"] = DOWNWASH_EXPOSURE_MS
            details["required_cruise_speed"] = round(
                step_size / (DOWNWASH_EXPOSURE_MS / 1000.0), 4
            )
            details["step_size"] = step_size
        if result.fixed_conflicts:
            details["code"] = "FIXED_PATH_CONFLICT"
            details["fixed_conflicts"] = [
                {
                    "drone": f"drone-{c['drone'] + 1}",
                    "blockedBy": [f"drone-{b + 1}" for b in c["blocked_by"]],
                    "blockedByFixed": [
                        f"drone-{b + 1}" for b in c["fixed_blockers"]
                    ],
                }
                for c in result.fixed_conflicts
            ]
        raise PlanningError(
            f"planning failed in segment '{label}': {result.failure_reason}",
            details=details,
        )

    step_offset = combined_steps[-1].step
    for record in result.steps[1:]:
        combined_steps.append(
            StepRecord(
                step=step_offset + record.step,
                positions=deepcopy(record.positions),
                collisions=list(record.collisions),
                reverted_drones=list(record.reverted_drones),
                verified=record.verified,
                yaws={idx: current_yaws[idx] for idx in range(num_drones)},
                constant_speed=constant_speed,
            )
        )
    return [tuple(result.steps[-1].positions[idx]) for idx in range(num_drones)]


def _plan_formation_phases(
    *,
    start_positions: list,
    phases: list[dict],
    step_size: float,
    duration_ms: int,
    seed: Optional[int],
    return_to_initial: bool = True,
    min_z: float = 0.0,
    initial_yaws: list[float] | None = None,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
    staging_targets: Optional[list[tuple[float, float, float]]] = None,
    landing_targets: Optional[list[tuple[float, float, float]]] = None,
    min_separation: float = HARD_MIN_SEPARATION,
) -> tuple[SolverResult, list[dict]]:
    """Plan synced formation phases with collision avoidance between phases.

    ``start_positions`` are the hover positions right after takeoff. When
    ``staging_targets`` is given, a staging segment spreads the fleet out
    before the first phase (a no-op when the take-off layout is already
    spread out), and ``return_to_initial`` brings it back to the hover
    positions at the end (so landing descends onto the original ground
    spots). ``landing_targets``, when given, replaces those hover positions
    for the final return leg only (e.g. a wider spread computed the same way
    as ``staging_targets``), so landing touches down under wherever that
    leg ends instead of directly below the original take-off spot.

    Raises :class:`PlanningError` on any unsolvable segment.
    """
    num_drones = len(start_positions)
    min_separation = clamp_separation(min_separation)
    duration_sec = duration_ms / 1000.0
    current_positions = [
        tuple(float(v) for v in point) for point in start_positions
    ]
    original_initials = list(current_positions)
    current_yaws = list(initial_yaws or [0.0] * num_drones)
    original_yaws = list(current_yaws)
    neutral_yaws = [0.0] * num_drones

    combined_steps: list[StepRecord] = [
        StepRecord(
            step=0,
            positions={idx: list(pos) for idx, pos in enumerate(current_positions)},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={idx: current_yaws[idx] for idx in range(num_drones)},
        )
    ]
    phase_summaries: list[dict] = []
    segment_counter = 0
    # Set while the phase loop still has a looser fallback notch to drop to,
    # so a failed attempt logs as debug instead of shouting FAILED at an
    # operator who will never see the retry succeed a line later.
    tentative_run = False

    def summarize(
        name: str,
        arrival_step: int,
        hold_ms: int,
        hold_steps: int,
        notes: Sequence[str] = (),
    ) -> None:
        phase_summaries.append(
            {
                "name": name,
                "arrivalStep": arrival_step,
                "arrivalTimeMs": arrival_step * duration_ms,
                "holdMs": hold_ms,
                "holdSteps": hold_steps,
                "endStep": combined_steps[-1].step,
                "endTimeMs": combined_steps[-1].step * duration_ms,
                "success": True,
                "warnings": list(notes),
            }
        )

    def run_stage(
        targets,
        label: str,
        *,
        stage_step_size,
        constant_speed,
        fixed_routes=None,
        lockstep_groups=None,
    ) -> None:
        nonlocal current_positions, segment_counter
        if _positions_match(current_positions, targets):
            current_positions = [tuple(t) for t in targets]
            return
        current_positions = _extend_with_solver_run(
            combined_steps,
            current_positions,
            targets,
            step_size=stage_step_size,
            seed=_segment_seed(seed, segment_counter),
            min_z=min_z,
            current_yaws=current_yaws,
            label=label,
            constant_speed=constant_speed,
            fixed_routes=fixed_routes,
            lockstep_groups=lockstep_groups,
            step_duration_ms=duration_ms,
            min_separation=min_separation,
            tentative=tentative_run,
        )
        segment_counter += 1

    def run_segment(
        targets,
        label: str,
        fixed_routes=None,
        lockstep_groups=None,
        auto_cluster: bool = True,
        entry_mode: str = "direct",
    ) -> None:
        """One formation move: approach stage plus staged stack-entry climbs.

        Stacked lower drones stop :data:`STACK_APPROACH_OFFSET` below their
        target during the approach and climb the final stretch vertically at
        a constant :data:`STACK_CLIMB_SPEED`, wave by wave (top first), only
        after the drones above them have settled.

        Drones with a ``fixed_routes`` entry are exempt from the staged
        stack entry — their user-defined path *is* the entry — and fly
        straight to their real target along it during the approach stage.
        """
        # Exempt from the staged entry: drones already parked on their
        # target (no re-entry dip) and pinned drones — user fixed paths,
        # user clusters and auto-detected rigid groups (the pinned line IS
        # the entry). Must be known *before* the stack analysis — otherwise
        # an all-pinned column could still fail its collapse check.
        stationary = {
            i
            for i in range(len(targets))
            if Drone.distance(current_positions[i], targets[i]) < 1e-9
        }
        # Plane entry: fly the whole existing pipeline at a copy of the
        # formation standing off its own plane, then close the gap with one
        # shared displacement. Drones already parked on target stay put.
        entry_leg = None
        if entry_mode == "plane":
            plane_plan = _plane_entry_targets(
                targets,
                current_positions,
                min_separation=min_separation,
                exempt=stationary,
            )
            if plane_plan is not None:
                approach_copy, normal_axis = plane_plan
                entry_leg = [tuple(t) for t in targets]
                targets = approach_copy
                if log:
                    log.info(
                        f"segment '{label}': planar formation — assembling "
                        f"{PLANE_STANDOFF_FACTOR * min_separation:.1f} m off the "
                        f"{'XYZ'[normal_axis]} plane, then entering as one block"
                    )

        combined_fixed = {
            did: route
            for did, route in (fixed_routes or {}).items()
            if did not in stationary
        }

        # Automatic rigid-group clustering: formation blocks that translate
        # as a unit between the phases fly in lockstep on straight lines.
        # It is an optimisation, so a caller retrying a failed segment can
        # switch it off — otherwise it would re-pin the very block that was
        # just released and reproduce the failure.
        auto_pinned = (
            _detect_rigid_groups(
                [tuple(p) for p in current_positions],
                [tuple(t) for t in targets],
                excluded=stationary | set(combined_fixed),
                static_positions=[current_positions[i] for i in stationary],
                # A user-pinned drone never detours either, so an
                # auto-clustered corridor has to keep clear of its whole
                # route, not just of where it starts.
                pinned_routes=[
                    [tuple(current_positions[did]), *(tuple(wp) for wp in route)]
                    for did, route in combined_fixed.items()
                ],
                separation=min_separation,
            )
            if auto_cluster
            else {}
        )
        if auto_pinned:
            combined_fixed.update(auto_pinned)
            if log:
                log.info(
                    f"segment '{label}': auto-clustered rigid group(s) "
                    f"{[f'drone-{i + 1}' for i in sorted(auto_pinned)]}"
                )

        exempt = stationary | set(combined_fixed)

        try:
            approach_targets, climb_waves = _stack_entry_plan(
                targets,
                min_z=min_z,
                exempt=exempt,
                xy_tolerance=min_separation,
            )
        except PlanningError as exc:
            exc.details = {**exc.details, "segment": label}
            raise PlanningError(
                f"planning failed in segment '{label}': {exc}", details=exc.details
            ) from exc

        # Only blocks that are still moving constrain each other; a member
        # parked on its target holds nobody back.
        moving_groups = [
            group - stationary
            for group in (lockstep_groups or [])
            if len(group - stationary) > 1
        ]

        # Blocks that must vacate a spot before another block can take it
        # are dispatched in that order: each wave flies while the blocks
        # still to come hold where they are. Without it the two blocks race,
        # whoever wins parks on the other's lane, and a pinned drone that
        # can never detour is stuck behind it for good.
        # Auto-detected rigid groups are blocks too: regroup them by the
        # displacement they were detected on.
        auto_groups: dict[tuple[int, int, int], set[int]] = {}
        for index in auto_pinned:
            key = tuple(
                round((targets[index][axis] - current_positions[index][axis]) * 1000)
                for axis in range(3)
            )
            auto_groups.setdefault(key, set()).add(index)
        blocks = _pinned_blocks(
            combined_fixed, [*moving_groups, *auto_groups.values()]
        )
        waves = _dispatch_waves(
            blocks,
            current_positions=current_positions,
            targets=targets,
            separation=min_separation,
        )
        if len(waves) > 1 and log:
            log.info(
                f"segment '{label}': dispatching pinned blocks in "
                f"{len(waves)} waves "
                + " then ".join(
                    str([f"drone-{i + 1}" for block in wave for i in sorted(block)])
                    for wave in waves
                )
            )

        held_back = {index for wave in waves[1:] for block in wave for index in block}
        for wave_index, wave in enumerate(waves or [[]]):
            released = {index for block in wave for index in block}
            held_back -= released
            stage_targets = [
                tuple(current_positions[index]) if index in held_back else point
                for index, point in enumerate(approach_targets)
            ]
            run_stage(
                stage_targets,
                label if len(waves) < 2 else f"{label}/wave-{wave_index + 1}",
                stage_step_size=step_size,
                constant_speed=False,
                fixed_routes={
                    index: route
                    for index, route in combined_fixed.items()
                    if index not in held_back
                }
                or None,
                lockstep_groups=[
                    group for group in moving_groups if not (group & held_back)
                ]
                or None,
            )
        climb_step = STACK_CLIMB_SPEED * duration_sec
        for wave_index, wave in enumerate(climb_waves):
            wave_targets = list(current_positions)
            for drone_index in wave:
                wave_targets[drone_index] = tuple(targets[drone_index])
            run_stage(
                wave_targets,
                f"{label}/stack-climb-{wave_index + 1}",
                stage_step_size=climb_step,
                constant_speed=True,
            )

        # The entry leg: every drone covers the same displacement, so all
        # pairwise distances are preserved and the leg is collision-free as
        # long as the formation itself is — which validation already checked.
        if entry_leg is not None:
            run_stage(
                entry_leg,
                f"{label}/plane-entry",
                stage_step_size=step_size,
                constant_speed=False,
            )

    # ── staging: move from the hover line-up into the grid ──────────────
    if staging_targets is not None:
        run_segment(staging_targets, "staging-grid")
        summarize("staging-grid", combined_steps[-1].step, 0, 0)

    # ── requested formation phases ───────────────────────────────────────
    for phase_index, phase in enumerate(phases):
        name = str(phase.get("name", f"phase-{phase_index + 1}"))
        targets = _phase_targets(phase, num_drones)
        target_yaws = _phase_target_yaws(phase, num_drones)

        # Pinned routes for this transition: explicit fixedPaths plus every
        # member of an explicit cluster whose straight lines can be flown.
        pinned_routes = _phase_fixed_routes(phase, num_drones)
        hand_pinned = set(pinned_routes)
        cluster_groups, cluster_notes = _pin_flyable_clusters(
            pinned_routes,
            _phase_cluster_indices(phase, num_drones),
            current_positions=current_positions,
            targets=targets,
            separation=min_separation,
            label=name,
        )

        # Whether a block's straight lines are schedulable *alongside the
        # rest of the fleet* is the solver's own greedy, per-step decision —
        # geometry alone cannot predict it (a block can have a valid flight
        # order on paper that the solver never finds). So try the segment
        # with everything pinned and, if it will not solve, give up the
        # pins the *server* chose rather than fail a show whose formations
        # are all perfectly legal, loosening one notch at a time:
        #
        #   1. everything — hand-drawn paths, cluster pins, lockstep timing
        #      and automatic rigid-group clustering
        #   2. release the user's clusters (kept: hand-drawn paths + auto)
        #   3. also switch off automatic clustering — otherwise it re-pins
        #      the very block just released and reproduces the failure
        #
        # Only the drone paths the user drew by hand survive to the end, so
        # a failure at the last notch is genuinely about those and is
        # reported as such.
        hand_only = {
            index: route
            for index, route in pinned_routes.items()
            if index in hand_pinned
        }
        # Entry strategy ladder. The single-shot entry is ALWAYS tried first:
        # it is what every show has flown so far, and staging costs a visible
        # standoff-and-return excursion that a formation the solver can
        # already reach must not be made to fly.
        #
        # Staged entry is a fallback, and only worth offering when the
        # formation is genuinely too tight to fly into in one go — packed
        # closer than ENTRY_CORRIDOR_FACTOR x the separation, so no two of its
        # drones leave a corridor and the last arrivals are walled out by the
        # first. It sits right after the first attempt because it addresses
        # that failure directly, whereas the cluster-release notches below
        # loosen unrelated constraints.
        corridor = ENTRY_CORRIDOR_FACTOR * min_separation
        formation_gap = _min_pairwise_gap(targets)
        staged_fallback = formation_gap < corridor

        attempts: list[tuple[str, dict, list[set[int]] | None, bool, str]] = [
            ("direct", pinned_routes, cluster_groups or None, True, "")
        ]
        if staged_fallback:
            attempts.append(
                ("plane", pinned_routes, cluster_groups or None, True, "")
            )
        if cluster_groups:
            attempts.append(
                ("direct", hand_only, None, True, "the cluster(s) were released")
            )
        attempts.append(
            (
                "direct",
                hand_only,
                None,
                False,
                "the cluster(s) and the automatically detected rigid groups "
                "were released",
            )
        )

        for attempt, (mode, routes, groups, auto, _label) in enumerate(attempts):
            # A failed attempt appends nothing, but a segment can fail
            # *after* its approach stage, so rewind before retrying.
            checkpoint = (list(current_positions), len(combined_steps), segment_counter)
            tentative_run = attempt < len(attempts) - 1
            try:
                run_segment(
                    targets,
                    name,
                    fixed_routes=routes or None,
                    lockstep_groups=groups,
                    auto_cluster=auto,
                    entry_mode=mode,
                )
                break
            except PlanningError as exc:
                if attempt == len(attempts) - 1:
                    raise
                # Describe the notch we are about to drop to, not the one
                # that just failed. The solver's own reason goes to debug:
                # it names a conflict that the next notch is about to
                # dissolve, so repeating it as a warning only reads as an
                # unresolved error.
                next_mode, next_label = (
                    attempts[attempt + 1][0],
                    attempts[attempt + 1][4],
                )
                if next_label:
                    note = (
                        f"{next_label}: the solver could not schedule their "
                        "straight lines alongside the rest of the fleet, so "
                        "those drones were planned normally and still reach "
                        "the same formation"
                    )
                else:
                    note = (
                        f"retrying with the '{next_mode}' entry: the '{mode}' "
                        "entry could not be scheduled"
                    )
                cluster_notes.append(note)
                if log:
                    log.warning(f"segment '{name}': {note}")
                    log.debug(f"segment '{name}': attempt {attempt + 1} failed: {exc}")
                current_positions, checkpoint_steps, segment_counter = checkpoint
                del combined_steps[checkpoint_steps:]
        tentative_run = False
        arrival_step = combined_steps[-1].step

        hold_ms = int(phase.get("holdMs", 0))
        hold_steps = ceil(hold_ms / duration_ms) if hold_ms > 0 else 0
        current_yaws = _append_yaw_transition(
            combined_steps,
            current_positions,
            current_yaws,
            target_yaws,
            duration_sec=duration_sec,
            max_yaw_rate_deg_s=max_yaw_rate_deg_s,
            min_steps=hold_steps,
        )

        # Reset yaw to neutral before the next translation (if any actually
        # moves the fleet), so all cruising happens at a known heading.
        if phase_index < len(phases) - 1:
            next_targets = _phase_targets(phases[phase_index + 1], num_drones)
            moves_next = not _positions_match(current_positions, next_targets)
        else:
            moves_next = return_to_initial and not _positions_match(
                current_positions, original_initials
            )
        if moves_next and not _yaw_lists_match(current_yaws, neutral_yaws):
            current_yaws = _append_yaw_transition(
                combined_steps,
                current_positions,
                current_yaws,
                neutral_yaws,
                duration_sec=duration_sec,
                max_yaw_rate_deg_s=max_yaw_rate_deg_s,
            )

        summarize(name, arrival_step, hold_ms, hold_steps, cluster_notes)

    # ── return to the staging hover positions ────────────────────────────
    # landing_targets, when given, spreads the final hover leg out (same
    # solver as the take-off staging spread) instead of returning to the
    # original, possibly tight, take-off hover spots.
    return_targets = (
        [tuple(t) for t in landing_targets]
        if landing_targets is not None
        else original_initials
    )
    final_targets = (
        return_targets
        if return_to_initial
        else [tuple(t) for t in _phase_targets(phases[-1], num_drones)]
    )
    if return_to_initial:
        run_segment(return_targets, "return-to-start")
        arrival_step = combined_steps[-1].step
        if not _yaw_lists_match(current_yaws, original_yaws):
            current_yaws = _append_yaw_transition(
                combined_steps,
                current_positions,
                current_yaws,
                original_yaws,
                duration_sec=duration_sec,
                max_yaw_rate_deg_s=max_yaw_rate_deg_s,
            )
        summarize("return-to-start", arrival_step, 0, 0)

    drones = [
        Drone(
            drone_id=idx,
            initial=tuple(original_initials[idx]),
            target=tuple(final_targets[idx]),
        )
        for idx in range(num_drones)
    ]
    for idx, drone in enumerate(drones):
        drone.position = list(current_positions[idx])
        drone.arrived = True

    return (
        SolverResult(
            steps=combined_steps,
            total_steps=combined_steps[-1].step,
            drones=drones,
            success=True,
        ),
        phase_summaries,
    )


# ── shared request helpers ───────────────────────────────────────────────


async def _fetch_uav_params_for_validation(validation_payload: dict) -> dict:
    """Fetch validator-declared firmware params from the first UAV."""
    uav_params: dict = {}
    param_names = collect_required_params()
    if param_names and app is not None:
        try:
            from flockwave.server.model.uav import UAV

            uav_ids = sorted(app.object_registry.ids_by_type(UAV))
            if uav_ids:
                first_uav = app.object_registry.find_by_id(uav_ids[0])
                uav_params = await fetch_required_params(
                    first_uav, param_names, log=log
                )
                validation_payload["param_source_uav"] = uav_ids[0]
        except Exception as exc:
            if log:
                log.warning(f"Could not fetch UAV parameters for validation: {exc}")
            validation_payload["param_fetch_error"] = str(exc)
    return uav_params


def _resolve_output_dir(requested: Optional[str]):
    """Resolve the output directory, refusing paths outside the base dir.

    Returns ``(path, None)`` or ``(None, error_response)``.
    """
    base = Path(output_base_dir) if output_base_dir else Path.cwd().parent
    if requested is None or requested == "":
        return base, None
    candidate = Path(requested)
    if candidate.is_absolute() or ".." in candidate.parts:
        return None, (
            jsonify(
                {
                    "error": (
                        "'output_dir' must be a relative subdirectory (no "
                        "absolute paths or '..'); files are stored under the "
                        "server's configured output directory"
                    )
                }
            ),
            400,
        )
    return base / candidate, None


def _verification_failure_response(violations: list[dict], validation_payload: dict):
    return (
        jsonify(
            {
                "error": (
                    "Trajectory verification failed: drones violate the "
                    "collision envelope in the generated show"
                ),
                "code": "VERIFICATION_FAILED",
                "details": {"violations": violations},
                "validation": validation_payload,
            }
        ),
        422,
    )


def _planning_failure_response(exc: PlanningError, validation_payload: dict):
    return (
        jsonify(
            {
                "error": str(exc),
                "code": "PLANNING_FAILED",
                "details": exc.details,
                "validation": validation_payload,
            }
        ),
        422,
    )


def _limit_failure_response(exc: Exception, validation_payload: dict):
    return (
        jsonify(
            {
                "error": str(exc),
                "code": "TRAJECTORY_LIMIT_EXCEEDED",
                "validation": validation_payload,
            }
        ),
        422,
    )


def _build_and_verify_shows(
    build_fn, smoothing: float, separation: float = HARD_MIN_SEPARATION
):
    """Build show dicts and run the final verification gate (sync, threaded).

    ``build_fn(smoothing)`` must return the show dicts. When the smoothed
    show fails verification the build is retried without smoothing; if even
    that fails, the violations are returned so the caller can abort.

    Returns ``(show_dicts, violations, applied_smoothing)``.
    """
    show_dicts = build_fn(smoothing)
    violations = verify_show_dicts(show_dicts, separation=separation)
    if not violations:
        return show_dicts, [], smoothing
    if smoothing > 0.0:
        fallback_dicts = build_fn(0.0)
        fallback_violations = verify_show_dicts(fallback_dicts, separation=separation)
        if not fallback_violations:
            return fallback_dicts, [], 0.0
        return fallback_dicts, fallback_violations, 0.0
    return show_dicts, violations, smoothing


# ── Path delivery (pre-built per-drone paths) ────────────────────────────


def _validate_delivery_drones(drones):
    """Validate the ``drones`` payload of a path-delivery request."""
    if not isinstance(drones, list) or len(drones) == 0:
        return jsonify({"error": "'drones' must be a non-empty array"}), 400

    for i, d in enumerate(drones):
        if not isinstance(d, dict):
            return jsonify({"error": f"'drones[{i}]' must be an object"}), 400
        if not _is_vec3(d.get("initial_position")):
            return (
                jsonify(
                    {"error": f"'drones[{i}].initial_position' must be [x, y, z]"}
                ),
                400,
            )
        path = d.get("path")
        if not isinstance(path, list) or len(path) == 0:
            return (
                jsonify({"error": f"'drones[{i}].path' must be a non-empty array"}),
                400,
            )
        for j, p in enumerate(path):
            if not isinstance(p, dict) or not all(
                isinstance(p.get(k), (int, float)) for k in ("x", "y", "z")
            ):
                return (
                    jsonify(
                        {
                            "error": (
                                f"'drones[{i}].path[{j}]' must have numeric x, y, z"
                            )
                        }
                    ),
                    400,
                )
    return None


async def _handle_path_delivery(body: dict):
    """Turn a pre-built per-drone ``drones`` payload into a show.

    The paths are already decided by the caller (the 3D view's "path
    delivery"); this re-times them, applies velocity smoothing, wraps them
    with takeoff/landing segments and packages the result — but only after
    they pass the same validators and the same spatio-temporal verification
    gate as generated shows.
    """
    drones = body.get("drones")
    err = _validate_delivery_drones(drones)
    if err is not None:
        return err

    smoothing = float(body.get("velocity_smoothing", velocity_smoothing))
    if not (0.0 <= smoothing <= 1.0):
        return jsonify({"error": "'velocity_smoothing' must be between 0 and 1"}), 400
    profile_exp, profile_log, velocity_profile, profile_err = _parse_profile_knobs(
        body, smoothing
    )
    if profile_err is not None:
        return profile_err
    # Delivery paths honour the same separation floor as generated plans;
    # requests may raise it, never lower it (silently clamped up here since
    # the delivery UI has no separation field yet).
    min_separation = clamp_separation(
        body.get("min_separation", body.get("min_spacing", HARD_MIN_SEPARATION))
    )
    takeoff_speed = float(body.get("takeoff_speed", DEFAULT_TAKEOFF_SPEED_M_S))
    landing_speed = float(body.get("landing_speed", DEFAULT_LANDING_SPEED_M_S))
    if takeoff_speed <= 0 or landing_speed <= 0:
        return (
            jsonify({"error": "'takeoff_speed' and 'landing_speed' must be > 0"}),
            400,
        )

    takeoff_time = float(body.get("takeoff_time", 0.0))
    takeoff_time_adjusted = takeoff_time < MIN_TAKEOFF_TIME
    if takeoff_time_adjusted:
        takeoff_time = MIN_TAKEOFF_TIME

    # Normalize initial_position to a plain [x, y, z] list (dict form accepted).
    normalized: list[dict] = []
    for d in drones:
        ip = d["initial_position"]
        if isinstance(ip, dict):
            ip = [ip["x"], ip["y"], ip["z"]]
        normalized.append(
            {
                "id": d.get("id"),
                "initial_position": [float(ip[0]), float(ip[1]), float(ip[2])],
                "ground_z": float(d.get("ground_z", 0.0)),
                "path": d["path"],
            }
        )

    # Pre-flight validators (same pipeline as generated plans); the min-alt
    # validator scans drones[].path via the request body. Airborne initial
    # positions participate as "initial".
    skip_validation = bool(body.get("skip_validation", False))
    validation_payload: dict = {"skipped": skip_validation, "issues": []}
    if not skip_validation:
        uav_params = await _fetch_uav_params_for_validation(validation_payload)
        ctx = ValidationContext(
            initial=[
                d["initial_position"]
                for d in normalized
                if d["initial_position"][2] > d["ground_z"]
            ],
            target=[],
            step_size=0.0,
            duration_ms=0,
            takeoff_time=takeoff_time,
            uav_params=uav_params,
            body=body,
        )
        issues = run_validators(ctx)
        validation_payload["issues"] = [i.to_dict() for i in issues]
        validation_payload["params"] = dict(uav_params)
        blocking = [i for i in issues if i.severity == SEVERITY_ERROR]
        if blocking:
            if log:
                log.warning(
                    "path-planner request rejected -- validation failed: "
                    + "; ".join(str(i.message) for i in blocking[:10])
                )
            return (
                jsonify(
                    {
                        "error": "Path validation failed",
                        "code": "VALIDATION_FAILED",
                        "validation": validation_payload,
                    }
                ),
                422,
            )

    coordinate_system = body.get("coordinate_system") or None
    amsl_reference = body.get("amsl_reference")
    if coordinate_system is None:
        coordinate_system = _derive_coordinate_system_from_first_uav()
    if amsl_reference is None:
        amsl_reference = _derive_amsl_reference_from_first_uav()

    def build(smoothing_value: float):
        return build_delivery_show_dicts(
            normalized,
            takeoff_time=takeoff_time,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
            velocity_smoothing=smoothing_value,
            takeoff_speed=takeoff_speed,
            landing_speed=landing_speed,
            geofence=body.get("geofence"),
            profile_exp=profile_exp,
            profile_log=profile_log,
            profile=velocity_profile,
        )

    try:
        show_dicts, violations, applied_smoothing = await to_thread.run_sync(
            lambda: _build_and_verify_shows(build, smoothing, min_separation)
        )
    except TrajectoryLimitError as exc:
        return _limit_failure_response(exc, validation_payload)

    if violations:
        return _verification_failure_response(violations, validation_payload)

    output: dict = {
        "success": True,
        "mode": "path_delivery",
        "num_drones": len(show_dicts),
        "validation": validation_payload,
        "verification": {"checked": True, "violations": 0},
        "smoothing": {
            "requested": smoothing,
            "applied": applied_smoothing,
            "profile_exp": profile_exp,
            "profile_log": profile_log,
            "profile": _profile_report(velocity_profile),
        },
    }
    if takeoff_time_adjusted:
        output["adjustments"] = {"takeoff_time": takeoff_time}

    # Delivery does not upload by default (the UI's action is a .skyc
    # download); honour an explicit auto_upload once verification has passed.
    if bool(body.get("auto_upload", False)):
        output["upload"] = await _upload_show_dicts(
            show_dicts,
            explicit_uav_ids=body.get("uav_ids"),
            origin_available=coordinate_system is not None,
        )

    output_type = str(body.get("output", "skyc")).lower()
    if output_type not in ("path", "show", "skyc"):
        return (
            jsonify({"error": "'output' must be one of 'path', 'show', 'skyc'"}),
            400,
        )

    download = bool(body.get("download", output_type == "skyc"))
    if output_type == "skyc" and download:
        response = Response(
            skyc_bytes_from_show_dicts(show_dicts),
            mimetype="application/zip",
        )
        response.headers["Content-Disposition"] = (
            'attachment; filename="path-planner.skyc"'
        )
        return response

    if output_type in ("show", "skyc"):
        output["format"] = "show-upload-v1"
        output["shows"] = show_dicts

    return jsonify(output)


# ── REST endpoint ────────────────────────────────────────────────────────


@blueprint.route("/progress", methods=["GET"])
async def planning_progress():
    """Live progress of the current planning job.

    The frontend polls this while a plan request is in flight to show how
    far the greedy computation has gotten: current segment, step count,
    remaining fleet distance, per-segment percent and an ETA extrapolated
    from the segment's progress rate.
    """
    snapshot = dict(_progress_state)
    now = wall_time()
    started_at = snapshot.get("started_at")
    if started_at:
        snapshot["elapsed_sec"] = round(now - started_at, 1)
    segment_started_at = snapshot.get("segment_started_at")
    percent = snapshot.get("percent")
    if (
        segment_started_at
        and isinstance(percent, (int, float))
        and percent > 3.0
    ):
        segment_elapsed = now - segment_started_at
        snapshot["segment_elapsed_sec"] = round(segment_elapsed, 1)
        snapshot["segment_eta_sec"] = round(
            segment_elapsed * (100.0 - percent) / percent, 1
        )
    return jsonify(snapshot)


@blueprint.route("/plan", methods=["POST"])
async def plan():
    """Run the path-planning algorithm and return per-drone paths."""
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

    # Debugging aid: dump the incoming request verbatim so planning failures
    # can be traced back to the exact data the client sent.
    if log:
        try:
            body_json = dumps(body, ensure_ascii=False)
        except (TypeError, ValueError):
            body_json = repr(body)
        if len(body_json) > 30000:
            body_json = (
                body_json[:30000] + f"... (truncated, total {len(body_json)} chars)"
            )
        log.info(f"path-planner request body: {body_json}")

    # Path *delivery* mode: the caller supplies ready-made per-drone paths
    # (the 3D view's "path delivery") instead of initial/target/phases.
    if body.get("drones") is not None:
        return await _handle_path_delivery(body)

    # --- validate required fields ---
    initial = body.get("initial")
    phases = body.get("phases")
    target = body.get("target")
    uses_phases = phases is not None

    initial_error = _validate_vec3_array("initial", initial)
    if initial_error is not None:
        return initial_error
    try:
        initial = _normalize_vec3_array("initial", initial)
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400

    if uses_phases:
        phases_error = _validate_phases(phases, num_drones=len(initial))
        if phases_error is not None:
            return phases_error
        target = [list(point) for point in _phase_targets(phases[-1], len(initial))]
    else:
        target_error = _validate_vec3_array("target", target)
        if target_error is not None:
            return target_error
        try:
            target = _normalize_vec3_array("target", target)
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

        if len(initial) != len(target):
            return (
                jsonify(
                    {
                        "error": (
                            f"'initial' ({len(initial)}) and 'target' ({len(target)}) "
                            "must have the same length"
                        )
                    }
                ),
                400,
            )

    # --- optional parameters ---
    step_size: float = float(body.get("step_size", 1.0))
    cruise_speed: float = float(
        body.get("cruise_speed", body.get("formation_speed", DEFAULT_CRUISE_SPEED_M_S))
    )
    takeoff_speed: float = float(body.get("takeoff_speed", DEFAULT_TAKEOFF_SPEED_M_S))
    landing_speed: float = float(body.get("landing_speed", DEFAULT_LANDING_SPEED_M_S))
    seed: Optional[int] = body.get("seed")
    max_yaw_rate_deg_s: float = float(
        body.get("max_yaw_rate_deg_s", DEFAULT_MAX_YAW_RATE_DEG_S)
    )
    smoothing: float = float(body.get("velocity_smoothing", velocity_smoothing))

    if step_size <= 0:
        return jsonify({"error": "'step_size' must be > 0"}), 400
    if cruise_speed <= 0:
        return jsonify({"error": "'cruise_speed' must be > 0"}), 400
    if takeoff_speed <= 0 or landing_speed <= 0:
        return (
            jsonify({"error": "'takeoff_speed' and 'landing_speed' must be > 0"}),
            400,
        )
    # Time per solver step. When omitted, derive it from step_size and the
    # target cruise speed (default 1 m/s).
    if "duration_ms" in body:
        duration_ms: int = int(body["duration_ms"])
    else:
        duration_ms = duration_ms_for_cruise_speed(step_size, cruise_speed)
    if duration_ms <= 0:
        return jsonify({"error": "'duration_ms' must be > 0"}), 400
    if max_yaw_rate_deg_s <= 0:
        return jsonify({"error": "'max_yaw_rate_deg_s' must be > 0"}), 400
    if not (0.0 <= smoothing <= 1.0):
        return jsonify({"error": "'velocity_smoothing' must be between 0 and 1"}), 400
    profile_exp, profile_log, velocity_profile, profile_err = _parse_profile_knobs(
        body, smoothing
    )
    if profile_err is not None:
        return profile_err

    # Minimum inter-drone separation (per-axis / Chebyshev). Adjustable per
    # request but NEVER below the hard floor — requests trying to lower it
    # are rejected loudly instead of being clamped silently.
    raw_separation = body.get("min_separation", body.get("min_spacing"))
    if raw_separation is None:
        min_separation = HARD_MIN_SEPARATION
    else:
        try:
            min_separation = float(raw_separation)
        except (TypeError, ValueError):
            return jsonify({"error": "'min_separation' must be a number"}), 400
        if min_separation < HARD_MIN_SEPARATION:
            return (
                jsonify(
                    {
                        "error": (
                            f"'min_separation' of {min_separation} m is below "
                            f"the hard operational floor of "
                            f"{HARD_MIN_SEPARATION} m and cannot be lowered"
                        ),
                        "code": "SEPARATION_BELOW_HARD_FLOOR",
                        "hard_min_separation": HARD_MIN_SEPARATION,
                    }
                ),
                400,
            )
    min_separation = clamp_separation(min_separation)

    # Staging parameters: hover altitude above each drone's ground position
    # and the spacing the take-off layout is spread out to.
    # ``initial_altitude`` is accepted as a legacy alias for the staging
    # altitude.
    staging_grid: bool = bool(body.get("staging_grid", uses_phases))
    staging_altitude: float = float(
        body.get(
            "staging_altitude",
            body.get("initial_altitude", body.get("takeoff_altitude", DEFAULT_STAGING_ALTITUDE)),
        )
    )
    grid_spacing: float = float(body.get("grid_spacing", DEFAULT_GRID_SPACING))
    if uses_phases and staging_altitude <= 0:
        return jsonify({"error": "'staging_altitude' must be > 0"}), 400
    if staging_grid and not uses_phases:
        return (
            jsonify(
                {
                    "error": (
                        "'staging_grid' is only supported together with "
                        "'phases'; point-to-point requests treat 'initial' "
                        "as in-air positions"
                    )
                }
            ),
            400,
        )
    if grid_spacing < min_separation:
        return (
            jsonify(
                {
                    "error": (
                        f"'grid_spacing' of {grid_spacing} m is below the "
                        f"required minimum separation of "
                        f"{min_separation:.2f} m"
                    )
                }
            ),
            400,
        )

    # Landing parameters: opt-in wider spread for the final return-to-start
    # leg, independent of the take-off staging spacing above. Off by default
    # so existing callers keep landing on their original take-off spot.
    landing_grid: bool = bool(body.get("landing_grid", False))
    landing_spacing: float = float(
        body.get("landing_spacing", DEFAULT_LANDING_SPACING)
    )
    if landing_grid and not uses_phases:
        return (
            jsonify(
                {
                    "error": (
                        "'landing_grid' is only supported together with "
                        "'phases'; point-to-point requests have no "
                        "return-to-start leg"
                    )
                }
            ),
            400,
        )
    if landing_grid and landing_spacing < min_separation:
        return (
            jsonify(
                {
                    "error": (
                        f"'landing_spacing' of {landing_spacing} m is below "
                        f"the required minimum separation of "
                        f"{min_separation:.2f} m"
                    )
                }
            ),
            400,
        )

    # --- takeoff time & output directory ---
    takeoff_time: float = float(body.get("takeoff_time", 0.0))
    takeoff_time_adjusted = takeoff_time < MIN_TAKEOFF_TIME
    if takeoff_time_adjusted:
        takeoff_time = MIN_TAKEOFF_TIME

    output_dir, output_dir_error = _resolve_output_dir(body.get("output_dir"))
    if output_dir_error is not None:
        return output_dir_error

    auto_upload: bool = body.get("auto_upload", True)

    coordinate_system: Optional[dict] = body.get("coordinate_system", None)
    amsl_reference: Optional[float] = body.get("amsl_reference", None)
    # Resolve the coordinate system *now* so that the on-disk files and the
    # MAVFTP upload share the same origin.
    if coordinate_system is None:
        coordinate_system = _derive_coordinate_system_from_first_uav()
    if amsl_reference is None:
        amsl_reference = _derive_amsl_reference_from_first_uav()

    # --- staging geometry -------------------------------------------------
    if uses_phases:
        ground_positions = [list(point) for point in initial]
        hover_positions = [
            [point[0], point[1], point[2] + staging_altitude]
            for point in ground_positions
        ]
        if staging_grid:
            staging_targets = _staging_spread_targets(hover_positions, grid_spacing)
            if log:
                if _positions_match(hover_positions, staging_targets):
                    log.info(
                        "staging: take-off layout already clears "
                        f"{grid_spacing} m -- keeping it as flown"
                    )
                else:
                    log.info(
                        f"staging: spreading the take-off layout to {grid_spacing} m "
                        "about drone-1"
                    )
        else:
            staging_targets = None
        if landing_grid:
            landing_targets = _staging_spread_targets(hover_positions, landing_spacing)
            if log:
                if _positions_match(hover_positions, landing_targets):
                    log.info(
                        "landing: take-off layout already clears "
                        f"{landing_spacing} m -- returning to the original spot"
                    )
                else:
                    log.info(
                        f"landing: spreading the return leg to {landing_spacing} m "
                        "about drone-1"
                    )
        else:
            landing_targets = None
        planning_start = hover_positions
    else:
        ground_positions = None
        hover_positions = []
        staging_targets = None
        landing_targets = None
        planning_start = initial

    # --- pre-flight validation -------------------------------------------
    skip_validation: bool = bool(body.get("skip_validation", False))
    validation_payload: dict = {"skipped": skip_validation, "issues": []}
    uav_params: dict = {}
    if not skip_validation:
        uav_params = await _fetch_uav_params_for_validation(validation_payload)

        ctx = ValidationContext(
            initial=planning_start,
            target=target,
            step_size=step_size,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
            uav_params=uav_params,
            body=body,
        )
        issues = run_validators(ctx)
        validation_payload["issues"] = [i.to_dict() for i in issues]
        validation_payload["params"] = dict(uav_params)

        blocking = [i for i in issues if i.severity == SEVERITY_ERROR]
        if blocking:
            if log:
                log.warning(
                    "path-planner request rejected -- validation failed: "
                    + "; ".join(str(i.message) for i in blocking[:10])
                )
            return (
                jsonify(
                    {
                        "error": "Path validation failed",
                        "code": "VALIDATION_FAILED",
                        "validation": validation_payload,
                    }
                ),
                422,
            )

    # Altitude floor for the solver: the firmware's minimum show altitude
    # (or its fallback). Detours and every planned waypoint stay above it.
    min_z, min_z_source = resolve_min_alt(uav_params)
    validation_payload["altitude_floor"] = {"min_z": min_z, "source": min_z_source}
    if log:
        log.info(
            f"path-planner altitude floor: min_z={min_z} m (source={min_z_source})"
        )
    if uses_phases and staging_altitude + 1e-9 < min_z:
        return (
            jsonify(
                {
                    "error": (
                        f"'staging_altitude' of {staging_altitude} m is below "
                        f"the minimum flight altitude of {min_z} m"
                    ),
                    "code": "BELOW_ALTITUDE_FLOOR",
                    "validation": validation_payload,
                }
            ),
            422,
        )

    # --- spacing & altitude feasibility (both modes) ----------------------
    spacing_groups: list[tuple[str, Sequence]] = []
    floor_groups: list[tuple[str, Sequence]] = []
    if uses_phases:
        # Initial ground positions must respect the separation too — the
        # drones sit there together before takeoff.
        spacing_groups.append(("initial-ground", ground_positions))
        spacing_groups.append(("staging-hover", hover_positions))
        floor_groups.append(("staging-hover", hover_positions))
        if staging_targets is not None:
            spacing_groups.append(("staging-grid", staging_targets))
        if landing_targets is not None:
            spacing_groups.append(("landing-grid", landing_targets))
            floor_groups.append(("landing-grid", landing_targets))
        for phase_index, phase in enumerate(phases):
            targets = _phase_targets(phase, len(initial))
            label = f"phases[{phase_index}]:{phase.get('name') or f'phase-{phase_index + 1}'}"
            spacing_groups.append((label, targets))
            floor_groups.append((label, targets))
            fixed_routes = _phase_fixed_routes(phase, len(initial))
            if fixed_routes:
                floor_groups.append(
                    (
                        f"{label}:fixedPaths",
                        [wp for route in fixed_routes.values() for wp in route],
                    )
                )
    else:
        spacing_groups.append(("initial", initial))
        spacing_groups.append(("target", target))
        floor_groups.append(("initial", initial))
        floor_groups.append(("target", target))

    spacing_error = _validate_point_group_spacing(spacing_groups, min_separation)
    if spacing_error is not None:
        return spacing_error
    floor_error = _altitude_floor_error(floor_groups, min_z)
    if floor_error is not None:
        return floor_error

    # --- yaw setup ---------------------------------------------------------
    initial_yaws: list[float] | None = None
    if uses_phases:
        try:
            initial_yaws = _normalize_yaw_array(
                "initial", body.get("initial", []), len(initial)
            )
        except ValueError as exc:
            return jsonify({"error": str(exc)}), 400

    # --- run the solver in a worker thread ---------------------------------
    def run_planning():
        if uses_phases:
            return _plan_formation_phases(
                start_positions=planning_start,
                phases=phases,
                step_size=step_size,
                duration_ms=duration_ms,
                seed=seed,
                return_to_initial=bool(body.get("return_to_initial", True)),
                min_z=min_z,
                initial_yaws=initial_yaws,
                max_yaw_rate_deg_s=max_yaw_rate_deg_s,
                staging_targets=staging_targets,
                landing_targets=landing_targets,
                min_separation=min_separation,
            )
        solver = PathSolver(
            initials=[tuple(p) for p in initial],
            targets=[tuple(p) for p in target],
            step_size=step_size,
            seed=seed,
            min_z=min_z,
            min_separation=min_separation,
            step_duration_ms=duration_ms,
        )
        result = solver.solve()
        if not result.success:
            raise PlanningError(
                f"planning failed: {result.failure_reason}",
                details={
                    "reason": result.failure_reason,
                    "stuck_drones": [
                        f"drone-{i + 1}" for i in result.stuck_drones
                    ],
                    "steps_completed": result.total_steps,
                },
            )
        return result, []

    _progress_state.clear()
    _progress_state.update(
        {
            "active": True,
            "started_at": wall_time(),
            "num_drones": len(initial),
            "mode": "formation_phases" if uses_phases else "point_to_point",
            "phases_total": len(phases) if uses_phases else 1,
            "segment": None,
            "segment_index": 0,
            "finished": None,
        }
    )

    planning_started_at = perf_counter()
    try:
        result, phase_summaries = await to_thread.run_sync(run_planning)
    except PlanningError as exc:
        _progress_update(active=False, finished="failure", message=str(exc))
        if log:
            log.warning(f"Path planning failed: {exc}")
        return _planning_failure_response(exc, validation_payload)
    planning_sec = round(perf_counter() - planning_started_at, 2)
    if log:
        log.info(
            f"path-planner: solved {len(initial)} drones in {planning_sec}s "
            f"({result.total_steps} steps total)"
        )

    # --- build the shows once, verify, and only then save/upload -----------
    def build(smoothing_value: float):
        return build_show_dicts(
            result,
            duration_ms,
            takeoff_time,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
            max_yaw_rate_deg_s=max_yaw_rate_deg_s,
            velocity_smoothing=smoothing_value,
            takeoff_speed=takeoff_speed,
            landing_speed=landing_speed,
            ground_positions=ground_positions,
            geofence=body.get("geofence"),
            profile_exp=profile_exp,
            profile_log=profile_log,
            profile=velocity_profile,
        )

    _progress_update(segment="build+verify", percent=None, step=None)
    verify_started_at = perf_counter()
    try:
        show_dicts, violations, applied_smoothing = await to_thread.run_sync(
            lambda: _build_and_verify_shows(build, smoothing, min_separation)
        )
    except TrajectoryLimitError as exc:
        _progress_update(active=False, finished="failure", message=str(exc))
        if log:
            log.warning(f"Trajectory limit exceeded: {exc}")
        return _limit_failure_response(exc, validation_payload)
    build_verify_sec = round(perf_counter() - verify_started_at, 2)
    if log:
        log.info(f"path-planner: build+verify took {build_verify_sec}s")

    if violations:
        _progress_update(
            active=False, finished="failure", message="verification rejected"
        )
        if log:
            log.error(
                f"Verification gate rejected the generated show "
                f"({len(violations)} violation(s))"
            )
        return _verification_failure_response(violations, validation_payload)
    _progress_update(active=False, finished="success")

    # --- response scaffolding ----------------------------------------------
    output = build_output(result, duration_ms)
    output["success"] = True
    output["total_steps"] = result.total_steps
    output["validation"] = validation_payload
    output["verification"] = {"checked": True, "violations": 0}
    output["spacing"] = {
        "min_separation": min_separation,
        "hard_min_separation": HARD_MIN_SEPARATION,
        "semantics": "per-axis (Chebyshev)",
    }
    output["smoothing"] = {
        "requested": smoothing,
        "applied": applied_smoothing,
        "profile_exp": profile_exp,
        "profile_log": profile_log,
        "profile": _profile_report(velocity_profile),
    }
    output["timing"] = {
        "duration_ms": duration_ms,
        "cruise_speed": cruise_speed,
        "takeoff_speed": takeoff_speed,
        "landing_speed": landing_speed,
        "planning_sec": planning_sec,
        "build_verify_sec": build_verify_sec,
    }
    if takeoff_time_adjusted:
        output["adjustments"] = {"takeoff_time": takeoff_time}
    if uses_phases:
        # Absolute show-timeline seconds: solver step s happens at
        # takeoff_time (ground wait) + takeoff climb duration + s×step time.
        # Every drone climbs exactly ``staging_altitude`` during takeoff, so
        # the offset is fleet-wide. Lets clients (e.g. the LED timeline)
        # place phase markers without re-deriving the takeoff profile.
        takeoff_duration_sec = vertical_transit_duration_sec(
            staging_altitude, takeoff_speed, applied_smoothing,
            profile_exp, profile_log, profile=velocity_profile,
        )
        show_offset_sec = round(takeoff_time + takeoff_duration_sec, 4)
        for summary in phase_summaries:
            summary["arrivalTimeAbsSec"] = round(
                show_offset_sec + summary["arrivalTimeMs"] / 1000.0, 3
            )
            summary["endTimeAbsSec"] = round(
                show_offset_sec + summary["endTimeMs"] / 1000.0, 3
            )
        output["timing"]["takeoff_time_sec"] = takeoff_time
        output["timing"]["takeoff_duration_sec"] = takeoff_duration_sec
        output["timing"]["show_time_offset_sec"] = show_offset_sec
        output["mode"] = "formation_phases"
        output["phases"] = phase_summaries
        output["ground_initial"] = ground_positions
        output["staging"] = {
            "enabled": staging_targets is not None,
            "altitude": staging_altitude,
            "grid_spacing": grid_spacing,
            "hover_positions": hover_positions,
            # Legacy key name: these are the spread-out staging positions,
            # which equal ``hover_positions`` when no spreading was needed.
            "grid_slots": [list(slot) for slot in (staging_targets or [])],
        }
        output["landing"] = {
            "enabled": landing_targets is not None,
            "spacing": landing_spacing,
            "grid_slots": [list(slot) for slot in (landing_targets or [])],
        }

    # --- save Skybrush files (post-verification only) -----------------------
    try:
        saved = await save_skyb_files(show_dicts, output_dir=output_dir)
        output["skybrush_files"] = saved
        if log:
            log.info(
                f"Saved {sum(1 for k in saved if not k.startswith('_'))} "
                f".skyb file(s) and show.json under {output_dir}"
            )
    except Exception as exc:
        output["skybrush_files_error"] = str(exc)
        if log:
            log.error(f"Failed to save .skyb files to {output_dir}: {exc}")

    # --- auto-upload to connected UAVs (post-verification only) -------------
    if auto_upload:
        output["upload"] = await _upload_show_dicts(
            show_dicts,
            explicit_uav_ids=body.get("uav_ids"),
            origin_available=coordinate_system is not None,
        )

    # --- optional: return compiled .skyc or show JSON as download -----------
    default_output_type = "skyc" if uses_phases else "path"
    output_type = str(body.get("output", default_output_type)).lower()
    if output_type in ("show", "skyc"):
        output["shows"] = build_show_specifications(show_dicts)
        output["format"] = "show-upload-v1"
        should_download = bool(body.get("download", output_type == "skyc"))
        if should_download:
            if output_type == "skyc":
                response = Response(
                    skyc_bytes_from_show_dicts(show_dicts),
                    mimetype="application/zip",
                )
                filename = "path-planner.skyc"
            else:
                response = Response(
                    dumps(output, ensure_ascii=False),
                    mimetype="application/json",
                )
                filename = "path-planner-show.json"
            response.headers["Content-Disposition"] = (
                f'attachment; filename="{filename}"'
            )
            return response
    elif output_type != "path":
        return (
            jsonify({"error": "'output' must be one of 'path', 'show', 'skyc'"}),
            400,
        )

    return jsonify(output)


# ── Upload helpers ───────────────────────────────────────────────────────


def _derive_coordinate_system_from_first_uav() -> Optional[dict]:
    """Return a NWU coordinate system dict whose origin is the GPS position
    of the first connected UAV, or ``None`` if no UAV with a valid fix is
    available. Used so that both the saved ``.skyb`` files and the
    over-the-wire upload share the same origin.
    """
    from flockwave.server.model.uav import UAV

    global app, log
    if app is None:
        return None

    uav_ids = sorted(app.object_registry.ids_by_type(UAV), key=_natural_sort_key)
    if not uav_ids:
        return None

    first_uav = app.object_registry.find_by_id(uav_ids[0])
    pos = getattr(getattr(first_uav, "status", None), "position", None)
    lat = getattr(pos, "lat", None)
    lon = getattr(pos, "lon", None)
    if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
        if log:
            log.warning(
                f"Could not derive show origin from {uav_ids[0]} "
                "(no GPS fix yet); saved files will use origin (0, 0) and "
                "uploads will be refused."
            )
        return None

    if log:
        log.info(f"Auto-derived show origin from {uav_ids[0]}: lat={lat}, lon={lon}")
    return {"type": "nwu", "origin": [lon, lat], "orientation": 0}


def _derive_amsl_reference_from_first_uav() -> Optional[float]:
    """Return the current AMSL altitude (in meters) of the first connected
    UAV, or ``None`` if no UAV reports a usable AMSL value yet.

    Used so that the show specification includes an ``amslReference`` field
    (matching the "AMSL" altitude reference in the Skybrush Live UI). Without
    this the firmware sees ``SHOW_ORIGIN_AMSL = -32768000`` (the sentinel
    "no AMSL reference" value) and may refuse to take off.
    """
    from flockwave.server.model.uav import UAV

    global app, log
    if app is None:
        return None

    uav_ids = sorted(app.object_registry.ids_by_type(UAV), key=_natural_sort_key)
    if not uav_ids:
        return None

    first_uav = app.object_registry.find_by_id(uav_ids[0])
    pos = getattr(getattr(first_uav, "status", None), "position", None)
    amsl = getattr(pos, "amsl", None)
    if amsl is None:
        if log:
            log.warning(
                f"Could not derive AMSL reference from {uav_ids[0]} "
                "(no AMSL fix yet); show will be uploaded without an AMSL "
                "reference and the firmware will treat Z as relative to home."
            )
        return None

    try:
        amsl_value = float(amsl)
    except (TypeError, ValueError):
        return None

    # Reject obviously invalid sentinels (e.g. -32768.0 if the field was
    # never populated). Real AMSL values are typically within +/-10000 m.
    if amsl_value < -10000.0 or amsl_value > 10000.0:
        if log:
            log.warning(
                f"AMSL reference from {uav_ids[0]} is out of range "
                f"({amsl_value}); skipping."
            )
        return None

    if log:
        log.info(f"Auto-derived AMSL reference from {uav_ids[0]}: {amsl_value:.2f} m")
    return amsl_value


def _natural_sort_key(value: str):
    """Sort key treating digit runs numerically, so uav-2 < uav-10."""
    return [
        int(token) if token.isdigit() else token
        for token in re.split(r"(\d+)", value)
    ]


async def _upload_show_dicts(
    show_dicts: list,
    *,
    explicit_uav_ids: Optional[list] = None,
    origin_available: bool = True,
) -> dict:
    """Upload ready-made per-drone show dicts to connected UAVs.

    Mapping: ``drone-k`` goes to the k-th entry of ``explicit_uav_ids`` when
    the request provides one, otherwise to the k-th connected UAV in
    *natural* ID order (uav-2 before uav-10). Uploads are refused entirely
    when no real show origin is available — a (0, 0) origin would place the
    waypoints on the far side of the planet.
    """
    from flockwave.server.model.uav import UAV, is_uav

    global app, log
    if app is None:
        return {"error": "Server app not available"}

    if not origin_available:
        return {
            "error": (
                "upload refused: no coordinate system origin available "
                "(no UAV GPS fix and no 'coordinate_system' in the request)"
            ),
            "uploaded": 0,
            "details": {},
        }

    if explicit_uav_ids is not None:
        if not isinstance(explicit_uav_ids, list) or not all(
            isinstance(uid, str) for uid in explicit_uav_ids
        ):
            return {"error": "'uav_ids' must be an array of UAV id strings"}
        uav_ids = list(explicit_uav_ids)
    else:
        uav_ids = sorted(
            app.object_registry.ids_by_type(UAV), key=_natural_sort_key
        )
    if not uav_ids:
        return {"error": "No UAVs connected", "uploaded": 0, "details": {}}

    num_drones = len(show_dicts)
    if len(uav_ids) < num_drones:
        if log:
            log.warning(
                f"Only {len(uav_ids)} UAV(s) available but the show has "
                f"{num_drones} drones -- uploading to available UAVs only"
            )

    details: dict = {}
    uploaded = 0

    for idx, show_dict in enumerate(show_dicts):
        if idx >= len(uav_ids):
            details[f"drone-{idx + 1}"] = "skipped -- no UAV available"
            continue

        uav_id = uav_ids[idx]
        uav = app.object_registry.find_by_id(uav_id)
        if uav is None or not is_uav(uav):
            details[uav_id] = "not a UAV or not found"
            continue

        driver = uav.driver
        handler = getattr(driver, "handle_command___show_upload", None)
        if handler is None:
            details[uav_id] = "driver does not support show upload"
            continue

        try:
            await handler(uav, show=show_dict)
            details[uav_id] = "ok"
            uploaded += 1
            if log:
                log.info(f"Show uploaded to {uav_id} (drone-{idx + 1})")
        except Exception as exc:
            details[uav_id] = f"error: {exc}"
            if log:
                log.error(f"Failed to upload show to {uav_id}: {exc}")

    return {"uploaded": uploaded, "total_uavs": len(uav_ids), "details": details}


# ── Skybrush extension boilerplate ───────────────────────────────────────


class PathPlannerExtension(Extension):
    """Skybrush server extension that exposes the path-planner API."""

    async def run(self, app, configuration, logger):
        route = configuration.get("route", "/api/v1/path-planner")
        http_server = app.import_api("http_server")

        # Global default velocity smoothing, adjustable from the config UI and
        # applied to every planned path (clamped defensively to [0, 1]).
        smoothing = float(
            configuration.get("velocity_smoothing", DEFAULT_VELOCITY_SMOOTHING)
        )
        smoothing = max(0.0, min(1.0, smoothing))

        base_dir = str(configuration.get("output_dir", ""))

        with ExitStack() as stack:
            stack.enter_context(
                overridden(
                    globals(),
                    app=app,
                    log=logger,
                    velocity_smoothing=smoothing,
                    output_base_dir=base_dir,
                )
            )
            stack.enter_context(http_server.mounted(blueprint, path=route))
            logger.info(
                f"Path-planner API mounted at {route}/plan "
                f"(velocity_smoothing={smoothing})"
            )
            await sleep_forever()


construct = PathPlannerExtension

description = "REST API for automatic 3D drone path planning with collision avoidance"

schema = {
    "properties": {
        "route": {
            "type": "string",
            "title": "URL root",
            "description": (
                "URL prefix where the path-planner endpoints are mounted "
                "within the HTTP namespace of the server"
            ),
            "default": "/api/v1/path-planner",
        },
        "velocity_smoothing": {
            "type": "number",
            "title": "Velocity smoothing",
            "description": (
                "How much to ease the speed up/down between waypoints, from 0 "
                "to 1. 0 keeps constant-velocity motion (abrupt start and stop "
                "at every waypoint). Any value above 0 ramps the speed "
                "smoothly to/from zero at the start, end and every hold; "
                "larger values also slow the drone down more at "
                "direction-change corners (1 = come to a full stop at each "
                "corner). Applies to every generated path."
            ),
            "minimum": 0,
            "maximum": 1,
            "default": DEFAULT_VELOCITY_SMOOTHING,
        },
        "output_dir": {
            "type": "string",
            "title": "Output directory",
            "description": (
                "Base directory where generated .skyb/show.json files are "
                "stored. Requests may only select subdirectories of this. "
                "Empty means the parent of the server's working directory."
            ),
            "default": "",
        },
    }
}
