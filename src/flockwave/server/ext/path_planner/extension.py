"""Path-planner extension — REST API for automatic 3D drone path generation.

Endpoint
--------
POST ``/api/v1/path-planner/plan``

Formation-phase flow (``phases`` present)::

    ground (per-drone [x, y, z_ground])
      │  vertical takeoff
      ▼
    staging hover  (z_ground + staging_altitude, default 5 m)
      │  solver: collision-avoided move
      ▼
    staging grid   (grid_spacing apart, default 2 m — the algorithm's start)
      │  solver: phase 1, phase 2, ... (+ per-phase holds and yaw changes)
      ▼
    return to staging hover  (when return_to_initial, default true)
      │  vertical landing
      ▼
    ground

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
from json import dumps
from logging import Logger
from math import ceil, sqrt
from pathlib import Path
from typing import TYPE_CHECKING, Optional, Sequence

from quart import Blueprint, Response, jsonify, request
from trio import sleep_forever, to_thread

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .converter import (
    DEFAULT_DURATION_MS,
    DEFAULT_MAX_YAW_RATE_DEG_S,
    DEFAULT_VELOCITY_SMOOTHING,
    TrajectoryLimitError,
    build_delivery_show_dicts,
    build_show_dicts,
    lerp_yaw_deg,
    save_skyb_files,
    yaw_delta_deg,
)
from .drone import Drone
from .output import (
    build_output,
    build_show_specifications,
    skyc_bytes_from_show_dicts,
)
from .collision_volume import (
    PLANNED_XY_CLEARANCE,
    PLANNING_MARGIN,
    describe_collision_envelope,
    envelope_overlap,
)
from .solver import PathSolver, SolverResult, StepRecord
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

# Base directory for generated files; requests may only choose subdirectories
# of this. Empty string means "parent of the server's working directory".
output_base_dir: str = ""

# Skybrush firmware tends to silently reject very short shows or shows with a
# zero takeoff time; every mode enforces this same minimum ground wait.
MIN_TAKEOFF_TIME = 5.0

# Staging defaults: hover this high above each drone's ground position, then
# form a grid with this spacing before the requested formation phases start.
DEFAULT_STAGING_ALTITUDE = 5.0
DEFAULT_GRID_SPACING = 2.0


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

    return None


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
) -> list[dict]:
    """Pairs of points closer than the planner's inflated envelope allows."""
    pairs: list[dict] = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            a = points[i]
            b = points[j]
            if envelope_overlap(a, b, margin=PLANNING_MARGIN):
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


def _spacing_error_response(violations: list[dict]):
    return (
        jsonify(
            {
                "error": "Formation points are too close to each other",
                "code": "FORMATION_SPACING_TOO_CLOSE",
                "details": {
                    "collision_envelope": describe_collision_envelope(),
                    "required_xy_clearance": PLANNED_XY_CLEARANCE,
                    "violations": violations,
                },
            }
        ),
        422,
    )


def _validate_point_group_spacing(groups: list[tuple[str, Sequence]]):
    """422 response when any labelled point set violates the clearance."""
    violations: list[dict] = []
    for label, points in groups:
        violations.extend(_find_clearance_violations(label, points))
    if violations:
        return _spacing_error_response(violations)
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


# ── staging grid ─────────────────────────────────────────────────────────


def _staging_grid_slots(
    hover_positions: Sequence[Sequence[float]], spacing: float
) -> list[tuple[float, float, float]]:
    """Grid slot positions centered on the fleet's hover centroid.

    ``ceil(sqrt(n))`` columns, row-major, all at the highest hover altitude
    so drones over uneven ground meet on one flat plane.
    """
    n = len(hover_positions)
    cx = sum(p[0] for p in hover_positions) / n
    cy = sum(p[1] for p in hover_positions) / n
    altitude = max(p[2] for p in hover_positions)

    cols = ceil(sqrt(n))
    rows = ceil(n / cols)
    slots: list[tuple[float, float, float]] = []
    for r in range(rows):
        for c in range(cols):
            if len(slots) >= n:
                break
            slots.append(
                (
                    round(cx + (c - (cols - 1) / 2.0) * spacing, 4),
                    round(cy + (r - (rows - 1) / 2.0) * spacing, 4),
                    round(altitude, 4),
                )
            )
    return slots


def _assign_grid_slots(
    positions: Sequence[Sequence[float]],
    slots: Sequence[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    """Assign each drone the closest free grid slot (greedy global matching).

    Deterministic: candidate pairs are sorted by distance with the drone and
    slot indices as tie-breakers, which keeps transition paths short and
    mostly crossing-free.
    """
    n = len(positions)
    candidates = sorted(
        (
            (
                (positions[i][0] - slots[j][0]) ** 2
                + (positions[i][1] - slots[j][1]) ** 2
                + (positions[i][2] - slots[j][2]) ** 2,
                i,
                j,
            )
            for i in range(n)
            for j in range(n)
        )
    )
    drone_to_slot: dict[int, int] = {}
    used_slots: set[int] = set()
    for _dist, i, j in candidates:
        if i in drone_to_slot or j in used_slots:
            continue
        drone_to_slot[i] = j
        used_slots.add(j)
        if len(drone_to_slot) == n:
            break
    return [slots[drone_to_slot[i]] for i in range(n)]


# ── formation planning (runs in a worker thread) ─────────────────────────


def _segment_seed(seed: Optional[int], index: int) -> Optional[int]:
    """Distinct-but-reproducible seed per solver segment."""
    return None if seed is None else seed + index


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
) -> list[tuple[float, float, float]]:
    """Run one solver segment and append its steps to the combined timeline.

    Raises :class:`PlanningError` when the segment cannot be solved — the
    caller never sees a partial path.
    """
    num_drones = len(current_positions)
    solver = PathSolver(
        initials=current_positions,
        targets=list(targets),
        step_size=step_size,
        seed=seed,
        min_z=min_z,
    )
    result = solver.solve()
    if not result.success:
        raise PlanningError(
            f"planning failed in segment '{label}': {result.failure_reason}",
            details={
                "segment": label,
                "reason": result.failure_reason,
                "stuck_drones": [f"drone-{i + 1}" for i in result.stuck_drones],
                "steps_completed": result.total_steps,
            },
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
) -> tuple[SolverResult, list[dict]]:
    """Plan synced formation phases with collision avoidance between phases.

    ``start_positions`` are the hover positions right after takeoff. When
    ``staging_targets`` is given, a staging segment moves the fleet into the
    grid before the first phase, and ``return_to_initial`` brings it back to
    the hover positions at the end (so landing descends onto the original
    ground spots).

    Raises :class:`PlanningError` on any unsolvable segment.
    """
    num_drones = len(start_positions)
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

    def summarize(name: str, arrival_step: int, hold_ms: int, hold_steps: int) -> None:
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
            }
        )

    def run_segment(targets, label: str) -> None:
        nonlocal current_positions, segment_counter
        if _positions_match(current_positions, targets):
            current_positions = [tuple(t) for t in targets]
            return
        current_positions = _extend_with_solver_run(
            combined_steps,
            current_positions,
            targets,
            step_size=step_size,
            seed=_segment_seed(seed, segment_counter),
            min_z=min_z,
            current_yaws=current_yaws,
            label=label,
        )
        segment_counter += 1

    # ── staging: move from the hover line-up into the grid ──────────────
    if staging_targets is not None:
        run_segment(staging_targets, "staging-grid")
        summarize("staging-grid", combined_steps[-1].step, 0, 0)

    # ── requested formation phases ───────────────────────────────────────
    for phase_index, phase in enumerate(phases):
        name = str(phase.get("name", f"phase-{phase_index + 1}"))
        targets = _phase_targets(phase, num_drones)
        target_yaws = _phase_target_yaws(phase, num_drones)

        run_segment(targets, name)
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

        summarize(name, arrival_step, hold_ms, hold_steps)

    # ── return to the staging hover positions ────────────────────────────
    final_targets = (
        original_initials
        if return_to_initial
        else [tuple(t) for t in _phase_targets(phases[-1], num_drones)]
    )
    if return_to_initial:
        run_segment(original_initials, "return-to-start")
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


def _build_and_verify_shows(build_fn, smoothing: float):
    """Build show dicts and run the final verification gate (sync, threaded).

    ``build_fn(smoothing)`` must return the show dicts. When the smoothed
    show fails verification the build is retried without smoothing; if even
    that fails, the violations are returned so the caller can abort.

    Returns ``(show_dicts, violations, applied_smoothing)``.
    """
    show_dicts = build_fn(smoothing)
    violations = verify_show_dicts(show_dicts)
    if not violations:
        return show_dicts, [], smoothing
    if smoothing > 0.0:
        fallback_dicts = build_fn(0.0)
        fallback_violations = verify_show_dicts(fallback_dicts)
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
            geofence=body.get("geofence"),
        )

    try:
        show_dicts, violations, applied_smoothing = await to_thread.run_sync(
            lambda: _build_and_verify_shows(build, smoothing)
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
        "smoothing": {"requested": smoothing, "applied": applied_smoothing},
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


@blueprint.route("/plan", methods=["POST"])
async def plan():
    """Run the path-planning algorithm and return per-drone paths."""
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

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
    duration_ms: int = int(body.get("duration_ms", DEFAULT_DURATION_MS))
    seed: Optional[int] = body.get("seed")
    max_yaw_rate_deg_s: float = float(
        body.get("max_yaw_rate_deg_s", DEFAULT_MAX_YAW_RATE_DEG_S)
    )
    smoothing: float = float(body.get("velocity_smoothing", velocity_smoothing))

    if step_size <= 0:
        return jsonify({"error": "'step_size' must be > 0"}), 400
    if duration_ms <= 0:
        return jsonify({"error": "'duration_ms' must be > 0"}), 400
    if max_yaw_rate_deg_s <= 0:
        return jsonify({"error": "'max_yaw_rate_deg_s' must be > 0"}), 400
    if not (0.0 <= smoothing <= 1.0):
        return jsonify({"error": "'velocity_smoothing' must be between 0 and 1"}), 400

    # Staging parameters: hover altitude above each drone's ground position
    # and the grid spacing. ``initial_altitude`` is accepted as a legacy
    # alias for the staging altitude.
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
    if grid_spacing < PLANNED_XY_CLEARANCE:
        return (
            jsonify(
                {
                    "error": (
                        f"'grid_spacing' of {grid_spacing} m is below the "
                        f"required clearance of {PLANNED_XY_CLEARANCE:.2f} m"
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
            grid_slots = _staging_grid_slots(hover_positions, grid_spacing)
            staging_targets = _assign_grid_slots(hover_positions, grid_slots)
        else:
            grid_slots = []
            staging_targets = None
        planning_start = hover_positions
    else:
        ground_positions = None
        hover_positions = []
        grid_slots = []
        staging_targets = None
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
        spacing_groups.append(("staging-hover", hover_positions))
        floor_groups.append(("staging-hover", hover_positions))
        if staging_targets is not None:
            spacing_groups.append(("staging-grid", staging_targets))
        for phase_index, phase in enumerate(phases):
            targets = _phase_targets(phase, len(initial))
            label = f"phases[{phase_index}]:{phase.get('name') or f'phase-{phase_index + 1}'}"
            spacing_groups.append((label, targets))
            floor_groups.append((label, targets))
    else:
        spacing_groups.append(("initial", initial))
        spacing_groups.append(("target", target))
        floor_groups.append(("initial", initial))
        floor_groups.append(("target", target))

    spacing_error = _validate_point_group_spacing(spacing_groups)
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
            )
        solver = PathSolver(
            initials=[tuple(p) for p in initial],
            targets=[tuple(p) for p in target],
            step_size=step_size,
            seed=seed,
            min_z=min_z,
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

    try:
        result, phase_summaries = await to_thread.run_sync(run_planning)
    except PlanningError as exc:
        if log:
            log.warning(f"Path planning failed: {exc}")
        return _planning_failure_response(exc, validation_payload)

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
            ground_positions=ground_positions,
            geofence=body.get("geofence"),
        )

    try:
        show_dicts, violations, applied_smoothing = await to_thread.run_sync(
            lambda: _build_and_verify_shows(build, smoothing)
        )
    except TrajectoryLimitError as exc:
        if log:
            log.warning(f"Trajectory limit exceeded: {exc}")
        return _limit_failure_response(exc, validation_payload)

    if violations:
        if log:
            log.error(
                f"Verification gate rejected the generated show "
                f"({len(violations)} violation(s))"
            )
        return _verification_failure_response(violations, validation_payload)

    # --- response scaffolding ----------------------------------------------
    output = build_output(result, duration_ms)
    output["success"] = True
    output["total_steps"] = result.total_steps
    output["validation"] = validation_payload
    output["verification"] = {"checked": True, "violations": 0}
    output["smoothing"] = {"requested": smoothing, "applied": applied_smoothing}
    if takeoff_time_adjusted:
        output["adjustments"] = {"takeoff_time": takeoff_time}
    if uses_phases:
        output["mode"] = "formation_phases"
        output["phases"] = phase_summaries
        output["ground_initial"] = ground_positions
        output["staging"] = {
            "enabled": staging_targets is not None,
            "altitude": staging_altitude,
            "grid_spacing": grid_spacing,
            "hover_positions": hover_positions,
            "grid_slots": [list(slot) for slot in (staging_targets or [])],
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
                f"{num_drones} drones — uploading to available UAVs only"
            )

    details: dict = {}
    uploaded = 0

    for idx, show_dict in enumerate(show_dicts):
        if idx >= len(uav_ids):
            details[f"drone-{idx + 1}"] = "skipped — no UAV available"
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
