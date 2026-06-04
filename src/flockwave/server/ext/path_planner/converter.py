"""Convert path-planner algorithm output → Skybrush show files.

This module takes a ``SolverResult`` (per-drone waypoint lists produced by
the greedy path-planning solver) and produces:

1. **Per-drone Skybrush trajectory specification dicts** – the JSON structure
   that ``TrajectorySpecification`` expects (version 1, linear segments).
2. **Per-drone ``.skyb`` binary show files** – the compact binary format that
   can be uploaded to MAVLink drones.
3. A **combined show JSON file** containing all drone trajectories and a
   placeholder coordinate system / light program.

All generated files are written to a caller-supplied output directory
(typically the workspace parent folder).
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .solver import SolverResult

# Conservative default for small quadrotors (e.g. Crazyflie) during in-place turns.
DEFAULT_MAX_YAW_RATE_DEG_S = 90.0


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------


def solver_result_to_trajectory_dicts(
    result: SolverResult,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    takeoff_speed: float = 1.5,
    landing_speed: float = 1.0,
) -> List[dict]:
    """Convert a *SolverResult* into a list of Skybrush trajectory dicts.

    Each dict can be passed directly to ``TrajectorySpecification(d)``
    and has the form::

        {
            "version": 1,
            "takeoffTime": <float>,
            "points": [
                [t, [x, y, z], []],          # first point (no control pts)
                [t, [x, y, z], []],           # subsequent – linear segment
                ...
            ]
        }

    The generated trajectory includes a takeoff segment (ground → first
    waypoint altitude) at the beginning and a landing segment (last
    waypoint altitude → ground) at the end.

    Parameters:
        result: output of ``PathSolver.solve()``
        duration_ms: milliseconds per step (from the API request)
        takeoff_time: seconds to wait on the ground before takeoff
        takeoff_speed: vertical speed during takeoff in m/s
        landing_speed: vertical speed during landing in m/s
    """
    num_drones = len(result.drones)
    duration_sec = duration_ms / 1000.0

    # Build per-drone position timelines from the step records.
    # steps[0] is the initial position (step == 0).
    trajectories: List[dict] = []

    for drone in result.drones:
        did = drone.drone_id
        points: List[list] = []

        # Collect raw waypoints from solver
        raw_points: List[list] = []
        for rec in result.steps:
            t_sec = round(rec.step * duration_sec, 4)
            pos = rec.positions[did]
            raw_points.append(
                [t_sec, [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)], []]
            )

        if not raw_points:
            trajectories.append(
                {"version": 1, "takeoffTime": takeoff_time, "points": []}
            )
            continue

        first_pos = raw_points[0][1]  # [x, y, z]
        last_pos = raw_points[-1][1]

        # Ground position: same x, y but z=0
        ground_start = [first_pos[0], first_pos[1], 0]
        ground_end = [last_pos[0], last_pos[1], 0]

        # Takeoff duration based on altitude and speed
        takeoff_alt = abs(first_pos[2])
        takeoff_duration = (
            round(takeoff_alt / takeoff_speed, 4) if takeoff_alt > 0 else 0
        )

        # Landing duration based on altitude and speed
        landing_alt = abs(last_pos[2])
        landing_duration = (
            round(landing_alt / landing_speed, 4) if landing_alt > 0 else 0
        )

        # Build full trajectory:
        # 1) Ground start at t=0
        # 2) Top of takeoff at t=takeoff_duration
        # 3) Solver waypoints shifted by takeoff_duration
        # 4) Landing to ground

        # (1) ground start
        points.append([0, ground_start, []])

        # (2) top of takeoff (= first solver position)
        if takeoff_duration > 0:
            points.append([round(takeoff_duration, 4), list(first_pos), []])

        # (3) solver waypoints (time-shifted)
        for raw_pt in raw_points:
            t_shifted = round(raw_pt[0] + takeoff_duration, 4)
            # Skip duplicate of first point (already added as takeoff end)
            if takeoff_duration > 0 and raw_pt is raw_points[0]:
                continue
            # At ground level, ground_start at t=0 already matches raw_points[0].
            if takeoff_duration == 0 and raw_pt is raw_points[0] and first_pos[2] == 0:
                continue
            points.append([t_shifted, list(raw_pt[1]), []])

        # (4) landing to ground
        last_t = points[-1][0]
        if landing_duration > 0:
            points.append([round(last_t + landing_duration, 4), ground_end, []])

        # Optimisation: collapse consecutive identical positions into a
        # single keyframe (the encoder will produce a constant segment).
        optimised = _collapse_stationary(points)

        trajectories.append(
            {
                "version": 1,
                "takeoffTime": takeoff_time,
                "points": optimised,
            }
        )

    return trajectories


def _lerp_yaw_deg(start: float, end: float, fraction: float) -> float:
    """Linearly interpolate yaw along the shortest path on the circle."""
    delta = _yaw_delta_deg(start, end)
    return (start + delta * fraction) % 360.0


def _yaw_delta_deg(start: float, end: float) -> float:
    """Shortest signed yaw change from *start* to *end* in degrees."""
    return (end - start + 180.0) % 360.0 - 180.0


def _append_yaw_setpoint(setpoints: list[list[float]], t: float, yaw: float) -> None:
    key = round(t, 4)
    yaw_rounded = round(yaw, 4)
    if setpoints and setpoints[-1][0] == key:
        setpoints[-1][1] = yaw_rounded
    else:
        setpoints.append([key, yaw_rounded])


def _append_yaw_ramp_setpoints(
    setpoints: list[list[float]],
    *,
    t_start: float,
    t_budget: float,
    yaw_start: float,
    yaw_end: float,
    max_yaw_rate_deg_s: float,
    min_segment_s: float = 0.05,
) -> float:
    """Append rate-limited yaw setpoints; return the ramp end time."""
    delta = _yaw_delta_deg(yaw_start, yaw_end)
    if abs(delta) < 1e-9:
        return t_start

    needed = abs(delta) / max_yaw_rate_deg_s
    duration = min(t_budget, needed) if t_budget > 0 else needed
    duration = max(duration, 0.001)

    n_segments = max(1, int(math.ceil(duration / min_segment_s)))
    for i in range(1, n_segments + 1):
        frac = i / n_segments
        t = round(t_start + duration * frac, 4)
        yaw = _lerp_yaw_deg(yaw_start, yaw_end, frac)
        _append_yaw_setpoint(setpoints, t, yaw)

    return round(t_start + duration, 4)


def _yaw_at_step(
    record_yaws: Dict[int, float] | None, drone_idx: int, default: float = 0.0
) -> float:
    if not record_yaws:
        return default
    return float(record_yaws.get(drone_idx, default))


def build_yaw_control_dict(
    result: SolverResult,
    drone_idx: int,
    duration_ms: int,
    *,
    takeoff_speed: float = 1.5,
    landing_speed: float = 1.0,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
) -> dict[str, Any] | None:
    """Build a Skybrush ``yawControl`` block for one drone.

    Yaw setpoint times follow the same timeline as
    :func:`solver_result_to_trajectory_dicts`, including takeoff and landing
    segments. Between setpoints the firmware interpolates yaw linearly.

    In-place yaw changes (same position, different yaw) are spread over time
    according to *max_yaw_rate_deg_s*, up to the solver step interval.
    """
    if not result.steps or not any(rec.yaws for rec in result.steps):
        return None

    did = result.drones[drone_idx].drone_id
    duration_sec = duration_ms / 1000.0

    raw_points: list[tuple[float, int]] = []
    for rec in result.steps:
        t_sec = round(rec.step * duration_sec, 4)
        raw_points.append((t_sec, rec.step))

    if not raw_points:
        return None

    first_pos = result.steps[0].positions[did]
    last_pos = result.steps[-1].positions[did]
    takeoff_alt = abs(first_pos[2])
    takeoff_duration = round(takeoff_alt / takeoff_speed, 4) if takeoff_alt > 0 else 0.0
    landing_alt = abs(last_pos[2])
    landing_duration = round(landing_alt / landing_speed, 4) if landing_alt > 0 else 0.0

    def yaw_for_step(step: int) -> float:
        for rec in result.steps:
            if rec.step == step:
                return _yaw_at_step(rec.yaws, did)
        return 0.0

    def position_for_step(step: int) -> list[float] | None:
        for rec in result.steps:
            if rec.step == step:
                pos = rec.positions.get(did)
                return list(pos) if pos is not None else None
        return None

    if max_yaw_rate_deg_s <= 0:
        raise ValueError("max_yaw_rate_deg_s must be > 0")

    setpoints: list[list[float]] = []

    def append_setpoint(t: float, step: int) -> None:
        _append_yaw_setpoint(setpoints, t, yaw_for_step(step))

    append_setpoint(0.0, 0)
    if takeoff_duration > 0:
        append_setpoint(takeoff_duration, 0)

    for raw_idx, (t_sec, step) in enumerate(raw_points):
        t_shifted = round(t_sec + takeoff_duration, 4)
        if takeoff_duration > 0 and step == 0:
            continue
        if takeoff_duration == 0 and step == 0 and first_pos[2] == 0:
            continue
        if raw_idx > 0:
            prev_t_sec, prev_step = raw_points[raw_idx - 1]
            prev_t_shifted = round(prev_t_sec + takeoff_duration, 4)
            prev_pos = position_for_step(prev_step)
            curr_pos = position_for_step(step)
            prev_yaw = yaw_for_step(prev_step)
            curr_yaw = yaw_for_step(step)
            if (
                prev_pos is not None
                and curr_pos is not None
                and prev_pos == curr_pos
                and abs(prev_yaw - curr_yaw) > 1e-9
            ):
                t_budget = max(t_shifted - prev_t_shifted, 0.001)
                ramp_end = _append_yaw_ramp_setpoints(
                    setpoints,
                    t_start=prev_t_shifted,
                    t_budget=t_budget,
                    yaw_start=prev_yaw,
                    yaw_end=curr_yaw,
                    max_yaw_rate_deg_s=max_yaw_rate_deg_s,
                )
                if ramp_end < t_shifted - 1e-6:
                    append_setpoint(t_shifted, step)
                continue
        append_setpoint(t_shifted, step)

    last_t = setpoints[-1][0]
    if landing_duration > 0:
        append_setpoint(last_t + landing_duration, result.steps[-1].step)

    if len(setpoints) < 2:
        return None

    return {
        "version": 1,
        "autoYaw": False,
        "autoYawOffset": setpoints[0][1],
        "setpoints": setpoints,
    }


def build_show_dicts(
    result: SolverResult,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
) -> List[dict]:
    """Build a list of full *show specification* dicts (one per drone).

    Each dict mirrors what Skybrush Live sends to the server during
    ``OBJ-CMD`` / ``__show_upload``::

        {
            "trajectory": { ... },
            "lights": { "version": 1, "data": "AA==" },
            "home": [x, y, z],
            "coordinateSystem": { ... },
        }

    Parameters:
        coordinate_system: optional dict like
            ``{"type": "nwu", "origin": [lon, lat], "orientation": 0}``.
            Defaults to a WGS-84 origin at ``[0, 0]`` with 0° orientation
            when *None*.
    """
    import base64

    if coordinate_system is None:
        coordinate_system = {"type": "nwu", "origin": [0, 0], "orientation": 0}

    traj_dicts = solver_result_to_trajectory_dicts(result, duration_ms, takeoff_time)
    shows: List[dict] = []

    for idx, (drone, traj) in enumerate(zip(result.drones, traj_dicts)):
        home = [
            round(drone.initial[0], 4),
            round(drone.initial[1], 4),
            round(drone.initial[2], 4),
        ]

        # Minimal light program: a single END (0x00) byte
        minimal_light = base64.b64encode(b"\x00").decode("ascii")

        # Permissive default geofence so the firmware does not reject the
        # show on reload because of missing fence info. The values are wide
        # enough not to interfere with typical small flights.
        geofence = {
            "version": 1,
            "enabled": True,
            "maxAltitude": 100.0,
            "maxDistance": 500.0,
            "minAltitude": -5.0,
            "action": "land",
            "polygons": [],
            "rallyPoints": [],
        }

        show_dict = {
            "trajectory": traj,
            "lights": {"version": 1, "data": minimal_light},
            "home": home,
            "coordinateSystem": coordinate_system,
            "geofence": geofence,
        }
        # Setting an AMSL reference makes the firmware interpret the
        # trajectory Z coordinates as offsets from this absolute altitude
        # (in meters) rather than treating them as relative to home. This
        # corresponds to the "AMSL" altitude reference in the Live UI.
        if amsl_reference is not None:
            show_dict["amslReference"] = float(amsl_reference)

        yaw_control = build_yaw_control_dict(
            result,
            idx,
            duration_ms,
            max_yaw_rate_deg_s=max_yaw_rate_deg_s,
        )
        if yaw_control is not None:
            show_dict["yawControl"] = yaw_control

        shows.append(show_dict)

    return shows


async def save_skyb_files(
    result: SolverResult,
    output_dir: str | Path,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
    max_yaw_rate_deg_s: float = DEFAULT_MAX_YAW_RATE_DEG_S,
) -> Dict[str, str]:
    """Generate ``.skyb`` files for every drone and save them to *output_dir*.

    Also writes a ``show.json`` containing all drone show specifications.

    The ``coordinate_system`` argument is forwarded to :func:`build_show_dicts`
    so the JSON saved on disk matches what the drone actually receives over
    MAVFTP. Pass the same dict you use for the auto-upload step.

    Returns a dict mapping drone id strings to their ``.skyb`` file paths.
    """
    from flockwave.server.show.trajectory import TrajectorySpecification
    from flockwave.server.show.formats import SkybrushBinaryShowFile

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    show_dicts = build_show_dicts(
        result,
        duration_ms,
        takeoff_time,
        coordinate_system=coordinate_system,
        amsl_reference=amsl_reference,
        max_yaw_rate_deg_s=max_yaw_rate_deg_s,
    )
    traj_dicts = solver_result_to_trajectory_dicts(result, duration_ms, takeoff_time)
    skyb_paths: Dict[str, str] = {}

    for idx, (show_dict, traj_dict) in enumerate(zip(show_dicts, traj_dicts)):
        drone_id = f"drone-{idx + 1}"

        # ── .skyb binary ────────────────────────────────────────────
        traj_spec = TrajectorySpecification(traj_dict)

        async with SkybrushBinaryShowFile.create_in_memory(version=2) as f:
            await f.add_trajectory(traj_spec)

            # Minimal light program (single END opcode)
            await f.add_encoded_light_program(b"\x00")

            await f.add_comment(f"path_planner:{drone_id}")
            await f.finalize()
            skyb_data = f.get_contents()

        skyb_path = output_dir / f"{drone_id}.skyb"
        skyb_path.write_bytes(skyb_data)
        skyb_paths[drone_id] = str(skyb_path)

    # ── Combined show JSON ──────────────────────────────────────────
    combined = {
        "version": 1,
        "num_drones": len(show_dicts),
        "drones": {},
    }
    for idx, show_dict in enumerate(show_dicts):
        drone_id = f"drone-{idx + 1}"
        combined["drones"][drone_id] = show_dict

    show_json_path = output_dir / "show.json"
    show_json_path.write_text(json.dumps(combined, indent=2), encoding="utf-8")
    skyb_paths["_show_json"] = str(show_json_path)

    return skyb_paths


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _collapse_stationary(points: List[list]) -> List[list]:
    """Remove consecutive keyframes with identical positions.

    When a drone sits still for multiple steps the algorithm records the
    same (x, y, z) repeatedly.  We collapse those into a single pair of
    keyframes (enter + exit) so the trajectory keeps its timing but the
    encoder produces a compact *constant* segment instead of many tiny
    linear segments with zero displacement.

    The first and last keyframe are always kept.
    """
    if len(points) <= 2:
        return points

    collapsed: List[list] = [points[0]]

    i = 1
    while i < len(points):
        # Look ahead: is the position the same as the previous kept point?
        prev_pos = collapsed[-1][1]
        cur_pos = points[i][1]

        if cur_pos == prev_pos:
            # Skip ahead to the last frame with this same position
            j = i
            while j + 1 < len(points) and points[j + 1][1] == cur_pos:
                j += 1
            # Keep only the exit keyframe (or the final point)
            collapsed.append(points[j])
            i = j + 1
        else:
            collapsed.append(points[i])
            i += 1

    return collapsed
