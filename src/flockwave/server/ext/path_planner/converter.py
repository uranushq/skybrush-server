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
from pathlib import Path
from typing import Any, Dict, List, Optional

from .solver import SolverResult

# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------


def _raw_timeline_to_trajectory_dict(
    raw_points: List[list],
    takeoff_time: float,
    takeoff_speed: float,
    landing_speed: float,
) -> dict:
    """Turn absolute-time keyframes into a Skybrush trajectory dict.

    ``raw_points`` items are ``[t_sec, [x, y, z], []]`` with *t* increasing
    from 0. The same takeoff/landing envelope as the solver export is
    applied (ground → cruise → ground).
    """
    if not raw_points:
        return {"version": 1, "takeoffTime": takeoff_time, "points": []}

    first_pos = raw_points[0][1]
    last_pos = raw_points[-1][1]

    ground_start = [first_pos[0], first_pos[1], 0]
    ground_end = [last_pos[0], last_pos[1], 0]

    takeoff_alt = abs(first_pos[2])
    takeoff_duration = round(takeoff_alt / takeoff_speed, 4) if takeoff_alt > 0 else 0

    landing_alt = abs(last_pos[2])
    landing_duration = round(landing_alt / landing_speed, 4) if landing_alt > 0 else 0

    points: List[list] = []
    points.append([0, ground_start, []])

    if takeoff_duration > 0:
        points.append([round(takeoff_duration, 4), list(first_pos), []])

    for raw_pt in raw_points:
        t_shifted = round(raw_pt[0] + takeoff_duration, 4)
        if takeoff_duration > 0 and raw_pt is raw_points[0]:
            continue
        if takeoff_duration == 0 and raw_pt is raw_points[0] and first_pos[2] == 0:
            continue
        points.append([t_shifted, list(raw_pt[1]), []])

    last_t = points[-1][0]
    if landing_duration > 0:
        points.append([round(last_t + landing_duration, 4), ground_end, []])

    optimised = _collapse_stationary(points)

    return {
        "version": 1,
        "takeoffTime": takeoff_time,
        "points": optimised,
    }


def waypoints_to_raw_timeline(path: List[dict]) -> List[list]:
    """Build ``[[t, [x, y, z], []], ...]`` from frontend waypoints.

    The first waypoint starts at *t* = 0; its ``durationMs`` is ignored.
    For each following waypoint, ``durationMs`` is the travel time **from
    the previous keyframe to this position** (ms). Optional ``holdMs`` on
    a waypoint adds a dwell segment at that position after arrival.

    Args:
        path: List of dicts with ``x``, ``y``, ``z`` and optional
            ``durationMs``, ``holdMs``.

    Returns:
        Skybrush-style raw point rows with absolute times in seconds.
    """
    if not path:
        return []

    first = path[0]
    x0 = round(float(first["x"]), 4)
    y0 = round(float(first["y"]), 4)
    z0 = round(float(first["z"]), 4)
    start_pos = [x0, y0, z0]
    raw_points: List[list] = [[0.0, start_pos, []]]
    t = 0.0
    last_dt_sec = 0.0

    for i in range(1, len(path)):
        wp = path[i]
        x = round(float(wp["x"]), 4)
        y = round(float(wp["y"]), 4)
        z = round(float(wp["z"]), 4)
        dt_ms = float(wp.get("durationMs", 0))
        last_dt_sec = dt_ms / 1000.0
        t = round(t + last_dt_sec, 4)
        raw_points.append([t, [x, y, z], []])
        hold_ms = float(wp.get("holdMs") or 0)
        if hold_ms > 0:
            t = round(t + hold_ms / 1000.0, 4)
            raw_points.append([t, [x, y, z], []])

    # Ensure we return to the starting position at the end of the timeline.
    end_pos = raw_points[-1][1]
    if end_pos != start_pos:
        extra = last_dt_sec if last_dt_sec > 0 else 0.0
        t = round(t + extra, 4)
        raw_points.append([t, start_pos, []])

    return raw_points


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
    duration_sec = duration_ms / 1000.0

    # Build per-drone position timelines from the step records.
    # steps[0] is the initial position (step == 0).
    trajectories: List[dict] = []

    for drone in result.drones:
        did = drone.drone_id

        # Collect raw waypoints from solver
        raw_points: List[list] = []
        for rec in result.steps:
            t_sec = round(rec.step * duration_sec, 4)
            pos = rec.positions[did]
            raw_points.append(
                [t_sec, [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)], []]
            )

        trajectories.append(
            _raw_timeline_to_trajectory_dict(
                raw_points, takeoff_time, takeoff_speed, landing_speed
            )
        )

    return trajectories


def explicit_drone_paths_to_trajectory_dicts(
    drones: List[dict],
    takeoff_time: float = 0.0,
    takeoff_speed: float = 1.5,
    landing_speed: float = 1.0,
) -> List[dict]:
    """Convert frontend ``drones[].path`` payloads to trajectory dicts."""
    out: List[dict] = []
    for d in drones:
        path = d.get("path") or []
        raw = waypoints_to_raw_timeline(path if isinstance(path, list) else [])
        out.append(
            _raw_timeline_to_trajectory_dict(
                raw, takeoff_time, takeoff_speed, landing_speed
            )
        )
    return out


def build_show_dicts(
    result: SolverResult,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
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

    for drone, traj in zip(result.drones, traj_dicts):
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

        show_dict: dict[str, Any] = {
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
        shows.append(show_dict)

    return shows


def build_show_dicts_from_explicit_drone_paths(
    drones: List[dict],
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
) -> List[dict]:
    """Build *show* dicts from ``{ "id", "path": [ waypoints... ] }`` entries."""
    import base64

    if coordinate_system is None:
        coordinate_system = {"type": "nwu", "origin": [0, 0], "orientation": 0}

    traj_dicts = explicit_drone_paths_to_trajectory_dicts(
        drones, takeoff_time=takeoff_time
    )
    shows: List[dict] = []

    for d, traj in zip(drones, traj_dicts):
        path = d.get("path") or []
        if path and isinstance(path[0], dict):
            p0 = path[0]
            home = [
                round(float(p0.get("x", 0)), 4),
                round(float(p0.get("y", 0)), 4),
                round(float(p0.get("z", 0)), 4),
            ]
        else:
            home = [0.0, 0.0, 0.0]

        minimal_light = base64.b64encode(b"\x00").decode("ascii")

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

        show_dict: dict[str, Any] = {
            "trajectory": traj,
            "lights": {"version": 1, "data": minimal_light},
            "home": home,
            "coordinateSystem": coordinate_system,
            "geofence": geofence,
        }
        if amsl_reference is not None:
            show_dict["amslReference"] = float(amsl_reference)
        shows.append(show_dict)

    return shows


async def save_skyb_files(
    result: SolverResult,
    output_dir: str | Path,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
) -> Dict[str, str]:
    """Generate ``.skyb`` files for every drone and save them to *output_dir*.

    Also writes a ``show.json`` containing all drone show specifications.

    The ``coordinate_system`` argument is forwarded to :func:`build_show_dicts`
    so the JSON saved on disk matches what the drone actually receives over
    MAVFTP. Pass the same dict you use for the auto-upload step.

    Returns a dict mapping drone id strings to their ``.skyb`` file paths.
    """
    from flockwave.server.show.formats import SkybrushBinaryShowFile
    from flockwave.server.show.trajectory import TrajectorySpecification

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    show_dicts = build_show_dicts(
        result,
        duration_ms,
        takeoff_time,
        coordinate_system=coordinate_system,
        amsl_reference=amsl_reference,
    )
    traj_dicts = solver_result_to_trajectory_dicts(result, duration_ms, takeoff_time)
    skyb_paths: Dict[str, str] = {}

    for idx, (_show, traj_dict) in enumerate(zip(show_dicts, traj_dicts)):
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


async def save_skyb_files_from_explicit_drone_paths(
    drones: List[dict],
    output_dir: str | Path,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
) -> Dict[str, str]:
    """Write ``.skyb`` and ``show.json`` for frontend waypoint payloads."""
    from flockwave.server.show.formats import SkybrushBinaryShowFile
    from flockwave.server.show.trajectory import TrajectorySpecification

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    show_dicts = build_show_dicts_from_explicit_drone_paths(
        drones,
        takeoff_time=takeoff_time,
        coordinate_system=coordinate_system,
        amsl_reference=amsl_reference,
    )
    traj_dicts = explicit_drone_paths_to_trajectory_dicts(
        drones, takeoff_time=takeoff_time
    )
    skyb_paths: Dict[str, str] = {}

    for idx, (_, traj_dict) in enumerate(zip(show_dicts, traj_dicts)):
        src = drones[idx] if idx < len(drones) else {}
        drone_id = str(src.get("id") or f"drone-{idx + 1}")

        traj_spec = TrajectorySpecification(traj_dict)

        async with SkybrushBinaryShowFile.create_in_memory(version=2) as f:
            await f.add_trajectory(traj_spec)
            await f.add_encoded_light_program(b"\x00")

            await f.add_comment(f"path_planner:{drone_id}")
            await f.finalize()
            skyb_data = f.get_contents()

        safe_name = "".join(c if c.isalnum() or c in "-_" else "_" for c in drone_id)
        skyb_path = output_dir / f"{safe_name}.skyb"
        skyb_path.write_bytes(skyb_data)
        skyb_paths[drone_id] = str(skyb_path)

    combined = {
        "version": 1,
        "num_drones": len(show_dicts),
        "drones": {},
    }
    for idx, show_dict in enumerate(show_dicts):
        src = drones[idx] if idx < len(drones) else {}
        key = str(src.get("id") or f"drone-{idx + 1}")
        combined["drones"][key] = show_dict

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
