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
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .solver import SolverResult


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def solver_result_to_trajectory_dicts(
    result: SolverResult,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
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

    Because the algorithm already outputs discrete waypoints joined by
    straight-line moves, every segment is **linear** (no Bézier control
    points between keyframes — the ``[]`` placeholder is sufficient since
    ``iter_segments`` will build a 2-point ``[start, end]`` pair which
    the binary encoder writes as a linear segment).

    Parameters:
        result: output of ``PathSolver.solve()``
        duration_ms: milliseconds per step (from the API request)
        takeoff_time: seconds of ground-wait before the first movement
    """
    num_drones = len(result.drones)
    duration_sec = duration_ms / 1000.0

    # Build per-drone position timelines from the step records.
    # steps[0] is the initial position (step == 0).
    trajectories: List[dict] = []

    for drone in result.drones:
        did = drone.drone_id
        points: List[list] = []

        for rec in result.steps:
            t_sec = round(rec.step * duration_sec, 4)
            pos = rec.positions[did]
            # [time, [x,y,z], [control_points]]
            points.append([t_sec, [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)], []])

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


def build_show_dicts(
    result: SolverResult,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
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
        home = [round(drone.initial[0], 4), round(drone.initial[1], 4), round(drone.initial[2], 4)]

        # Minimal light program: a single END (0x00) byte
        minimal_light = base64.b64encode(b"\x00").decode("ascii")

        shows.append(
            {
                "trajectory": traj,
                "lights": {"version": 1, "data": minimal_light},
                "home": home,
                "coordinateSystem": coordinate_system,
            }
        )

    return shows


async def save_skyb_files(
    result: SolverResult,
    output_dir: str | Path,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
) -> Dict[str, str]:
    """Generate ``.skyb`` files for every drone and save them to *output_dir*.

    Also writes a ``show.json`` containing all drone show specifications.

    Returns a dict mapping drone id strings to their ``.skyb`` file paths.
    """
    from flockwave.server.show.trajectory import TrajectorySpecification
    from flockwave.server.show.formats import SkybrushBinaryShowFile

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    show_dicts = build_show_dicts(result, duration_ms, takeoff_time)
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
