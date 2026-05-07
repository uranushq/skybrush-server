"""SolverResult → JSON output builder.

Output format
-------------
{
  "drones": [
    {
      "id": "drone-1",
      "name": "Drone 1",
      "battery": 100,
      "status": "Flying",
      "pos": [x, y, z],
      "path": [
        {"x": ..., "y": ..., "z": ..., "durationMs": 300},
        ...
      ]
    },
    ...
  ]
}
"""

from __future__ import annotations

from io import BytesIO
from json import dumps
from typing import Any, List, Optional
from zipfile import ZIP_DEFLATED, ZipFile

from .converter import build_show_dicts
from .solver import SolverResult

DEFAULT_DURATION_MS = 300


def build_output(result: SolverResult, duration_ms: int = DEFAULT_DURATION_MS) -> dict:
    """Convert a SolverResult into the JSON-serialisable output dict."""
    drones_out: List[dict] = []

    for drone in result.drones:
        path: List[dict] = []

        for step_rec in result.steps:
            if step_rec.step == 0:
                continue
            step_pos = step_rec.positions[drone.drone_id]
            path.append(
                {
                    "x": round(step_pos[0], 4),
                    "y": round(step_pos[1], 4),
                    "z": round(step_pos[2], 4),
                    "durationMs": duration_ms,
                }
            )

        drone_entry = {
            "id": f"drone-{drone.drone_id + 1}",
            "name": f"Drone {drone.drone_id + 1}",
            "battery": 100,
            "status": "Flying",
            "pos": [
                round(drone.initial[0], 4),
                round(drone.initial[1], 4),
                round(drone.initial[2], 4),
            ],
            "path": path,
        }
        drones_out.append(drone_entry)

    return {"drones": drones_out}


def build_show_specifications(
    result: SolverResult, duration_ms: int = DEFAULT_DURATION_MS
) -> list[dict[str, Any]]:
    """Build per-drone show upload payloads compatible with ``__show_upload``."""
    dt = duration_ms / 1000.0
    shows: list[dict[str, Any]] = []

    for drone in result.drones:
        points: list[list[Any]] = [[0.0, [*drone.initial], []]]
        t = 0.0

        for step_rec in result.steps:
            if step_rec.step == 0:
                continue
            t = round(t + dt, 3)
            x, y, z = step_rec.positions[drone.drone_id]
            points.append([t, [x, y, z], []])

        shows.append(
            {
                "mission": {
                    "id": f"path-planner-drone-{drone.drone_id + 1}",
                    "numDrones": 1,
                },
                "coordinateSystem": {
                    "type": "nwu",
                    "origin": {"lat": 0.0, "lon": 0.0},
                    "orientation": 0.0,
                },
                "home": [*drone.initial],
                "trajectory": {
                    "version": 1,
                    "home": [*drone.initial],
                    "takeoffTime": 0.0,
                    "points": points,
                },
            }
        )

    return shows


def build_skyc_bytes(
    result: SolverResult,
    duration_ms: int = DEFAULT_DURATION_MS,
    *,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
) -> bytes:
    """Build a ``.skyc`` ZIP for Skybrush Viewer.

    Uses the same per-drone payload as Live / MAV upload (:func:`build_show_dicts`)
    and **inlines** trajectories (no ``$ref``), so the reader does not need to
    resolve JSON references inside the archive.
    """
    show_dicts = build_show_dicts(
        result,
        duration_ms,
        takeoff_time,
        coordinate_system=coordinate_system,
        amsl_reference=amsl_reference,
    )
    cues = {"version": 1, "items": [{"time": 0.0, "name": "start"}]}

    drones_swarm: list[dict[str, Any]] = []
    for idx, show_dict in enumerate(show_dicts):
        drone_name = f"drone-{idx + 1}"
        entry = {
            "type": "generic",
            "settings": {"name": drone_name, **show_dict},
        }
        drones_swarm.append(entry)

    show = {
        "version": 1,
        "format": "show-upload-v1",
        "settings": {
            "cues": cues,
            "validation": {
                "maxAltitude": 150,
                "maxVelocityXY": 8,
                "maxVelocityZ": 2.5,
                "minDistance": 3,
            },
        },
        "swarm": {"drones": drones_swarm},
        "environment": {"type": "outdoor"},
        "meta": {
            "id": f"path-planner-{len(drones_swarm)}",
            "title": f"Path planner export ({len(drones_swarm)} drones)",
        },
        "media": {},
    }

    archive = BytesIO()
    with ZipFile(archive, mode="w", compression=ZIP_DEFLATED) as zf:
        zf.writestr("show.json", dumps(show, ensure_ascii=False, indent=2))
        zf.writestr("cues.json", dumps(cues, ensure_ascii=False, indent=2))

    return archive.getvalue()
