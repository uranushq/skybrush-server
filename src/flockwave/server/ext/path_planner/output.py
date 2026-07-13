"""SolverResult → JSON output builder.

Output format
-------------
{
  "drones": [
    {
      "id": "drone-1",
      "name": "Drone 1",
      "pos": [x, y, z],
      "path": [
        {"x": ..., "y": ..., "z": ..., "durationMs": 1000},
        ...
      ]
    },
    ...
  ]
}

All show-shaped payloads (the ``shows`` response field, the ``.skyc``
archive, the ``.skyb`` files and the MAVFTP upload) come from the **same**
per-drone show dicts built once by ``converter.build_show_dicts`` — there is
no second trajectory builder that could drift out of sync.
"""

from __future__ import annotations

import math
from io import BytesIO
from json import dumps
from typing import Any, List
from zipfile import ZIP_DEFLATED, ZipFile

from .collision_volume import MIN_DISTANCE_FOR_VALIDATION
from .converter import (
    DEFAULT_DURATION_MS,
    MAX_VELOCITY_XY,
    MAX_VELOCITY_Z,
    step_time_ms,
)
from .solver import SolverResult

__all__ = (
    "build_output",
    "build_show_specifications",
    "skyc_bytes_from_show_dicts",
)


def build_output(result: SolverResult, duration_ms: int = DEFAULT_DURATION_MS) -> dict:
    """Convert a SolverResult into the JSON-serialisable output dict.

    Consecutive steps at the same position are collapsed into a single
    entry with an accumulated ``durationMs`` so long holds don't bloat the
    payload with identical waypoints. When steps carry explicit ``time_ms``
    values, those drive the per-hop duration instead of the global default.
    """
    drones_out: List[dict] = []

    for drone in result.drones:
        path: List[dict] = []
        prev_time_ms = step_time_ms(result.steps[0], duration_ms) if result.steps else 0

        for step_rec in result.steps:
            if step_rec.step == 0:
                continue
            step_pos = step_rec.positions[drone.drone_id]
            curr_time_ms = step_time_ms(step_rec, duration_ms)
            hop_ms = max(1, curr_time_ms - prev_time_ms)
            prev_time_ms = curr_time_ms
            entry = {
                "x": round(step_pos[0], 4),
                "y": round(step_pos[1], 4),
                "z": round(step_pos[2], 4),
                "durationMs": hop_ms,
            }
            if (
                path
                and path[-1]["x"] == entry["x"]
                and path[-1]["y"] == entry["y"]
                and path[-1]["z"] == entry["z"]
            ):
                path[-1]["durationMs"] += hop_ms
            else:
                path.append(entry)

        drone_entry = {
            "id": f"drone-{drone.drone_id + 1}",
            "name": f"Drone {drone.drone_id + 1}",
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
    show_dicts: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Wrap ready-made per-drone show dicts as ``__show_upload`` payloads.

    Uses the exact show dicts that are uploaded to the UAVs / saved to disk,
    so the ``shows`` field of the API response is always identical to what
    the drones actually receive.
    """
    return [
        {
            "mission": {
                "id": f"path-planner-drone-{idx + 1}",
                "numDrones": 1,
            },
            **show_dict,
        }
        for idx, show_dict in enumerate(show_dicts)
    ]


def _max_geofence_altitude(show_dicts: list[dict[str, Any]]) -> float:
    max_alt = 30.0
    for show_dict in show_dicts:
        fence = show_dict.get("geofence") or {}
        try:
            max_alt = max(max_alt, float(fence.get("maxAltitude", 0.0)))
        except (TypeError, ValueError):
            continue
    return math.ceil(max_alt)


def skyc_bytes_from_show_dicts(show_dicts: list[dict[str, Any]]) -> bytes:
    """Pack a list of per-drone show dicts into a ``.skyc`` ZIP for Viewer.

    The validation block reuses the same limits the planner enforces
    (velocity caps from ``converter``, minimum distance from the collision
    envelope), so the Viewer never flags a show the planner considers safe
    and vice versa.
    """
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
                "maxAltitude": _max_geofence_altitude(show_dicts),
                "maxVelocityXY": MAX_VELOCITY_XY,
                "maxVelocityZ": MAX_VELOCITY_Z,
                "minDistance": MIN_DISTANCE_FOR_VALIDATION,
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
