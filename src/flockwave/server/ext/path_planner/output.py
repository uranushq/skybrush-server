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

from typing import List

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
