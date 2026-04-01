"""Path-planner extension — REST API for automatic 3D drone path generation.

Endpoint
--------
POST ``/api/v1/path-planner/plan``

Request body (JSON) — same shape as the algorithm's ``input.json``::

    {
      "initial": [[x, y, z], ...],
      "target":  [[x, y, z], ...],
      "step_size":    1.0,        // optional, default 1.0
      "duration_ms":  300,        // optional, per-step ms, default 300
      "seed":         42          // optional, for reproducibility
    }

Response body (JSON)::

    {
      "success": true,
      "total_steps": 27,
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

import os
from contextlib import ExitStack
from logging import Logger
from pathlib import Path
from quart import Blueprint, jsonify, request
from trio import sleep_forever
from typing import Optional

from flockwave.server.ext.base import Extension

from .solver import PathSolver
from .output import build_output
from .converter import save_skyb_files

blueprint = Blueprint("path_planner", __name__)


# ── REST endpoint ────────────────────────────────────────────────────────


@blueprint.route("/plan", methods=["POST"])
async def plan():
    """Run the path-planning algorithm and return per-drone paths."""
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

    # --- validate required fields ---
    initial = body.get("initial")
    target = body.get("target")

    if not isinstance(initial, list) or not isinstance(target, list):
        return (
            jsonify({"error": "'initial' and 'target' must be arrays of [x,y,z]"}),
            400,
        )

    if len(initial) == 0:
        return jsonify({"error": "'initial' must not be empty"}), 400

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

    # validate each coordinate is [x, y, z]
    for label, arr in [("initial", initial), ("target", target)]:
        for i, pt in enumerate(arr):
            if (
                not isinstance(pt, (list, tuple))
                or len(pt) != 3
                or not all(isinstance(v, (int, float)) for v in pt)
            ):
                return (
                    jsonify(
                        {"error": f"'{label}[{i}]' must be [x, y, z] (3 numbers)"}
                    ),
                    400,
                )

    # --- optional parameters ---
    step_size: float = float(body.get("step_size", 1.0))
    duration_ms: int = int(body.get("duration_ms", 300))
    seed: Optional[int] = body.get("seed")

    if step_size <= 0:
        return jsonify({"error": "'step_size' must be > 0"}), 400
    if duration_ms <= 0:
        return jsonify({"error": "'duration_ms' must be > 0"}), 400

    # --- optional: output directory & takeoff_time ---
    takeoff_time: float = float(body.get("takeoff_time", 0.0))

    # Default output dir: parent of skybrush-server (i.e. the DCS/ workspace)
    default_output = str(Path(__file__).resolve().parents[6])  # …/DCS/
    output_dir: str = body.get("output_dir", default_output)

    # --- run solver ---
    initials = [tuple(p) for p in initial]
    targets = [tuple(p) for p in target]

    solver = PathSolver(
        initials=initials,
        targets=targets,
        step_size=step_size,
        seed=seed,
    )
    result = solver.solve()

    # --- build response ---
    output = build_output(result, duration_ms)
    output["success"] = result.success
    output["total_steps"] = result.total_steps

    # --- generate & save Skybrush .skyb files ---
    try:
        saved = await save_skyb_files(
            result,
            output_dir=output_dir,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
        )
        output["skybrush_files"] = saved
    except Exception as exc:
        output["skybrush_files_error"] = str(exc)

    return jsonify(output)


# ── Skybrush extension boilerplate ───────────────────────────────────────


class PathPlannerExtension(Extension):
    """Skybrush server extension that exposes the path-planner API."""

    async def run(self, app, configuration, logger):
        route = configuration.get("route", "/api/v1/path-planner")
        http_server = app.import_api("http_server")

        with ExitStack() as stack:
            stack.enter_context(http_server.mounted(blueprint, path=route))
            logger.info(f"Path-planner API mounted at {route}/plan")
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
    }
}
