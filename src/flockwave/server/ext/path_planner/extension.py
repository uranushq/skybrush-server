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

from contextlib import ExitStack
from logging import Logger
from pathlib import Path
from quart import Blueprint, jsonify, request
from trio import sleep_forever
from typing import Optional, TYPE_CHECKING

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .solver import PathSolver
from .output import build_output
from .converter import build_show_dicts, save_skyb_files

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("path_planner", __name__)

# Module-level globals injected at runtime via `overridden(globals(), ...)`.
# Available only while the extension is loaded.
app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None


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

    # Default output dir: parent of the CWD where the server was launched.
    # e.g. if launched from skybrush-server/, output goes to its parent (DCS/).
    default_output = str(Path.cwd().parent)
    output_dir: str = body.get("output_dir", default_output)

    # Whether to auto-upload the generated show to connected UAVs
    auto_upload: bool = body.get("auto_upload", True)

    # Optional coordinate system for the show (required for real drones).
    # Default: local NWU at lon=0, lat=0
    coordinate_system: Optional[dict] = body.get("coordinate_system", None)

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

    # --- auto-upload show to connected UAVs ---
    if auto_upload and result.success:
        upload_results = await _upload_to_connected_uavs(
            result,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
            coordinate_system=coordinate_system,
        )
        output["upload"] = upload_results

    return jsonify(output)


# ── Upload helper ────────────────────────────────────────────────────────


async def _upload_to_connected_uavs(
    result,
    *,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
    coordinate_system: Optional[dict] = None,
) -> dict:
    """Upload per-drone show specs to connected UAVs.

    Looks up all UAVs currently registered in the server's object registry,
    sorts them by ID, and uploads a show specification to each one
    (in order, matching drone-1 → first UAV, drone-2 → second UAV, …).

    Returns a summary dict describing what happened for each UAV.
    """
    from flockwave.server.model.uav import UAV, is_uav

    global app, log
    if app is None:
        return {"error": "Server app not available"}

    # Build per-drone show dicts (same format Skybrush Live would send)
    show_dicts = build_show_dicts(
        result,
        duration_ms=duration_ms,
        takeoff_time=takeoff_time,
        coordinate_system=coordinate_system,
    )
    num_drones = len(show_dicts)

    # Gather connected UAV IDs, sorted so assignment is deterministic
    uav_ids = sorted(app.object_registry.ids_by_type(UAV))

    if not uav_ids:
        return {"error": "No UAVs connected", "uploaded": 0, "details": {}}

    if len(uav_ids) < num_drones:
        if log:
            log.warning(
                f"Only {len(uav_ids)} UAV(s) connected but path was planned "
                f"for {num_drones} drones — uploading to available UAVs only"
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

        with ExitStack() as stack:
            stack.enter_context(
                overridden(globals(), app=app, log=logger)
            )
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
