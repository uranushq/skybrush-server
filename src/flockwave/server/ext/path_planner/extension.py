"""Path-planner extension — REST API for automatic 3D drone path generation.

Endpoint
--------
POST ``/api/v1/path-planner/plan``

Request body (JSON). Either **solver mode** (same shape as ``input.json``)
or **explicit waypoints** from the frontend::

    // Solver (collision avoidance)
    {
      "initial": [[x, y, z], ...],
      "target":  [[x, y, z], ...],
      "step_size":    1.0,
      "duration_ms":  5000,
      "seed":         42
    }

    // Explicit paths (per-drone keyframes)
    {
      "drones": [
        {
          "id": "drone-1",
          "path": [
            { "x": 0, "y": 0, "z": 2.5, "durationMs": 0 },
            { "x": -1, "y": 2, "z": 8, "durationMs": 4000, "holdMs": 5000 },
            { "x": 0, "y": 0, "z": 2.5, "durationMs": 4000 }
          ]
        }
      ]
    }

The first waypoint's ``durationMs`` is ignored (start time). Each following
waypoint's ``durationMs`` is travel time from the previous point; optional
``holdMs`` adds a dwell at that keyframe after arrival.

Per-drone waypoint lists may use the key ``path``, or ``waypoints`` / ``points``
/ ``trajectory`` as aliases.

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
from json import dumps, loads
from logging import Logger
from pathlib import Path
from typing import TYPE_CHECKING, Any, Optional, cast

from quart import Blueprint, Response, jsonify, request
from trio import sleep_forever

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .converter import (
    build_show_dicts,
    build_show_dicts_from_explicit_drone_paths,
    save_skyb_files,
    save_skyb_files_from_explicit_drone_paths,
)
from .output import (
    build_output,
    build_output_from_explicit_drone_paths,
    build_show_specifications,
    build_skyc_bytes,
    build_skyc_bytes_from_explicit_drone_paths,
)
from .solver import PathSolver
from .validators import (
    SEVERITY_ERROR,
    ValidationContext,
    collect_required_params,
    fetch_required_params,
    run_validators,
)

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("path_planner", __name__)


def _coerce_plan_request_body(body: Any) -> None:
    """Normalize client shapes so explicit waypoint mode is detected reliably.

    Handles:

    - ``{ "data": { "drones": ... } }`` and other one-level wrappers
    - A single-key envelope ``{ "planRequest": { "drones": ... } }``
    - ``drones`` as an object map ``{ "drone-1": { "path": [...] } }``
    - ``drones`` sent as a JSON string (double-encoded)
    - ``waypoints`` / ``points`` aliases merged into ``path``
    """
    if not isinstance(body, dict):
        return

    if body.get("drones") is None:
        for wk in (
            "data",
            "payload",
            "body",
            "request",
            "plan",
            "json",
            "params",
            "input",
        ):
            inner = body.get(wk)
            if isinstance(inner, dict) and "drones" in inner:
                body["drones"] = inner["drones"]
                break

    if body.get("drones") is None and len(body) == 1:
        inner = next(iter(body.values()))
        if isinstance(inner, dict) and "drones" in inner:
            merged = dict(inner)
            body.clear()
            body.update(merged)

    raw_drones = body.get("drones")
    if isinstance(raw_drones, str):
        try:
            body["drones"] = loads(raw_drones)
        except Exception:
            pass
        raw_drones = body.get("drones")

    if isinstance(raw_drones, dict) and raw_drones:
        out: list[dict[str, Any]] = []
        for k, v in raw_drones.items():
            if not isinstance(v, dict):
                continue
            ent = cast(dict[str, Any], dict(v))
            ent.setdefault("id", str(k))
            out.append(ent)
        body["drones"] = out

    _normalize_explicit_request_body(body)


def _normalize_explicit_request_body(body: Any) -> None:
    """Merge common frontend aliases into ``drones[].path`` before routing.

    Some clients send ``waypoints`` / ``points`` instead of ``path``, or leave
    ``path`` unset; without this we would not match explicit mode and the API
    would incorrectly require ``initial`` / ``target``.
    """
    if not isinstance(body, dict):
        return
    drones = body.get("drones")
    if not isinstance(drones, list):
        return
    for d in drones:
        if not isinstance(d, dict):
            continue
        d_map = cast(dict[str, Any], d)
        path_val = d_map.get("path")
        if not (isinstance(path_val, list) and len(path_val) > 0):
            for key in ("waypoints", "points", "trajectory"):
                alt = d_map.get(key)
                if isinstance(alt, list) and len(alt) > 0:
                    d_map["path"] = alt
                    break

        initial_raw = None
        for key in ("initial_position", "initialPosition", "initial"):
            if key in d_map:
                initial_raw = d_map.get(key)
                break

        initial_wp: dict[str, float] | None = None
        if isinstance(initial_raw, (list, tuple)) and len(initial_raw) >= 3:
            try:
                initial_wp = {
                    "x": float(initial_raw[0]),
                    "y": float(initial_raw[1]),
                    "z": float(initial_raw[2]),
                }
            except (TypeError, ValueError):
                initial_wp = None
        elif isinstance(initial_raw, dict):
            try:
                initial_wp = {
                    "x": float(initial_raw["x"]),
                    "y": float(initial_raw["y"]),
                    "z": float(initial_raw["z"]),
                }
            except (KeyError, TypeError, ValueError):
                initial_wp = None

        path_any = d_map.get("path")
        if initial_wp is not None and isinstance(path_any, list):
            if len(path_any) == 0:
                d_map["path"] = [{**initial_wp, "durationMs": 0}]
            else:
                d_map["path"] = [{**initial_wp, "durationMs": 0}, *path_any]


def _is_explicit_drone_paths_request(body: dict) -> bool:
    """True when the body is a per-drone waypoint plan (not ``initial``/``target``)."""
    drones = body.get("drones")
    if not isinstance(drones, list) or len(drones) == 0:
        return False
    return all(isinstance(d, dict) for d in drones)


def _validate_explicit_drone_paths_body(body: dict[str, Any]):
    """Return ``(error_response, status)`` if invalid, else ``None``."""
    drones = body.get("drones")
    if not isinstance(drones, list) or len(drones) == 0:
        return (jsonify({"error": "'drones' must be a non-empty array"}), 400)

    for di, d in enumerate(drones):
        if not isinstance(d, dict):
            return (jsonify({"error": f"'drones[{di}]' must be an object"}), 400)
        d_obj = cast(dict[str, Any], d)
        path = d_obj.get("path")
        if not isinstance(path, list) or len(path) < 1:
            return (
                jsonify(
                    {
                        "error": (
                            f"'drones[{di}].path' must be a non-empty array of waypoints"
                        )
                    }
                ),
                400,
            )
        for wi, wp in enumerate(path):
            if not isinstance(wp, dict):
                return (
                    jsonify({"error": f"'drones[{di}].path[{wi}]' must be an object"}),
                    400,
                )
            wp_obj = cast(dict[str, Any], wp)
            for key in ("x", "y", "z"):
                if key not in wp_obj:
                    return (
                        jsonify(
                            {"error": f"'drones[{di}].path[{wi}]' missing {key!r}"}
                        ),
                        400,
                    )
                v = wp_obj[key]
                if not isinstance(v, (int, float)):
                    return (
                        jsonify(
                            {
                                "error": (
                                    f"'drones[{di}].path[{wi}].{key}' must be a number"
                                )
                            }
                        ),
                        400,
                    )
            if wi >= 1:
                if "durationMs" in wp_obj and wp_obj["durationMs"] is not None:
                    try:
                        dms = int(wp_obj["durationMs"])
                    except (TypeError, ValueError):
                        return (
                            jsonify(
                                {
                                    "error": (
                                        f"'drones[{di}].path[{wi}].durationMs' "
                                        "must be an integer"
                                    )
                                }
                            ),
                            400,
                        )
                    if dms < 0:
                        return (
                            jsonify(
                                {
                                    "error": (
                                        f"'drones[{di}].path[{wi}].durationMs' "
                                        "must be >= 0"
                                    )
                                }
                            ),
                            400,
                        )
                if "holdMs" in wp_obj and wp_obj["holdMs"] is not None:
                    try:
                        hms = int(wp_obj["holdMs"])
                    except (TypeError, ValueError):
                        return (
                            jsonify(
                                {
                                    "error": (
                                        f"'drones[{di}].path[{wi}].holdMs' "
                                        "must be an integer"
                                    )
                                }
                            ),
                            400,
                        )
                    if hms < 0:
                        return (
                            jsonify(
                                {
                                    "error": (
                                        f"'drones[{di}].path[{wi}].holdMs' must be >= 0"
                                    )
                                }
                            ),
                            400,
                        )
    return None


def _explicit_path_segment_count(drones: list) -> int:
    """Count travel legs plus optional hold segments (for ``total_steps``)."""
    n = 0
    for d in drones:
        path = d.get("path") or []
        if not isinstance(path, list):
            continue
        for i in range(1, len(path)):
            n += 1
            wp = path[i]
            if isinstance(wp, dict) and int(wp.get("holdMs") or 0) > 0:
                n += 1
    return n


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
        # Fallback for clients that forget `Content-Type: application/json`.
        try:
            raw_data = await request.get_data()
            raw = raw_data.decode("utf-8") if isinstance(raw_data, bytes) else raw_data
            body = loads(raw) if raw else None
        except Exception:
            body = None

    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

    # Accept top-level list payloads as shorthand for {"drones": [...]}
    if isinstance(body, list):
        body = {"drones": body}

    _coerce_plan_request_body(body)
    explicit_paths = _is_explicit_drone_paths_request(body)

    # --- optional parameters (shared) ---
    step_size: float = float(body.get("step_size", 1.0))
    duration_ms: int = int(body.get("duration_ms", 5000))
    seed: Optional[int] = body.get("seed")

    if step_size <= 0:
        return jsonify({"error": "'step_size' must be > 0"}), 400
    if duration_ms <= 0:
        return jsonify({"error": "'duration_ms' must be > 0"}), 400

    takeoff_time: float = float(body.get("takeoff_time", 0.0))

    MIN_TAKEOFF_TIME = 5.0
    if takeoff_time < MIN_TAKEOFF_TIME:
        takeoff_time = MIN_TAKEOFF_TIME

    default_output = str(Path.cwd().parent)
    output_dir: str = body.get("output_dir", default_output)
    auto_upload: bool = body.get("auto_upload", True)
    coordinate_system: Optional[dict] = body.get("coordinate_system", None)
    amsl_reference: Optional[float] = body.get("amsl_reference", None)

    if coordinate_system is None:
        coordinate_system = _derive_coordinate_system_from_first_uav()

    if amsl_reference is None:
        amsl_reference = _derive_amsl_reference_from_first_uav()

    skip_validation: bool = bool(body.get("skip_validation", False))
    validation_payload: dict = {"skipped": skip_validation, "issues": []}

    # --- Frontend waypoint payload: no collision solver -----------------
    if explicit_paths:
        explicit_err = _validate_explicit_drone_paths_body(body)
        if explicit_err is not None:
            return explicit_err

        drones_payload: list = body["drones"]
        initial: list = []
        target: list = []
        for d in drones_payload:
            path = d["path"]
            p0 = path[0]
            pn = path[-1]
            initial.append([float(p0["x"]), float(p0["y"]), float(p0["z"])])
            target.append([float(pn["x"]), float(pn["y"]), float(pn["z"])])

        if not skip_validation:
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
                        log.warning(
                            f"Could not fetch UAV parameters for validation: {exc}"
                        )
                    validation_payload["param_fetch_error"] = str(exc)

            ctx = ValidationContext(
                initial=initial,
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

        output = build_output_from_explicit_drone_paths(drones_payload)
        output["success"] = True
        output["total_steps"] = _explicit_path_segment_count(drones_payload)
        output["validation"] = validation_payload
        output["mode"] = "explicit_waypoints"

        try:
            saved = await save_skyb_files_from_explicit_drone_paths(
                drones_payload,
                output_dir=output_dir,
                takeoff_time=takeoff_time,
                coordinate_system=coordinate_system,
                amsl_reference=amsl_reference,
            )
            output["skybrush_files"] = saved
            if log:
                log.info(
                    f"Saved {sum(1 for k in saved if not k.startswith('_'))} "
                    f".skyb file(s) and show.json under {output_dir}"
                )
                for drone_id, path in saved.items():
                    log.info(f"  {drone_id}: {path}")
        except Exception as exc:
            output["skybrush_files_error"] = str(exc)
            if log:
                log.error(f"Failed to save .skyb files to {output_dir}: {exc}")

        if auto_upload:
            show_dicts_exp = build_show_dicts_from_explicit_drone_paths(
                drones_payload,
                takeoff_time=takeoff_time,
                coordinate_system=coordinate_system,
                amsl_reference=amsl_reference,
            )
            upload_results = await _upload_to_connected_uavs(
                show_dicts_exp,
                coordinate_system=coordinate_system,
                amsl_reference=amsl_reference,
            )
            output["upload"] = upload_results

        output_type = str(body.get("output", "path")).lower()
        if output_type in ("show", "skyc"):
            output["shows"] = build_show_dicts_from_explicit_drone_paths(
                drones_payload,
                takeoff_time=takeoff_time,
                coordinate_system=coordinate_system,
                amsl_reference=amsl_reference,
            )
            output["format"] = "show-upload-v1"
            should_download = bool(body.get("download", output_type == "skyc"))
            if should_download:
                if output_type == "skyc":
                    response = Response(
                        build_skyc_bytes_from_explicit_drone_paths(
                            drones_payload,
                            takeoff_time=takeoff_time,
                            coordinate_system=coordinate_system,
                            amsl_reference=amsl_reference,
                        ),
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

    # --- Legacy: initial / target + collision solver --------------------
    initial = body.get("initial")
    target = body.get("target")

    if not isinstance(initial, list) or not isinstance(target, list):
        return (
            jsonify(
                {
                    "error": (
                        "'initial' and 'target' must be arrays of [x,y,z], "
                        "or supply a non-empty 'drones' array with per-drone "
                        "'path' (or 'waypoints' / 'points') waypoint lists"
                    )
                }
            ),
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

    for label, arr in [("initial", initial), ("target", target)]:
        for i, pt in enumerate(arr):
            if (
                not isinstance(pt, (list, tuple))
                or len(pt) != 3
                or not all(isinstance(v, (int, float)) for v in pt)
            ):
                return (
                    jsonify({"error": f"'{label}[{i}]' must be [x, y, z] (3 numbers)"}),
                    400,
                )

    if not skip_validation:
        uav_params = {}
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

        ctx = ValidationContext(
            initial=initial,
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

    initials = [tuple(p) for p in initial]
    targets = [tuple(p) for p in target]

    solver = PathSolver(
        initials=initials,
        targets=targets,
        step_size=step_size,
        seed=seed,
    )
    result = solver.solve()

    output = build_output(result, duration_ms)
    output["success"] = result.success
    output["total_steps"] = result.total_steps
    output["validation"] = validation_payload

    try:
        saved = await save_skyb_files(
            result,
            output_dir=output_dir,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
        )
        output["skybrush_files"] = saved
        if log:
            log.info(
                f"Saved {sum(1 for k in saved if not k.startswith('_'))} "
                f".skyb file(s) and show.json under {output_dir}"
            )
            for drone_id, path in saved.items():
                log.info(f"  {drone_id}: {path}")
    except Exception as exc:
        output["skybrush_files_error"] = str(exc)
        if log:
            log.error(f"Failed to save .skyb files to {output_dir}: {exc}")

    if auto_upload and result.success:
        show_dicts = build_show_dicts(
            result,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
        )
        upload_results = await _upload_to_connected_uavs(
            show_dicts,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
        )
        output["upload"] = upload_results

    output_type = str(body.get("output", "path")).lower()
    if output_type in ("show", "skyc"):
        output["shows"] = build_show_specifications(result, duration_ms)
        output["format"] = "show-upload-v1"
        should_download = bool(body.get("download", output_type == "skyc"))
        if should_download:
            if output_type == "skyc":
                response = Response(
                    build_skyc_bytes(
                        result,
                        duration_ms,
                        takeoff_time=takeoff_time,
                        coordinate_system=coordinate_system,
                        amsl_reference=amsl_reference,
                    ),
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


# ── Upload helper ────────────────────────────────────────────────────────


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

    uav_ids = sorted(app.object_registry.ids_by_type(UAV))
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
                "(no GPS fix yet); saved files and uploads will use "
                "origin (0, 0)."
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

    uav_ids = sorted(app.object_registry.ids_by_type(UAV))
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


async def _upload_to_connected_uavs(
    show_dicts: list,
    *,
    coordinate_system: Optional[dict] = None,
    amsl_reference: Optional[float] = None,
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

    # Gather connected UAV IDs, sorted so assignment is deterministic
    uav_ids = sorted(app.object_registry.ids_by_type(UAV))

    if not uav_ids:
        return {"error": "No UAVs connected", "uploaded": 0, "details": {}}

    # If the caller did not specify a coordinate system, derive its origin
    # from the current GPS position of the first connected UAV. Without a
    # real origin, the firmware will reject the show because the resulting
    # waypoints land thousands of km away from the drone.
    if coordinate_system is None:
        coordinate_system = _derive_coordinate_system_from_first_uav()

    # Same for the AMSL reference: without it the firmware uses the
    # sentinel "no AMSL" value and the takeoff altitude check fails.
    if amsl_reference is None:
        amsl_reference = _derive_amsl_reference_from_first_uav()

    num_drones = len(show_dicts)

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

        driver = cast(Any, uav).driver
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
            stack.enter_context(overridden(globals(), app=app, log=logger))
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
