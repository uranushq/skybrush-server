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
from copy import deepcopy
from json import dumps
from logging import Logger
from math import ceil
from pathlib import Path
from typing import TYPE_CHECKING, Optional

from quart import Blueprint, Response, jsonify, request
from trio import sleep_forever

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .converter import build_show_dicts, save_skyb_files
from .drone import Drone
from .output import build_output, build_show_specifications, build_skyc_bytes
from .solver import (
    COLLISION_X,
    COLLISION_Y,
    COLLISION_Z,
    PathSolver,
    SolverResult,
    StepRecord,
)
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

# Module-level globals injected at runtime via `overridden(globals(), ...)`.
# Available only while the extension is loaded.
app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None


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
    has_drone_ids = any(isinstance(point, dict) and "droneId" in point for point in value)
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
                jsonify({"error": f"'phases[{phase_index}].holdMs' must be an integer"}),
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


def _phase_point_drone_index(point: dict, fallback_index: int, num_drones: int) -> int | None:
    drone_id = point.get("droneId", point.get("id"))
    if drone_id is None:
        return fallback_index if fallback_index < num_drones else None
    index = _drone_index_from_id(drone_id)
    if index is None or not (0 <= index < num_drones):
        return None
    return index


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
    positions: list[tuple[float, float, float]],
    targets: list[tuple[float, float, float]],
) -> bool:
    return all(
        abs(pos[axis] - target[axis]) < 1e-9
        for pos, target in zip(positions, targets)
        for axis in range(3)
    )


def _find_collision_envelope_pairs(
    label: str,
    points: list[tuple[float, float, float]] | list[list[float]],
) -> list[dict]:
    pairs: list[dict] = []
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            a = points[i]
            b = points[j]
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            dz = abs(a[2] - b[2])
            if dx < COLLISION_X and dy < COLLISION_Y and dz < COLLISION_Z:
                pairs.append(
                    {
                        "label": label,
                        "first": f"drone-{i + 1}",
                        "second": f"drone-{j + 1}",
                        "delta": [round(dx, 4), round(dy, 4), round(dz, 4)],
                    }
                )
    return pairs


def _validate_phase_spacing(
    *,
    initial: list[list[float]],
    phases: list[dict],
):
    violations = _find_collision_envelope_pairs("initial", initial)
    for phase_index, phase in enumerate(phases):
        targets = _phase_targets(phase, len(initial))
        label = str(phase.get("name") or f"phase-{phase_index + 1}")
        violations.extend(
            _find_collision_envelope_pairs(f"phases[{phase_index}]:{label}", targets)
        )

    if not violations:
        return None

    return (
        jsonify(
            {
                "error": "Formation points are too close to each other",
                "code": "FORMATION_SPACING_TOO_CLOSE",
                "details": {
                    "collision_envelope": {
                        "x": COLLISION_X,
                        "y": COLLISION_Y,
                        "z": COLLISION_Z,
                    },
                    "violations": violations,
                },
            }
        ),
        422,
    )


def _plan_formation_phases(
    *,
    initial: list,
    phases: list[dict],
    step_size: float,
    duration_ms: int,
    seed: Optional[int],
    return_to_initial: bool = True,
    min_z: float = 0.0,
) -> tuple[SolverResult, list[dict]]:
    """Plan synced formation phases with collision avoidance between phases."""
    num_drones = len(initial)
    current_positions = [tuple(float(v) for v in point) for point in initial]
    original_initials = list(current_positions)
    combined_steps: list[StepRecord] = [
        StepRecord(
            step=0,
            positions={idx: list(pos) for idx, pos in enumerate(current_positions)},
            collisions=[],
            reverted_drones=[],
            verified=True,
        )
    ]
    phase_summaries: list[dict] = []
    success = True

    for phase_index, phase in enumerate(phases):
        targets = _phase_targets(phase, num_drones)
        phase_success = True
        if _positions_match(current_positions, targets):
            current_positions = list(targets)
        else:
            solver = PathSolver(
                initials=current_positions,
                targets=targets,
                step_size=step_size,
                seed=seed,
                min_z=min_z,
            )
            result = solver.solve()
            step_offset = combined_steps[-1].step

            for record in result.steps[1:]:
                combined_steps.append(
                    StepRecord(
                        step=step_offset + record.step,
                        positions=deepcopy(record.positions),
                        collisions=list(record.collisions),
                        reverted_drones=list(record.reverted_drones),
                        verified=record.verified,
                    )
                )

            current_positions = [
                tuple(result.steps[-1].positions[idx]) for idx in range(num_drones)
            ]
            phase_success = result.success
            success = success and phase_success

        arrival_step = combined_steps[-1].step
        hold_ms = int(phase.get("holdMs", 0))
        hold_steps = ceil(hold_ms / duration_ms) if hold_ms > 0 else 0
        for _ in range(hold_steps):
            combined_steps.append(
                StepRecord(
                    step=combined_steps[-1].step + 1,
                    positions={idx: list(pos) for idx, pos in enumerate(current_positions)},
                    collisions=[],
                    reverted_drones=[],
                    verified=True,
                )
            )

        phase_summaries.append(
            {
                "name": phase.get("name", f"phase-{phase_index + 1}"),
                "arrivalStep": arrival_step,
                "arrivalTimeMs": arrival_step * duration_ms,
                "holdMs": hold_ms,
                "holdSteps": hold_steps,
                "endStep": combined_steps[-1].step,
                "endTimeMs": combined_steps[-1].step * duration_ms,
                "success": phase_success,
            }
        )

    final_targets = _phase_targets(phases[-1], num_drones)
    if return_to_initial:
        final_targets = original_initials
        return_success = True
        if _positions_match(current_positions, final_targets):
            current_positions = list(final_targets)
        else:
            solver = PathSolver(
                initials=current_positions,
                targets=final_targets,
                step_size=step_size,
                seed=seed,
                min_z=min_z,
            )
            result = solver.solve()
            step_offset = combined_steps[-1].step

            for record in result.steps[1:]:
                combined_steps.append(
                    StepRecord(
                        step=step_offset + record.step,
                        positions=deepcopy(record.positions),
                        collisions=list(record.collisions),
                        reverted_drones=list(record.reverted_drones),
                        verified=record.verified,
                    )
                )

            current_positions = [
                tuple(result.steps[-1].positions[idx]) for idx in range(num_drones)
            ]
            return_success = result.success
            success = success and return_success

        phase_summaries.append(
            {
                "name": "return-to-initial",
                "arrivalStep": combined_steps[-1].step,
                "arrivalTimeMs": combined_steps[-1].step * duration_ms,
                "holdMs": 0,
                "holdSteps": 0,
                "endStep": combined_steps[-1].step,
                "endTimeMs": combined_steps[-1].step * duration_ms,
                "success": return_success,
            }
        )

    drones = [
        Drone(drone_id=idx, initial=original_initials[idx], target=final_targets[idx])
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
            success=success,
        ),
        phase_summaries,
    )


# ── REST endpoint ────────────────────────────────────────────────────────


@blueprint.route("/plan", methods=["POST"])
async def plan():
    """Run the path-planning algorithm and return per-drone paths."""
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

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
    # Time per solver step (i.e. for one `step_size` worth of horizontal
    # motion). 5000 ms keeps horizontal speed below the ArduPilot DRONE_SHOW
    # firmware acceptance limits and avoids reload rejection on small shows.
    duration_ms: int = int(body.get("duration_ms", 5000))
    seed: Optional[int] = body.get("seed")

    if step_size <= 0:
        return jsonify({"error": "'step_size' must be > 0"}), 400
    if duration_ms <= 0:
        return jsonify({"error": "'duration_ms' must be > 0"}), 400
    initial_altitude: float = float(
        body.get("initial_altitude", body.get("takeoff_altitude", 2.5))
    )
    if uses_phases and initial_altitude <= 0:
        return jsonify({"error": "'initial_altitude' must be > 0"}), 400
    planning_initial = (
        [
            [point[0], point[1], max(point[2], initial_altitude)]
            for point in initial
        ]
        if uses_phases
        else initial
    )
    if uses_phases:
        spacing_error = _validate_phase_spacing(
            initial=planning_initial,
            phases=phases,
        )
        if spacing_error is not None:
            return spacing_error

    # --- optional: output directory & takeoff_time ---
    takeoff_time: float = float(body.get("takeoff_time", 0.0))

    # Skybrush firmware tends to silently reject very short shows or shows
    # with a zero takeoff time. Force a sensible minimum so the drone has
    # a ground-wait segment in front of the trajectory.
    MIN_TAKEOFF_TIME = 5.0
    if takeoff_time < MIN_TAKEOFF_TIME:
        takeoff_time = MIN_TAKEOFF_TIME

    # Default output dir: parent of the CWD where the server was launched.
    # e.g. if launched from skybrush-server/, output goes to its parent (DCS/).
    default_output = str(Path.cwd().parent)
    output_dir: str = body.get("output_dir", default_output)

    # Whether to auto-upload the generated show to connected UAVs
    auto_upload: bool = body.get("auto_upload", True)

    # Optional coordinate system for the show (required for real drones).
    # Default: local NWU at lon=0, lat=0
    coordinate_system: Optional[dict] = body.get("coordinate_system", None)

    # Optional AMSL reference (in meters). When set, the firmware
    # interprets the trajectory Z as offsets from this absolute altitude
    # instead of as relative-to-home. The Skybrush Live "Outdoor
    # Environment" editor sets the same field as ``amslReference``.
    amsl_reference: Optional[float] = body.get("amsl_reference", None)

    # Resolve the coordinate system *now* (before the solver runs) so that
    # both the on-disk ``.skyb`` files and the per-drone MAVFTP upload use
    # the same origin. Without this the saved files would have origin
    # (0, 0) while the drone receives a real GPS origin — i.e. the saved
    # file could not be replayed against the same drone afterwards.
    if coordinate_system is None:
        coordinate_system = _derive_coordinate_system_from_first_uav()

    # Same trick for the AMSL reference: derive it from the first UAV so
    # the show is uploaded with a real ``amslReference`` rather than the
    # firmware sentinel ``SHOW_ORIGIN_AMSL = -32768000`` (= "no AMSL").
    if amsl_reference is None:
        amsl_reference = _derive_amsl_reference_from_first_uav()

    # --- pre-flight validation -------------------------------------------
    # Validators inspect the request payload together with parameters
    # fetched from the first connected UAV. Any "error"-severity issue
    # blocks the request with HTTP 422; warnings are returned but do not
    # block. Set ``"skip_validation": true`` in the body to bypass.
    skip_validation: bool = bool(body.get("skip_validation", False))
    validation_payload: dict = {"skipped": skip_validation, "issues": []}
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
                    log.warning(f"Could not fetch UAV parameters for validation: {exc}")
                validation_payload["param_fetch_error"] = str(exc)

        ctx = ValidationContext(
            initial=planning_initial,
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

    # --- run solver ---
    if uses_phases:
        result, phase_summaries = _plan_formation_phases(
            initial=planning_initial,
            phases=phases,
            step_size=step_size,
            duration_ms=duration_ms,
            seed=seed,
            return_to_initial=bool(body.get("return_to_initial", True)),
            min_z=initial_altitude,
        )
    else:
        initials = [tuple(p) for p in initial]
        targets = [tuple(p) for p in target]

        solver = PathSolver(
            initials=initials,
            targets=targets,
            step_size=step_size,
            seed=seed,
        )
        result = solver.solve()
        phase_summaries = []

    # --- build response ---
    output = build_output(result, duration_ms)
    output["success"] = result.success
    output["total_steps"] = result.total_steps
    output["validation"] = validation_payload
    if uses_phases:
        output["mode"] = "formation_phases"
        output["phases"] = phase_summaries
        output["ground_initial"] = initial
        output["initial_altitude"] = initial_altitude

    # --- generate & save Skybrush .skyb files ---
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

    # --- auto-upload show to connected UAVs ---
    if auto_upload and result.success:
        upload_results = await _upload_to_connected_uavs(
            result,
            duration_ms=duration_ms,
            takeoff_time=takeoff_time,
            coordinate_system=coordinate_system,
            amsl_reference=amsl_reference,
        )
        output["upload"] = upload_results

    # --- optional: return compiled .skyc or show JSON as download ---
    default_output_type = "skyc" if uses_phases else "path"
    output_type = str(body.get("output", default_output_type)).lower()
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
        log.info(
            f"Auto-derived show origin from {uav_ids[0]}: "
            f"lat={lat}, lon={lon}"
        )
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
        log.info(
            f"Auto-derived AMSL reference from {uav_ids[0]}: {amsl_value:.2f} m"
        )
    return amsl_value


async def _upload_to_connected_uavs(
    result,
    *,
    duration_ms: int = 300,
    takeoff_time: float = 0.0,
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

    # Build per-drone show dicts (same format Skybrush Live would send)
    show_dicts = build_show_dicts(
        result,
        duration_ms=duration_ms,
        takeoff_time=takeoff_time,
        coordinate_system=coordinate_system,
        amsl_reference=amsl_reference,
    )
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
