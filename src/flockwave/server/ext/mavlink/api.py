"""REST API for MAVLink flight-mode parameter management."""

from __future__ import annotations

from logging import Logger
from typing import TYPE_CHECKING, Any

from quart import Blueprint, jsonify, request

from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.flight_modes import (
    SHOW_MODE_CUSTOM_MODE,
    SHOW_MODE_FLIGHT_MODE_PARAM_NAMES,
    configure_show_mode_flight_mode_slots,
    flight_mode_slot_errors,
    read_show_mode_flight_mode_slots,
)
from flockwave.server.model.uav import UAV, is_uav

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("mavlink_api", __name__)

app: SkybrushServer | None = None
log: Logger | None = None


def _resolve_mavlink_uavs(
    requested_ids: list[str] | None,
) -> tuple[list[tuple[str, MAVLinkUAV]], list[str]]:
    if app is None:
        raise RuntimeError("Server app not available")

    resolved: list[tuple[str, MAVLinkUAV]] = []
    skipped: list[str] = []

    if requested_ids is None:
        candidate_ids = sorted(app.object_registry.ids_by_type(UAV))
    else:
        candidate_ids = requested_ids

    for uav_id in candidate_ids:
        uav = app.object_registry.find_by_id(uav_id)
        if uav is None or not is_uav(uav):
            skipped.append(uav_id)
            continue
        if not isinstance(uav, MAVLinkUAV):
            skipped.append(uav_id)
            continue
        resolved.append((uav_id, uav))

    return resolved, skipped


async def apply_flight_mode_slots(
    *,
    mode: float = SHOW_MODE_CUSTOM_MODE,
    requested_ids: list[str] | None = None,
) -> tuple[dict[str, Any], int]:
    """Apply FLTMODE5/FLTMODE6 updates and return (payload, HTTP status)."""
    uavs, skipped = _resolve_mavlink_uavs(requested_ids)
    if not uavs:
        return (
            {
                "error": "No MAVLink UAVs available",
                "skipped": skipped,
            },
            404,
        )

    results: dict[str, dict[str, Any]] = {}
    for uav_id, uav in uavs:
        results[uav_id] = await configure_show_mode_flight_mode_slots(
            uav, mode=mode
        )
        if log:
            log.info(
                "Flight mode slots updated on %s: %s",
                uav_id,
                results[uav_id],
            )

    payload: dict[str, Any] = {
        "mode": mode,
        "parameters": list(SHOW_MODE_FLIGHT_MODE_PARAM_NAMES),
        "results": results,
        "skipped": skipped,
    }
    errors = flight_mode_slot_errors(results)
    if errors:
        payload["errors"] = errors
        return payload, 207
    return payload, 200


async def read_flight_mode_slots(
    requested_ids: list[str] | None = None,
) -> tuple[dict[str, Any], int]:
    """Read FLTMODE5/FLTMODE6 values and return (payload, HTTP status)."""
    uavs, skipped = _resolve_mavlink_uavs(requested_ids)
    if not uavs:
        return (
            {
                "error": "No MAVLink UAVs available",
                "skipped": skipped,
            },
            404,
        )

    results: dict[str, dict[str, Any]] = {}
    for uav_id, uav in uavs:
        results[uav_id] = await read_show_mode_flight_mode_slots(uav)

    payload: dict[str, Any] = {
        "parameters": list(SHOW_MODE_FLIGHT_MODE_PARAM_NAMES),
        "results": results,
        "skipped": skipped,
    }
    errors = flight_mode_slot_errors(results)
    if errors:
        payload["errors"] = errors
    return payload, 200


@blueprint.route("/flight-modes", methods=["GET"])
async def get_flight_mode_slots():
    """Read FLTMODE5 and FLTMODE6 from connected MAVLink UAVs."""
    requested = request.args.getlist("uavs") or None

    try:
        payload, status = await read_flight_mode_slots(requested)
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503
    return jsonify(payload), status


@blueprint.route("/flight-modes", methods=["POST"])
async def set_flight_mode_slots():
    """Set FLTMODE5 and FLTMODE6 on connected MAVLink UAVs.

    Request body (JSON)::

        {
          "mode": 127,
          "uavs": ["1", "2"]
        }

    *mode* defaults to 127 (Skybrush DRONE_SHOW). When *uavs* is omitted, all
    connected MAVLink UAVs are updated.
    """
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

    mode = body.get("mode", SHOW_MODE_CUSTOM_MODE)
    if not isinstance(mode, (int, float)):
        return jsonify({"error": "'mode' must be a number"}), 400
    if mode < 0 or mode > 255:
        return jsonify({"error": "'mode' must be between 0 and 255"}), 400

    requested = body.get("uavs")
    if requested is not None and not isinstance(requested, list):
        return jsonify({"error": "'uavs' must be a list of UAV IDs"}), 400

    try:
        payload, status = await apply_flight_mode_slots(
            mode=float(mode),
            requested_ids=requested,
        )
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 503
    return jsonify(payload), status
