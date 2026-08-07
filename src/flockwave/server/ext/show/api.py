"""REST API for drone show start readiness."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable

from quart import Blueprint, jsonify

if TYPE_CHECKING:
    from .readiness import ShowStartReadiness

blueprint = Blueprint("show_api", __name__)

get_start_readiness: Callable[[], "ShowStartReadiness"] | None = None
"""Callable bound by the show extension while it is running."""


@blueprint.route("/start-readiness", methods=["GET"])
async def start_readiness():
    """Return whether all mapped show UAVs have start time and authorization.

    Example::

        GET /api/v1/show/start-readiness

    Response body contains ``ready``, per-UAV details under ``uavs``, and lists
    of UAV IDs that are still missing a start time or authorization.
    """
    if get_start_readiness is None:
        return jsonify({"error": "Show extension is not available"}), 503

    payload: dict[str, Any] = dict(get_start_readiness())
    return jsonify(payload), 200
