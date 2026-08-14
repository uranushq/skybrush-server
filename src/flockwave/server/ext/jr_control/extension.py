"""JR-board control extension — REST API for ARM broadcast + board control.

Endpoints (mounted under ``/api/v1/jr`` by default)
---------------------------------------------------
POST ``/arm``                broadcast a JRPT ARM (or other) sync packet over UDP
GET  ``/health/<ip>``        latest status pushed by the board over health UDP
POST ``/reboot/<ip>``        proxy a board's ``POST /reboot``
POST ``/redownload/<ip>``    proxy a board's ``POST /redownload``

Health is received passively: boards push their status over UDP (port
``health_port``, default 16550) instead of being polled over HTTP, so
``/health/<ip>`` just returns whatever was last received -- see
``health_udp.py``.

``POST /arm`` request body (JSON, all optional)::

    {
      "startIn": 5.0,        // seconds from now -> start_time_us
      "fpsNum": 30, "fpsDen": 1,
      "frameCount": 300,
      "seq": 1,
      "repeat": 5,
      "interval": 0.05,      // < 0.1 enforced
      "target": "255.255.255.255",
      "port": 8765,
      "cmd": "ARM"           // ARM | ABORT | PING | LOAD_FILE
    }
"""

from __future__ import annotations

from contextlib import ExitStack
from functools import partial
from logging import Logger
from typing import TYPE_CHECKING, Optional

from quart import Blueprint, jsonify, request
from trio import to_thread

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .arm import broadcast_arm
from .health import JRBoardError, post_reboot, post_redownload
from .health_udp import get_cached_health, run_listener as run_health_udp_listener

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("jr_control", __name__)

app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None


@blueprint.route("/arm", methods=["POST"])
async def arm_endpoint():
    """Broadcast an ARM (or other) JRPT sync packet over UDP."""
    body = await request.get_json(silent=True) or {}

    kwargs = dict(
        start_in=float(body.get("startIn", 5.0)),
        fps_num=int(body.get("fpsNum", 30)),
        fps_den=int(body.get("fpsDen", 1)),
        frame_count=int(body.get("frameCount", 300)),
        show_id=int(body.get("showId", 1)),
        file_id=int(body.get("fileId", 99)),
        seq=int(body.get("seq", 1)),
        repeat=int(body.get("repeat", 5)),
        interval=float(body.get("interval", 0.05)),
        # None -> fan out a limited broadcast across every local interface so the
        # whole subnet receives the ARM; pass an explicit address to override.
        target=body.get("target") or None,
        port=int(body.get("port", 8765)),
        cmd=str(body.get("cmd", "ARM")),
    )

    try:
        # Blocking socket sends + sleeps -> run off the event loop.
        summary = await to_thread.run_sync(partial(broadcast_arm, **kwargs))
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except OSError as exc:
        return jsonify({"error": f"broadcast failed: {exc}"}), 502

    if log:
        log.info(
            f"JR ARM broadcast: cmd={summary['cmd']} "
            f"targets={summary['targets']} sent={summary['sent']} "
            f"startTimeUs={summary['startTimeUs']}"
        )
    return jsonify({"success": True, **summary})


@blueprint.route("/health/<ip>", methods=["GET"])
async def health_endpoint(ip: str):
    try:
        return jsonify(get_cached_health(ip))
    except JRBoardError as exc:
        return jsonify({"error": str(exc)}), 502


@blueprint.route("/reboot/<ip>", methods=["POST"])
async def reboot_endpoint(ip: str):
    try:
        return jsonify(await post_reboot(ip))
    except JRBoardError as exc:
        return jsonify({"error": str(exc)}), 502


@blueprint.route("/redownload/<ip>", methods=["POST"])
async def redownload_endpoint(ip: str):
    try:
        return jsonify(await post_redownload(ip))
    except JRBoardError as exc:
        return jsonify({"error": str(exc)}), 502


class JRControlExtension(Extension):
    """Skybrush server extension that exposes the JR-board control API."""

    async def run(self, app, configuration, logger):  # type: ignore[override]
        route = configuration.get("route", "/api/v1/jr")
        health_host = configuration.get("health_host", "")
        health_port = int(configuration.get("health_port", 16550))
        http_server = app.import_api("http_server")

        with ExitStack() as stack:
            stack.enter_context(overridden(globals(), app=app, log=logger))
            stack.enter_context(http_server.mounted(blueprint, path=route))
            logger.info(f"JR-control API mounted at {route}")
            # Blocks forever, receiving boards' health-UDP pushes.
            await run_health_udp_listener(health_host, health_port, log=logger)


construct = JRControlExtension

description = "REST API for JR LED-board control (ARM broadcast, health, reboot)"

schema = {
    "properties": {
        "route": {
            "type": "string",
            "title": "URL root",
            "description": (
                "URL prefix where the JR-control endpoints are mounted within "
                "the HTTP namespace of the server"
            ),
            "default": "/api/v1/jr",
        },
        "health_host": {
            "type": "string",
            "title": "Health UDP listen host",
            "description": (
                "IP address to listen on for JR boards' health UDP pushes. "
                "Use an empty string to listen on all interfaces."
            ),
            "default": "",
        },
        "health_port": {
            "type": "integer",
            "title": "Health UDP listen port",
            "description": (
                "UDP port that JR boards push their health status to (must "
                "match CFG_HEALTH_UDP_PORT in the board firmware's config.h)."
            ),
            "minimum": 1,
            "maximum": 65535,
            "default": 16550,
        },
    }
}
