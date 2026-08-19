"""JR-board control extension — REST API for ARM broadcast + board control.

Endpoints (mounted under ``/api/v1/jr`` by default)
---------------------------------------------------
POST ``/arm``                broadcast a JRPT ARM (or other) sync packet over UDP
GET  ``/health/<ip>``        latest status pushed by the board over health UDP
POST ``/reboot/<ip>``        send a UDP reboot command, wait for the board's ack
POST ``/redownload/<ip>``    send a UDP redownload command, wait for the board's ack
POST ``/led/<ip>``           light the board's LEDs solid (wiring check) / turn off

The board has no HTTP server at all -- everything in this extension talks to
it over UDP on ``health_port`` (default 16550, must match the firmware's
``CFG_HEALTH_UDP_PORT``): boards push their status passively (see
``health_udp.py``, consumed by ``/health/<ip>``), and ``/reboot``,
``/redownload`` actively send a small JSON command to that same port and
wait for the board's ack (see ``commands.py``).

``POST /arm`` request body (JSON, all optional)::

    {
      "startIn": 5.0,        // seconds from now -> start_time_us
      "seq": 1,
      "repeat": 5,
      "interval": 0.05,      // < 0.1 enforced
      "target": "255.255.255.255",
      "port": 8765,
      "cmd": "ARM"           // ARM | ABORT | PING | LOAD_FILE
    }

The show's length and frame rate are deliberately *not* accepted here. The
firmware plays exactly the ``frame_count`` the ARM packet carries and never
checks it against the file it downloaded, so a controller-side guess silently
overrides the real show. The boards report the true numbers in their health
pushes and ``derive_show_params`` reads them back, which leaves ``startIn``
as the only playback knob a caller owns. An ARM is refused with 409 while no
board has a show loaded to read them from.
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
from .commands import JRBoardError, post_led, post_reboot, post_redownload
from .health_udp import (
    ShowParams,
    derive_show_params,
    get_cached_health,
    run_listener as run_health_udp_listener,
)

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("jr_control", __name__)

app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None
#: UDP port boards listen on for reboot/redownload commands -- set from the
#: extension's own `health_port` configuration when it starts.
health_port: int = 16550


@blueprint.route("/arm", methods=["POST"])
async def arm_endpoint():
    """Broadcast an ARM (or other) JRPT sync packet over UDP."""
    body = await request.get_json(silent=True) or {}
    cmd = str(body.get("cmd", "ARM"))

    kwargs = dict(
        start_in=float(body.get("startIn", 5.0)),
        show_id=int(body.get("showId", 1)),
        file_id=int(body.get("fileId", 99)),
        seq=int(body.get("seq", 1)),
        repeat=int(body.get("repeat", 5)),
        interval=float(body.get("interval", 0.05)),
        # None -> fan out a limited broadcast across every local interface so the
        # whole subnet receives the ARM; pass an explicit address to override.
        target=body.get("target") or None,
        port=int(body.get("port", 8765)),
        cmd=cmd,
    )

    # Length and frame rate come from the boards, never from the request -- see
    # this module's docstring. Only ARM carries them: the firmware drops every
    # other command before it reads those fields (``on_sync_pkt`` returns early
    # unless cmd == ARM), so a PING still works with no show loaded anywhere.
    show: Optional[ShowParams] = None
    if cmd == "ARM":
        try:
            show = derive_show_params()
        except JRBoardError as exc:
            return jsonify({"error": str(exc)}), 409
        kwargs.update(
            frame_count=show.frame_count,
            fps_num=show.fps_num,
            fps_den=show.fps_den,
        )

    try:
        # Blocking socket sends + sleeps -> run off the event loop.
        summary = await to_thread.run_sync(partial(broadcast_arm, **kwargs))
    except ValueError as exc:
        return jsonify({"error": str(exc)}), 400
    except OSError as exc:
        return jsonify({"error": f"broadcast failed: {exc}"}), 502

    detail = ""
    if show is not None:
        summary["frameCountPerBoard"] = show.per_board
        detail = f" frames={show.frame_count} fps={show.fps_num}/{show.fps_den}"
        if show.disagreement:
            # Not fatal -- the longest show wins and shorter boards just hold
            # their last frame -- but it means the fleet was not uploaded from
            # one show, which is worth seeing.
            summary["frameCountMismatch"] = True
            detail += f" (boards disagree: {show.per_board})"

    if log:
        log.info(
            f"JR ARM broadcast: cmd={summary['cmd']} "
            f"targets={summary['targets']} sent={summary['sent']} "
            f"startTimeUs={summary['startTimeUs']}{detail}"
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
        return jsonify(await post_reboot(ip, port=health_port))
    except JRBoardError as exc:
        return jsonify({"error": str(exc)}), 502


@blueprint.route("/redownload/<ip>", methods=["POST"])
async def redownload_endpoint(ip: str):
    try:
        return jsonify(await post_redownload(ip, port=health_port))
    except JRBoardError as exc:
        return jsonify({"error": str(exc)}), 502


@blueprint.route("/led/<ip>", methods=["POST"])
async def led_endpoint(ip: str):
    """Light one board's LEDs solid, or turn them off.

    Body (JSON, all optional)::

        {"red": 255, "green": 0, "blue": 0, "white": 0}
        {"off": true}

    Channels are clamped to 0..255. The reply echoes what the board reports it
    actually applied, e.g. ``{"ok": true, "action": "led 255,0,0,0"}``.
    """
    body = await request.get_json(silent=True) or {}
    try:
        kwargs = dict(
            off=bool(body.get("off", False)),
            red=int(body.get("red", 255)),
            green=int(body.get("green", 255)),
            blue=int(body.get("blue", 255)),
            white=int(body.get("white", 0)),
        )
    except (TypeError, ValueError):
        return jsonify({"error": "'red'/'green'/'blue'/'white' must be integers"}), 400

    try:
        return jsonify(await post_led(ip, port=health_port, **kwargs))
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
            stack.enter_context(
                overridden(globals(), app=app, log=logger, health_port=health_port)
            )
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
