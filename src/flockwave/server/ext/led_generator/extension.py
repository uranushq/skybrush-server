"""LED-generator extension — REST API that compiles an LED show into per-drone
``.bin`` files and uploads them to the external download server.

Endpoint
--------
POST ``/api/v1/led/compile``

Request body (JSON) — the ``LedShow`` model authored in Skybrush Live
(see :mod:`.compile` for the full schema)::

    {
      "ledsPerDrone": 4,
      "droneRows": 3,
      "droneCols": 7,
      "droneCount": 21,
      "fps": 15,
      "boards": [
        {"id": "...", "name": "...", "pixels": [[r,g,b], ...],
         "startSec": 0.0, "durationSec": 2.0}
      ],
      "upload": true,                 // optional, default true
      "upload_url": "http://..."      // optional, overrides the default server
    }

Response body (JSON)::

    {
      "success": true,
      "totalFrames": 30,
      "fps": 15,
      "tileWidth": 4,
      "tileHeight": 4,
      "droneCount": 21,
      "tiles": [
        {"droneIndex": 0, "bytes": 736, "filename": "file12.bin",
         "url": "/download/12", "message": "File uploaded successfully"},
        ...
      ]
    }
"""

from __future__ import annotations

from contextlib import ExitStack
from logging import Logger
from typing import TYPE_CHECKING, Optional

from quart import Blueprint, jsonify, request
from trio import sleep_forever

from flockwave.server.ext.base import Extension
from flockwave.server.utils import overridden

from .compile import CompileError, compile_show
from .upload import DEFAULT_UPLOAD_URL, UploadError, upload_bin

if TYPE_CHECKING:
    from flockwave.server.app import SkybrushServer

blueprint = Blueprint("led_generator", __name__)

# Injected at runtime via `overridden(globals(), ...)` while the extension runs.
app: Optional["SkybrushServer"] = None
log: Optional[Logger] = None
default_upload_url: str = DEFAULT_UPLOAD_URL


@blueprint.route("/compile", methods=["POST"])
async def compile_endpoint():
    """Compile an LED show and (optionally) upload the per-drone files."""
    body = await request.get_json(silent=True)
    if body is None:
        return jsonify({"error": "Request body must be valid JSON"}), 400

    try:
        compiled = compile_show(body)
    except CompileError as exc:
        return jsonify({"error": str(exc)}), 400

    do_upload = bool(body.get("upload", True))
    upload_url = str(body.get("upload_url", default_upload_url))

    tiles: list[dict] = []
    for per_drone in compiled.bins:
        entry: dict = {
            "droneIndex": per_drone.drone_index,
            "bytes": len(per_drone.data),
        }
        if do_upload:
            # The download server assigns the real filename/id; this is a hint.
            filename = f"drone{per_drone.drone_index + 1}.bin"
            try:
                result = await upload_bin(filename, per_drone.data, url=upload_url)
                entry["filename"] = result.get("filename")
                entry["url"] = result.get("url")
                entry["message"] = result.get("message")
            except UploadError as exc:
                entry["error"] = str(exc)
                if log:
                    log.error(
                        f"Upload failed for drone {per_drone.drone_index}: {exc}"
                    )
        tiles.append(entry)

    upload_errors = sum(1 for t in tiles if "error" in t)
    if log:
        log.info(
            f"Compiled LED show: {len(compiled.bins)} drone(s), "
            f"{compiled.total_frames} frame(s) @ {compiled.fps} fps"
            + (
                f"; {upload_errors} upload error(s)"
                if do_upload and upload_errors
                else ""
            )
        )

    return jsonify(
        {
            "success": do_upload is False or upload_errors == 0,
            "totalFrames": compiled.total_frames,
            "fps": compiled.fps,
            "tileWidth": compiled.tile_width,
            "tileHeight": compiled.tile_height,
            "droneCount": len(compiled.bins),
            "uploaded": do_upload,
            "tiles": tiles,
        }
    )


class LedGeneratorExtension(Extension):
    """Skybrush server extension that exposes the LED-generator API."""

    async def run(self, app, configuration, logger):  # type: ignore[override]
        route = configuration.get("route", "/api/v1/led")
        upload_url = configuration.get("upload_url", DEFAULT_UPLOAD_URL)
        http_server = app.import_api("http_server")

        with ExitStack() as stack:
            stack.enter_context(
                overridden(
                    globals(), app=app, log=logger, default_upload_url=upload_url
                )
            )
            stack.enter_context(http_server.mounted(blueprint, path=route))
            logger.info(f"LED-generator API mounted at {route}/compile")
            await sleep_forever()


construct = LedGeneratorExtension

description = "REST API that compiles LED shows into per-drone .bin files"

schema = {
    "properties": {
        "route": {
            "type": "string",
            "title": "URL root",
            "description": (
                "URL prefix where the LED-generator endpoints are mounted "
                "within the HTTP namespace of the server"
            ),
            "default": "/api/v1/led",
        },
        "upload_url": {
            "type": "string",
            "title": "Download-server upload URL",
            "description": (
                "Where compiled .bin files are POSTed for the LED boards to "
                "later download"
            ),
            "default": DEFAULT_UPLOAD_URL,
        },
    }
}
