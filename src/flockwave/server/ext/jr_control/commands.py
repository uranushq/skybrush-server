"""UDP control commands sent to a JR board (reboot / redownload).

Replaces the old ``POST http://<ip>/reboot`` and ``POST http://<ip>/redownload``
HTTP calls -- the board no longer runs an HTTP server at all. Commands are
sent as small JSON UDP datagrams to the board's health/control port (see
``health_udp.py``, default 16550) and the board acks back over the same
socket with ``{"ok": true, "action": "<cmd>"}``
(see ``JR_precise_timing/firmware/dr_node/main/health_udp.c``).

The board ignores commands while it is ``PLAYING`` a show (mirrors the old
behaviour, where its HTTP server was stopped for the same window), so a
command sent during playback times out here exactly like an unreachable
board would.
"""

from __future__ import annotations

import json
import socket
from functools import partial
from typing import Any

from trio import to_thread

__all__ = ("JRBoardError", "post_reboot", "post_redownload", "send_command")


class JRBoardError(RuntimeError):
    """Raised when a command to a JR board fails."""


def _send_and_wait(ip: str, port: int, cmd: str, timeout: float) -> Any:
    """Blocking send-then-wait-for-ack; must run off the event loop."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(json.dumps({"cmd": cmd}).encode("utf-8"), (ip, port))
        data, _addr = sock.recvfrom(1024)
    except OSError as exc:
        raise JRBoardError(
            f"no ack from JR board {ip}:{port} for '{cmd}' within {timeout}s "
            f"(unreachable, or busy PLAYING a show): {exc}"
        ) from exc
    finally:
        sock.close()

    try:
        return json.loads(data)
    except ValueError:
        return {"raw": data.decode("utf-8", "replace")}


async def send_command(ip: str, cmd: str, *, port: int, timeout: float = 3.0) -> Any:
    """Sends a control command to a JR board over UDP and waits for its ack."""
    return await to_thread.run_sync(partial(_send_and_wait, ip, port, cmd, timeout))


async def post_reboot(ip: str, *, port: int, timeout: float = 3.0) -> Any:
    """Trigger a remote reboot of a JR board."""
    return await send_command(ip, "reboot", port=port, timeout=timeout)


async def post_redownload(ip: str, *, port: int, timeout: float = 3.0) -> Any:
    """Trigger a re-download of the show file on a JR board (ARM_WAIT only)."""
    return await send_command(ip, "redownload", port=port, timeout=timeout)
