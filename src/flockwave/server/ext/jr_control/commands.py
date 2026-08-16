"""UDP control commands sent to a JR board (reboot / redownload / led).

Replaces the old ``POST http://<ip>/reboot`` and ``POST http://<ip>/redownload``
HTTP calls -- the board no longer runs an HTTP server at all. Commands are
sent as small UDP datagrams to the board's health/control port (see
``health_udp.py``, default 16550) and the board acks back over the same
socket with ``{"ok": true, "action": "<what it did>"}``
(see ``JR_precise_timing/firmware/dr_node/main/health_udp.c``).

The board does not parse the body -- ``handle_command`` substring-matches it,
in the order ``redownload`` -> ``reboot`` -> ``led``. So a body must never
contain a keyword that outranks the one it means: ``led`` bodies in particular
must not carry the word "reboot" anywhere.

The board ignores commands while it is ``PLAYING`` a show (mirrors the old
behaviour, where its HTTP server was stopped for the same window), so a
command sent during playback times out here exactly like an unreachable
board would. Every other state -- including ``GNSS_WAIT_PPS`` -- accepts them.
"""

from __future__ import annotations

import json
import socket
from functools import partial
from typing import Any

from trio import to_thread

__all__ = (
    "JRBoardError",
    "format_led_body",
    "post_led",
    "post_reboot",
    "post_redownload",
    "send_command",
    "send_raw_command",
)


class JRBoardError(RuntimeError):
    """Raised when a command to a JR board fails."""


def _send_and_wait(ip: str, port: int, body: bytes, label: str, timeout: float) -> Any:
    """Blocking send-then-wait-for-ack; must run off the event loop."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(body, (ip, port))
        data, _addr = sock.recvfrom(1024)
    except OSError as exc:
        raise JRBoardError(
            f"no ack from JR board {ip}:{port} for '{label}' within {timeout}s "
            f"(unreachable, or busy PLAYING a show): {exc}"
        ) from exc
    finally:
        sock.close()

    try:
        return json.loads(data)
    except ValueError:
        return {"raw": data.decode("utf-8", "replace")}


async def send_command(ip: str, cmd: str, *, port: int, timeout: float = 3.0) -> Any:
    """Sends a control command to a JR board over UDP and waits for its ack.

    The command travels as ``{"cmd": "<cmd>"}``; the board only substring-matches
    the body, so the JSON wrapper is incidental but harmless.
    """
    body = json.dumps({"cmd": cmd}).encode("utf-8")
    return await to_thread.run_sync(
        partial(_send_and_wait, ip, port, body, cmd, timeout)
    )


async def send_raw_command(
    ip: str, body: str, *, port: int, timeout: float = 3.0
) -> Any:
    """Sends a verbatim plain-text command body (no JSON wrapper) and waits.

    Used for commands whose *arguments* are parsed out of the body by the
    firmware, where a JSON wrapper's punctuation would sit in the middle of the
    text the board scans.
    """
    return await to_thread.run_sync(
        partial(_send_and_wait, ip, port, body.encode("utf-8"), body, timeout)
    )


async def post_reboot(ip: str, *, port: int, timeout: float = 3.0) -> Any:
    """Trigger a remote reboot of a JR board."""
    return await send_command(ip, "reboot", port=port, timeout=timeout)


async def post_redownload(ip: str, *, port: int, timeout: float = 3.0) -> Any:
    """Trigger a re-download of the show file on a JR board (ARM_WAIT only)."""
    return await send_command(ip, "redownload", port=port, timeout=timeout)


def _clamp_u8(value: Any) -> int:
    """Clamp one channel into 0..255, mirroring the firmware's ``clamp_u8``."""
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"LED channel must be an integer, got {value!r}") from exc
    return max(0, min(255, number))


def format_led_body(
    red: int = 255,
    green: int = 255,
    blue: int = 255,
    white: int = 0,
    *,
    off: bool = False,
) -> str:
    """Build the plain-text ``led`` body for :func:`post_led`.

    The firmware (``handle_led`` in ``health_udp.c``) scans the text *after* the
    first ``"led"``: it checks for ``"off"`` first, otherwise ``sscanf``-s four
    integers and falls back to full white when it reads fewer than three. Two
    consequences shape what we emit:

    * ``off`` is its own word, never mixed with numbers.
    * every colour sends all four channels explicitly, so the "fewer than three
      numbers" fallback can never fire and the board lights exactly what was
      asked for. Black is ``led 0 0 0 0``, which is the same result as ``off``.

    The body is deliberately *not* JSON: ``sscanf`` would otherwise have to step
    over the wrapper's quotes and braces.
    """
    if off:
        return "led off"
    return (
        f"led {_clamp_u8(red)} {_clamp_u8(green)} "
        f"{_clamp_u8(blue)} {_clamp_u8(white)}"
    )


async def post_led(
    ip: str,
    *,
    port: int,
    red: int = 255,
    green: int = 255,
    blue: int = 255,
    white: int = 0,
    off: bool = False,
    timeout: float = 3.0,
) -> Any:
    """Light a JR board's LEDs solid (wiring check), or turn them off.

    Ignored while the board is ``PLAYING`` (the command never reaches the LED
    driver, so this times out like an unreachable board); any other state,
    including ``GNSS_WAIT_PPS``, accepts it. A lit board stays lit until it is
    turned off or the next show's player overwrites it.
    """
    body = format_led_body(red, green, blue, white, off=off)
    return await send_raw_command(ip, body, port=port, timeout=timeout)
