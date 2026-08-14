"""Proxy helpers for the JR boards' on-board HTTP API (port 80).

Endpoints (see ``JR_precise_timing/firmware/dr_node/main/http_api.c``)::

    POST http://<ip>/reboot       -> {"ok": true, "action": "reboot"}
    POST http://<ip>/redownload   -> {"ok": true, "action": "redownload"}

Health is no longer part of this on-board HTTP API -- the board pushes its
status over UDP instead (see ``health_udp.py``), since polling ~30 boards'
``/health`` synchronously over HTTP was the source of the "no telem"
bottleneck this replaced.
"""

from __future__ import annotations

from typing import Any

import httpx

__all__ = ("JRBoardError", "post_reboot", "post_redownload")


class JRBoardError(RuntimeError):
    """Raised when a request to a JR board fails."""


async def _request(method: str, ip: str, path: str, *, timeout: float) -> Any:
    url = f"http://{ip}{path}"
    try:
        async with httpx.AsyncClient(timeout=timeout) as client:
            response = await client.request(method, url)
    except httpx.HTTPError as exc:
        raise JRBoardError(f"could not reach JR board at {url}: {exc}") from exc

    if response.status_code // 100 != 2:
        raise JRBoardError(
            f"JR board {url} returned HTTP {response.status_code}: "
            f"{response.text[:200]}"
        )

    try:
        return response.json()
    except ValueError:
        # Some firmware actions reply with a bare string; surface it as-is.
        return {"raw": response.text}


async def post_reboot(ip: str, *, timeout: float = 5.0) -> Any:
    """Trigger a remote reboot of a JR board."""
    return await _request("POST", ip, "/reboot", timeout=timeout)


async def post_redownload(ip: str, *, timeout: float = 5.0) -> Any:
    """Trigger a re-download of the show file on a JR board (ARM_WAIT only)."""
    return await _request("POST", ip, "/redownload", timeout=timeout)
