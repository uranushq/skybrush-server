"""UDP listener that receives JR boards' health status pushes.

Replaces the old ``GET http://<ip>/health`` HTTP proxy (see ``health.py``'s
history / ``JR_precise_timing/firmware/dr_node/main/http_api.c``): polling
~30 boards over synchronous HTTP every few seconds created a packet
bottleneck ("no telem") and occasional dropped connections. The board now
pushes its status over UDP instead -- immediately on every state change and
as a periodic heartbeat (see ``main/health_udp.c`` in the firmware) -- and
this module just caches the latest push per board IP for instant lookup.
"""

from __future__ import annotations

import json
from logging import Logger
from time import monotonic
from typing import Any, Optional

from flockwave.networking import create_socket
from trio.socket import SOCK_DGRAM

from .health import JRBoardError

__all__ = ("get_cached_health", "run_listener")

#: source IP -> {"data": <parsed JSON dict>, "received_at": monotonic() at receipt}
_cache: dict[str, dict[str, Any]] = {}

#: a cached entry older than this is treated as "board unreachable", mirroring
#: the old HTTP proxy's behaviour of failing once the board stops answering.
#: Generous vs. the firmware's 5s heartbeat so a couple of missed packets
#: don't flip a live board to "offline".
STALE_AFTER_SECONDS = 15.0


def _record_status(ip: str, payload: dict[str, Any]) -> None:
    _cache[ip] = {"data": payload, "received_at": monotonic()}


def get_cached_health(ip: str, *, stale_after: float = STALE_AFTER_SECONDS) -> Any:
    """Returns the most recent status JSON pushed by the board at ``ip``.

    Raises:
        JRBoardError: if the board never reported in, or its last report is
            older than ``stale_after`` seconds (the board is presumed
            unreachable/powered off, matching the old HTTP proxy's failure
            mode when a board stopped answering).
    """
    entry = _cache.get(ip)
    if entry is None:
        raise JRBoardError(f"no health UDP report received yet from {ip}")

    age = monotonic() - entry["received_at"]
    if age > stale_after:
        raise JRBoardError(
            f"JR board {ip} health report is stale ({age:.1f}s old, "
            f"limit {stale_after:.0f}s) -- board may be unreachable"
        )

    data = dict(entry["data"])
    data["_age_sec"] = round(age, 1)
    return data


async def run_listener(host: str, port: int, *, log: Optional[Logger] = None) -> None:
    """Background task: binds a UDP socket and caches every status push.

    Runs forever (until the extension is unloaded / the socket is closed);
    intended to be the sole long-running body of the extension's ``run()``.
    """
    sock = create_socket(SOCK_DGRAM)
    await sock.bind((host, port))
    if log:
        log.info(f"JR health UDP listener up on {host or '*'}:{port}")

    try:
        while True:
            data, address = await sock.recvfrom(65536)
            ip = address[0]
            try:
                payload = json.loads(data)
            except ValueError:
                if log:
                    log.warning(f"JR health UDP: malformed packet from {ip}, ignored")
                continue

            if not isinstance(payload, dict):
                if log:
                    log.warning(
                        f"JR health UDP: non-object payload from {ip}, ignored"
                    )
                continue

            _record_status(ip, payload)
    finally:
        await sock.aclose()
