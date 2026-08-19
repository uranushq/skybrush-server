"""UDP listener that receives JR boards' health status pushes.

Replaces the old ``GET http://<ip>/health`` HTTP proxy (see ``health.py``'s
history / ``JR_precise_timing/firmware/dr_node/main/http_api.c``): polling
~30 boards over synchronous HTTP every few seconds created a packet
bottleneck ("no telem") and occasional dropped connections. The board now
pushes its status over UDP instead -- immediately on every state change and
as a periodic heartbeat (see ``main/health_udp.c`` in the firmware) -- and
this module just caches the latest push per board IP for instant lookup.

Those pushes are also the only place the controller can learn how long the
show actually is, so :func:`derive_show_params` reads the ARM packet's
``frame_count``/``fps`` back out of them rather than letting the caller guess.
"""

from __future__ import annotations

import json
from logging import Logger
from time import monotonic
from typing import Any, NamedTuple, Optional

from flockwave.networking import create_socket
from trio.socket import SOCK_DGRAM

from .commands import JRBoardError

__all__ = (
    "ShowParams",
    "derive_show_params",
    "get_cached_health",
    "live_health_reports",
    "run_listener",
)

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


def live_health_reports(
    *, stale_after: float = STALE_AFTER_SECONDS
) -> dict[str, dict[str, Any]]:
    """Every board's latest status push that is still fresh, keyed by source IP.

    Unlike :func:`get_cached_health` this never raises for an individual board:
    one that has gone quiet simply drops out of the result.
    """
    now = monotonic()
    return {
        ip: entry["data"]
        for ip, entry in _cache.items()
        if now - entry["received_at"] <= stale_after
    }


class ShowParams(NamedTuple):
    """Playback parameters read back from the boards' own health reports."""

    #: Total frames to play -- the largest any board reports.
    frame_count: int
    #: Frame-rate numerator; a show header carries whole fps only.
    fps_num: int
    #: Always 1 -- see :attr:`fps_num`.
    fps_den: int
    #: Frame count each contributing board reported, keyed by IP.
    per_board: dict[str, int]

    @property
    def disagreement(self) -> bool:
        """True when the boards do not all hold a show of the same length."""
        return len(set(self.per_board.values())) > 1


def _positive_int(value: Any) -> Optional[int]:
    """Coerce a JSON number to a positive ``int``; ``None`` if it is neither."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = int(value)
    return number if number > 0 else None


#: The one state in which a board both runs its UDP sync listener and has the
#: show it is about to play already loaded (``stage_arm_wait`` in
#: ``app_state.c``). Boards in any other state cannot act on an ARM.
ARMABLE_STATE = "ARM_WAIT"


def derive_show_params(*, stale_after: float = STALE_AFTER_SECONDS) -> ShowParams:
    """Reads an ARM packet's playback parameters off the boards themselves.

    The firmware takes the show's length and frame rate purely from the ARM
    packet: ``stage_playing`` hands ``frame_count``/``fps_num``/``fps_den``
    straight to ``frame_scheduler_start`` and never compares them against the
    show file the board downloaded. Whatever the controller puts here therefore
    silently overrides the real show -- too small a ``frame_count`` cuts
    playback short, and 1 makes the boards flash a single frame and go dark.

    The boards already publish the authoritative numbers in their health push
    (``show.frames`` / ``show.fps``, filled in from the downloaded file's header
    by ``sd_storage_load_show``), so read them back from there instead of asking
    the operator -- or an editor timeline -- to keep a duplicate in sync.

    Only boards in :data:`ARMABLE_STATE` are consulted, for two reasons: they
    are the only ones whose sync listener is up, so nobody else can act on this
    ARM anyway; and a board that is re-downloading still advertises the
    *previous* cycle's show (``stage_load_show`` only clears ``s_show_loaded``
    once the new file is about to be read), which would otherwise let stale
    numbers through after a show swap.

    Raises:
        JRBoardError: if no board is reporting, if none of them is waiting for
            an ARM with a show loaded, or if they disagree about the frame rate
            (one broadcast cannot serve two rates).
    """
    reports = live_health_reports(stale_after=stale_after)
    if not reports:
        raise JRBoardError(
            "no JR board has pushed a health report in the last "
            f"{stale_after:.0f}s, so the show's frame count and fps are unknown "
            "-- power the boards up and let them report in before arming"
        )

    per_board: dict[str, int] = {}
    fps_by_board: dict[str, int] = {}
    for ip, data in sorted(reports.items()):
        if data.get("state") != ARMABLE_STATE:
            continue
        show = data.get("show")
        if not isinstance(show, dict) or not show.get("loaded"):
            continue
        frames = _positive_int(show.get("frames"))
        fps = _positive_int(show.get("fps"))
        if frames is None or fps is None:
            continue
        per_board[ip] = frames
        fps_by_board[ip] = fps

    if not per_board:
        states = ", ".join(
            f"{ip}={data.get('state', '?')}" for ip, data in sorted(reports.items())
        )
        raise JRBoardError(
            f"no JR board is in {ARMABLE_STATE} with a show loaded, so there is "
            f"nothing to arm and no frame count to read -- boards are: {states}"
        )

    rates = set(fps_by_board.values())
    if len(rates) > 1:
        detail = ", ".join(f"{ip}={fps}" for ip, fps in sorted(fps_by_board.items()))
        raise JRBoardError(
            "JR boards report different frame rates and a single ARM broadcast "
            f"cannot serve both: {detail}"
        )

    # Take the longest show any board holds. A board that runs out of frames
    # early simply stops updating its LEDs -- ``led_player_render`` returns once
    # ``frame_idx`` passes that board's own ``total_frames`` -- whereas a frame
    # count below the real length would truncate every board's show, which is
    # the failure this function exists to prevent.
    return ShowParams(
        frame_count=max(per_board.values()),
        fps_num=rates.pop(),
        fps_den=1,
        per_board=per_board,
    )


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
