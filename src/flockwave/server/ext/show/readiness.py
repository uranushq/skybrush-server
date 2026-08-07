"""Helpers that summarize whether show participants are ready to start.

A participating UAV is considered ready when it is connected, supports
scheduled takeoff, has a scheduled start time, and has a non-``NONE``
authorization scope.
"""

from __future__ import annotations

from typing import Any, Protocol, TypedDict

from .config import AuthorizationScope, DroneShowConfiguration

__all__ = (
    "ShowStartReadiness",
    "UAVStartReadiness",
    "collect_show_start_readiness",
)


class SupportsScheduledTakeoffStatus(Protocol):
    """Minimal protocol for objects that expose scheduled takeoff state."""

    @property
    def id(self) -> str: ...

    @property
    def is_connected(self) -> bool: ...

    @property
    def supports_scheduled_takeoff(self) -> bool: ...

    @property
    def scheduled_takeoff_time(self) -> int | None: ...

    @property
    def scheduled_takeoff_authorization_scope(self) -> AuthorizationScope: ...


class UAVStartReadiness(TypedDict):
    """Per-UAV start readiness snapshot exposed to clients."""

    ready: bool
    connected: bool
    supportsScheduledTakeoff: bool
    hasStartTime: bool
    startTime: int | None
    hasAuthorization: bool
    authorizationScope: str


class ShowStartReadiness(TypedDict):
    """Aggregate start readiness for all mapped show participants."""

    ready: bool
    total: int
    readyCount: int
    missingStartTime: list[str]
    missingAuthorization: list[str]
    missing: list[str]
    disconnected: list[str]
    unsupported: list[str]
    uavs: dict[str, UAVStartReadiness]


def _authorization_scope_value(scope: Any) -> str:
    if isinstance(scope, AuthorizationScope):
        return scope.value
    if scope is None:
        return AuthorizationScope.NONE.value
    return str(scope)


def _has_authorization(scope: Any) -> bool:
    if isinstance(scope, AuthorizationScope):
        return scope is not AuthorizationScope.NONE
    if scope is None:
        return False
    return str(scope) not in ("", AuthorizationScope.NONE.value)


def describe_uav_start_readiness(
    uav: SupportsScheduledTakeoffStatus,
) -> UAVStartReadiness:
    """Returns the start readiness snapshot of a single UAV."""
    connected = bool(getattr(uav, "is_connected", True))
    supports = bool(getattr(uav, "supports_scheduled_takeoff", False))
    start_time = getattr(uav, "scheduled_takeoff_time", None)
    scope = getattr(
        uav, "scheduled_takeoff_authorization_scope", AuthorizationScope.NONE
    )
    has_start_time = start_time is not None
    has_authorization = _has_authorization(scope)

    return {
        "ready": connected and supports and has_start_time and has_authorization,
        "connected": connected,
        "supportsScheduledTakeoff": supports,
        "hasStartTime": has_start_time,
        "startTime": int(start_time) if start_time is not None else None,
        "hasAuthorization": has_authorization,
        "authorizationScope": _authorization_scope_value(scope),
    }


def collect_show_start_readiness(
    config: DroneShowConfiguration,
    *,
    find_uav,
) -> ShowStartReadiness:
    """Collects start readiness for every UAV mapped into the show.

    Args:
        config: current drone show configuration
        find_uav: callable that returns a UAV for an ID, or ``None`` if the UAV
            is not registered

    Returns:
        aggregate readiness payload suitable for Flockwave / HTTP responses
    """
    uav_ids = [uav_id for uav_id in config.uav_ids if uav_id is not None]

    missing: list[str] = []
    disconnected: list[str] = []
    unsupported: list[str] = []
    missing_start_time: list[str] = []
    missing_authorization: list[str] = []
    uavs: dict[str, UAVStartReadiness] = {}
    ready_count = 0

    for uav_id in uav_ids:
        uav = find_uav(uav_id)
        if uav is None:
            missing.append(uav_id)
            uavs[uav_id] = {
                "ready": False,
                "connected": False,
                "supportsScheduledTakeoff": False,
                "hasStartTime": False,
                "startTime": None,
                "hasAuthorization": False,
                "authorizationScope": AuthorizationScope.NONE.value,
            }
            continue

        # UAVs that do not expose scheduled-takeoff helpers cannot be ready.
        if not hasattr(uav, "scheduled_takeoff_time") or not hasattr(
            uav, "scheduled_takeoff_authorization_scope"
        ):
            unsupported.append(uav_id)
            uavs[uav_id] = {
                "ready": False,
                "connected": bool(getattr(uav, "is_connected", False)),
                "supportsScheduledTakeoff": False,
                "hasStartTime": False,
                "startTime": None,
                "hasAuthorization": False,
                "authorizationScope": AuthorizationScope.NONE.value,
            }
            continue

        status = describe_uav_start_readiness(uav)
        uavs[uav_id] = status

        if not status["connected"]:
            disconnected.append(uav_id)
        if not status["supportsScheduledTakeoff"]:
            unsupported.append(uav_id)
        if not status["hasStartTime"]:
            missing_start_time.append(uav_id)
        if not status["hasAuthorization"]:
            missing_authorization.append(uav_id)
        if status["ready"]:
            ready_count += 1

    total = len(uav_ids)
    return {
        "ready": total > 0 and ready_count == total,
        "total": total,
        "readyCount": ready_count,
        "missingStartTime": missing_start_time,
        "missingAuthorization": missing_authorization,
        "missing": missing,
        "disconnected": disconnected,
        "unsupported": unsupported,
        "uavs": uavs,
    }
