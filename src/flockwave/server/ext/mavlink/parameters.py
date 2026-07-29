"""Helpers for bulk MAVLink parameter listing via MAVFTP."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from flockwave.server.ext.mavlink.driver import MAVLinkUAV

ParameterEntry = dict[str, Any]
UAVParameterResult = dict[str, Any]


async def read_all_parameters(uav: MAVLinkUAV) -> UAVParameterResult:
    """Download and decode the full parameter set from a single UAV.

    Returns a payload with either ``parameters`` / ``count`` on success, or
    ``error`` on failure.
    """
    try:
        entries = await uav.get_all_parameters()
    except Exception as exc:
        return {"error": str(exc)}
    return {"count": len(entries), "parameters": entries}


def parameter_list_errors(
    results: dict[str, UAVParameterResult],
) -> list[dict[str, Any]]:
    """Return per-UAV errors from bulk parameter list results."""
    errors: list[dict[str, Any]] = []
    for uav_id, result in results.items():
        error = result.get("error")
        if isinstance(error, str) and error:
            errors.append({"uav": uav_id, "error": error})
    return errors
