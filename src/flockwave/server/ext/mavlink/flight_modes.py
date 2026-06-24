"""ArduPilot flight-mode switch slot helpers for Skybrush drone shows."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from flockwave.server.ext.mavlink.driver import MAVLinkUAV

# ArduPilot custom_mode values used during manual show starts.
LOITER_CUSTOM_MODE = 5
SHOW_MODE_CUSTOM_MODE = 127

# RC switch positions mapped to show mode during show upload.
SHOW_MODE_FLIGHT_MODE_PARAM_NAMES = ("FLTMODE5", "FLTMODE6")

ParameterResult = float | str


async def configure_show_mode_flight_mode_slots(
    uav: MAVLinkUAV,
    *,
    mode: float = SHOW_MODE_CUSTOM_MODE,
    parameter_names: tuple[str, ...] = SHOW_MODE_FLIGHT_MODE_PARAM_NAMES,
) -> dict[str, ParameterResult]:
    """Set FLTMODE5/FLTMODE6 (or similar) to *mode* on a single UAV."""
    results: dict[str, ParameterResult] = {}
    for name in parameter_names:
        try:
            await uav.set_parameter(name, mode)
            results[name] = mode
        except Exception as exc:
            results[name] = f"error: {exc}"
    return results


async def read_show_mode_flight_mode_slots(
    uav: MAVLinkUAV,
    *,
    parameter_names: tuple[str, ...] = SHOW_MODE_FLIGHT_MODE_PARAM_NAMES,
) -> dict[str, ParameterResult]:
    """Read FLTMODE5/FLTMODE6 values from a single UAV."""
    results: dict[str, ParameterResult] = {}
    for name in parameter_names:
        try:
            results[name] = await uav.get_parameter(name)
        except Exception as exc:
            results[name] = f"error: {exc}"
    return results


def flight_mode_slot_errors(
    results: dict[str, dict[str, ParameterResult]],
) -> list[dict[str, Any]]:
    """Return a list of per-UAV parameter errors from configure/read results."""
    errors: list[dict[str, Any]] = []
    for uav_id, params in results.items():
        for name, value in params.items():
            if isinstance(value, str) and value.startswith("error:"):
                errors.append({"uav": uav_id, "parameter": name, "error": value[7:]})
    return errors
