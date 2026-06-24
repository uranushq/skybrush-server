"""Manual show start sequence for Skybrush-compatible MAVLink drones."""

from __future__ import annotations

import random
from time import time
from typing import TYPE_CHECKING

from trio import TooSlowError, sleep

from flockwave.server.ext.show.config import AuthorizationScope

from .channel import Channel
from .flight_modes import LOITER_CUSTOM_MODE, SHOW_MODE_CUSTOM_MODE
from .packets import DroneShowExecutionStage

if TYPE_CHECKING:
    from flockwave.server.model.transport import TransportOptions

    from .driver import MAVLinkUAV

__all__ = ("perform_manual_show_start",)

SHOW_START_LEAD_TIME_MIN_SEC = 30
SHOW_START_LEAD_TIME_MAX_SEC = 60
MODE_SWITCH_DELAY_SEC = 1.0
STAGE_WAIT_TIMEOUT_SEC = 15.0
STAGE_POLL_INTERVAL_SEC = 0.2
MAX_LANDED_RETRIES = 1


async def perform_manual_show_start(
    uav: MAVLinkUAV,
    authorization_scope: AuthorizationScope,
    *,
    transport: TransportOptions | None = None,
) -> None:
    """Runs the full manual show start sequence on a single UAV.

    The sequence clears any previous start configuration, optionally
    re-initializes the show subsystem after a landed or error state, schedules
    a new GPS-based start time, and waits until the drone enters the Wait stage.

    Args:
        uav: the UAV to start the show on
        authorization_scope: the authorization scope to grant (typically LIVE)
        transport: optional transport options for sending commands

    Raises:
        RuntimeError: if the show start sequence fails
    """
    if authorization_scope is AuthorizationScope.NONE:
        raise RuntimeError("Show start is not authorized")

    if not uav.supports_scheduled_takeoff:
        raise RuntimeError("UAV does not support scheduled show starts")

    channel = _command_channel(transport)

    for attempt in range(MAX_LANDED_RETRIES + 1):
        await _clear_start_configuration(uav)

        stage = uav.show_execution_stage
        if stage in (
            DroneShowExecutionStage.LANDED,
            DroneShowExecutionStage.ERROR,
        ):
            await _reset_show_mode(uav, channel=channel)
            stage = uav.show_execution_stage

        lead_time = random.randint(
            SHOW_START_LEAD_TIME_MIN_SEC, SHOW_START_LEAD_TIME_MAX_SEC
        )
        start_time = int(time()) + lead_time
        await uav.send_start_config(
            authorization_scope=authorization_scope,
            start_time=start_time,
        )

        try:
            await uav.wait_for_show_execution_stage(
                DroneShowExecutionStage.WAIT_FOR_START_TIME,
                timeout=STAGE_WAIT_TIMEOUT_SEC,
            )
            return
        except TooSlowError:
            stage = uav.show_execution_stage
            if stage is DroneShowExecutionStage.LANDED and attempt < MAX_LANDED_RETRIES:
                continue

            if stage is DroneShowExecutionStage.LANDED:
                raise RuntimeError(
                    "Show start failed: drone is still in Landed stage; "
                    "press Show start again or switch LOITER → SHOW"
                ) from None

            raise RuntimeError(
                "Show start failed: expected Wait stage, "
                f"got {stage.description or stage.name}"
            ) from None


async def _clear_start_configuration(uav: MAVLinkUAV) -> None:
    """Sends a START_CONFIG packet that clears past start time and authorization."""
    await uav.send_start_config(
        authorization_scope=AuthorizationScope.NONE,
        start_time=None,
    )


async def _reset_show_mode(uav: MAVLinkUAV, *, channel: str) -> None:
    """Switches LOITER → SHOW to re-enter the show subsystem after Landed/Error."""
    await uav.set_mode(LOITER_CUSTOM_MODE, channel=channel)
    await sleep(MODE_SWITCH_DELAY_SEC)
    await uav.set_mode(SHOW_MODE_CUSTOM_MODE, channel=channel)
    await sleep(MODE_SWITCH_DELAY_SEC)


def _command_channel(transport: TransportOptions | None) -> str:
    if transport is not None and getattr(transport, "channel", 0) > 0:
        return Channel.SECONDARY
    return Channel.PRIMARY
