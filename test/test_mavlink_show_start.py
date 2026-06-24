from unittest.mock import AsyncMock, Mock

import pytest
from trio import TooSlowError

from flockwave.server.ext.mavlink.autopilots.unknown import UnknownAutopilot
from flockwave.server.ext.mavlink.channel import Channel
from flockwave.server.ext.mavlink.driver import MAVLinkDriver, MAVLinkUAV
from flockwave.server.ext.mavlink.packets import DroneShowExecutionStage
from flockwave.server.ext.mavlink.show_start import perform_manual_show_start
from flockwave.server.ext.show.config import AuthorizationScope


class ScheduledTakeoffAutopilot(UnknownAutopilot):
    @property
    def supports_scheduled_takeoff(self):
        return True


class FakeDroneShowStatus:
    def __init__(self, stage: DroneShowExecutionStage):
        self.stage = stage


@pytest.fixture
def uav() -> MAVLinkUAV:
    driver = Mock(spec=MAVLinkDriver)
    driver.autopilot_factory = ScheduledTakeoffAutopilot
    driver.log = Mock()
    uav = MAVLinkUAV(driver=driver, id="test")
    uav.send_start_config = AsyncMock()
    uav.set_mode = AsyncMock()
    uav.wait_for_show_execution_stage = AsyncMock()
    return uav


async def test_show_start_clears_config_before_scheduling(uav: MAVLinkUAV):
    uav._last_skybrush_status_info = FakeDroneShowStatus(
        DroneShowExecutionStage.WAIT_FOR_START_TIME
    )

    await perform_manual_show_start(uav, AuthorizationScope.LIVE)

    assert uav.send_start_config.await_count == 2
    clear_call = uav.send_start_config.await_args_list[0]
    schedule_call = uav.send_start_config.await_args_list[1]
    assert clear_call.kwargs["authorization_scope"] is AuthorizationScope.NONE
    assert clear_call.kwargs["start_time"] is None
    assert schedule_call.kwargs["authorization_scope"] is AuthorizationScope.LIVE
    assert schedule_call.kwargs["start_time"] is not None
    uav.set_mode.assert_not_awaited()


async def test_show_start_resets_mode_after_landed(uav: MAVLinkUAV):
    uav._last_skybrush_status_info = FakeDroneShowStatus(DroneShowExecutionStage.LANDED)

    await perform_manual_show_start(uav, AuthorizationScope.LIVE)

    assert uav.set_mode.await_count == 2
    uav.set_mode.assert_any_await(5, channel=Channel.PRIMARY)
    uav.set_mode.assert_any_await(127, channel=Channel.PRIMARY)


async def test_show_start_resets_mode_after_error(uav: MAVLinkUAV):
    uav._last_skybrush_status_info = FakeDroneShowStatus(DroneShowExecutionStage.ERROR)

    await perform_manual_show_start(uav, AuthorizationScope.LIVE)

    assert uav.set_mode.await_count == 2


async def test_show_start_waits_for_wait_stage(uav: MAVLinkUAV):
    uav._last_skybrush_status_info = FakeDroneShowStatus(DroneShowExecutionStage.OFF)

    await perform_manual_show_start(uav, AuthorizationScope.REHEARSAL)

    uav.wait_for_show_execution_stage.assert_awaited_once_with(
        DroneShowExecutionStage.WAIT_FOR_START_TIME,
        timeout=15.0,
    )


async def test_show_start_retries_when_still_landed(uav: MAVLinkUAV):
    uav._last_skybrush_status_info = FakeDroneShowStatus(DroneShowExecutionStage.LANDED)
    uav.wait_for_show_execution_stage.side_effect = TooSlowError()

    with pytest.raises(RuntimeError, match="still in Landed stage"):
        await perform_manual_show_start(uav, AuthorizationScope.LIVE)

    assert uav.send_start_config.await_count == 4


async def test_show_start_rejects_unauthorized_scope(uav: MAVLinkUAV):
    with pytest.raises(RuntimeError, match="not authorized"):
        await perform_manual_show_start(uav, AuthorizationScope.NONE)
