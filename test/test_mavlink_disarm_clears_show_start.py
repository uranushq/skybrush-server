from unittest.mock import AsyncMock, Mock

import pytest

from flockwave.server.ext.mavlink.autopilots.unknown import UnknownAutopilot
from flockwave.server.ext.mavlink.channel import Channel
from flockwave.server.ext.mavlink.driver import FORCE_MAGIC, MAVLinkDriver, MAVLinkUAV
from flockwave.server.ext.mavlink.enums import MAVCommand
from flockwave.server.ext.mavlink.packets import DroneShowExecutionStage
from flockwave.server.ext.show.config import AuthorizationScope


class ScheduledTakeoffAutopilot(UnknownAutopilot):
    @property
    def supports_scheduled_takeoff(self):
        return True


@pytest.fixture
def uav() -> MAVLinkUAV:
    driver = Mock(spec=MAVLinkDriver)
    driver.autopilot_factory = ScheduledTakeoffAutopilot
    driver.log = Mock()
    return MAVLinkUAV(driver=driver, id="test")


async def test_disarm_with_clear_show_start_clears_configuration(uav: MAVLinkUAV):
    uav._set_armed_state = AsyncMock()
    uav.clear_show_start_configuration = AsyncMock()

    await uav.disarm(clear_show_start=True)

    uav._set_armed_state.assert_awaited_once_with(
        False, force=False, channel=Channel.PRIMARY
    )
    uav.clear_show_start_configuration.assert_awaited_once()


async def test_disarm_without_clear_show_start_leaves_configuration(uav: MAVLinkUAV):
    uav._set_armed_state = AsyncMock()
    uav.clear_show_start_configuration = AsyncMock()

    await uav.disarm()

    uav._set_armed_state.assert_awaited_once()
    uav.clear_show_start_configuration.assert_not_awaited()


async def test_clear_show_start_configuration_updates_local_state(uav: MAVLinkUAV):
    uav.set_parameter = AsyncMock()
    uav._scheduled_takeoff_authorization_scope = AuthorizationScope.LIVE
    uav._scheduled_takeoff_time = 1_700_000_000
    uav._scheduled_takeoff_time_gps_time_of_week = 123

    await uav.clear_show_start_configuration()

    assert uav.set_parameter.await_count == 2
    assert (
        uav._scheduled_takeoff_authorization_scope is AuthorizationScope.NONE
    )
    assert uav._scheduled_takeoff_time is None
    assert uav._scheduled_takeoff_time_gps_time_of_week is None


async def test_motor_stop_signal_clears_show_start_configuration():
    driver = MAVLinkDriver()
    uav = Mock()
    uav.disarm = AsyncMock()

    await driver._send_motor_start_stop_signal_single(uav, start=False)

    uav.disarm.assert_awaited_once_with(
        force=False, channel=Channel.PRIMARY, clear_show_start=True
    )


def test_landed_transition_schedules_show_start_clear(uav: MAVLinkUAV):
    uav.driver.run_in_background = Mock()

    uav._on_show_execution_stage_changed(DroneShowExecutionStage.PERFORMING)
    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)

    uav.driver.run_in_background.assert_called_once_with(
        uav._clear_show_start_configuration_after_landing
    )


def test_landed_while_already_landed_does_not_clear_again(uav: MAVLinkUAV):
    uav.driver.run_in_background = Mock()

    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)
    uav.driver.run_in_background.reset_mock()
    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)

    uav.driver.run_in_background.assert_not_called()


def test_landed_does_not_clear_without_scheduled_takeoff_support(uav: MAVLinkUAV):
    uav._autopilot = UnknownAutopilot()
    uav.driver.run_in_background = Mock()

    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)

    uav.driver.run_in_background.assert_not_called()


async def test_clear_show_start_configuration_after_landing(uav: MAVLinkUAV):
    uav.clear_show_start_configuration = AsyncMock()

    await uav._clear_show_start_configuration_after_landing()

    uav.clear_show_start_configuration.assert_awaited_once()


async def test_arm_sends_single_command_without_retries(uav: MAVLinkUAV):
    uav.driver.send_command_long = AsyncMock(return_value=True)

    await uav.arm()

    uav.driver.send_command_long.assert_awaited_once_with(
        uav,
        MAVCommand.COMPONENT_ARM_DISARM,
        1,
        0,
        channel=Channel.PRIMARY,
        retries=0,
    )


async def test_arm_with_force_uses_force_flag_once(uav: MAVLinkUAV):
    uav.driver.send_command_long = AsyncMock(return_value=True)

    await uav.arm(force=True)

    uav.driver.send_command_long.assert_awaited_once_with(
        uav,
        MAVCommand.COMPONENT_ARM_DISARM,
        1,
        FORCE_MAGIC,
        channel=Channel.PRIMARY,
        retries=0,
    )


async def test_arm_does_not_retry_with_force_after_rejection(uav: MAVLinkUAV):
    uav.driver.send_command_long = AsyncMock(return_value=False)

    with pytest.raises(RuntimeError, match="Failed to arm"):
        await uav.arm()

    uav.driver.send_command_long.assert_awaited_once()


async def test_disarm_sends_single_command_without_retries(uav: MAVLinkUAV):
    uav.driver.send_command_long = AsyncMock(return_value=True)
    uav.clear_show_start_configuration = AsyncMock()

    await uav.disarm(clear_show_start=True)

    uav.driver.send_command_long.assert_awaited_once_with(
        uav,
        MAVCommand.COMPONENT_ARM_DISARM,
        0,
        0,
        channel=Channel.PRIMARY,
        retries=0,
    )
