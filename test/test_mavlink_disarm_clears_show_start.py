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
    uav._landing_clear_generation = 0

    uav._on_show_execution_stage_changed(DroneShowExecutionStage.PERFORMING)
    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)

    uav.driver.run_in_background.assert_called_once()
    scheduled = uav.driver.run_in_background.call_args.args[0]
    assert scheduled.__name__ == "<lambda>"


async def test_clear_show_start_configuration_after_landing(uav: MAVLinkUAV):
    uav.clear_show_start_configuration = AsyncMock()
    uav._landing_clear_generation = 1

    await uav._clear_show_start_configuration_after_landing(1)

    uav.clear_show_start_configuration.assert_awaited_once()


async def test_stale_landing_clear_is_ignored_after_show_start_prepare(
    uav: MAVLinkUAV,
):
    uav.clear_show_start_configuration = AsyncMock()
    uav.set_mode = AsyncMock()
    uav.reload_show = AsyncMock()
    uav.driver.run_in_background = Mock()
    uav._last_show_execution_stage = DroneShowExecutionStage.LANDED
    uav._landing_clear_generation = 0

    uav._on_show_execution_stage_changed(DroneShowExecutionStage.PERFORMING)
    uav._on_show_execution_stage_changed(DroneShowExecutionStage.LANDED)

    await uav.prepare_for_show_start()
    await uav._clear_show_start_configuration_after_landing(1)

    assert uav.clear_show_start_configuration.await_count == 1


async def test_prepare_for_show_start_reloads_show_after_rtl(uav: MAVLinkUAV):
    uav.clear_show_start_configuration = AsyncMock()
    uav.set_mode = AsyncMock()
    uav.reload_show = AsyncMock()
    uav._last_show_execution_stage = DroneShowExecutionStage.RTL

    await uav.prepare_for_show_start()

    uav.clear_show_start_configuration.assert_awaited_once()
    uav.set_mode.assert_awaited_once_with(127)
    uav.reload_show.assert_awaited_once()


async def test_prepare_for_show_start_reuploads_cached_show(uav: MAVLinkUAV):
    uav.clear_show_start_configuration = AsyncMock()
    uav.set_mode = AsyncMock()
    uav.remove_show = AsyncMock()
    uav.upload_show = AsyncMock()
    uav.reload_show = AsyncMock()
    cached = {"trajectory": {"version": 1, "points": []}}
    uav._uploaded_show = cached

    await uav.prepare_for_show_start()

    uav.remove_show.assert_awaited_once()
    uav.upload_show.assert_awaited_once_with(cached)
    uav.reload_show.assert_not_awaited()


async def test_prepare_for_show_start_reloads_show_when_landed(uav: MAVLinkUAV):
    uav.clear_show_start_configuration = AsyncMock()
    uav.set_mode = AsyncMock()
    uav.reload_show = AsyncMock()
    uav._last_show_execution_stage = DroneShowExecutionStage.LANDED

    await uav.prepare_for_show_start()

    uav.reload_show.assert_awaited_once()


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
