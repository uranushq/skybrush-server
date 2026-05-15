from logging import getLogger
from time import time

from flockwave.spec.errors import FlockwaveErrorCode

from flockwave.server.ext.mavlink.driver import MAVLinkDriver
from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.extension import DroneShowExtension
from flockwave.server.model.builders import FlockwaveMessageBuilder
from flockwave.server.model.uav import UAVBase


class MockHub:
    def __init__(self):
        self._builder = FlockwaveMessageBuilder(id_generator=lambda: "response")

    def acknowledge(self, message, outcome=True, reason=None):
        body = {"type": "ACK-ACK", "outcome": bool(outcome)}
        if reason:
            body["reason"] = reason
        return self._builder.create_response_to(message, body)

    def create_response_or_notification(self, body, in_response_to=None):
        assert in_response_to is not None
        return self._builder.create_response_to(in_response_to, body)


class MockApp:
    def __init__(self, uavs):
        self.uavs = {uav.id: uav for uav in uavs}

    def sort_uavs_by_drivers(self, uav_ids, response=None):
        result = {}
        for uav_id in uav_ids:
            uav = self.uavs.get(uav_id)
            if uav is None:
                if response is not None:
                    response.add_error(uav_id, "No such UAV")
                continue
            result.setdefault(uav.driver, []).append(uav)
        return result


class MockShowStartDriver:
    def __init__(self):
        self.calls = []

    def send_show_start_signal(self, uavs, *, authorization_scope=None, transport=None):
        self.calls.append((list(uavs), authorization_scope))
        return {uav: None for uav in uavs}


class MockUAV(UAVBase):
    def __init__(self, driver, id):
        super().__init__(driver=driver, id=id)


async def test_show_start_uses_manual_hook_and_skips_disarmed_uavs():
    driver = MockShowStartDriver()
    armed_uav = MockUAV(driver, "armed")
    disarmed_uav = MockUAV(driver, "disarmed")
    disarmed_uav.ensure_error(FlockwaveErrorCode.DISARMED)

    extension = DroneShowExtension()
    extension.app = MockApp([armed_uav, disarmed_uav])
    extension.log = getLogger("test_ext_show_manual_start")
    extension._config.update_from_json(
        {
            "start": {
                "authorized": True,
                "authorizationScope": "live",
                "uavIds": ["armed", "disarmed"],
            }
        }
    )

    message = FlockwaveMessageBuilder(id_generator=lambda: "request").create_message(
        {"type": "X-SHOW-START"}
    )
    response = await extension.handle_SHOW_START(message, None, MockHub())

    assert driver.calls == [([armed_uav], AuthorizationScope.LIVE)]
    assert response.body["result"] == {"armed": True}
    assert response.body["error"] == {"disarmed": "UAV is not armed"}


async def test_mavlink_manual_show_start_sets_authorization_and_immediate_time():
    driver = MAVLinkDriver()
    uav = MockMAVLinkUAV()

    before = int(time())
    await driver._send_show_start_signal_single(
        uav, authorization_scope=AuthorizationScope.REHEARSAL
    )
    after = int(time()) + 1

    assert uav.authorization_scope is AuthorizationScope.REHEARSAL
    assert before <= uav.scheduled_takeoff_time <= after
    assert not uav.armed
    assert not uav.took_off


class MockMAVLinkUAV:
    def __init__(self):
        self.armed = False
        self.authorization_scope = None
        self.scheduled_takeoff_time = None
        self.took_off = False

    async def set_authorization_scope(self, scope):
        self.authorization_scope = scope

    async def set_scheduled_takeoff_time(self, seconds):
        self.scheduled_takeoff_time = seconds

    async def arm(self):
        self.armed = True

    async def takeoff_to_relative_altitude(self, altitude, *, channel):
        self.took_off = True
