from flockwave.server.ext.show.config import AuthorizationScope
from flockwave.server.ext.show.extension import DroneShowExtension
from flockwave.server.ext.show.readiness import collect_show_start_readiness
from flockwave.server.model.builders import FlockwaveMessageBuilder


class MockHub:
    def __init__(self):
        self._builder = FlockwaveMessageBuilder(id_generator=lambda: "response")

    def create_response_or_notification(self, body, in_response_to=None):
        assert in_response_to is not None
        return self._builder.create_response_to(in_response_to, body)


class MockApp:
    def __init__(self, uavs):
        self.uavs = {uav.id: uav for uav in uavs}

    def find_uav_by_id(self, uav_id, response=None):
        return self.uavs.get(uav_id)


class MockScheduledTakeoffUAV:
    def __init__(
        self,
        uav_id,
        *,
        connected=True,
        supports=True,
        start_time=None,
        auth=AuthorizationScope.NONE,
    ):
        self.id = uav_id
        self.is_connected = connected
        self.supports_scheduled_takeoff = supports
        self.scheduled_takeoff_time = start_time
        self.scheduled_takeoff_authorization_scope = auth


class PlainUAV:
    def __init__(self, uav_id):
        self.id = uav_id
        self.is_connected = True


def test_collect_show_start_readiness_all_ready():
    uavs = [
        MockScheduledTakeoffUAV(
            "01", start_time=1786066744, auth=AuthorizationScope.LIVE
        ),
        MockScheduledTakeoffUAV(
            "02", start_time=1786066744, auth=AuthorizationScope.REHEARSAL
        ),
    ]
    app = MockApp(uavs)
    extension = DroneShowExtension()
    extension.app = app
    extension._config.update_from_json(
        {"start": {"uavIds": ["01", None, "02"], "method": "auto"}}
    )

    result = collect_show_start_readiness(
        extension._config, find_uav=app.find_uav_by_id
    )

    assert result["ready"] is True
    assert result["total"] == 2
    assert result["readyCount"] == 2
    assert result["missingStartTime"] == []
    assert result["missingAuthorization"] == []
    assert result["uavs"]["01"]["authorizationScope"] == "live"
    assert result["uavs"]["02"]["startTime"] == 1786066744


def test_collect_show_start_readiness_reports_gaps():
    uavs = [
        MockScheduledTakeoffUAV("01", start_time=None, auth=AuthorizationScope.NONE),
        MockScheduledTakeoffUAV(
            "02", start_time=1786066744, auth=AuthorizationScope.NONE
        ),
        MockScheduledTakeoffUAV(
            "03",
            connected=False,
            start_time=1786066744,
            auth=AuthorizationScope.LIVE,
        ),
        MockScheduledTakeoffUAV(
            "04",
            supports=False,
            start_time=1786066744,
            auth=AuthorizationScope.LIVE,
        ),
        PlainUAV("05"),
    ]
    app = MockApp(uavs)
    config = DroneShowExtension()._config
    config.update_from_json(
        {
            "start": {
                "uavIds": ["01", "02", "03", "04", "05", "99"],
                "method": "auto",
            }
        }
    )

    result = collect_show_start_readiness(config, find_uav=app.find_uav_by_id)

    assert result["ready"] is False
    assert result["total"] == 6
    assert result["readyCount"] == 0
    assert result["missingStartTime"] == ["01"]
    assert result["missingAuthorization"] == ["01", "02"]
    assert result["disconnected"] == ["03"]
    assert result["unsupported"] == ["04", "05"]
    assert result["missing"] == ["99"]


def test_handle_SHOW_READY_returns_readiness_payload():
    uavs = [
        MockScheduledTakeoffUAV(
            "01", start_time=1786066744, auth=AuthorizationScope.LIVE
        ),
        MockScheduledTakeoffUAV("02", start_time=None, auth=AuthorizationScope.NONE),
    ]
    extension = DroneShowExtension()
    extension.app = MockApp(uavs)
    extension._config.update_from_json(
        {"start": {"uavIds": ["01", "02"], "method": "auto"}}
    )

    message = FlockwaveMessageBuilder(id_generator=lambda: "request").create_message(
        {"type": "X-SHOW-READY"}
    )
    response = extension.handle_SHOW_READY(message, None, MockHub())
    assert response.body["type"] == "X-SHOW-READY"

    assert response.body["ready"] is False
    assert response.body["readyCount"] == 1
    assert response.body["missingStartTime"] == ["02"]
    assert response.body["missingAuthorization"] == ["02"]
    assert response.body["uavs"]["01"]["ready"] is True
