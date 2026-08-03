from math import hypot
from time import monotonic
from unittest.mock import MagicMock

import pytest
from flockwave.gps.vectors import GPSCoordinate

from flockwave.server.ext.virtual_uavs.driver import VirtualUAVDriver, VirtualUAVState
from flockwave.server.ext.virtual_uavs.extension import VirtualUAVProviderExtension


def _configured_extension(
    *, count: int = 3, active: bool = True
) -> VirtualUAVProviderExtension:
    ext = VirtualUAVProviderExtension()
    app = MagicMock()
    app.extension_manager.import_api.side_effect = Exception("no radiation")
    ext.app = app
    ext.configure(
        {
            "count": count,
            "active": active,
            "id_format": "{0}",
            "origin": [18.915125, 47.486305, 215],
            "orientation": 0,
            "takeoff_area": {"type": "grid", "spacing": 5},
            "arm_after_boot": True,
        }
    )
    return ext


def test_virtual_uav_lands_after_show_with_low_final_altitude():
    driver = VirtualUAVDriver()
    uav = driver.create_uav("test", GPSCoordinate(lat=0, lon=0))

    uav.handle_show_upload(
        {
            "coordinateSystem": {"type": "nwu", "origin": [0, 0], "orientation": 0},
            "trajectory": {
                "version": 1,
                "takeoffTime": 0,
                "points": [
                    [0, [0, 0, 0.05], []],
                    [1, [0, 0, 0.05], []],
                ],
            },
        }
    )
    uav.takeoff()
    uav.state = VirtualUAVState.AIRBORNE
    uav._position_xyz.z = 0.05
    uav._mission_started_at = monotonic() - 10

    for _ in range(4):
        uav.step(0.1)

    assert uav.state is VirtualUAVState.LANDED


def test_virtual_uav_follows_low_altitude_show_without_xy_catchup_burst():
    """Show trajectories below the default takeoff altitude must not block XY."""
    driver = VirtualUAVDriver()
    uav = driver.create_uav("test", GPSCoordinate(lat=0, lon=0))

    uav.handle_show_upload(
        {
            "home": [0, 0, 2.5],
            "coordinateSystem": {"type": "nwu", "origin": [0, 0], "orientation": 0},
            "trajectory": {
                "version": 1,
                "takeoffTime": 0,
                "points": [
                    [0, [0, 0, 0], []],
                    [1.667, [0, 0, 2.5], []],
                    [2.667, [-0.7, 0, 2.55], []],
                ],
            },
        }
    )

    assert uav.takeoff_altitude == 2.5

    uav.takeoff()
    base_time = monotonic()

    max_speed = 0.0
    for step in range(30):
        uav._mission_started_at = base_time - step * 0.1
        uav.step(0.1)
        max_speed = max(max_speed, hypot(uav._velocity_xyz.x, uav._velocity_xyz.y))

    # Must track XY before reaching the old 2.8m airborne gate (takeoff_altitude - eps).
    assert uav._position_xyz.z < 2.8
    assert uav._position_xyz.x < -0.3
    assert max_speed < 2.5


def test_fleet_status_reports_enabled_count_and_ids():
    ext = _configured_extension(count=3)
    status = ext.fleet_status()
    assert status["enabled"] is True
    assert status["count"] == 3
    assert status["uavIds"] == ["0", "1", "2"]


def test_configure_with_active_false_creates_no_uavs():
    ext = _configured_extension(count=5, active=False)
    status = ext.fleet_status()
    assert status["enabled"] is False
    assert status["count"] == 5
    assert status["uavIds"] == []
    assert ext.uavs == []


@pytest.mark.trio
async def test_set_fleet_count_rebuilds_without_worker():
    ext = _configured_extension(count=2)
    status = await ext.set_fleet_count(4)
    assert status["count"] == 4
    assert status["enabled"] is True
    assert len(status["uavIds"]) == 4
    assert ext._configuration["count"] == 4


@pytest.mark.trio
async def test_set_fleet_enabled_toggles_uavs_without_worker():
    ext = _configured_extension(count=3)

    disabled = await ext.set_fleet_enabled(False)
    assert disabled["enabled"] is False
    assert disabled["count"] == 3
    assert disabled["uavIds"] == []
    assert ext.uavs == []
    assert ext._configuration["active"] is False

    enabled = await ext.set_fleet_enabled(True)
    assert enabled["enabled"] is True
    assert enabled["count"] == 3
    assert len(enabled["uavIds"]) == 3
    assert ext._configuration["active"] is True


@pytest.mark.trio
async def test_set_fleet_count_rejects_negative():
    ext = _configured_extension(count=1)
    with pytest.raises(ValueError, match="count must be >= 0"):
        await ext.set_fleet_count(-1)
