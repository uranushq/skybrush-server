from math import hypot
from time import monotonic

from flockwave.gps.vectors import GPSCoordinate

from flockwave.server.ext.virtual_uavs.driver import VirtualUAVDriver, VirtualUAVState


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
            }
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
