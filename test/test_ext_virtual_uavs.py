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
