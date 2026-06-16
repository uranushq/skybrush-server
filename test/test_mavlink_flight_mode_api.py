"""Tests for MAVLink flight-mode slot helpers and REST API."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from flockwave.server.ext.mavlink import api as mavlink_api
from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.flight_modes import (
    SHOW_MODE_CUSTOM_MODE,
    configure_show_mode_flight_mode_slots,
    read_show_mode_flight_mode_slots,
)
from flockwave.server.utils import overridden


@pytest.mark.trio
async def test_configure_show_mode_flight_mode_slots_sets_both_params() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    results = await configure_show_mode_flight_mode_slots(uav, mode=127.0)

    assert results == {"FLTMODE5": 127.0, "FLTMODE6": 127.0}
    assert uav.set_parameter.await_count == 2
    uav.set_parameter.assert_any_await("FLTMODE5", 127.0)
    uav.set_parameter.assert_any_await("FLTMODE6", 127.0)


@pytest.mark.trio
async def test_configure_show_mode_flight_mode_slots_reports_partial_failure() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)

    async def _set_parameter(name: str, value: float) -> None:
        if name == "FLTMODE5":
            raise RuntimeError("timeout")

    uav.set_parameter.side_effect = _set_parameter
    results = await configure_show_mode_flight_mode_slots(uav, mode=5.0)

    assert results["FLTMODE5"] == "error: timeout"
    assert results["FLTMODE6"] == 5.0


@pytest.mark.trio
async def test_read_show_mode_flight_mode_slots() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_parameter.side_effect = [127.0, 3.0]

    results = await read_show_mode_flight_mode_slots(uav)

    assert results == {"FLTMODE5": 127.0, "FLTMODE6": 3.0}


def _mock_app_with_uav(uav: MAVLinkUAV, uav_id: str = "drone-1") -> MagicMock:
    registry = MagicMock()
    registry.ids_by_type.return_value = [uav_id]
    registry.find_by_id.return_value = uav

    app = MagicMock()
    app.object_registry = registry
    return app


@pytest.mark.trio
async def test_apply_flight_mode_slots_updates_all_mavlink_uavs() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.set_parameter = AsyncMock()
    app = _mock_app_with_uav(uav)

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.apply_flight_mode_slots(
            mode=SHOW_MODE_CUSTOM_MODE
        )

    assert status == 200
    assert payload["mode"] == 127.0
    assert payload["results"]["drone-1"] == {"FLTMODE5": 127.0, "FLTMODE6": 127.0}


@pytest.mark.trio
async def test_apply_flight_mode_slots_returns_207_on_partial_failure() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)

    async def _set_parameter(name: str, value: float) -> None:
        if name == "FLTMODE6":
            raise RuntimeError("nack")

    uav.set_parameter.side_effect = _set_parameter
    app = _mock_app_with_uav(uav)

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.apply_flight_mode_slots(
            mode=127.0,
            requested_ids=["drone-1"],
        )

    assert status == 207
    assert payload["results"]["drone-1"]["FLTMODE6"] == "error: nack"
    assert payload["errors"][0]["parameter"] == "FLTMODE6"


@pytest.mark.trio
async def test_read_flight_mode_slots_api_logic() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_parameter.side_effect = [127.0, 127.0]
    app = _mock_app_with_uav(uav)

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_flight_mode_slots()

    assert status == 200
    assert payload["results"]["drone-1"] == {"FLTMODE5": 127.0, "FLTMODE6": 127.0}
