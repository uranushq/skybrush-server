"""Tests for MAVLink bulk parameter listing helpers and REST API."""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock

import pytest

from flockwave.server.ext.mavlink import api as mavlink_api
from flockwave.server.ext.mavlink.driver import MAVLinkUAV
from flockwave.server.ext.mavlink.parameters import (
    parameter_list_errors,
    read_all_parameters,
)
from flockwave.server.utils import overridden


@pytest.mark.trio
async def test_read_all_parameters_success() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_all_parameters.return_value = [
        {"name": "SYSID_THISMAV", "value": 3.0, "type": "INT16", "default": 1.0},
    ]

    result = await read_all_parameters(uav)

    assert result == {
        "count": 1,
        "parameters": [
            {
                "name": "SYSID_THISMAV",
                "value": 3.0,
                "type": "INT16",
                "default": 1.0,
            }
        ],
    }


@pytest.mark.trio
async def test_read_all_parameters_reports_error() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_all_parameters.side_effect = RuntimeError("timeout")

    result = await read_all_parameters(uav)

    assert result == {"error": "timeout"}


def test_parameter_list_errors() -> None:
    errors = parameter_list_errors(
        {
            "1": {"count": 1, "parameters": []},
            "2": {"error": "unsupported"},
        }
    )
    assert errors == [{"uav": "2", "error": "unsupported"}]


def _mock_app_with_uavs(uavs: dict[str, MAVLinkUAV]) -> MagicMock:
    registry = MagicMock()
    registry.ids_by_type.return_value = list(uavs)
    registry.contains.side_effect = lambda uav_id: uav_id in uavs
    registry.find_by_id.side_effect = lambda uav_id: uavs[uav_id]

    app = MagicMock()
    app.object_registry = registry
    return app


def _mock_app_with_uav(uav: MAVLinkUAV, uav_id: str = "drone-1") -> MagicMock:
    return _mock_app_with_uavs({uav_id: uav})


@pytest.mark.trio
async def test_read_parameter_lists_api_logic() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_all_parameters.return_value = [
        {"name": "FOO", "value": 1.0, "type": "REAL32", "default": None},
    ]
    app = _mock_app_with_uav(uav)

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_parameter_lists(
            requested_ids=["drone-1"]
        )

    assert status == 200
    assert payload["skipped"] == []
    assert payload["results"]["drone-1"]["count"] == 1
    assert payload["results"]["drone-1"]["parameters"][0]["name"] == "FOO"


@pytest.mark.trio
async def test_read_parameter_lists_returns_207_on_failure() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_all_parameters.side_effect = NotImplementedError("no mavftp")
    app = _mock_app_with_uav(uav)

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_parameter_lists()

    assert status == 207
    assert payload["results"]["drone-1"]["error"] == "no mavftp"
    assert payload["errors"][0]["uav"] == "drone-1"


@pytest.mark.trio
async def test_read_parameter_lists_returns_404_when_empty() -> None:
    registry = MagicMock()
    registry.ids_by_type.return_value = []
    registry.contains.return_value = False
    registry.find_by_id.return_value = None
    app = MagicMock()
    app.object_registry = registry

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_parameter_lists()

    assert status == 404
    assert payload["error"] == "No MAVLink UAVs available"


@pytest.mark.trio
async def test_read_parameter_lists_skips_unknown_ids() -> None:
    uav = AsyncMock(spec=MAVLinkUAV)
    uav.get_all_parameters.return_value = [
        {"name": "FOO", "value": 1.0, "type": "REAL32", "default": None},
    ]
    app = _mock_app_with_uav(uav, uav_id="17")

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_parameter_lists(
            requested_ids=["0", "17"]
        )

    assert status == 200
    assert payload["skipped"] == ["0"]
    assert "17" in payload["results"]


@pytest.mark.trio
async def test_read_parameter_lists_downloads_all_requested_uavs() -> None:
    uav_a = AsyncMock(spec=MAVLinkUAV)
    uav_a.get_all_parameters.return_value = [
        {"name": "FOO", "value": 1.0, "type": "REAL32", "default": None},
    ]
    uav_b = AsyncMock(spec=MAVLinkUAV)
    uav_b.get_all_parameters.return_value = [
        {"name": "BAR", "value": 2.0, "type": "REAL32", "default": None},
    ]
    app = _mock_app_with_uavs({"a": uav_a, "b": uav_b})

    with overridden(mavlink_api, app=app, log=None):
        payload, status = await mavlink_api.read_parameter_lists()

    assert status == 200
    assert set(payload["results"]) == {"a", "b"}
    assert payload["results"]["a"]["parameters"][0]["name"] == "FOO"
    assert payload["results"]["b"]["parameters"][0]["name"] == "BAR"
