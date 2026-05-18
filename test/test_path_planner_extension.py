"""Tests for path-planner request parsing helpers."""

from __future__ import annotations

from flockwave.server.ext.path_planner.extension import (
    _drone_index_from_id,
    _normalize_vec3_array,
    _phase_point_drone_index,
)


def test_drone_index_from_id_accepts_drone_and_show_drone_prefixes() -> None:
    assert _drone_index_from_id(1) == 0
    assert _drone_index_from_id("drone-3") == 2
    assert _drone_index_from_id("show-drone-4") == 3
    assert _drone_index_from_id("unknown-2") is None


def test_phase_point_drone_index_show_drone_ids() -> None:
    point = {"droneId": "show-drone-2", "x": 0, "y": 0, "z": 0}
    assert _phase_point_drone_index(point, 0, 4) == 1


def test_normalize_vec3_array_orders_by_show_drone_id() -> None:
    value = [
        {"droneId": "show-drone-2", "x": 3, "y": 0, "z": 0},
        {"droneId": "show-drone-1", "x": 0, "y": 0, "z": 0},
    ]
    assert _normalize_vec3_array("initial", value) == [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
