"""Tests for path-planner request parsing helpers."""

from __future__ import annotations

from flockwave.server.ext.path_planner.converter import (
    DEFAULT_CRUISE_SPEED_M_S,
    DEFAULT_LANDING_SPEED_M_S,
    duration_ms_for_cruise_speed,
    solver_result_to_trajectory_dicts,
)
from flockwave.server.ext.path_planner.extension import (
    _drone_index_from_id,
    _normalize_vec3_array,
    _phase_point_drone_index,
)
from flockwave.server.ext.path_planner.solver import Drone, SolverResult, StepRecord


def test_duration_ms_for_default_cruise_speed() -> None:
    assert duration_ms_for_cruise_speed(1.0, DEFAULT_CRUISE_SPEED_M_S) == 10000


def test_solver_result_landing_uses_slower_default_speed() -> None:
    result = SolverResult(
        drones=[Drone(drone_id=0, initial=(0.0, 0.0, 2.5), target=(0.0, 0.0, 2.5))],
        steps=[
            StepRecord(
                step=0,
                positions={0: [0.0, 0.0, 2.5]},
                collisions=[],
                reverted_drones=[],
                verified=True,
            )
        ],
        success=True,
        total_steps=0,
    )
    traj = solver_result_to_trajectory_dicts(
        result,
        duration_ms=5000,
        landing_speed=DEFAULT_LANDING_SPEED_M_S,
        velocity_smoothing=0.0,
    )[0]
    landing_start = traj["points"][-2][0]
    landing_end = traj["points"][-1][0]
    assert round(landing_end - landing_start, 4) == round(2.5 / DEFAULT_LANDING_SPEED_M_S, 4)


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
