"""Tests for path-planner request parsing helpers."""

from __future__ import annotations

from flockwave.server.ext.path_planner.converter import (
    DEFAULT_CRUISE_SPEED_M_S,
    DEFAULT_LANDING_SPEED_M_S,
    duration_ms_for_cruise_speed,
    solver_result_to_trajectory_dicts,
    step_time_ms,
    step_time_sec,
)
from flockwave.server.ext.path_planner.extension import (
    _drone_index_from_id,
    _normalize_vec3_array,
    _phase_point_drone_index,
    _plan_formation_phases,
)
from flockwave.server.ext.path_planner.output import build_output
from flockwave.server.ext.path_planner.solver import Drone, SolverResult, StepRecord


def test_duration_ms_for_default_cruise_speed() -> None:
    # Default cruise is 1 m/s, so a 1 m step takes 1000 ms.
    assert DEFAULT_CRUISE_SPEED_M_S == 1.0
    assert duration_ms_for_cruise_speed(1.0, DEFAULT_CRUISE_SPEED_M_S) == 1000
    assert duration_ms_for_cruise_speed(1.0, 0.1) == 10000


def test_step_time_prefers_explicit_time_ms() -> None:
    rec = StepRecord(
        step=3,
        positions={0: [0.0, 0.0, 2.5]},
        collisions=[],
        reverted_drones=[],
        verified=True,
        time_ms=25000,
    )
    assert step_time_ms(rec, duration_ms=1000) == 25000
    assert step_time_sec(rec, duration_ms=1000) == 25.0
    assert (
        step_time_ms(
            StepRecord(
                step=3,
                positions={0: [0.0, 0.0, 2.5]},
                collisions=[],
                reverted_drones=[],
                verified=True,
            ),
            duration_ms=1000,
        )
        == 3000
    )


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
    assert round(landing_end - landing_start, 4) == round(
        2.5 / DEFAULT_LANDING_SPEED_M_S, 4
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


def test_plan_formation_phases_honours_per_phase_duration_ms() -> None:
    start = [[0.0, 0.0, 5.0]]
    phases = [
        {
            "name": "slow",
            "durationMs": 10000,
            "holdMs": 0,
            "points": [{"x": 2.0, "y": 0.0, "z": 5.0}],
        },
        {
            "name": "fast",
            "durationMs": 1000,
            "holdMs": 0,
            "points": [{"x": 4.0, "y": 0.0, "z": 5.0}],
        },
    ]
    result, summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        staging_targets=None,
    )
    assert result.success
    by_name = {item["name"]: item for item in summaries}
    assert by_name["slow"]["durationMs"] == 10000
    assert by_name["fast"]["durationMs"] == 1000

    # First hops into the slow phase should advance by 10 s each.
    slow_times = [
        rec.time_ms
        for rec in result.steps
        if rec.time_ms is not None
        and 0 < rec.time_ms <= by_name["slow"]["arrivalTimeMs"]
    ]
    assert slow_times
    assert all(
        slow_times[i] - (slow_times[i - 1] if i else 0) == 10000
        for i in range(len(slow_times))
    )

    output = build_output(result, duration_ms=1000)
    hop_durations = [entry["durationMs"] for entry in output["drones"][0]["path"]]
    assert 10000 in hop_durations
    assert 1000 in hop_durations

    # Start already at cruise altitude so takeoff duration is zero.
    traj = solver_result_to_trajectory_dicts(
        result,
        duration_ms=1000,
        velocity_smoothing=0.0,
        ground_positions=[[0.0, 0.0, 5.0]],
    )[0]
    assert any(
        abs(pt[0] - (by_name["slow"]["arrivalTimeMs"] / 1000.0)) < 1e-6
        for pt in traj["points"]
    )
