"""Tests for path-planner yaw interpolation and skyc yawControl output."""

from __future__ import annotations

from flockwave.server.ext.path_planner.converter import (
    build_show_dicts,
    build_yaw_control_dict,
)
from flockwave.server.ext.path_planner.extension import _plan_formation_phases
from flockwave.server.ext.path_planner.solver import StepRecord


def test_plan_formation_phases_changes_yaw_after_arrival() -> None:
    initial = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    phases = [
        {
            "name": "heart",
            "holdMs": 0,
            "points": [
                {"droneId": "drone-1", "x": 0, "y": 5, "z": 0, "yaw": 90.0},
                {"droneId": "drone-2", "x": 2, "y": 5, "z": 0, "yaw": 0.0},
            ],
        }
    ]
    result, _ = _plan_formation_phases(
        initial=initial,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=42,
        return_to_initial=False,
        initial_yaws=[0.0, 0.0],
    )

    move_records = [rec for rec in result.steps if rec.step > 0]
    assert move_records
    assert all(abs(rec.yaws[0] - 0.0) < 1e-6 for rec in move_records[:-1])
    assert abs(move_records[-1].yaws[0] - 90.0) < 1e-6
    assert move_records[-1].positions == move_records[-2].positions


def test_build_yaw_control_dict_matches_phase_target() -> None:
    steps = [
        StepRecord(
            step=0,
            positions={0: [0, 0, 1], 1: [2, 0, 1]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 0.0, 1: 0.0},
        ),
        StepRecord(
            step=2,
            positions={0: [0, 5, 0], 1: [2, 5, 0]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 45.0, 1: 0.0},
        ),
    ]
    from flockwave.server.ext.path_planner.drone import Drone
    from flockwave.server.ext.path_planner.solver import SolverResult

    result = SolverResult(
        steps=steps,
        total_steps=2,
        drones=[
            Drone(drone_id=0, initial=(0, 0, 1), target=(0, 5, 0)),
            Drone(drone_id=1, initial=(2, 0, 1), target=(2, 5, 0)),
        ],
        success=True,
    )

    yaw_control = build_yaw_control_dict(result, 0, duration_ms=1000)
    assert yaw_control is not None
    assert yaw_control["setpoints"][-1][1] == 45.0

    shows = build_show_dicts(result, duration_ms=1000, takeoff_time=5.0)
    assert "yawControl" in shows[0]
    assert shows[0]["yawControl"]["setpoints"][-1][1] == 45.0
    assert shows[0]["yawControl"]["setpoints"][-1][0] == 1.7167


def test_phase_hold_starts_after_yaw_change() -> None:
    initial = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    phases = [
        {
            "name": "pose",
            "holdMs": 2000,
            "points": [
                {"droneId": "drone-1", "x": 0, "y": 0, "z": 1, "yaw": 90.0},
                {"droneId": "drone-2", "x": 2, "y": 0, "z": 1, "yaw": 45.0},
            ],
        }
    ]
    result, summaries = _plan_formation_phases(
        initial=initial,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=42,
        return_to_initial=False,
        initial_yaws=[0.0, 0.0],
    )

    assert summaries[0]["holdSteps"] == 2
    assert summaries[0]["arrivalStep"] == 1
    assert summaries[0]["endStep"] == 3
    assert result.steps[1].positions == result.steps[0].positions
    assert result.steps[1].yaws[0] == 90.0
    assert result.steps[2].yaws[0] == 90.0
    assert result.steps[3].yaws[0] == 90.0


def test_plan_formation_phases_resets_yaw_after_hold_before_return() -> None:
    initial = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    phases = [
        {
            "name": "pose",
            "holdMs": 1000,
            "points": [
                {"droneId": "drone-1", "x": 0, "y": 5, "z": 0, "yaw": 90.0},
                {"droneId": "drone-2", "x": 2, "y": 5, "z": 0, "yaw": 45.0},
            ],
        }
    ]
    result, summaries = _plan_formation_phases(
        initial=initial,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=42,
        return_to_initial=True,
        initial_yaws=[0.0, 0.0],
    )

    hold_end_step = summaries[0]["endStep"]
    reset_step_idx = next(
        idx for idx, rec in enumerate(result.steps) if rec.step == hold_end_step + 1
    )
    assert result.steps[reset_step_idx - 1].yaws[0] == 90.0
    assert result.steps[reset_step_idx].yaws[0] == 0.0
    assert (
        result.steps[reset_step_idx].positions
        == result.steps[reset_step_idx - 1].positions
    )

    return_move_records = [
        rec
        for rec in result.steps[reset_step_idx + 1 :]
        if rec.positions[0] != list(initial[0])
    ]
    assert return_move_records
    assert all(abs(rec.yaws[0] - 0.0) < 1e-6 for rec in return_move_records[:-1])
