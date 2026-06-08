"""Tests for path-planner yaw interpolation and skyc yawControl output."""

from __future__ import annotations

from flockwave.server.ext.path_planner.converter import (
    DEFAULT_MAX_YAW_RATE_DEG_S,
    build_show_dicts,
    build_yaw_control_dict,
)
from flockwave.server.ext.path_planner.extension import _plan_formation_phases
from flockwave.server.ext.path_planner.solver import StepRecord


def _formation_arrival_index(steps) -> int:
    return next(
        idx
        for idx, rec in enumerate(steps)
        if round(rec.positions[0][1], 2) >= 5.0 and abs(rec.positions[0][2]) < 1e-6
    )


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
    arrival_idx = _formation_arrival_index(result.steps)
    assert all(abs(rec.yaws[0] - 0.0) < 1e-6 for rec in result.steps[1 : arrival_idx + 1])
    assert abs(result.steps[arrival_idx + 1].yaws[0] - 90.0) < 1e-6
    assert (
        result.steps[arrival_idx].positions == result.steps[arrival_idx + 1].positions
    )


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


def test_in_place_yaw_change_is_rate_limited() -> None:
    steps = [
        StepRecord(
            step=0,
            positions={0: [0, 0, 1]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 0.0},
        ),
        StepRecord(
            step=1,
            positions={0: [0, 0, 1]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 90.0},
        ),
    ]
    from flockwave.server.ext.path_planner.drone import Drone
    from flockwave.server.ext.path_planner.solver import SolverResult

    result = SolverResult(
        steps=steps,
        total_steps=1,
        drones=[Drone(drone_id=0, initial=(0, 0, 1), target=(0, 0, 1))],
        success=True,
    )

    yaw_control = build_yaw_control_dict(
        result,
        0,
        duration_ms=1000,
        max_yaw_rate_deg_s=90.0,
    )
    assert yaw_control is not None
    setpoints = yaw_control["setpoints"]
    assert setpoints[-1][1] == 90.0
    assert len(setpoints) >= 3

    first_90_t = min(t for t, yaw in setpoints if yaw == 90.0)
    last_0_before_turn = max(
        t for t, yaw in setpoints if yaw == 0.0 and t <= first_90_t
    )
    assert round(first_90_t - last_0_before_turn, 4) == round(90.0 / 90.0, 4)

    for idx in range(1, len(setpoints)):
        dt = setpoints[idx][0] - setpoints[idx - 1][0]
        dyaw = abs(
            (setpoints[idx][1] - setpoints[idx - 1][1] + 180.0) % 360.0 - 180.0
        )
        if dt > 1e-6 and dyaw > 1e-6:
            assert dyaw / dt <= 90.0 + 1.0


def test_negative_yaw_interpolates_without_wrapping_through_360() -> None:
    """0 -> -44 must ramp through negative values, not 349/316 degrees."""
    steps = [
        StepRecord(
            step=0,
            positions={0: [0, 0, 1]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 0.0},
        ),
        StepRecord(
            step=1,
            positions={0: [0, 0, 1]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: -44.0},
        ),
    ]
    from flockwave.server.ext.path_planner.drone import Drone
    from flockwave.server.ext.path_planner.solver import SolverResult

    result = SolverResult(
        steps=steps,
        total_steps=1,
        drones=[Drone(drone_id=0, initial=(0, 0, 1), target=(0, 0, 1))],
        success=True,
    )

    yaw_control = build_yaw_control_dict(
        result,
        0,
        duration_ms=1000,
        max_yaw_rate_deg_s=90.0,
    )
    assert yaw_control is not None
    setpoints = yaw_control["setpoints"]
    assert setpoints[-1][1] == -44.0

    ramp_yaws = [yaw for _, yaw in setpoints if -44.0 < yaw < 0.0]
    assert ramp_yaws
    assert all(yaw < 0.0 for yaw in ramp_yaws)
    assert all(yaw <= 0.0 for _, yaw in setpoints)


def test_in_place_yaw_uses_full_step_when_rate_allows() -> None:
    """90 deg in 2 s at 90 deg/s needs 1 s; hold setpoint remains at step end."""
    steps = [
        StepRecord(
            step=0,
            positions={0: [0, 5, 0]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 0.0},
        ),
        StepRecord(
            step=2,
            positions={0: [0, 5, 0]},
            collisions=[],
            reverted_drones=[],
            verified=True,
            yaws={0: 90.0},
        ),
    ]
    from flockwave.server.ext.path_planner.drone import Drone
    from flockwave.server.ext.path_planner.solver import SolverResult

    result = SolverResult(
        steps=steps,
        total_steps=2,
        drones=[Drone(drone_id=0, initial=(0, 5, 0), target=(0, 5, 0))],
        success=True,
    )

    yaw_control = build_yaw_control_dict(result, 0, duration_ms=1000)
    setpoints = yaw_control["setpoints"]
    ramp_times = [t for t, yaw in setpoints if 0.0 < yaw < 90.0]
    assert ramp_times
    assert setpoints[-1] == [2.0, 90.0]
    assert DEFAULT_MAX_YAW_RATE_DEG_S == 90.0


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
    assert summaries[0]["arrivalStep"] == 0
    assert summaries[0]["endStep"] == 2
    assert result.steps[0].positions == result.steps[1].positions
    assert result.steps[0].yaws[0] == 0.0
    assert result.steps[1].yaws[0] == 90.0
    assert result.steps[2].yaws[0] == 90.0


def test_yaw_changes_during_hold_after_arrival_not_before() -> None:
    """Yaw target is reached during hold; reset happens only after hold ends."""
    initial = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    phases = [
        {
            "name": "pose",
            "holdMs": 3000,
            "points": [
                {"droneId": "drone-1", "x": 0, "y": 5, "z": 0, "yaw": -44.0},
                {"droneId": "drone-2", "x": 2, "y": 5, "z": 0, "yaw": 44.0},
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

    arrival_idx = _formation_arrival_index(result.steps)
    hold_end_step = summaries[0]["endStep"]
    reset_idx = next(
        idx
        for idx, rec in enumerate(result.steps)
        if rec.step == hold_end_step + 1
    )

    assert result.steps[arrival_idx].yaws[0] == 0.0
    assert all(
        abs(result.steps[idx].yaws[0] + 44.0) < 1e-6
        for idx in range(arrival_idx + 1, reset_idx)
    )
    assert result.steps[reset_idx].yaws[0] == 0.0

    yaw_control = build_show_dicts(
        result, duration_ms=1000, takeoff_time=5.0
    )[0]["yawControl"]
    assert yaw_control is not None
    setpoints = yaw_control["setpoints"]
    takeoff_time = 5.0
    takeoff_duration = round(1.0 / 1.5, 4)
    arrival_t = round(takeoff_time + arrival_idx * 1.0 + takeoff_duration, 4)
    reset_t = round(
        takeoff_time + (hold_end_step + 1) * 1.0 + takeoff_duration, 4
    )
    first_ramp_t = min(t for t, yaw in setpoints if yaw < -1.0)
    first_reset_ramp_t = min(
        t for t, yaw in setpoints if t >= reset_t - 0.001 and yaw > -1.0
    )
    assert first_ramp_t >= arrival_t - 1e-3
    assert first_reset_ramp_t >= reset_t - 1e-3


def test_yaw_setpoints_match_trajectory_player_timeline() -> None:
    """Yaw changes must align with TrajectoryPlayer when takeoffTime is set."""
    from flockwave.server.show.player import TrajectoryPlayer
    from flockwave.server.show.trajectory import TrajectorySpecification

    initial = [[0.0, 0.0, 1.0], [2.0, 0.0, 1.0]]
    phases = [
        {
            "name": "pose",
            "holdMs": 3000,
            "points": [
                {"droneId": "drone-1", "x": 0, "y": 5, "z": 0, "yaw": -44.0},
                {"droneId": "drone-2", "x": 2, "y": 5, "z": 0, "yaw": 44.0},
            ],
        }
    ]
    result, _ = _plan_formation_phases(
        initial=initial,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=42,
        return_to_initial=True,
        initial_yaws=[0.0, 0.0],
    )
    takeoff_time = 5.0
    show = build_show_dicts(result, duration_ms=1000, takeoff_time=takeoff_time)[0]
    player = TrajectoryPlayer(TrajectorySpecification(show["trajectory"]))
    setpoints = show["yawControl"]["setpoints"]

    arrival_t = next(
        t
        for t in range(0, 40)
        if player.position_at(float(t))[1] >= 4.9 and player.position_at(float(t))[2] < 0.1
    )
    first_ramp_t = min(t for t, yaw in setpoints if yaw < -1.0)
    assert first_ramp_t >= float(arrival_t) - 0.05
    assert setpoints[0][0] == 0.0
    assert setpoints[1][0] == takeoff_time


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
