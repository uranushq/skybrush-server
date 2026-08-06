"""Tests for path-planner request parsing helpers and phase planning."""

from __future__ import annotations

import pytest

from flockwave.server.ext.path_planner.converter import (
    DEFAULT_CRUISE_SPEED_M_S,
    DEFAULT_LANDING_SPEED_M_S,
    duration_ms_for_cruise_speed,
    solver_result_to_trajectory_dicts,
)
from flockwave.server.ext.path_planner.extension import (
    STACK_APPROACH_OFFSET,
    STACK_CLIMB_SPEED,
    PlanningError,
    _drone_index_from_id,
    _normalize_vec3_array,
    _phase_point_drone_index,
    _plan_formation_phases,
    _stack_entry_plan,
)
from flockwave.server.ext.path_planner.solver import Drone, SolverResult, StepRecord


def test_duration_ms_for_default_cruise_speed() -> None:
    # Default cruise is 1 m/s, so a 1 m step takes 1000 ms.
    assert DEFAULT_CRUISE_SPEED_M_S == 1.0
    assert duration_ms_for_cruise_speed(1.0, DEFAULT_CRUISE_SPEED_M_S) == 1000
    assert duration_ms_for_cruise_speed(1.0, 0.1) == 10000


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


# ── staged vertical stack entry ──────────────────────────────────────────


def test_stack_entry_plan_detects_column() -> None:
    targets = [(0.0, 0.0, 8.0), (0.0, 0.0, 6.0), (10.0, 0.0, 6.0)]
    approach, waves = _stack_entry_plan(targets, min_z=0.0)
    assert approach[0] == (0.0, 0.0, 8.0)  # top of the stack: normal entry
    assert approach[1] == (0.0, 0.0, 6.0 - STACK_APPROACH_OFFSET)
    assert approach[2] == (10.0, 0.0, 6.0)  # horizontally far: untouched
    assert waves == [[1]]


def test_stack_entry_plan_ignores_wide_vertical_gaps() -> None:
    targets = [(0.0, 0.0, 9.0), (0.0, 0.0, 6.0)]  # gap 3.0 m > 2.5 m
    approach, waves = _stack_entry_plan(targets, min_z=0.0)
    assert approach == [tuple(t) for t in targets]
    assert waves == []


def test_stack_entry_plan_three_deep_column_climbs_top_first() -> None:
    targets = [(0.0, 0.0, 9.0), (0.0, 0.0, 7.0), (0.0, 0.0, 5.0)]
    approach, waves = _stack_entry_plan(targets, min_z=3.0)
    assert approach[1] == (0.0, 0.0, 4.5)
    assert approach[2] == (0.0, 0.0, 3.0)  # clamped at min_z
    assert waves == [[1], [2]]  # drone above always settles first


def test_stack_entry_side_by_side_pairs_do_not_collapse() -> None:
    # Regression (image-wall field failure): two columns standing side by
    # side (lateral gap 1.455 m) must not be chained into one "column"; the
    # equal approach altitudes of their lower members are NOT a collapse
    # because they are on different vertical lines.
    targets = [
        (0.0, 0.0, 10.0),
        (0.0, 0.0, 8.8),
        (0.0, 1.455, 10.0),
        (0.0, 1.455, 8.8),
    ]
    approach, waves = _stack_entry_plan(targets, min_z=2.5, xy_tolerance=1.45)
    assert approach[0] == targets[0]
    assert approach[2] == targets[2]
    assert approach[1][2] == 8.8 - STACK_APPROACH_OFFSET
    assert approach[3][2] == 8.8 - STACK_APPROACH_OFFSET
    assert waves == [[1, 3]]


def test_stack_entry_image_wall_plane_passes() -> None:
    # A 9x8 vertical image wall (all x = 0, lateral gaps 1.5 m, vertical
    # gaps 1.455 m): every column stages independently, approaches inside a
    # column stay strictly ordered, and no false collapse is raised.
    targets = []
    for row in range(8):
        for col in range(9):
            targets.append((0.0, -6.0 + col * 1.5, 5.0 + row * 1.455))
    approach, waves = _stack_entry_plan(targets, min_z=2.5, xy_tolerance=1.45)
    # Top row untouched; every lower row staged.
    for col in range(9):
        top = 7 * 9 + col
        assert approach[top] == targets[top]
    for row in range(7):
        for col in range(9):
            index = row * 9 + col
            assert approach[index][2] == max(
                2.5, targets[index][2] - STACK_APPROACH_OFFSET
            )
    # Within one column the approach altitudes stay strictly increasing.
    for col in range(9):
        zs = [approach[row * 9 + col][2] for row in range(8)]
        assert all(b - a > 1e-9 for a, b in zip(zs, zs[1:]))
    assert len(waves) == 7  # one wave per stacked depth


def test_stack_entry_clamped_approaches_are_lifted_apart() -> None:
    # min_z clamping compresses the lower approaches of a chain; the lift
    # cascade must restore at least one separation of vertical gap between
    # staged approaches (bottom 2.5, middle lifted to 2.5 + 1.45).
    targets = [(0.0, 0.0, 6.0), (0.0, 0.0, 4.5), (0.0, 0.0, 3.0)]
    approach, _waves = _stack_entry_plan(targets, min_z=2.5)
    assert approach[2][2] == 2.5
    assert approach[1][2] == pytest.approx(2.5 + 1.45)
    assert approach[0] == targets[0]  # top of the chain: no staging


def test_stack_entry_plan_fails_loudly_when_approaches_collapse() -> None:
    # Targets that themselves violate the vertical separation (0.8 m gap)
    # cannot be staged: even after the lift cascade the approaches stay
    # closer than the separation and the plan must fail loudly.
    targets = [(0.0, 0.0, 4.0), (0.0, 0.0, 2.8), (0.0, 0.0, 2.0)]
    with pytest.raises(PlanningError):
        _stack_entry_plan(targets, min_z=2.5)


def test_stacked_phase_enters_from_below_at_constant_speed() -> None:
    # Drone 1 already sits at the stack top; drone 2 must approach 2.5 m
    # below its target and climb the last stretch vertically at
    # STACK_CLIMB_SPEED.
    start = [(0.0, 0.0, 10.0), (8.0, 0.0, 10.0)]
    phases = [
        {
            "name": "stack",
            "points": [
                {"x": 0.0, "y": 0.0, "z": 10.0},
                {"x": 0.0, "y": 0.0, "z": 8.0},
            ],
        }
    ]
    result, _summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=7,
        return_to_initial=False,
        min_z=0.0,
    )

    climb_steps = [rec for rec in result.steps if rec.constant_speed]
    assert climb_steps, "expected a staged climb segment"

    first_climb_index = next(
        i for i, rec in enumerate(result.steps) if rec.constant_speed
    )
    before_climb = result.steps[first_climb_index - 1].positions[1]
    assert before_climb == [0.0, 0.0, 8.0 - STACK_APPROACH_OFFSET]

    previous = before_climb
    for rec in climb_steps:
        pos = rec.positions[1]
        # purely vertical, exactly one climb step per record
        assert pos[0] == previous[0] and pos[1] == previous[1]
        assert pos[2] - previous[2] == pytest.approx(STACK_CLIMB_SPEED * 1.0)
        # the drone above never moves while someone climbs underneath
        assert rec.positions[0] == [0.0, 0.0, 10.0]
        previous = pos

    assert result.steps[-1].positions[1] == [0.0, 0.0, 8.0]
