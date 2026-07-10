"""Tests for :mod:`flockwave.server.ext.path_planner.solver`."""

from __future__ import annotations

import random

from flockwave.server.ext.path_planner.collision_volume import (
    PLANNING_MARGIN,
    envelope_overlap,
    volumes_overlap_at_times,
)
from flockwave.server.ext.path_planner.solver import PathSolver


def test_swept_colliding_crossing_paths() -> None:
    """Endpoints are separated on x but paths cross inside the safety volumes."""
    a0, a1 = [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
    b0, b1 = [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    assert volumes_overlap_at_times(a0, a1, b0, b1)


def test_swept_not_colliding_parallel_offset() -> None:
    """Constant 10 m separation on y clears the envelopes for all t."""
    a0, a1 = [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]
    b0, b1 = [0.0, 10.0, 0.0], [10.0, 10.0, 0.0]
    assert not volumes_overlap_at_times(a0, a1, b0, b1)


def test_swept_overlap_endpoints_only() -> None:
    solver = PathSolver([(0.0, 0.0, 0.0)], [(1.0, 0.0, 0.0)])
    assert solver._swept_overlap(
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [0.2, 0.0, 0.0],
        [0.4, 0.0, 0.0],
    )


def _all_steps_clear(result) -> bool:
    for rec in result.steps:
        ids = sorted(rec.positions)
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                if envelope_overlap(
                    rec.positions[ids[i]],
                    rec.positions[ids[j]],
                    margin=PLANNING_MARGIN,
                ):
                    return False
    return True


def test_square_corner_swap_is_collision_free() -> None:
    initials = [(0, 0, 5), (10, 0, 5), (10, 10, 5), (0, 10, 5)]
    targets = [(10, 10, 5), (0, 10, 5), (0, 0, 5), (10, 0, 5)]
    result = PathSolver(initials, targets, seed=42, min_z=2.5).solve()
    assert result.success
    assert all(rec.verified for rec in result.steps)
    assert _all_steps_clear(result)


def test_min_z_floor_is_respected() -> None:
    initials = [(0, 0, 5), (10, 0, 5)]
    targets = [(10, 0, 5), (0, 0, 5)]
    result = PathSolver(initials, targets, seed=7, min_z=2.5).solve()
    min_seen = min(
        rec.positions[d][2] for rec in result.steps for d in rec.positions
    )
    assert min_seen >= 2.5 - 1e-9


def test_overlapping_targets_fail_fast_with_reason() -> None:
    result = PathSolver(
        [(0, 0, 5), (5, 0, 5)], [(10, 0, 5), (10.3, 0, 5)], seed=1
    ).solve()
    assert not result.success
    assert result.failure_reason is not None
    assert "target" in result.failure_reason


def test_overlapping_initials_fail_fast_with_reason() -> None:
    result = PathSolver(
        [(0, 0, 5), (0.3, 0, 5)], [(10, 0, 5), (20, 0, 5)], seed=1
    ).solve()
    assert not result.success
    assert result.failure_reason is not None
    assert "initial" in result.failure_reason


def test_global_rng_is_not_polluted() -> None:
    state_before = random.getstate()
    PathSolver([(0, 0, 5)], [(5, 0, 5)], seed=99).solve()
    assert random.getstate() == state_before


def test_seeded_runs_are_reproducible() -> None:
    initials = [(0, 0, 5), (10, 0, 5), (10, 10, 5), (0, 10, 5)]
    targets = [(10, 10, 5), (0, 10, 5), (0, 0, 5), (10, 0, 5)]
    a = PathSolver(initials, targets, seed=5, min_z=2.5).solve()
    b = PathSolver(initials, targets, seed=5, min_z=2.5).solve()
    assert [r.positions for r in a.steps] == [r.positions for r in b.steps]
