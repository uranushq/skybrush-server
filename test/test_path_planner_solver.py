"""Tests for :mod:`flockwave.server.ext.path_planner.solver`."""

from __future__ import annotations

from flockwave.server.ext.path_planner.solver import PathSolver


def test_swept_colliding_crossing_paths() -> None:
    """Endpoints are separated on x but paths cross inside the 1 m envelope."""
    a0, a1 = [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
    b0, b1 = [2.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    assert PathSolver._swept_colliding(a0, a1, b0, b1)


def test_swept_not_colliding_parallel_offset() -> None:
    """Constant separation on y keeps the axis envelope clear for all t."""
    a0, a1 = [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]
    b0, b1 = [0.0, 2.0, 0.0], [10.0, 2.0, 0.0]
    assert not PathSolver._swept_colliding(a0, a1, b0, b1)


def test_pair_step_conflict_endpoints_only() -> None:
    solver = PathSolver([(0.0, 0.0, 0.0)], [(1.0, 0.0, 0.0)])
    assert solver._pair_step_conflict(
        [0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0],
        [0.2, 0.0, 0.0],
        [0.4, 0.0, 0.0],
    )
