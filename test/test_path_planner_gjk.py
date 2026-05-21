"""Tests for Voronoi/GJK path-planner helpers."""

from __future__ import annotations

import math

from flockwave.server.ext.path_planner.solver import PathSolver
from flockwave.server.ext.path_planner.voronoi import (
    add_bounding_box,
    build_bvc_halfspaces,
    closest_point_in_polytope,
    contains_point,
    support_point,
)


def test_query_inside_unit_cube_returns_query() -> None:
    halfspaces = add_bounding_box([], (0.0, 0.0, 0.0), 1.0)
    query = (0.1, 0.2, 0.3)
    assert contains_point(halfspaces, query)
    assert closest_point_in_polytope(halfspaces, query) == query


def test_query_outside_box_projects_to_face() -> None:
    halfspaces = add_bounding_box([], (0.0, 0.0, 0.0), 1.0)
    query = (2.0, 0.0, 0.0)
    closest = closest_point_in_polytope(
        halfspaces, query, interior_hint=(0.0, 0.0, 0.0)
    )
    assert closest[0] == 1.0
    assert closest[1] == 0.0
    assert closest[2] == 0.0


def test_bvc_separates_close_neighbors() -> None:
    """Two drones 2 m apart with 1.5 m envelope should keep centers separated."""
    p_i = (0.0, 0.0, 0.0)
    p_j = (2.0, 0.0, 0.0)
    halfspaces = build_bvc_halfspaces(
        p_i,
        [p_j],
        radii=(0.75, 0.75, 0.75),
        use_ellipsoid=True,
    )
    midpoint_goal = (1.0, 0.0, 0.0)
    closest = closest_point_in_polytope(halfspaces, midpoint_goal, interior_hint=p_i)
    assert closest[0] < 1.0


def test_solver_two_drones_separate_goals() -> None:
    """Neighboring drones reach distinct goals without overlapping."""
    solver = PathSolver(
        initials=[(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)],
        targets=[(0.0, 5.0, 0.0), (6.0, 5.0, 0.0)],
        step_size=1.0,
        seed=1,
    )
    result = solver.solve()
    assert result.success
    assert result.total_steps < 200
    final = {d.drone_id: d.position for d in result.drones}
    assert not PathSolver._is_colliding(final[0], final[1])


def test_voronoi_waypoint_moves_toward_goal_when_alone() -> None:
    solver = PathSolver([(0.0, 0.0, 0.0)], [(10.0, 0.0, 0.0)])
    drone = solver.drones[0]
    wp = solver._voronoi_waypoint(drone, [])
    assert wp == drone.target


def test_four_drone_formation_reaches_targets() -> None:
    solver = PathSolver(
        initials=[(0.0, 0.0, 0.0), (3.0, 0.0, 0.0), (0.0, 3.0, 0.0), (3.0, 3.0, 0.0)],
        targets=[(3.0, 3.0, 0.0), (0.0, 3.0, 0.0), (3.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
        step_size=1.0,
        seed=1,
    )
    result = solver.solve()
    assert result.success
    assert result.total_steps < 500


def test_ten_drone_formation_completes_quickly() -> None:
    """Ten-drone plans must finish within a practical step budget."""
    import time

    n = 10
    initials = [(float(i % 5), float(i // 5), 0.0) for i in range(n)]
    targets = [(float((i + 2) % 5), float((i + 3) % 5), 2.0) for i in range(n)]
    t0 = time.perf_counter()
    result = PathSolver(initials=initials, targets=targets, step_size=1.0, seed=1).solve()
    elapsed = time.perf_counter() - t0
    assert elapsed < 5.0
    assert result.total_steps <= 3000


def test_support_point_scales_with_neighbor_count() -> None:
    """Support mapping must stay fast for large swarms."""
    import time

    center = (0.0, 0.0, 0.0)
    neighbors = [
        (
            10.0 * math.cos(i),
            10.0 * math.sin(i),
            float(i % 5),
        )
        for i in range(80)
    ]
    halfspaces = build_bvc_halfspaces(center, neighbors, radii=(0.75, 0.75, 0.75))
    halfspaces = add_bounding_box(halfspaces, center, 50.0)

    t0 = time.perf_counter()
    for _ in range(200):
        support_point(halfspaces, (1.0, 0.5, 0.2))
    elapsed = time.perf_counter() - t0
    assert elapsed < 2.0
