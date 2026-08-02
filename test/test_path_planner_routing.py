"""Tests for the route planner (lattice A* around parked drones).

Quality contract:

* a drone blocked by a parked formation flies a near-shortest route
  (no random wandering, no backtracking-by-accident);
* routes climb **over** obstacles — never below a parked drone's downwash
  column (the staged stack entry is the only sanctioned way underneath);
* the solver is fully deterministic.
"""

from __future__ import annotations

from flockwave.server.ext.path_planner.solver import PathSolver


def _drone_path(result, did):
    path = [result.steps[0].positions[did]]
    for rec in result.steps[1:]:
        if rec.positions[did] != path[-1]:
            path.append(rec.positions[did])
    return path


def _length(path):
    return sum(
        (
            (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2
        )
        ** 0.5
        for a, b in zip(path, path[1:])
    )


def _enters_downwash_column(path, obstacles, clearance=2.5) -> bool:
    """Whether any path point sits in an obstacle's padded vertical column.

    The column spans ``clearance`` above and below the parked drone (plus
    the envelope) within its XY footprint — routes must pass laterally
    (>= 0.7 m in XY) or keep the full vertical clearance.
    """
    for p in path:
        for obs in obstacles:
            if (
                abs(p[0] - obs[0]) < 0.7 - 1e-9
                and abs(p[1] - obs[1]) < 0.7 - 1e-9
                and -(0.5 + clearance) < p[2] - obs[2] < 0.5 + clearance
            ):
                # start/goal legitimately near own row ends are excluded by
                # construction in these scenarios
                return True
    return False


def test_blocked_drone_routes_near_optimally_around_parked_grid() -> None:
    # A 5x5 parked grid (2 m spacing) sits across the straight line. The
    # planner must fly a tight route (the 2.5 m vertical clearance makes the
    # lateral dodge around the grid edge cheaper than a 3 m climb) without
    # ever entering any parked drone's padded vertical column.
    parked = [(2.0 * (i % 5), 2.0 * (i // 5), 10.0) for i in range(25)]
    initials = parked + [(-4.0, 4.0, 10.0)]
    targets = parked + [(12.0, 4.0, 10.0)]
    result = PathSolver(initials, targets).solve()
    assert result.success

    path = _drone_path(result, 25)
    assert _length(path) < 24.0
    z_values = [p[2] for p in path]
    assert min(z_values) >= 10.0 - 1e-9  # never dips below its cruise level
    assert not _enters_downwash_column(path, parked)


def test_route_never_skims_over_or_under_a_parked_drone() -> None:
    # One parked drone dead on the straight line at the same altitude: the
    # route must keep the 2.5 m vertical clearance — passing laterally or
    # high, never skimming right above or below the parked drone.
    initials = [(0.0, 0.0, 10.0), (5.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (5.0, 0.0, 10.0)]
    result = PathSolver(initials, targets).solve()
    assert result.success

    path = _drone_path(result, 0)
    assert min(p[2] for p in path) >= 10.0 - 1e-9
    assert not _enters_downwash_column(path, [(5.0, 0.0, 10.0)])
    assert _length(path) < 14.0  # tight dodge, no wandering


def test_symmetric_square_rotation_resolves() -> None:
    # Four drones rotating one corner clockwise meet head-on in the middle;
    # sequenced escape planning must break the symmetry deterministically.
    initials = [(0, 0, 5), (10, 0, 5), (10, 10, 5), (0, 10, 5)]
    targets = [(10, 10, 5), (0, 10, 5), (0, 0, 5), (10, 0, 5)]
    result = PathSolver(initials, targets, min_z=2.5).solve()
    assert result.success


def test_parallel_formation_shift_is_near_straight() -> None:
    # KEY efficiency metric: flown length vs theoretical straight line.
    # A pure translation has no unavoidable conflicts, so the fleet overhead
    # must stay in the low single digits (was +8.8% before convoy-ordered
    # dispatch, now +0.9%).
    initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
    targets = [(x + 30.0, y, z) for x, y, z in initials]
    result = PathSolver(initials, targets).solve()
    assert result.success

    flown = sum(_length(_drone_path(result, did)) for did in range(25))
    straight = 25 * 30.0
    assert flown / straight - 1.0 < 0.03


def test_rotation_stress_overhead_is_bounded() -> None:
    # 180-degree rotation: every straight line crosses the centre and every
    # goal starts squatted — the pathological stress case. Guard that it
    # SOLVES and its overhead does not regress silently.
    initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
    targets = [(10.0 - x, 10.0 - y, z) for x, y, z in initials]
    result = PathSolver(initials, targets).solve()
    assert result.success

    flown = sum(_length(_drone_path(result, did)) for did in range(25))
    straight = sum(
        (
            (targets[i][0] - initials[i][0]) ** 2
            + (targets[i][1] - initials[i][1]) ** 2
        )
        ** 0.5
        for i in range(25)
    )
    assert flown / straight - 1.0 < 0.60  # currently ~0.49


def test_solver_is_fully_deterministic() -> None:
    parked = [(2.0 * (i % 3), 2.0 * (i // 3), 10.0) for i in range(9)]
    initials = parked + [(-4.0, 2.0, 10.0)]
    targets = parked + [(8.0, 2.0, 10.0)]
    first = PathSolver(initials, targets).solve()
    second = PathSolver(initials, targets).solve()
    assert first.success and second.success
    assert all(
        a.positions == b.positions for a, b in zip(first.steps, second.steps)
    )
