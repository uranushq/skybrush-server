"""Tests for user-pinned fixed routes (경로 고정) in the path planner.

Contract under test (see ``solver`` module docstring item 6):

* a fixed-route drone follows its waypoint sequence verbatim — every flown
  position lies on the user's polyline, in order;
* it bypasses the dispatch gate (departs at step 1 regardless of priority);
* in conflicts the automatic drones yield/detour; the fixed drone at most
  holds in place on its own path;
* the extension exempts fixed-path drones from the staged stack entry and
  threads the route into the phase's approach segment.
"""

from __future__ import annotations

import pytest

from flockwave.server.ext.path_planner.extension import (
    _phase_fixed_routes,
    _plan_formation_phases,
)
from flockwave.server.ext.path_planner.solver import PathSolver


def _positions_of(result, did):
    return [tuple(rec.positions[did]) for rec in result.steps]


def _on_polyline(point, vertices, tolerance=1e-6) -> bool:
    """Whether *point* lies on the polyline through *vertices*."""
    for a, b in zip(vertices, vertices[1:]):
        ab = [b[i] - a[i] for i in range(3)]
        ap = [point[i] - a[i] for i in range(3)]
        ab_len_sq = sum(v * v for v in ab)
        if ab_len_sq < 1e-18:
            continue
        t = sum(ap[i] * ab[i] for i in range(3)) / ab_len_sq
        if -1e-9 <= t <= 1.0 + 1e-9:
            closest = [a[i] + t * ab[i] for i in range(3)]
            if (
                sum((point[i] - closest[i]) ** 2 for i in range(3))
                <= tolerance * tolerance
            ):
                return True
    return False


def test_fixed_route_followed_verbatim() -> None:
    start = (0.0, 0.0, 10.0)
    waypoints = [(5.0, 0.0, 10.0), (5.0, 5.0, 10.0), (10.0, 5.0, 10.0)]
    solver = PathSolver(
        [start], [waypoints[-1]], fixed_routes={0: list(waypoints)}
    )
    result = solver.solve()
    assert result.success

    flown = _positions_of(result, 0)
    polyline = [start, *waypoints]
    assert all(_on_polyline(pos, polyline) for pos in flown)
    # Each waypoint is visited exactly (unit steps divide the 5 m legs).
    for waypoint in waypoints:
        assert waypoint in flown
    # ...in order.
    indices = [flown.index(w) for w in waypoints]
    assert indices == sorted(indices)


def test_fixed_drone_wins_crossing_conflict() -> None:
    # Drone 0 is pinned to the straight y=0 lane; drone 1 crosses it at
    # (5, 0). The automatic drone must yield or detour — the fixed drone
    # never leaves its lane.
    initials = [(0.0, 0.0, 10.0), (5.0, -6.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (5.0, 6.0, 10.0)]
    solver = PathSolver(
        initials, targets, fixed_routes={0: [(10.0, 0.0, 10.0)]}
    )
    result = solver.solve()
    assert result.success

    lane = [initials[0], targets[0]]
    for pos in _positions_of(result, 0):
        assert _on_polyline(pos, lane), f"fixed drone left its lane at {pos}"
    assert tuple(result.steps[-1].positions[1]) == targets[1]


def test_fixed_route_departs_at_step_one() -> None:
    # Seven drones packed 2 m apart: the pinned drone departs at the very
    # first step alongside everyone else (all drones release simultaneously).
    n = 7
    initials = [(i * 2.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 2.0, 12.0, 10.0) for i in range(n)]
    solver = PathSolver(
        initials, targets, fixed_routes={6: [targets[6]]}
    )
    result = solver.solve()
    assert result.success
    first_step = result.steps[1]
    assert first_step.positions[6] != list(initials[6]), (
        "fixed-route drone should depart immediately"
    )


def test_all_fixed_drones_depart_simultaneously() -> None:
    # Ten drones, every one pinned to the straight line to its target: the
    # mover cap and release-distance stagger must not apply — all ten leave
    # at the very first step together.
    n = 10
    initials = [(i * 2.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 2.0, 15.0, 10.0) for i in range(n)]
    solver = PathSolver(
        initials,
        targets,
        fixed_routes={i: [targets[i]] for i in range(n)},
    )
    result = solver.solve()
    assert result.success
    first_step = result.steps[1]
    for did in range(n):
        assert first_step.positions[did] != list(initials[did]), (
            f"drone {did} did not depart at step 1"
        )
    # Parallel lanes 2 m apart: nobody should ever have to hold.
    assert result.total_steps == 15


def test_two_crossing_fixed_routes_resolve_by_holding() -> None:
    # Both drones pinned, lanes crossing at the origin: one must hold while
    # the other clears — neither may leave its polyline.
    initials = [(-6.0, 0.0, 10.0), (0.0, -6.0, 10.0)]
    targets = [(6.0, 0.0, 10.0), (0.0, 6.0, 10.0)]
    solver = PathSolver(
        initials,
        targets,
        fixed_routes={0: [targets[0]], 1: [targets[1]]},
    )
    result = solver.solve()
    assert result.success
    for did in (0, 1):
        lane = [initials[did], targets[did]]
        for pos in _positions_of(result, did):
            assert _on_polyline(pos, lane)


def test_solver_appends_missing_target_to_fixed_route() -> None:
    # Defensive: a route that stops short of the target is extended so the
    # arrival machinery still works.
    solver = PathSolver(
        [(0.0, 0.0, 10.0)],
        [(10.0, 0.0, 10.0)],
        fixed_routes={0: [(4.0, 3.0, 10.0)]},
    )
    result = solver.solve()
    assert result.success
    assert tuple(result.steps[-1].positions[0]) == (10.0, 0.0, 10.0)


def test_phase_fixed_routes_parses_drone_ids() -> None:
    phase = {
        "points": [],
        "fixedPaths": [
            {
                "droneId": "drone-2",
                "path": [{"x": 1.0, "y": 2.0, "z": 3.0}],
            }
        ],
    }
    assert _phase_fixed_routes(phase, 3) == {1: [(1.0, 2.0, 3.0)]}


def test_formation_phase_threads_fixed_route_through_transition() -> None:
    # Drone 1 must transit through the user's dogleg waypoint (3, 3, 5)
    # instead of flying the straight line to (6, 0, 5).
    start = [(0.0, 0.0, 5.0), (10.0, 0.0, 5.0)]
    phases = [
        {
            "name": "dogleg",
            "points": [
                {"droneId": "drone-1", "x": 6.0, "y": 0.0, "z": 5.0},
                {"droneId": "drone-2", "x": 10.0, "y": 0.0, "z": 5.0},
            ],
            "fixedPaths": [
                {
                    "droneId": "drone-1",
                    "path": [
                        {"x": 3.0, "y": 3.0, "z": 5.0},
                        {"x": 6.0, "y": 0.0, "z": 5.0},
                    ],
                }
            ],
        }
    ]
    result, summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=0.0,
    )
    flown = [tuple(rec.positions[0]) for rec in result.steps]
    assert (3.0, 3.0, 5.0) in flown
    polyline = [start[0], (3.0, 3.0, 5.0), (6.0, 0.0, 5.0)]
    assert all(_on_polyline(pos, polyline) for pos in flown)
    assert summaries and summaries[-1]["name"] == "dogleg"


def test_fixed_route_exempt_from_staged_stack_entry() -> None:
    # Drone 2's target sits 3 m under drone 1 (a stack). Normally it would
    # approach 4 m below and climb; with a pinned path it flies the user's
    # route directly and no constant-speed climb segment is emitted for it.
    start = [(0.0, 0.0, 10.0), (8.0, 0.0, 10.0)]
    phases = [
        {
            "name": "stack",
            "points": [
                {"droneId": "drone-1", "x": 0.0, "y": 0.0, "z": 10.0},
                {"droneId": "drone-2", "x": 0.0, "y": 0.0, "z": 7.0},
            ],
            "fixedPaths": [
                {
                    "droneId": "drone-2",
                    "path": [
                        {"x": 8.0, "y": 0.0, "z": 7.0},
                        {"x": 0.0, "y": 0.0, "z": 7.0},
                    ],
                }
            ],
        }
    ]
    result, _summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=0.0,
    )
    assert not any(rec.constant_speed for rec in result.steps)
    assert result.steps[-1].positions[1] == [0.0, 0.0, 7.0]
    flown = [tuple(rec.positions[1]) for rec in result.steps]
    polyline = [start[1], (8.0, 0.0, 7.0), (0.0, 0.0, 7.0)]
    assert all(_on_polyline(pos, polyline) for pos in flown)


def test_pinned_path_flies_through_downwash_column_when_collision_free() -> None:
    # A parked (arrived) drone hovers at z=10; the pinned lane passes 2 m
    # directly underneath — inside the 4 m downwash column that automatic
    # route planning treats as blocked. A pinned drone is exempt from every
    # such operational rule: only the physical envelope check applies, and a
    # 2 m vertical gap clears it, so the drone must fly its straight lane
    # without climbing over or waiting.
    initials = [(0.0, 0.0, 8.0), (5.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 8.0), (5.0, 0.0, 10.0)]  # drone 1 parked mid-lane above
    solver = PathSolver(
        initials, targets, fixed_routes={0: [targets[0]]}
    )
    result = solver.solve()
    assert result.success
    lane = [initials[0], targets[0]]
    for pos in _positions_of(result, 0):
        assert _on_polyline(pos, lane), f"pinned drone left its lane at {pos}"
    # No stalling either: the 10 m lane takes exactly 10 unit steps.
    assert result.total_steps == 10


def test_all_pinned_stack_descent_skips_staged_entry() -> None:
    # Regression: a vertical column (1.5 m gaps) descending as a unit, every
    # member pinned to its straight (vertical) line. The staged stack entry
    # used to run its collapse check BEFORE the pinned drones were exempted,
    # rejecting the plan with "approach points collapse at the minimum
    # altitude" even though no staged entry was needed at all.
    start = [
        (-3.0, 3.0, 12.5),
        (-3.0, 3.0, 11.0),
        (-3.0, 3.0, 9.5),
        (-3.0, 3.0, 8.0),
    ]
    targets = [
        {"droneId": "drone-1", "x": -3.0, "y": 3.0, "z": 9.0},
        {"droneId": "drone-2", "x": -3.0, "y": 3.0, "z": 7.5},
        {"droneId": "drone-3", "x": -3.0, "y": 3.0, "z": 6.0},
        {"droneId": "drone-4", "x": -3.0, "y": 3.0, "z": 4.5},
    ]
    phases = [
        {
            "name": "stack-down",
            "points": targets,
            "fixedPaths": [
                {"droneId": p["droneId"], "path": [dict(p)]} for p in targets
            ],
        }
    ]
    result, summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=2.5,
    )
    assert not any(rec.constant_speed for rec in result.steps)
    final = result.steps[-1].positions
    assert final[0] == [-3.0, 3.0, 9.0]
    assert final[3] == [-3.0, 3.0, 4.5]
    # Lockstep descent: the 1.5 m vertical gaps never shrink.
    for rec in result.steps:
        zs = [rec.positions[i][2] for i in range(4)]
        gaps = [round(zs[i] - zs[i + 1], 6) for i in range(3)]
        assert all(gap >= 1.5 - 1e-6 for gap in gaps), (rec.step, zs)
    assert summaries[-1]["name"] == "stack-down"


def test_pinned_convoy_with_crossing_descent_does_not_livelock() -> None:
    # Regression (user's phase-5): three pinned drones slide along the
    # z=5.5 lane in convoy (constant 1.5 m gaps) while a fourth pinned
    # drone descends into the lane at y=3 — crossing the convoy's path.
    # The old cluster resolution held EVERY cluster member but one, so the
    # descending drone's conflict froze a convoy member in front of its
    # follower and the trio livelocked until the stagnation guard fired.
    initials = [
        (5.0, 0.0, 5.5),
        (5.0, 1.5, 5.5),
        (5.0, 3.0, 5.5),
        (5.0, 3.0, 7.0),
    ]
    targets = [
        (5.0, 6.554, 5.5),
        (5.0, 8.054, 5.5),
        (5.0, 9.554, 5.5),
        (5.0, 3.0, 5.5),
    ]
    solver = PathSolver(
        initials,
        targets,
        fixed_routes={i: [targets[i]] for i in range(4)},
    )
    result = solver.solve()
    assert result.success, result.failure_reason
    for did in range(4):
        lane = [initials[did], targets[did]]
        for pos in _positions_of(result, did):
            assert _on_polyline(pos, lane), (did, pos)


def test_pinned_crossing_resolves_regardless_of_drone_id_order() -> None:
    # Same convoy-plus-descent crossing as the phase-5 regression, but with
    # the DESCENDING drone holding the lowest id. Who-flies-first must be
    # decided by geometry (drones standing on someone's lane leave first),
    # never by id order — with id-based ties this permutation livelocked.
    initials = [
        (5.0, 3.0, 7.0),   # descender (lowest id)
        (5.0, 0.0, 5.5),
        (5.0, 1.5, 5.5),
        (5.0, 3.0, 5.5),
    ]
    targets = [
        (5.0, 3.0, 5.5),
        (5.0, 6.554, 5.5),
        (5.0, 8.054, 5.5),
        (5.0, 9.554, 5.5),
    ]
    solver = PathSolver(
        initials,
        targets,
        fixed_routes={i: [targets[i]] for i in range(4)},
    )
    result = solver.solve()
    assert result.success, result.failure_reason
    for did in range(4):
        lane = [initials[did], targets[did]]
        for pos in _positions_of(result, did):
            assert _on_polyline(pos, lane), (did, pos)


def test_phase5_full_segment_from_field_data() -> None:
    # The exact 16-drone phase-4 -> phase-5 transition from the field log:
    # 8 drones pinned (lane spread + descents), 4 automatic, 4 stationary.
    start = [
        (14.6976, -3.0, 10.6639),
        (14.6976, -1.0, 10.6639),
        (14.6976, 1.0, 10.6639),
        (14.6976, 3.0, 10.6639),
        (5.0, 1.5, 10.5),
        (5.0, 1.5, 12.0),
        (5.0, 3.0, 12.0),
        (5.0, 1.5, 13.5),
        (5.0, -3.0, 5.5),
        (5.0, -1.5, 5.5),
        (5.0, -1.5, 7.0),
        (5.0, 0.0, 7.0),
        (5.0, 0.0, 5.5),
        (5.0, 1.5, 5.5),
        (5.0, 3.0, 5.5),
        (5.0, 3.0, 7.0),
    ]
    targets = [
        (8.0, 1.5, 12.0),
        (8.0, 0.0, 12.0),
        (8.0, 0.0, 10.5),
        (8.0, 1.5, 10.5),
        (5.0, 1.5, 10.5),
        (5.0, 1.5, 12.0),
        (5.0, 3.0, 12.0),
        (5.0, 1.5, 13.5),
        (5.0, -7.1596, 5.5),
        (5.0, -5.6596, 5.5),
        (5.0, -1.5, 5.5),
        (5.0, 0.0, 5.5),
        (5.0, 6.554, 5.5),
        (5.0, 8.054, 5.5),
        (5.0, 9.554, 5.5),
        (5.0, 3.0, 5.5),
    ]
    fixed = {i: [targets[i]] for i in range(8, 16)}
    solver = PathSolver(start, targets, fixed_routes=fixed)
    result = solver.solve()
    assert result.success, result.failure_reason
    for did in fixed:
        lane = [start[did], targets[did]]
        for pos in _positions_of(result, did):
            assert _on_polyline(pos, lane), (did, pos)
    # Downwash regression: the flown .skyc had drone-16 hovering only 0.5 m
    # above the lane while the convoy passed beneath. Every XY-overlapping
    # pair must now keep at least the hard minimum separation vertically.
    for rec in result.steps:
        for i in range(16):
            for j in range(i + 1, 16):
                pi, pj = rec.positions[i], rec.positions[j]
                if abs(pi[0] - pj[0]) < 0.7 and abs(pi[1] - pj[1]) < 0.7:
                    assert abs(pi[2] - pj[2]) >= 1.45 - 1e-6, (
                        rec.step, i, j, pi, pj,
                    )


def test_explicit_cluster_moves_as_rigid_block() -> None:
    # A 2x2 block declared as a cluster translates diagonally: lockstep,
    # internal gaps frozen, straight lines, minimal duration.
    start = [(0.0, 0.0, 5.0), (2.0, 0.0, 5.0), (0.0, 2.0, 5.0), (2.0, 2.0, 5.0)]
    phases = [
        {
            "name": "block-move",
            "points": [
                {"droneId": f"drone-{i + 1}", "x": s[0] + 6.0, "y": s[1] + 3.0, "z": 8.0}
                for i, s in enumerate(start)
            ],
            "clusters": [["drone-1", "drone-2", "drone-3", "drone-4"]],
        }
    ]
    result, _summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=0.0,
    )
    for rec in result.steps:
        p0 = rec.positions[0]
        for i in (1, 2, 3):
            pi = rec.positions[i]
            assert abs((pi[0] - p0[0]) - (start[i][0] - start[0][0])) < 1e-6
            assert abs((pi[1] - p0[1]) - (start[i][1] - start[0][1])) < 1e-6
            assert abs(pi[2] - p0[2]) < 1e-6


def test_rigid_group_is_auto_clustered_without_explicit_field() -> None:
    # A 1.5 m vertical column translating horizontally as a unit, WITHOUT
    # any clusters field. Auto-detection must pin the block: no staged
    # stack entry (its targets form a column, which would otherwise dip the
    # lower members 2.5 m down) and frozen internal geometry throughout.
    start = [(0.0, 0.0, 8.5), (0.0, 0.0, 7.0), (0.0, 0.0, 5.5)]
    phases = [
        {
            "name": "column-shift",
            "points": [
                {"droneId": f"drone-{i + 1}", "x": 8.0, "y": 4.0, "z": s[2]}
                for i, s in enumerate(start)
            ],
        }
    ]
    result, _summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=0.0,
    )
    assert not any(rec.constant_speed for rec in result.steps), (
        "auto-clustered column must skip the staged stack entry"
    )
    for rec in result.steps:
        p0 = rec.positions[0]
        for i in (1, 2):
            pi = rec.positions[i]
            assert abs(pi[0] - p0[0]) < 1e-6
            assert abs(pi[1] - p0[1]) < 1e-6
            assert abs((p0[2] - pi[2]) - 1.5 * i) < 1e-6, (rec.step, i)
    final = result.steps[-1].positions
    assert final[0] == [8.0, 4.0, 8.5]
    assert final[2] == [8.0, 4.0, 5.5]


def test_wall_to_stack_column_entry_resolves() -> None:
    # Regression (user's phase-3): three drones leave a vertical wall and
    # enter a single stacked column at 1.5 m gaps while a pinned group
    # departs the area. The staged approach points sit one separation below
    # already-parked drones — inside the A* downwash pads — and used to be
    # unreachable (deadlock): the goal-connect radius must bridge the
    # padded zone, and the skim-over guard must not mutually hold the
    # column-converging drones.
    start = [
        (12.6976, -3.0, 10.6639),
        (12.6976, -1.0, 10.6639),
        (12.6976, 3.0, 10.6639),
        (8.0, 0.0, 10.5),
        (8.0, 1.5, 10.5),
    ]
    phases = [
        {
            "name": "column",
            "points": [
                {"droneId": "drone-1", "x": 8.0, "y": 0.0, "z": 10.5},
                {"droneId": "drone-2", "x": 8.0, "y": 0.0, "z": 12.0},
                {"droneId": "drone-3", "x": 8.0, "y": 0.0, "z": 13.5},
                {"droneId": "drone-4", "x": 5.0, "y": -3.0, "z": 10.5},
                {"droneId": "drone-5", "x": 5.0, "y": -1.5, "z": 10.5},
            ],
            "fixedPaths": [
                {"droneId": "drone-4", "path": [{"x": 5.0, "y": -3.0, "z": 10.5}]},
                {"droneId": "drone-5", "path": [{"x": 5.0, "y": -1.5, "z": 10.5}]},
            ],
        }
    ]
    result, _summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=1,
        return_to_initial=False,
        min_z=2.5,
    )
    final = result.steps[-1].positions
    assert final[0] == [8.0, 0.0, 10.5]
    assert final[1] == [8.0, 0.0, 12.0]
    assert final[2] == [8.0, 0.0, 13.5]


def test_head_on_pinned_paths_fail_with_specific_error() -> None:
    # Two drones pinned to the SAME lane in opposite directions (a position
    # swap along one line): physically impossible without leaving the lane.
    # The failure must name the pinned-vs-pinned conflict, not the generic
    # "no progress" deadlock message.
    initials = [(0.0, 0.0, 10.0), (10.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (0.0, 0.0, 10.0)]
    solver = PathSolver(
        initials,
        targets,
        fixed_routes={0: [targets[0]], 1: [targets[1]]},
    )
    result = solver.solve()
    assert not result.success
    assert "user-pinned straight paths collide with each other" in (
        result.failure_reason or ""
    )
    assert result.fixed_conflicts
    conflicted = {c["drone"] for c in result.fixed_conflicts}
    assert conflicted & {0, 1}
    for conflict in result.fixed_conflicts:
        assert conflict["fixed_blockers"], "blocker should be a pinned drone"


def test_pinned_path_blocked_by_parked_drone_fails_with_specific_error() -> None:
    # An already-arrived drone sits permanently in the middle of the pinned
    # lane. The pinned drone can never detour, so the plan must fail and the
    # error must name the blocking drone (not a pinned one).
    initials = [(0.0, 0.0, 10.0), (5.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (5.0, 0.0, 10.0)]  # drone 1 already arrived
    solver = PathSolver(
        initials, targets, fixed_routes={0: [targets[0]]}
    )
    result = solver.solve()
    assert not result.success
    assert "user-pinned straight path is blocked" in (result.failure_reason or "")
    assert result.fixed_conflicts
    conflict = result.fixed_conflicts[0]
    assert conflict["drone"] == 0
    assert 1 in conflict["blocked_by"]
    assert conflict["fixed_blockers"] == []


def test_fixed_route_waits_out_a_parked_blocker_or_fails_loudly() -> None:
    # A waiting drone sits mid-lane on the fixed route. The fixed drone may
    # only hold (never detour); once the blocker is dispatched away the lane
    # clears. Either way the solver must terminate deterministically.
    initials = [(0.0, 0.0, 10.0), (5.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (5.0, 8.0, 10.0)]
    solver = PathSolver(
        initials, targets, fixed_routes={0: [targets[0]]}
    )
    result = solver.solve()
    assert result.success
    lane = [initials[0], targets[0]]
    for pos in _positions_of(result, 0):
        assert _on_polyline(pos, lane)
