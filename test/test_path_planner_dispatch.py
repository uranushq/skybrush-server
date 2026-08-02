"""Tests for the solver's release policy (simultaneous, cluster-coordinated).

The former dispatch gate (hard cap of 5 concurrent movers, 5 m horizontal
release distance, altitude-first staggered departure) was removed: forcing
members of a coherent group to depart separately created more close
encounters than it prevented. Rules under test now:

* every drone departs at the very first step — no cap, no stagger;
* a formation translating as a unit moves in lockstep, so all relative
  separations are preserved exactly and nobody ever holds;
* conflicting motions are still sequenced per step by the collision-cluster
  admission, so crossing scenarios remain collision-free by construction.
"""

from __future__ import annotations

from flockwave.server.ext.path_planner.solver import PathSolver


def _moving_ids_per_step(result):
    """Set of drones whose position changed at each step transition."""
    moving = []
    for prev, cur in zip(result.steps, result.steps[1:]):
        moving.append(
            {
                did
                for did in cur.positions
                if cur.positions[did] != prev.positions[did]
            }
        )
    return moving


def test_everyone_departs_at_step_one() -> None:
    # Ten parallel lanes 6 m apart: all ten fly at once from the start.
    n = 10
    initials = [(i * 6.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 6.0, 15.0, 10.0) for i in range(n)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    assert _moving_ids_per_step(result)[0] == set(range(n))


def test_dense_formation_translates_in_lockstep() -> None:
    # Dense same-altitude line (2 m apart) shifting sideways: the whole
    # formation moves as a unit — separations never shrink, nobody holds,
    # and the transition takes exactly the straight-line number of steps.
    n = 8
    initials = [(i * 2.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 2.0, 8.0, 10.0) for i in range(n)]
    result = PathSolver(initials, targets, seed=3).solve()
    assert result.success
    assert result.total_steps == 8
    for step in result.steps:
        xs = sorted(step.positions[i][0] for i in range(n))
        gaps = [round(xs[i + 1] - xs[i], 6) for i in range(n - 1)]
        assert all(gap >= 2.0 - 1e-6 for gap in gaps)
    final = result.steps[-1].positions
    for i in range(n):
        assert final[i] == [i * 2.0, 8.0, 10.0]


def test_close_drones_at_different_altitudes_fly_together() -> None:
    # 2 m apart horizontally, 6 m apart vertically: both fly immediately —
    # no release-distance rule holds the lower one back any more.
    initials = [(0.0, 0.0, 6.0), (0.0, 2.0, 12.0)]
    targets = [(20.0, 0.0, 6.0), (20.0, 2.0, 12.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    assert _moving_ids_per_step(result)[0] == {0, 1}
    assert result.total_steps == 20


def test_drone_parked_on_anothers_target_resolves() -> None:
    # Drone 1 starts exactly on drone 0's target. Both released at once:
    # drone 1 flies off toward its own target and drone 0 follows in.
    initials = [(0.0, 0.0, 10.0), (4.0, 0.0, 9.0)]
    targets = [(4.0, 0.0, 10.0), (4.0, 6.0, 9.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    final = result.steps[-1].positions
    assert final[0] == [4.0, 0.0, 10.0]
    assert final[1] == [4.0, 6.0, 9.0]


def test_crossing_paths_remain_collision_free() -> None:
    # Two drones swapping sides through a common corridor: simultaneous
    # release must still be sequenced safely by the cluster admission.
    initials = [(0.0, 0.0, 10.0), (10.0, 0.6, 10.0)]
    targets = [(10.0, 0.0, 10.0), (0.0, 0.6, 10.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    # Every recorded step is verified collision-free by construction.
    assert all(rec.verified for rec in result.steps)
