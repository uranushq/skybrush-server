"""Tests for the solver's dispatch gate (staggered, altitude-first release).

Rules under test (see ``solver`` module docstring):

* never more than ``MAX_CONCURRENT_MOVERS`` drones moving in one step;
* a waiting drone is released only when every active mover is at least
  ``RELEASE_DISTANCE`` away — so nearby drones depart staggered while far
  apart drones fly concurrently;
* release priority is highest start altitude first;
* a waiting drone parked on a mover's target is boosted out instead of
  deadlocking the plan.
"""

from __future__ import annotations

from flockwave.server.ext.path_planner.solver import (
    MAX_CONCURRENT_MOVERS,
    RELEASE_DISTANCE,
    PathSolver,
)


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


def test_mover_cap_is_never_exceeded() -> None:
    # Ten independent parallel lanes (6 m apart, beyond RELEASE_DISTANCE):
    # without the cap all ten would fly at once.
    n = 10
    initials = [(i * 6.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 6.0, 15.0, 10.0) for i in range(n)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    per_step = _moving_ids_per_step(result)
    assert max(len(ids) for ids in per_step) <= MAX_CONCURRENT_MOVERS
    # The cap should actually bite in this scenario.
    assert any(len(ids) == MAX_CONCURRENT_MOVERS for ids in per_step)


def test_far_apart_drones_fly_concurrently() -> None:
    initials = [(0.0, 0.0, 10.0), (20.0, 0.0, 6.0)]
    targets = [(0.0, 10.0, 10.0), (20.0, 10.0, 6.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    first_movers = _moving_ids_per_step(result)[0]
    assert first_movers == {0, 1}


def test_nearby_lower_drone_departs_staggered() -> None:
    # Same start altitude, 2 m apart: drone 0 wins the tie and departs;
    # drone 1 must wait until drone 0 has pulled RELEASE_DISTANCE away.
    initials = [(0.0, 0.0, 10.0), (0.0, 2.0, 10.0)]
    targets = [(20.0, 0.0, 10.0), (0.0, 2.0, 10.0), ]
    targets[1] = (20.0, 2.0, 10.0)
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success

    departure_step = {}
    for step_index, moving in enumerate(_moving_ids_per_step(result)):
        for did in moving:
            departure_step.setdefault(did, step_index)
    assert departure_step[0] == 0
    # Drone 0 travels 1 m/step along x; distance to drone 1 is sqrt(x²+4),
    # which reaches 5 m only after ceil(sqrt(21)) = 5 steps.
    assert departure_step[1] >= 5
    # At drone 1's departure, every prior mover really was far enough away.
    release_positions = result.steps[departure_step[1]].positions
    dx = release_positions[0][0] - release_positions[1][0]
    dy = release_positions[0][1] - release_positions[1][1]
    dz = release_positions[0][2] - release_positions[1][2]
    assert (dx * dx + dy * dy + dz * dz) ** 0.5 >= RELEASE_DISTANCE


def test_higher_drone_departs_first() -> None:
    # Two drones close together at different altitudes: the higher one goes
    # first even though it has the *smaller* drone id disadvantage reversed.
    initials = [(0.0, 0.0, 6.0), (0.0, 2.0, 12.0)]
    targets = [(20.0, 0.0, 6.0), (20.0, 2.0, 12.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success
    first_movers = _moving_ids_per_step(result)[0]
    assert first_movers == {1}


def test_higher_destination_departs_first() -> None:
    # Priority is the DESTINATION altitude: from the same hover altitude,
    # the drone bound for the upper formation slot leaves first so the
    # lower-bound drone never flies in under an unfilled upper slot.
    initials = [(0.0, 0.0, 10.0), (0.0, 2.0, 10.0)]
    targets = [(20.0, 0.0, 8.0), (20.0, 2.0, 14.0)]
    result = PathSolver(initials, targets).solve()
    assert result.success
    first_movers = _moving_ids_per_step(result)[0]
    assert first_movers == {1}


def test_waiting_drone_on_target_is_boosted_not_deadlocked() -> None:
    # Drone 1 (lower, waiting) sits exactly on drone 0's target while being
    # within RELEASE_DISTANCE of drone 0 — without the boost this would be a
    # guaranteed stagnation failure.
    initials = [(0.0, 0.0, 10.0), (4.0, 0.0, 9.0)]
    targets = [(4.0, 0.0, 10.0), (4.0, 6.0, 9.0)]
    result = PathSolver(initials, targets, seed=1).solve()
    assert result.success


def test_all_waiting_drones_eventually_arrive() -> None:
    # Dense same-altitude line (2 m apart, all within RELEASE_DISTANCE of
    # their neighbours) shifting sideways: heavy staggering, but everyone
    # must still make it.
    n = 8
    initials = [(i * 2.0, 0.0, 10.0) for i in range(n)]
    targets = [(i * 2.0, 8.0, 10.0) for i in range(n)]
    result = PathSolver(initials, targets, seed=3).solve()
    assert result.success
    final = result.steps[-1].positions
    for i in range(n):
        assert final[i] == [i * 2.0, 8.0, 10.0]
