"""Tests for path-planner request parsing helpers and phase planning."""

from __future__ import annotations

from itertools import combinations

import pytest

from flockwave.server.ext.path_planner.collision_volume import envelope_overlap_swept
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
    _corridors_conflict,
    _detect_rigid_groups,
    _dispatch_waves,
    _drone_index_from_id,
    _normalize_vec3_array,
    _phase_point_drone_index,
    _pin_flyable_clusters,
    _pinned_blocks,
    _pinned_corridors_deadlock,
    _plan_formation_phases,
    _stack_entry_plan,
    _staging_spread_targets,
    _validate_phase_clusters,
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


# ── staging layout ───────────────────────────────────────────────────────


def _chebyshev(a, b) -> float:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def test_staging_keeps_a_takeoff_layout_that_already_clears_the_spacing() -> None:
    # A 2 m take-off grid is already spread out, so staging must not move a
    # single drone -- it used to re-shuffle everyone onto a half-cell-offset
    # grid, which deadlocked the solver.
    hover = [(x, y, 20.0) for x in (0, 2, 4, 6, 8) for y in (0, 2, 4, 6, 8, 10)]

    assert _staging_spread_targets(hover, 2.0) == hover


def test_staging_spreads_a_tight_layout_about_drone_one() -> None:
    hover = [(x, y, 20.0) for x in (0, 1, 2) for y in (0, 1, 2)]
    targets = _staging_spread_targets(hover, 2.0)

    # drone-1 is the anchor and stays exactly where it took off
    assert targets[0] == hover[0]
    # every pair clears the requested spacing afterwards
    for a, b in combinations(targets, 2):
        assert _chebyshev(a, b) >= 2.0
    # the take-off shape is preserved: one uniform horizontal scale, no
    # altitude change
    assert [t[2] for t in targets] == [p[2] for p in hover]
    assert targets[1][1] / hover[1][1] == pytest.approx(targets[3][0] / hover[3][0])


def test_staging_spread_never_brings_two_drones_closer_together() -> None:
    # Scaling about a fixed point is monotone, which is what makes the
    # staging move collision-free without any avoidance planning.
    hover = [(0.0, 0.0, 20.0), (0.9, 0.4, 20.0), (2.3, 1.1, 20.0), (0.2, 1.8, 20.0)]
    targets = _staging_spread_targets(hover, 2.0)

    for i, j in combinations(range(len(hover)), 2):
        previous = _chebyshev(hover[i], hover[j])
        for step in range(1, 21):
            t = step / 20.0
            a = [hover[i][k] + (targets[i][k] - hover[i][k]) * t for k in range(3)]
            b = [hover[j][k] + (targets[j][k] - hover[j][k]) * t for k in range(3)]
            distance = _chebyshev(a, b)
            assert distance >= previous - 1e-9
            previous = distance


def test_staging_ignores_pairs_already_separated_by_altitude() -> None:
    # Separation is Chebyshev, so a pair split by altitude alone is fine.
    # Scaling it apart horizontally would blow the whole layout up.
    hover = [(0.0, 0.0, 20.0), (0.1, 0.0, 22.0), (3.0, 0.0, 20.0)]

    assert _staging_spread_targets(hover, 2.0) == hover


def test_staging_spread_survives_a_single_drone() -> None:
    assert _staging_spread_targets([(1.0, 2.0, 20.0)], 2.0) == [(1.0, 2.0, 20.0)]


# ── automatic rigid-group clustering ─────────────────────────────────────


def test_rigid_groups_pin_blocks_that_do_not_get_in_each_others_way() -> None:
    # Two blocks translating well clear of one another: both fly in lockstep.
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (0.0, 20.0, 10.0), (0.0, 23.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (-10.0, 20.0, 10.0), (-10.0, 23.0, 10.0)]

    pinned = _detect_rigid_groups(
        current, targets, excluded=set(), static_positions=[], separation=1.45
    )

    assert set(pinned) == {0, 1, 2, 3}


def test_rigid_groups_do_not_pin_two_blocks_onto_crossing_corridors() -> None:
    # Head-on blocks whose lanes are only 1 m apart. Pinning both would take
    # away everyone's right to detour and deadlock the segment, so only one
    # block keeps its pins and the other falls back to normal planning.
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (10.0, 1.0, 10.0), (10.0, 4.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (0.0, 1.0, 10.0), (0.0, 4.0, 10.0)]

    pinned = _detect_rigid_groups(
        current, targets, excluded=set(), static_positions=[], separation=1.45
    )

    assert set(pinned) == {0, 1}, "the larger/earlier block wins, the clashing one yields"


def test_rigid_groups_keep_clear_of_a_user_pinned_route() -> None:
    # drone-3 is pinned by hand head-on down drone-1's lane, swapping places
    # with it. Neither can detour and neither order works, so the block must
    # not be pinned against it.
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (10.0, 0.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (0.0, 0.0, 10.0)]
    routes = [[current[2], targets[2]]]

    assert _detect_rigid_groups(
        current, targets, excluded={2}, static_positions=[], separation=1.45
    ), "sanity: without the route the block is pinned"

    pinned = _detect_rigid_groups(
        current,
        targets,
        excluded={2},
        static_positions=[],
        pinned_routes=routes,
        separation=1.45,
    )

    assert pinned == {}


def test_corridors_conflict_compares_distance_flown_not_normalised_time() -> None:
    # A flies 20 m; B flies 4 m and then parks 1 m off A's lane. A passes
    # the parked B half way down its own run. Rescaling both onto t=[0, 1]
    # would put B at the far end of its path while A is only a fifth of the
    # way along and miss it entirely.
    a_start, a_end = (0.0, 0.0, 10.0), (20.0, 0.0, 10.0)
    b_start, b_end = (10.0, 5.0, 10.0), (10.0, 1.0, 10.0)

    assert _corridors_conflict(a_start, a_end, b_start, b_end, separation=1.45)
    # the naive same-duration reading of the very same corridors
    assert not envelope_overlap_swept(
        list(a_start), list(a_end), list(b_start), list(b_end), separation=1.45
    )


def test_corridors_conflict_clears_genuinely_separate_lanes() -> None:
    assert not _corridors_conflict(
        (0.0, 0.0, 10.0), (20.0, 0.0, 10.0),
        (10.0, 5.0, 10.0), (10.0, 2.0, 10.0),
        separation=1.45,
    )


# ── explicit per-phase clusters ──────────────────────────────────────────


def test_cluster_that_translates_keeps_its_lockstep_pins() -> None:
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (0.0, 6.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (10.0, 6.0, 10.0)]
    routes: dict = {}

    groups, notes = _pin_flyable_clusters(
        routes,
        [{0, 1, 2}],
        current_positions=current,
        targets=targets,
        separation=1.45,
        label="phase-1",
    )

    assert notes == []
    assert routes == {i: [targets[i]] for i in range(3)}
    assert groups == [{0, 1, 2}], "the block must also be handed over as one unit"


def test_cluster_that_contracts_is_planned_normally_instead() -> None:
    # The real phase-2 block: four drones fanned out over a 10 m line
    # converging onto a 1.5 m wedge. The target formation is legal, but two
    # of the straight lines to it block each other whichever goes first, so
    # the block must not be pinned.
    current = [(24.0, 8.0, 20.0), (24.0, 10.0, 20.0), (26.0, 0.0, 20.0), (26.0, 2.0, 20.0)]
    targets = [(18.0, 3.0, 39.5), (18.0, 1.5, 39.5), (18.0, 1.5, 38.0), (18.0, 0.0, 38.0)]
    routes: dict = {}

    groups, notes = _pin_flyable_clusters(
        routes,
        [{0, 1, 2, 3}],
        current_positions=current,
        targets=targets,
        separation=1.45,
        label="phase-2",
    )

    assert routes == {}, "a contracting block must keep its right to detour"
    assert groups == []
    assert len(notes) == 1
    assert "drone-1" in notes[0] and "drone-2" in notes[0]

    # sanity: the formation the block is heading for is itself legal
    for a, b in combinations(targets, 2):
        assert _chebyshev(a, b) >= 1.45


def test_unschedulable_cluster_is_released_instead_of_failing_the_show() -> None:
    # Geometry alone cannot tell whether a block's lines are schedulable
    # alongside the rest of the fleet — this one has a valid flight order on
    # paper that the solver never finds. Rather than failing a show whose
    # formations are all legal, the clusters are released and the segment is
    # replanned normally.
    start = [
        (24.0, 0.0, 20.0), (24.0, 2.0, 20.0), (24.0, 4.0, 20.0), (24.0, 6.0, 20.0),
        (24.0, 8.0, 20.0), (24.0, 10.0, 20.0), (26.0, 0.0, 20.0), (26.0, 2.0, 20.0),
    ]
    targets = [
        (12.0, 6.0, 39.5), (12.0, 6.0, 38.0), (12.0, 4.5, 38.0), (12.0, 3.0, 38.0),
        (18.0, 3.0, 39.5), (18.0, 1.5, 39.5), (18.0, 1.5, 38.0), (18.0, 0.0, 38.0),
    ]
    phases = [
        {
            "name": "wedge",
            "points": [{"x": x, "y": y, "z": z} for x, y, z in targets],
            "clusters": [
                ["drone-1", "drone-2", "drone-3", "drone-4"],
                ["drone-5", "drone-6", "drone-7", "drone-8"],
            ],
        }
    ]

    result, summaries = _plan_formation_phases(
        start_positions=start,
        phases=phases,
        step_size=1.0,
        duration_ms=1000,
        seed=7,
        return_to_initial=False,
        min_z=2.5,
    )

    assert result.success
    assert [tuple(result.steps[-1].positions[i]) for i in range(8)] == targets
    warnings = summaries[0]["warnings"]
    assert any("released" in w for w in warnings), warnings


def test_cluster_keeps_fixed_path_members_in_the_lockstep_group() -> None:
    # Every member is also pinned by hand. The old code filtered those out
    # and the group vanished, so the three drones left independently as
    # soon as each one's own lane cleared.
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (0.0, 6.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (10.0, 6.0, 10.0)]
    drawn = [(5.0, 6.0, 12.0), (10.0, 6.0, 10.0)]
    routes = {2: drawn}

    groups, notes = _pin_flyable_clusters(
        routes,
        [{0, 1, 2}],
        current_positions=current,
        targets=targets,
        separation=1.45,
        label="phase-1",
    )

    assert notes == []
    assert groups == [{0, 1, 2}]
    assert routes[2] == drawn, "a hand-drawn path is never overwritten"
    assert routes[0] == [targets[0]] and routes[1] == [targets[1]]


def test_clusters_may_overlap_fixed_paths() -> None:
    # The pin fixes a drone's geometry, the cluster fixes the block's
    # timing; a hand-drawn path joins a block by appearing in both.
    phase = {
        "clusters": [["drone-1", "drone-3"]],
        "fixedPaths": [{"droneId": "drone-3", "path": [{"x": 0, "y": 0, "z": 5}]}],
    }

    assert _validate_phase_clusters(phase, 0, 4) is None


def test_pinned_corridors_that_can_be_staggered_are_not_a_deadlock() -> None:
    # drone-1 descends onto the spot drone-2 is vacating. Flown together
    # they would pass 0.75 m apart, but sending drone-2 first resolves it --
    # exactly what the solver does, so this must not be rejected.
    descend = ((12.0, 6.0, 21.5), (12.0, 6.0, 20.0))
    slide = ((12.0, 6.0, 20.0), (12.0, 15.0, 20.0))

    assert _corridors_conflict(*descend, *slide, separation=1.45)
    assert not _pinned_corridors_deadlock(*descend, *slide, separation=1.45)


def test_pinned_corridors_swapping_one_lane_are_a_deadlock() -> None:
    # A head-on swap along a single line: neither order clears the other.
    assert _pinned_corridors_deadlock(
        (0.0, 0.0, 10.0), (10.0, 0.0, 10.0),
        (10.0, 0.0, 10.0), (0.0, 0.0, 10.0),
        separation=1.45,
    )


# ── dispatch order between pinned blocks ─────────────────────────────────


def test_block_bound_for_an_occupied_lane_is_dispatched_second() -> None:
    # The real phase-7 clash: one block descends the column at (12, 3, ·)
    # while another flies in from x=18 to fill the very altitudes it is
    # still passing through. Sent together, whoever arrives first parks and
    # the pinned descender can never get by, so the descent goes first.
    current = [
        (12.0, 3.0, 39.5), (12.0, 3.0, 38.0),          # block A: descends
        (18.0, 3.0, 36.5), (18.0, 3.0, 38.0),          # block B: flies in
    ]
    targets = [
        (12.0, 3.0, 23.0), (12.0, 3.0, 21.5),
        (12.0, 3.0, 36.5), (12.0, 3.0, 38.0),
    ]

    waves = _dispatch_waves(
        [{0, 1}, {2, 3}],
        current_positions=current,
        targets=targets,
        separation=1.45,
    )

    assert waves == [[{0, 1}], [{2, 3}]]


def test_independent_blocks_share_one_wave() -> None:
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (0.0, 20.0, 10.0), (0.0, 23.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (10.0, 20.0, 10.0), (10.0, 23.0, 10.0)]

    waves = _dispatch_waves(
        [{0, 1}, {2, 3}],
        current_positions=current,
        targets=targets,
        separation=1.45,
    )

    assert waves == [[{0, 1}, {2, 3}]]


def test_blocks_that_need_each_others_lanes_have_no_order() -> None:
    # Each block is bound for the lane the other still has to fly: no
    # running order can satisfy both, so none is imposed.
    current = [(0.0, 0.0, 10.0), (0.0, 3.0, 10.0), (10.0, 0.0, 10.0), (10.0, 3.0, 10.0)]
    targets = [(10.0, 0.0, 10.0), (10.0, 3.0, 10.0), (0.0, 0.0, 10.0), (0.0, 3.0, 10.0)]

    assert (
        _dispatch_waves(
            [{0, 1}, {2, 3}],
            current_positions=current,
            targets=targets,
            separation=1.45,
        )
        == []
    )


def test_only_whole_pinned_blocks_get_a_running_order() -> None:
    pinned = {0: [(0.0, 0.0, 0.0)], 1: [(0.0, 0.0, 0.0)], 2: [(0.0, 0.0, 0.0)]}

    # a block whose members are all pinned qualifies; a lone pinned drone
    # does not, and neither does a group with an unpinned member
    assert _pinned_blocks(pinned, [{0, 1}]) == [{0, 1}]
    assert _pinned_blocks(pinned, [{2}]) == []
    assert _pinned_blocks(pinned, [{2, 9}]) == []
    # overlapping groups are not double-counted
    assert _pinned_blocks(pinned, [{0, 1}, {1, 2}]) == [{0, 1}]


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
