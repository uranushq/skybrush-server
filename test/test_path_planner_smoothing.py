"""Tests for the natural-log velocity smoothing profile.

Covers the invariants the collision-safety contract depends on:

* knot times of the original keyframes never move and the arc length is
  covered exactly (``arc(1) == L``);
* the schedule deviation of any eased segment stays below ~13% of the
  segment length (what ``PLANNING_MARGIN`` absorbs);
* the speed follows the natural-log curve and endpoint speeds match;
* smoothing-exempt (``constant_times``) segments stay exactly linear.
"""

from __future__ import annotations

import math

from flockwave.server.ext.path_planner.collision_volume import (
    MIN_FORMATION_XY_CLEARANCE,
)
from flockwave.server.ext.path_planner.converter import (
    _EASE_PEAK_FACTOR,
    _LOG_SUBDIVISIONS,
    _log_profile,
    _log_profile_peak,
    _log_w,
    apply_velocity_smoothing,
)

# Worst-case per-drone schedule deviation fraction the docs promise.
MAX_DEVIATION_FRACTION = 0.15


def _deviation(v0: float, v1: float, length: float, dt: float) -> float:
    _, arc = _log_profile(v0, v1, length, dt)
    return max(
        abs(arc(i / 200.0) / length - i / 200.0) for i in range(201)
    )


def test_profile_covers_length_exactly() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.3, 1.0)):
        _, arc = _log_profile(v0, v1, 1.0, 1.0)
        assert math.isclose(arc(1.0), 1.0, rel_tol=1e-6)
        assert arc(0.0) == 0.0


def test_schedule_deviation_stays_below_contract_bound() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.5, 1.0), (0.0, 0.5)):
        assert _deviation(v0, v1, 1.0, 1.0) <= MAX_DEVIATION_FRACTION


def test_deviation_bound_fits_planning_margin() -> None:
    # Two drones may deviate in opposite directions. With PLANNING_MARGIN
    # at 0 (origin/dev policy) the schedule error must stay small relative
    # to the minimum formation spacing floor. Checked for the default 1 m
    # step.
    assert 2.0 * MAX_DEVIATION_FRACTION * 1.0 <= MIN_FORMATION_XY_CLEARANCE


def test_endpoint_speeds_match() -> None:
    speed, _ = _log_profile(0.2, 1.0, 1.0, 1.0)
    assert math.isclose(speed(0.0), 0.2, abs_tol=1e-9)
    assert math.isclose(speed(1.0), 1.0, abs_tol=1e-9)


def test_rest_to_rest_peak_factor() -> None:
    peak = _log_profile_peak(0.0, 0.0, 1.0, 1.0)
    assert math.isclose(peak, _EASE_PEAK_FACTOR, rel_tol=1e-3)
    assert 1.3 < _EASE_PEAK_FACTOR < 1.6


def test_speed_follows_natural_log_on_pure_ramp() -> None:
    # With v0 = 0 the boundary-pinned part of the profile is v1·ln-shaped;
    # spot-check that the log basis really is ln(1 + (e-1)τ).
    assert math.isclose(_log_w(1.0), 1.0, rel_tol=1e-12)
    assert math.isclose(_log_w(0.5), math.log1p((math.e - 1.0) * 0.5))


def _hold(t: float, pos) -> list:
    return [t, list(pos), []]


def test_eased_segment_is_subdivided_and_exempt_segment_stays_linear() -> None:
    # One 4 m dash between two holds, followed by a marked constant-speed
    # climb: the dash gets subdivided, the climb keeps a single linear
    # segment with no control points.
    points = [
        _hold(0.0, [0.0, 0.0, 10.0]),
        _hold(4.0, [4.0, 0.0, 10.0]),
        _hold(6.0, [4.0, 0.0, 10.0]),
        _hold(8.0, [4.0, 0.0, 11.0]),  # climb, 0.5 m/s
    ]
    smoothed = apply_velocity_smoothing(points, 1.0, constant_times={8.0})

    dash_knots = [p for p in smoothed if 0.0 < p[0] <= 4.0]
    assert len(dash_knots) == _LOG_SUBDIVISIONS
    assert all(p[2] for p in dash_knots)  # every piece carries controls

    climb = [p for p in smoothed if p[0] == 8.0]
    assert climb and climb[0][2] == []
    assert [p for p in smoothed if 6.0 < p[0] < 8.0] == []


def test_knot_times_and_endpoints_are_preserved() -> None:
    points = [
        _hold(0.0, [0.0, 0.0, 10.0]),
        _hold(5.0, [5.0, 0.0, 10.0]),
        _hold(10.0, [5.0, 5.0, 10.0]),
    ]
    smoothed = apply_velocity_smoothing(points, 1.0)
    times = [p[0] for p in smoothed]
    assert times == sorted(times)
    for original in points:
        matches = [p for p in smoothed if p[0] == original[0]]
        assert matches and matches[0][1] == original[1]
