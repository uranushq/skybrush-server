"""Tests for the exp-in / log-out velocity smoothing profile.

Covers the invariants the collision-safety contract depends on:

* knot times of the original keyframes never move and the arc length is
  covered exactly (``arc(1) == L``);
* the schedule deviation of any eased segment stays within the documented
  bound (what the minimum separation absorbs; ``verify`` is the real backstop);
* the speed follows an exponential ease-in / logarithmic ease-out curve and
  endpoint speeds match;
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
    _MAX_THICKNESS,
    _bump,
    _ease_in_exp,
    _ease_out_log,
    _log_profile,
    _log_profile_peak,
    _thickness_from_smoothing,
    apply_velocity_smoothing,
)

# Worst-case per-drone schedule deviation fraction the exp-in/log-out profile
# produces at the maximum thickness (the exponential ease-in lags an
# accelerating segment by ~23% of its length at k = 2).
MAX_DEVIATION_FRACTION = 0.25

# Thickness used by the profile-shape tests (full smoothing).
_K = _thickness_from_smoothing(1.0)


def _deviation(v0: float, v1: float, length: float, dt: float, k: float) -> float:
    _, arc = _log_profile(v0, v1, length, dt, k)
    return max(
        abs(arc(i / 200.0) / length - i / 200.0) for i in range(201)
    )


def test_profile_covers_length_exactly() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.3, 1.0)):
        _, arc = _log_profile(v0, v1, 1.0, 1.0, _K)
        assert math.isclose(arc(1.0), 1.0, rel_tol=1e-4)
        assert arc(0.0) == 0.0


def test_bump_area_is_half_for_every_thickness() -> None:
    # E and L are inverse functions, so the bump area is exactly 1/2 for all k,
    # which is what makes arc(1) == length independent of the thickness.
    for k in (_thickness_from_smoothing(s) for s in (0.05, 0.25, 0.5, 1.0)):
        _, arc = _log_profile(0.0, 0.0, 1.0, 1.0, k)
        # rest-to-rest arc(1) == length regardless of k
        assert math.isclose(arc(1.0), 1.0, rel_tol=1e-4)


def test_schedule_deviation_stays_below_contract_bound() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.5, 1.0), (0.0, 0.5)):
        assert _deviation(v0, v1, 1.0, 1.0, _K) <= MAX_DEVIATION_FRACTION


def test_deviation_bound_fits_minimum_separation() -> None:
    # Two drones may deviate in opposite directions. Even the worst-case mutual
    # approach (2 × max deviation × step for the default 1 m step) must stay
    # inside the minimum formation clearance floor.
    assert 2.0 * MAX_DEVIATION_FRACTION * 1.0 <= MIN_FORMATION_XY_CLEARANCE


def test_endpoint_speeds_match() -> None:
    speed, _ = _log_profile(0.2, 1.0, 1.0, 1.0, _K)
    assert math.isclose(speed(0.0), 0.2, abs_tol=1e-9)
    assert math.isclose(speed(1.0), 1.0, abs_tol=1e-9)


def test_rest_to_rest_peak_factor() -> None:
    peak = _log_profile_peak(0.0, 0.0, 1.0, 1.0, _K)
    assert math.isclose(peak, _EASE_PEAK_FACTOR, rel_tol=1e-3)
    assert math.isclose(_EASE_PEAK_FACTOR, 2.0, rel_tol=1e-9)


def test_ease_bases_are_inverses_and_monotonic() -> None:
    # E(u) is a convex ease-in, L(u) a concave ease-out, and L = E^{-1}.
    for k in (_thickness_from_smoothing(s) for s in (0.1, 0.5, 1.0)):
        assert math.isclose(_ease_in_exp(0.0, k), 0.0, abs_tol=1e-12)
        assert math.isclose(_ease_in_exp(1.0, k), 1.0, abs_tol=1e-12)
        assert math.isclose(_ease_out_log(0.0, k), 0.0, abs_tol=1e-12)
        assert math.isclose(_ease_out_log(1.0, k), 1.0, abs_tol=1e-12)
        for u in (0.1, 0.37, 0.6, 0.9):
            # inverse relationship: L(E(u)) == u
            assert math.isclose(_ease_out_log(_ease_in_exp(u, k), k), u, abs_tol=1e-9)


def test_bump_is_exp_rising_then_log_falling() -> None:
    # Peak at the midpoint, zero at both ends, and the exponential rise stays
    # BELOW the linear ramp (convex, gentle start) on the first half.
    assert math.isclose(_bump(0.0, _K), 0.0, abs_tol=1e-12)
    assert math.isclose(_bump(1.0, _K), 0.0, abs_tol=1e-12)
    assert math.isclose(_bump(0.5, _K), 1.0, abs_tol=1e-9)
    # convex exponential rise: value at τ=0.25 is below the linear 0.5
    assert _bump(0.25, _K) < 0.5
    # concave log fall: value at τ=0.75 is above the linear 0.5
    assert _bump(0.75, _K) > 0.5


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


def test_thickness_mapping_is_monotonic_and_bounded() -> None:
    assert _thickness_from_smoothing(0.0) > 0.0  # never exactly zero (no div/0)
    assert _thickness_from_smoothing(1.0) == _MAX_THICKNESS
    assert _thickness_from_smoothing(0.5) < _thickness_from_smoothing(1.0)
