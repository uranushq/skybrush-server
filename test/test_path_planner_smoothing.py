"""Tests for the exp-in / plateau / log-out (trapezoid) velocity profile.

Covers the invariants the collision-safety contract depends on:

* knot times of the original keyframes never move and the arc length is
  covered exactly (``arc(1) == L``);
* the schedule deviation of any eased segment stays within the documented
  bound (what the minimum separation absorbs; ``verify`` is the real backstop);
* the speed accelerates along an exponential ramp, holds a CONSTANT plateau
  (no pointy apex), then decelerates along a logarithmic ramp; endpoint
  speeds match;
* smoothing-exempt (``constant_times``) segments stay exactly linear.
"""

from __future__ import annotations

import math

from flockwave.server.ext.path_planner.collision_volume import (
    MIN_FORMATION_XY_CLEARANCE,
)
from flockwave.server.ext.path_planner.converter import (
    DEFAULT_PROFILE_EXP,
    DEFAULT_PROFILE_LOG,
    _RAMP_HALF_MAX,
    _RAMP_HALF_MIN,
    _THICKNESS_MAX,
    _THICKNESS_MIN,
    _clamp_thickness,
    _ease_in_exp,
    _ease_out_log,
    _ease_peak_factor,
    _log_profile,
    _log_profile_peak,
    _ramp_from_smoothing,
    apply_velocity_smoothing,
)

# Worst-case per-drone schedule deviation fraction of the trapezoid profile at
# full smoothing (ramp 0.4 per side): the accel ramp lags the constant-speed
# schedule by ~17% of the segment length at the ramp/plateau boundary.
MAX_DEVIATION_FRACTION = 0.25

# Ramp fraction and curvatures used by the profile-shape tests (full smoothing).
_RAMP = _ramp_from_smoothing(1.0)
_KE = DEFAULT_PROFILE_EXP
_KL = DEFAULT_PROFILE_LOG


def _deviation(v0: float, v1: float, length: float, dt: float) -> float:
    _, arc = _log_profile(v0, v1, length, dt, _KE, _KL, _RAMP)
    return max(abs(arc(i / 200.0) / length - i / 200.0) for i in range(201))


def test_profile_covers_length_exactly() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.3, 1.0)):
        _, arc = _log_profile(v0, v1, 1.0, 1.0, _KE, _KL, _RAMP)
        assert math.isclose(arc(1.0), 1.0, rel_tol=1e-9)
        assert arc(0.0) == 0.0


def test_length_is_exact_for_every_curvature_and_ramp() -> None:
    # vc is solved in closed form so arc(1) == length regardless of the
    # curvature knobs or ramp fraction.
    for ke in (0.05, 0.5, 2.0, 4.0):
        for kl in (0.05, 1.0, 4.0):
            for ramp in (_RAMP_HALF_MIN, 0.25, _RAMP_HALF_MAX):
                _, arc = _log_profile(0.0, 0.4, 1.0, 1.0, ke, kl, ramp)
                assert math.isclose(arc(1.0), 1.0, rel_tol=1e-9)


def test_schedule_deviation_stays_below_contract_bound() -> None:
    for v0, v1 in ((0.0, 0.0), (0.0, 1.0), (1.0, 0.0), (0.5, 1.0), (0.0, 0.5)):
        assert _deviation(v0, v1, 1.0, 1.0) <= MAX_DEVIATION_FRACTION


def test_deviation_bound_fits_minimum_separation() -> None:
    # Two drones may deviate in opposite directions. Even the worst-case mutual
    # approach (2 × max deviation × step for the default 1 m step) must stay
    # inside the minimum formation clearance floor.
    assert 2.0 * MAX_DEVIATION_FRACTION * 1.0 <= MIN_FORMATION_XY_CLEARANCE


def test_endpoint_speeds_match() -> None:
    speed, _ = _log_profile(0.2, 1.0, 1.0, 1.0, _KE, _KL, _RAMP)
    assert math.isclose(speed(0.0), 0.2, abs_tol=1e-9)
    assert math.isclose(speed(1.0), 1.0, abs_tol=1e-9)


def test_plateau_is_constant_with_no_apex() -> None:
    # The defining property of the trapezoid: between the two ramps the speed
    # is EXACTLY constant, and that plateau is also the profile's maximum —
    # there is no pointy apex above it.
    speed, _ = _log_profile(0.0, 0.0, 1.0, 1.0, _KE, _KL, _RAMP)
    vc = speed(0.5)
    for i in range(101):
        tau = _RAMP + (1.0 - 2.0 * _RAMP) * i / 100.0
        assert math.isclose(speed(tau), vc, rel_tol=1e-12)
    peak = max(speed(i / 400.0) for i in range(401))
    assert math.isclose(peak, vc, rel_tol=1e-9)


def test_rest_to_rest_peak_factor() -> None:
    # Rest-to-rest peak = plateau speed = 1 / (1 - ramp) for equal curvatures
    # (E and L are inverses, so their mean areas are complementary).
    peak = _log_profile_peak(0.0, 0.0, 1.0, 1.0, _KE, _KL, _RAMP)
    factor = _ease_peak_factor(1.0, _KE, _KL)
    assert math.isclose(peak, factor, rel_tol=1e-9)
    assert math.isclose(factor, 1.0 / (1.0 - _RAMP), rel_tol=1e-9)
    assert factor < 2.0  # gentler than the old pointy profile
    assert _ease_peak_factor(0.0, _KE, _KL) == 1.0  # no easing -> constant


def test_ease_bases_are_inverses_and_monotonic() -> None:
    # E(u) is a convex ease-in, L(u) a concave ease-out, and L = E^{-1}.
    for k in (0.1, 1.0, 2.0, 4.0):
        assert math.isclose(_ease_in_exp(0.0, k), 0.0, abs_tol=1e-12)
        assert math.isclose(_ease_in_exp(1.0, k), 1.0, abs_tol=1e-12)
        assert math.isclose(_ease_out_log(0.0, k), 0.0, abs_tol=1e-12)
        assert math.isclose(_ease_out_log(1.0, k), 1.0, abs_tol=1e-12)
        for u in (0.1, 0.37, 0.6, 0.9):
            # inverse relationship: L(E(u)) == u
            assert math.isclose(_ease_out_log(_ease_in_exp(u, k), k), u, abs_tol=1e-9)


def test_ramps_are_exp_shaped_then_log_shaped() -> None:
    # Accel ramp is convex (below the linear chord: gentle start); decel ramp
    # is the mirrored concave log (gentle finish).
    speed, _ = _log_profile(0.0, 0.0, 1.0, 1.0, _KE, _KL, _RAMP)
    vc = speed(0.5)
    # halfway up the accel ramp the exp curve is below the linear chord
    assert speed(_RAMP * 0.5) < 0.5 * vc
    # halfway down the decel ramp the log curve is above the linear chord
    assert speed(1.0 - _RAMP * 0.5) > 0.5 * vc


def test_independent_curvatures_shape_each_ramp() -> None:
    # Sharpening only the exp curvature changes the accel ramp but leaves the
    # decel ramp identical (and vice versa the plateau still covers length).
    gentle, _ = _log_profile(0.0, 0.0, 1.0, 1.0, 0.5, _KL, _RAMP)
    sharp, _ = _log_profile(0.0, 0.0, 1.0, 1.0, 4.0, _KL, _RAMP)
    # same plateau (area constraint with same mean is not required — plateau
    # differs slightly), but the normalized accel shapes must differ
    assert not math.isclose(
        gentle(_RAMP * 0.5) / gentle(0.5), sharp(_RAMP * 0.5) / sharp(0.5),
        rel_tol=1e-3,
    )


def _hold(t: float, pos) -> list:
    return [t, list(pos), []]


def test_eased_segment_is_subdivided_and_exempt_segment_stays_linear() -> None:
    # One 4 m dash between two holds, followed by a marked constant-speed
    # climb: the dash gets subdivided into the 5 trapezoid pieces (2 accel,
    # 1 plateau, 2 decel), the climb keeps a single linear segment with no
    # control points.
    points = [
        _hold(0.0, [0.0, 0.0, 10.0]),
        _hold(4.0, [4.0, 0.0, 10.0]),
        _hold(6.0, [4.0, 0.0, 10.0]),
        _hold(8.0, [4.0, 0.0, 11.0]),  # climb, 0.5 m/s
    ]
    smoothed = apply_velocity_smoothing(points, 1.0, constant_times={8.0})

    dash_knots = [p for p in smoothed if 0.0 < p[0] <= 4.0]
    assert len(dash_knots) == 5
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


def test_ramp_mapping_is_monotonic_and_bounded() -> None:
    assert _ramp_from_smoothing(0.0) == _RAMP_HALF_MIN
    assert _ramp_from_smoothing(1.0) == _RAMP_HALF_MAX
    assert _ramp_from_smoothing(0.5) < _ramp_from_smoothing(1.0)
    assert _RAMP_HALF_MAX < 0.5  # both ramps + plateau must fit the segment


def test_thickness_clamp() -> None:
    assert _clamp_thickness(0.0) == _THICKNESS_MIN
    assert _clamp_thickness(999.0) == _THICKNESS_MAX
    assert _clamp_thickness(2.0) == 2.0
