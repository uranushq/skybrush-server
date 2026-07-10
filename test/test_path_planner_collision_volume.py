"""Tests for composite drone collision volumes."""

from __future__ import annotations

import math

from flockwave.server.ext.path_planner.collision_volume import (
    BODY_SIZE_X,
    BODY_SIZE_Z,
    WAKE_ANGLE_DEG,
    WAKE_LENGTH,
    WAKE_RADIUS,
    volumes_overlap,
)


def test_bodies_overlap_when_close_on_x() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.3, 0.0, 10.0]
    assert volumes_overlap(a, b)


def test_components_separated_when_far_on_x() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.51, 0.0, 10.0]
    assert not volumes_overlap(a, b)


def test_bodies_overlap_when_close_on_y() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.2, 10.0]
    assert volumes_overlap(a, b)


def test_components_separated_when_far_on_y() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.6, 10.0]
    assert not volumes_overlap(a, b)


def test_components_separated_when_far_on_z() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.51]
    assert not volumes_overlap(a, b)


def test_bodies_overlap_when_touching_inside_threshold() -> None:
    half_x = BODY_SIZE_X * 0.5
    a = [0.0, 0.0, 10.0]
    b = [2.0 * half_x - 0.01, 0.0, 10.0]
    assert volumes_overlap(a, b)


def test_bodies_only_touching_on_x_do_not_overlap() -> None:
    """Wake stays inside the body on X, so exact body contact is allowed."""
    half_x = BODY_SIZE_X * 0.5
    a = [0.0, 0.0, 10.0]
    b = [2.0 * half_x, 0.0, 10.0]
    assert not volumes_overlap(a, b)


def test_wake_overlap_when_offset_below() -> None:
    theta = math.radians(WAKE_ANGLE_DEG)
    wake_dz = WAKE_LENGTH * math.cos(theta)
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.0 - wake_dz + 0.05]
    assert volumes_overlap(a, b)


def test_wake_clears_when_far_below() -> None:
    theta = math.radians(WAKE_ANGLE_DEG)
    wake_dz = WAKE_LENGTH * math.cos(theta)
    half_z = BODY_SIZE_Z * 0.5
    # Body-body needs 2*half_z; wake may or may not extend past the body.
    clearance = max(2.0 * half_z, half_z + wake_dz + WAKE_RADIUS) + 0.01
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.0 - clearance]
    assert not volumes_overlap(a, b)


def test_component_check_allows_spacing_union_would_reject() -> None:
    """Union AABB overlaps, but no body/wake component pair does."""
    a = [0.0, 0.0, 10.0]
    b = [0.17, 0.55, 10.0]
    assert not volumes_overlap(a, b)


def test_forward_wake_overlap_for_small_y_shift() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.25, 10.0]
    assert volumes_overlap(a, b)


def test_large_forward_shift_clears_all_components() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.7, 10.0]
    assert not volumes_overlap(a, b)


def test_zero_offset_overlap() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.0]
    assert volumes_overlap(a, b)


def test_formation_spacing_0_7m_on_y_is_allowed() -> None:
    """Formation targets spaced 0.7 m on Y must clear the collision envelope."""
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.7, 10.0]
    assert not volumes_overlap(a, b)
