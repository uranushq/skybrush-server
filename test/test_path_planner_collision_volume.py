"""Tests for the yaw-invariant drone collision envelope.

The checking envelope is a single bounding box whose horizontal half-side is
the horizontal circumradius of the detailed body+wake model, so overlap
results do not depend on either drone's commanded yaw.
"""

from __future__ import annotations

import math

from flockwave.server.ext.path_planner.collision_volume import (
    BODY_SIZE_Z,
    ENVELOPE_XY_HALF,
    ENVELOPE_Z_HEIGHT,
    GUARANTEED_XY_CLEARANCE,
    PLANNED_XY_CLEARANCE,
    PLANNING_MARGIN,
    WAKE_ANGLE_DEG,
    WAKE_LENGTH,
    WAKE_RADIUS,
    envelope_overlap,
    envelope_overlap_swept,
    volumes_overlap,
)


def test_envelope_covers_wake_reach() -> None:
    """The circumradius must cover the farthest wake corner."""
    theta = math.radians(WAKE_ANGLE_DEG)
    wake_y_reach = 0.183 + math.sin(theta) * WAKE_LENGTH + WAKE_RADIUS
    assert ENVELOPE_XY_HALF >= wake_y_reach


def test_zero_offset_overlap() -> None:
    assert volumes_overlap([0.0, 0.0, 10.0], [0.0, 0.0, 10.0])


def test_bodies_overlap_when_close_on_x() -> None:
    assert volumes_overlap([0.0, 0.0, 10.0], [0.3, 0.0, 10.0])


def test_separated_beyond_guaranteed_clearance_on_x() -> None:
    a = [0.0, 0.0, 10.0]
    b = [GUARANTEED_XY_CLEARANCE + 0.01, 0.0, 10.0]
    assert not volumes_overlap(a, b)


def test_separated_beyond_guaranteed_clearance_on_y() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, GUARANTEED_XY_CLEARANCE + 0.01, 10.0]
    assert not volumes_overlap(a, b)


def test_yaw_invariance_diagonal_equals_axis() -> None:
    """The envelope is a square: the same offset rotated 90° gives the
    same verdict, and near-clearance diagonal offsets still overlap."""
    d = GUARANTEED_XY_CLEARANCE - 0.05
    assert volumes_overlap([0, 0, 10], [d, 0, 10]) == volumes_overlap(
        [0, 0, 10], [0, d, 10]
    )
    assert volumes_overlap([0, 0, 10], [d, d, 10])


def test_separated_when_far_on_z() -> None:
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.0 + ENVELOPE_Z_HEIGHT + 0.01]
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
    a = [0.0, 0.0, 10.0]
    b = [0.0, 0.0, 10.0 - half_z - wake_dz - WAKE_RADIUS - 0.01]
    assert not volumes_overlap(a, b)


def test_margin_inflates_envelope() -> None:
    a = [0.0, 0.0, 10.0]
    b = [GUARANTEED_XY_CLEARANCE + 0.1, 0.0, 10.0]
    assert not envelope_overlap(a, b)
    assert envelope_overlap(a, b, margin=PLANNING_MARGIN)
    assert not envelope_overlap(
        [0.0, 0.0, 10.0], [PLANNED_XY_CLEARANCE + 0.01, 0.0, 10.0],
        margin=PLANNING_MARGIN,
    )


def test_swept_check_is_exact_no_tunneling() -> None:
    """A fast crossing that any point-sampling would miss must be caught."""
    assert envelope_overlap_swept(
        [-100, 0, 10], [100, 0, 10], [100, 0.1, 10], [-100, 0.1, 10]
    )


def test_swept_parallel_lanes_clear() -> None:
    d = GUARANTEED_XY_CLEARANCE + 0.05
    assert not envelope_overlap_swept(
        [0, 0, 10], [10, 0, 10], [0, d, 10], [10, d, 10]
    )
