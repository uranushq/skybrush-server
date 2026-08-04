"""Final spatio-temporal collision verification for built trajectories.

This is the last line of defense before a show is saved or uploaded: it
re-checks the *actual* trajectories (including takeoff/landing segments and
velocity-smoothing Bézier re-timing) against the yaw-invariant collision
envelope, independently of any guarantee the solver made.

The sampling interval is derived from the envelope size and the maximum
speed present in the trajectories so that two drones cannot pass through
each other's envelope entirely between two consecutive samples.
"""

from __future__ import annotations

from math import floor
from typing import Dict, List, Sequence, Tuple

from .collision_volume import (
    HARD_MIN_SEPARATION,
    clamp_separation,
    envelope_overlap,
)

__all__ = ("verify_trajectories", "verify_show_dicts", "sample_trajectory_position")

# Never sample coarser / finer than these bounds (seconds).
_MAX_SAMPLE_DT = 0.5
_MIN_SAMPLE_DT = 0.02

# Peak speed of a fully eased segment is 1.5× its average speed.
_EASE_PEAK_FACTOR = 1.5


def _bezier_point(
    p0: Sequence[float],
    p1: Sequence[float],
    p2: Sequence[float],
    p3: Sequence[float],
    s: float,
) -> List[float]:
    om = 1.0 - s
    b0 = om * om * om
    b1 = 3.0 * om * om * s
    b2 = 3.0 * om * s * s
    b3 = s * s * s
    return [
        b0 * p0[i] + b1 * p1[i] + b2 * p2[i] + b3 * p3[i] for i in range(3)
    ]


def sample_trajectory_position(
    points: List[list], t: float, segment_hint: int = 0
) -> Tuple[List[float], int]:
    """Position on a trajectory at time *t*.

    ``points`` is the ``[t, [x, y, z], control]`` keyframe list of a
    trajectory dict. Before the first keyframe the first position is
    returned; after the last keyframe the drone is assumed to hold its final
    position. ``segment_hint`` lets a monotonically advancing caller resume
    the segment search where the previous call left off; the (possibly
    advanced) hint is returned alongside the position.
    """
    n = len(points)
    if n == 0:
        return [0.0, 0.0, 0.0], 0
    if t <= points[0][0]:
        return list(points[0][1]), 0

    k = max(1, segment_hint)
    while k < n and points[k][0] < t:
        k += 1
    if k >= n:
        return list(points[-1][1]), n - 1

    t0, p0 = points[k - 1][0], points[k - 1][1]
    t1, p1, ctrl = points[k][0], points[k][1], points[k][2]
    if t1 <= t0:
        return list(p1), k
    s = (t - t0) / (t1 - t0)
    if ctrl and len(ctrl) == 2:
        return _bezier_point(p0, ctrl[0], ctrl[1], p1, s), k
    return [p0[i] + s * (p1[i] - p0[i]) for i in range(3)], k


def _max_speed_estimate(trajectories: List[List[list]]) -> float:
    """Upper bound on any drone's speed across all trajectories."""
    max_speed = 0.0
    for points in trajectories:
        for k in range(1, len(points)):
            dt = points[k][0] - points[k - 1][0]
            if dt <= 1e-9:
                continue
            a, b = points[k - 1][1], points[k][1]
            length = (
                (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2
            ) ** 0.5
            max_speed = max(max_speed, _EASE_PEAK_FACTOR * length / dt)
    return max_speed


def verify_trajectories(
    trajectories: List[List[list]],
    *,
    margin: float = 0.0,
    separation: float = HARD_MIN_SEPARATION,
    max_violations: int = 20,
) -> List[dict]:
    """Check every drone pair over the whole show timeline.

    Returns a list of violation dicts (empty when the show is safe). Each
    violation carries the time, the two 0-based drone indices and their
    positions at that time.
    """
    n = len(trajectories)
    if n < 2:
        return []

    end_time = max(
        (points[-1][0] for points in trajectories if points), default=0.0
    )
    if end_time <= 0:
        return []

    separation = clamp_separation(separation)
    pair_reach_xy = separation + 2.0 * margin
    pair_reach_z = separation + 2.0 * margin
    min_reach = min(pair_reach_xy, pair_reach_z)

    max_speed = _max_speed_estimate(trajectories)
    if max_speed > 1e-9:
        # Two drones approach at most at 2×max_speed; a quarter of the
        # narrowest envelope reach per sample cannot jump over an overlap.
        dt = 0.25 * min_reach / max_speed
    else:
        dt = _MAX_SAMPLE_DT
    dt = max(_MIN_SAMPLE_DT, min(_MAX_SAMPLE_DT, dt))

    cell = max(pair_reach_xy, pair_reach_z)
    hints = [0] * n
    violations: List[dict] = []
    reported: set[Tuple[int, int]] = set()

    steps = int(end_time / dt) + 2
    for step in range(steps):
        t = min(step * dt, end_time)
        positions: List[List[float]] = []
        for i in range(n):
            pos, hints[i] = sample_trajectory_position(
                trajectories[i], t, hints[i]
            )
            positions.append(pos)

        # Broad-phase: hash points into cells sized to the pair reach; any
        # interacting pair sits in the same or an adjacent cell.
        buckets: Dict[Tuple[int, int, int], List[int]] = {}
        for i, pos in enumerate(positions):
            key = (
                floor(pos[0] / cell),
                floor(pos[1] / cell),
                floor(pos[2] / cell),
            )
            buckets.setdefault(key, []).append(i)

        for (cx, cy, cz), ids in buckets.items():
            neighbours: List[int] = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        neighbours.extend(
                            buckets.get((cx + dx, cy + dy, cz + dz), ())
                        )
            for i in ids:
                for j in neighbours:
                    if j <= i:
                        continue
                    if (i, j) in reported:
                        continue
                    if envelope_overlap(
                        positions[i],
                        positions[j],
                        margin=margin,
                        separation=separation,
                    ):
                        reported.add((i, j))
                        violations.append(
                            {
                                "time": round(t, 3),
                                "first": i,
                                "second": j,
                                "first_position": [round(v, 3) for v in positions[i]],
                                "second_position": [round(v, 3) for v in positions[j]],
                            }
                        )
                        if len(violations) >= max_violations:
                            return violations
        if t >= end_time:
            break

    return violations


def verify_show_dicts(
    show_dicts: List[dict],
    *,
    margin: float = 0.0,
    separation: float = HARD_MIN_SEPARATION,
    max_violations: int = 20,
) -> List[dict]:
    """Convenience wrapper: verify the trajectories inside show dicts."""
    trajectories = [
        (show.get("trajectory") or {}).get("points") or [] for show in show_dicts
    ]
    return verify_trajectories(
        trajectories,
        margin=margin,
        separation=separation,
        max_violations=max_violations,
    )
