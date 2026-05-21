"""Buffered Voronoi cells for distributed collision avoidance.

Based on Zhou et al., Sensors 2022, 22(5), 1855 — Equations (21)–(25).
Each drone's feasible region is an intersection of half-spaces derived from
neighbor positions.  An ellipsoid-aware buffer shrinks the cell so that
axis-aligned collision envelopes stay separated.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import combinations
from typing import Iterable, Sequence

Vec3 = tuple[float, float, float]

# ``add_bounding_box()`` always appends this many half-spaces at the end.
_BBOX_FACE_COUNT = 6


@dataclass(frozen=True)
class HalfSpace:
    """Feasible set {x | normal · x <= offset}."""

    normal: Vec3
    offset: float


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _norm(v: Vec3) -> float:
    return math.sqrt(_dot(v, v))


def _ellipsoid_buffer(p_ij: Vec3, radii: Vec3) -> float:
    """||Λ Rᵀ p_ij|| with R = I and Λ = diag(radii)."""
    return math.sqrt(
        (radii[0] * p_ij[0]) ** 2
        + (radii[1] * p_ij[1]) ** 2
        + (radii[2] * p_ij[2]) ** 2
    )


def build_bvc_halfspaces(
    position: Vec3,
    neighbor_positions: Iterable[Vec3],
    *,
    radii: Vec3 = (0.75, 0.75, 0.75),
    sphere_radius: float | None = None,
    use_ellipsoid: bool = True,
) -> list[HalfSpace]:
    """Build buffered Voronoi half-spaces for *position* vs *neighbors*.

    Eq. (23) sphere buffer when ``use_ellipsoid`` is False; Eq. (25) style
    ellipsoid buffer when True (R = identity).
    """
    halfspaces: list[HalfSpace] = []
    p_i = position

    for p_j in neighbor_positions:
        p_ij = _sub(p_j, p_i)
        length = _norm(p_ij)
        if length < 1e-12:
            continue

        midpoint = _scale(_add(p_i, p_j), 0.5)
        if use_ellipsoid:
            buffer = _ellipsoid_buffer(p_ij, radii)
        else:
            r_d = sphere_radius if sphere_radius is not None else max(radii)
            buffer = r_d * length

        # p_ij · p <= p_ij · midpoint - buffer
        offset = _dot(p_ij, midpoint) - buffer
        halfspaces.append(HalfSpace(normal=p_ij, offset=offset))

    return halfspaces


def add_bounding_box(
    halfspaces: list[HalfSpace],
    center: Vec3,
    radius: float,
) -> list[HalfSpace]:
    """Add axis-aligned box constraints so the Voronoi cell is bounded."""
    cx, cy, cz = center
    r = radius
    extras = [
        HalfSpace((1.0, 0.0, 0.0), cx + r),
        HalfSpace((-1.0, 0.0, 0.0), -cx + r),
        HalfSpace((0.0, 1.0, 0.0), cy + r),
        HalfSpace((0.0, -1.0, 0.0), -cy + r),
        HalfSpace((0.0, 0.0, 1.0), cz + r),
        HalfSpace((0.0, 0.0, -1.0), -cz + r),
    ]
    return halfspaces + extras


def contains_point(
    halfspaces: Sequence[HalfSpace], point: Vec3, eps: float = 1e-6
) -> bool:
    return all(_dot(h.normal, point) <= h.offset + eps for h in halfspaces)


def _bbox_extreme_corner(bbox_hs: Sequence[HalfSpace], direction: Vec3) -> Vec3:
    """Corner of the axis-aligned box that maximizes ``direction · x``."""
    # Order matches :func:`add_bounding_box`.
    pos_x, neg_x, pos_y, neg_y, pos_z, neg_z = bbox_hs
    x = pos_x.offset if direction[0] >= 0.0 else -neg_x.offset
    y = pos_y.offset if direction[1] >= 0.0 else -neg_y.offset
    z = pos_z.offset if direction[2] >= 0.0 else -neg_z.offset
    return (x, y, z)


def _collect_vertex(
    candidates: list[Vec3],
    halfspaces: Sequence[HalfSpace],
    h1: HalfSpace,
    h2: HalfSpace,
    h3: HalfSpace,
) -> None:
    vertex = _plane_intersection(h1, h2, h3)
    if vertex is not None and contains_point(halfspaces, vertex):
        candidates.append(vertex)


def support_point(halfspaces: Sequence[HalfSpace], direction: Vec3) -> Vec3:
    """Support mapping s(d) = argmax d·p over the half-space polytope.

    In 3D the maximum of a linear function over a polytope occurs at a
    vertex (intersection of three faces).  We only check triplets formed by
    the six bounding-box faces plus one Voronoi face — O(N) in the number of
    neighbors instead of O(N³).
    """
    n = len(halfspaces)
    if n == 0:
        return (0.0, 0.0, 0.0)

    bbox_start = max(0, n - _BBOX_FACE_COUNT)
    voronoi_hs = halfspaces[:bbox_start]
    bbox_hs = halfspaces[bbox_start:]

    candidates: list[Vec3] = []

    if len(bbox_hs) == _BBOX_FACE_COUNT:
        candidates.append(_bbox_extreme_corner(bbox_hs, direction))
        for i, j, k in combinations(range(_BBOX_FACE_COUNT), 3):
            _collect_vertex(candidates, halfspaces, bbox_hs[i], bbox_hs[j], bbox_hs[k])

    if bbox_hs:
        for h in voronoi_hs:
            for i, j in combinations(range(len(bbox_hs)), 2):
                _collect_vertex(candidates, halfspaces, h, bbox_hs[i], bbox_hs[j])

    if not candidates:
        for h in halfspaces:
            n_len = _norm(h.normal)
            if n_len < 1e-12:
                continue
            n_hat = _scale(h.normal, 1.0 / n_len)
            candidates.append(_scale(n_hat, h.offset / n_len))

    return max(candidates, key=lambda p: _dot(direction, p))


def closest_point_in_polytope(
    halfspaces: Sequence[HalfSpace],
    query: Vec3,
    *,
    interior_hint: Vec3 | None = None,
    max_iterations: int = 16,
) -> Vec3:
    """Closest point in ``{x | n·x <= b}`` via half-space projection (O(N), fast).

    Used by the path planner instead of GJK for real-time swarms.
    """
    if not halfspaces:
        return query

    if contains_point(halfspaces, query):
        return query

    # Project the query onto the feasible set (Euclidean closest point).
    point: Vec3 = query

    for _ in range(max_iterations):
        moved = False
        for h in halfspaces:
            violation = _dot(h.normal, point) - h.offset
            if violation <= 1e-9:
                continue
            n_len_sq = _dot(h.normal, h.normal)
            if n_len_sq < 1e-12:
                continue
            scale = violation / n_len_sq
            point = (
                point[0] - scale * h.normal[0],
                point[1] - scale * h.normal[1],
                point[2] - scale * h.normal[2],
            )
            moved = True
        if not moved:
            break

    if contains_point(halfspaces, point):
        return point

    # Fallback: closest point on individual faces, pick minimum distance to query.
    best = point
    best_dist_sq = sum((query[k] - point[k]) ** 2 for k in range(3))
    for h in halfspaces:
        n_len_sq = _dot(h.normal, h.normal)
        if n_len_sq < 1e-12:
            continue
        t = (h.offset - _dot(h.normal, query)) / n_len_sq
        on_plane = (
            query[0] + t * h.normal[0],
            query[1] + t * h.normal[1],
            query[2] + t * h.normal[2],
        )
        candidate = on_plane
        for h2 in halfspaces:
            violation = _dot(h2.normal, candidate) - h2.offset
            if violation > 1e-6:
                n2_len_sq = _dot(h2.normal, h2.normal)
                if n2_len_sq < 1e-12:
                    continue
                candidate = (
                    candidate[0] - violation / n2_len_sq * h2.normal[0],
                    candidate[1] - violation / n2_len_sq * h2.normal[1],
                    candidate[2] - violation / n2_len_sq * h2.normal[2],
                )
        dist_sq = sum((query[k] - candidate[k]) ** 2 for k in range(3))
        if dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best = candidate
    return best


def _plane_intersection(h1: HalfSpace, h2: HalfSpace, h3: HalfSpace) -> Vec3 | None:
    """Solve three plane equations n·x = b."""
    n1, n2, n3 = h1.normal, h2.normal, h3.normal
    b1, b2, b3 = h1.offset, h2.offset, h3.offset

    det = (
        n1[0] * (n2[1] * n3[2] - n2[2] * n3[1])
        - n1[1] * (n2[0] * n3[2] - n2[2] * n3[0])
        + n1[2] * (n2[0] * n3[1] - n2[1] * n3[0])
    )
    if abs(det) < 1e-12:
        return None

    def det_replace(col: int, vals: tuple[float, float, float]) -> float:
        rows = [n1, n2, n3]
        m = [list(r) for r in rows]
        m[0][col] = vals[0]
        m[1][col] = vals[1]
        m[2][col] = vals[2]
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    x = det_replace(0, (b1, b2, b3)) / det
    y = det_replace(1, (b1, b2, b3)) / det
    z = det_replace(2, (b1, b2, b3)) / det
    return (x, y, z)
