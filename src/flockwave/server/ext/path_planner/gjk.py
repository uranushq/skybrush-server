"""GJK closest-point query for convex polytopes.

Finds the point in a half-space polytope that is closest to a query point,
following the GJK distance algorithm (Zhou et al., Sensors 2022, 22(5), 1855).
"""

from __future__ import annotations

import math
from typing import Sequence

from .voronoi import HalfSpace, contains_point, support_point

Vec3 = tuple[float, float, float]

_MAX_ITER = 64
_EPS = 1e-9


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(v: Vec3) -> float:
    return math.sqrt(_dot(v, v))


def closest_point_in_polytope(
    halfspaces: Sequence[HalfSpace],
    query: Vec3,
    *,
    interior_hint: Vec3 | None = None,
) -> Vec3:
    """Closest point in the polytope to *query*; returns *query* if inside."""
    if not halfspaces:
        return query

    if contains_point(halfspaces, query):
        return query

    # Search in Minkowski difference P - {query}: find point in P closest to query.
    def support(direction: Vec3) -> Vec3:
        return _sub(support_point(halfspaces, direction), query)

    if interior_hint is not None and contains_point(halfspaces, interior_hint):
        toward = _sub(interior_hint, query)
        init_dir = toward if _norm(toward) > _EPS else (1.0, 0.0, 0.0)
        simplex: list[Vec3] = [support(init_dir)]
    else:
        simplex = [support((1.0, 0.0, 0.0))]

    direction = _scale(_closest_point_on_simplex_to_origin(simplex), -1.0)
    if _norm(direction) < _EPS:
        direction = (1.0, 0.0, 0.0)

    for _ in range(_MAX_ITER):
        closest_rel = _closest_point_on_simplex_to_origin(simplex)
        direction = _scale(closest_rel, -1.0)
        if _norm(direction) < _EPS:
            return _add(closest_rel, query)

        w = support(direction)
        if _dot(direction, w) <= _dot(direction, closest_rel) + _EPS:
            return _add(closest_rel, query)

        simplex.append(w)
        simplex = _reduce_simplex_to_origin(
            simplex, _closest_point_on_simplex_to_origin(simplex)
        )

        if len(simplex) >= 4:
            return query

    closest_rel = _closest_point_on_simplex_to_origin(simplex)
    return _add(closest_rel, query)


def _reduce_simplex_to_origin(simplex: list[Vec3], closest: Vec3) -> list[Vec3]:
    """Keep only vertices needed to express *closest* as a convex combination."""
    if len(simplex) <= 1:
        return simplex

    # Drop vertices that are not on the minimal feature supporting closest.
    tol = 1e-7
    reduced = [v for v in simplex if _norm(_sub(v, closest)) < tol]
    if reduced:
        return reduced

    if len(simplex) == 2:
        return simplex
    if len(simplex) == 3:
        a, b, c = simplex
        if _point_on_segment(closest, a, b, tol):
            return [a, b]
        if _point_on_segment(closest, b, c, tol):
            return [b, c]
        if _point_on_segment(closest, a, c, tol):
            return [a, c]
    return simplex


def _point_on_segment(p: Vec3, a: Vec3, b: Vec3, tol: float) -> bool:
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom < _EPS:
        return _norm(_sub(p, a)) < tol
    t = _dot(_sub(p, a), ab) / denom
    if t < -tol or t > 1.0 + tol:
        return False
    proj = _add(a, _scale(ab, max(0.0, min(1.0, t))))
    return _norm(_sub(p, proj)) < tol


def _closest_point_on_simplex_to_origin(simplex: list[Vec3]) -> Vec3:
    if len(simplex) == 1:
        return simplex[0]

    if len(simplex) == 2:
        return _closest_on_segment(simplex[0], simplex[1])

    if len(simplex) == 3:
        return _closest_on_triangle(simplex[0], simplex[1], simplex[2])

    # Tetrahedron contains the origin.
    return (0.0, 0.0, 0.0)


def _closest_on_segment(a: Vec3, b: Vec3) -> Vec3:
    ab = _sub(b, a)
    denom = _dot(ab, ab)
    if denom < _EPS:
        return a if _norm(a) <= _norm(b) else b
    t = max(0.0, min(1.0, -_dot(a, ab) / denom))
    return _add(a, _scale(ab, t))


def _closest_on_triangle(a: Vec3, b: Vec3, c: Vec3) -> Vec3:
    candidates = [
        _closest_on_segment(a, b),
        _closest_on_segment(b, c),
        _closest_on_segment(c, a),
        a,
        b,
        c,
    ]

    ab = _sub(b, a)
    ac = _sub(c, a)
    n = _cross(ab, ac)
    n_len_sq = _dot(n, n)
    if n_len_sq >= _EPS:
        t = -_dot(n, a) / n_len_sq
        plane_proj = _add(a, _scale(n, t))
        if _barycentric_inside(a, b, c, plane_proj):
            candidates.append(plane_proj)

    return min(candidates, key=_norm)


def _barycentric_inside(a: Vec3, b: Vec3, c: Vec3, p: Vec3) -> bool:
    v0 = _sub(c, a)
    v1 = _sub(b, a)
    v2 = _sub(p, a)
    dot00 = _dot(v0, v0)
    dot01 = _dot(v0, v1)
    dot02 = _dot(v0, v2)
    dot11 = _dot(v1, v1)
    dot12 = _dot(v1, v2)
    denom = dot00 * dot11 - dot01 * dot01
    if abs(denom) < _EPS:
        return False
    inv = 1.0 / denom
    u = (dot11 * dot02 - dot01 * dot12) * inv
    v = (dot00 * dot12 - dot01 * dot02) * inv
    return u >= -_EPS and v >= -_EPS and u + v <= 1.0 + _EPS
