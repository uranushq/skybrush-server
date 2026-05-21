"""Distributed Voronoi–GJK path planner with collision fallback.

Algorithm
---------
1. Each step, build a buffered Voronoi cell (BVC) from neighbor positions.
2. Use GJK to find the point in the BVC closest to the goal (Zhou et al.,
   Sensors 2022, 22(5), 1855).
3. Move one step toward that point (ellipsoid-aware buffer).
4. Collision check: endpoints *and* swept linear motion share the same
   axis-aligned envelope (|dx| < COLLISION_X on all three axes).
5. Colliding drones revert (priority by remaining distance); deadlocks get
   detour candidates.
6. Repeats until all drones arrive or MAX_STEPS is reached.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Set, Tuple

from .drone import Drone, Vec3
from .voronoi import (
    HalfSpace,
    add_bounding_box,
    build_bvc_halfspaces,
    closest_point_in_polytope,
    contains_point,
)

# collision envelope per axis
COLLISION_X = 1.5
COLLISION_Y = 1.5
COLLISION_Z = 1.5

MAX_STEPS = 10_000
# Per-solve cap derived from longest route (prevents API hangs).
STEP_BUDGET_PER_DRONE = 200
DEADLOCK_THRESHOLD = 1
# Only the nearest neighbors define the local Voronoi cell (distributed approx).
MAX_VORONOI_NEIGHBORS = 16
# Fall back to greedy goal pursuit when stuck in the BVC.
STAGNATION_LIMIT = 50
# Exponential smoothing for Voronoi guidance (0 = frozen, 1 = no smoothing).
WAYPOINT_SMOOTHING = 0.4
# Blend goal pursuit vs Voronoi boundary correction.
GOAL_BLEND = 0.85

# Numerical slack for swept-interval intersection (open inequalities vs. FP).
_SWEPT_TIME_EPS = 1e-9


@dataclass
class StepRecord:
    step: int
    positions: Dict[int, List[float]]
    collisions: List[Tuple[int, int]]
    reverted_drones: List[int]
    verified: bool


@dataclass
class SolverResult:
    steps: List[StepRecord]
    total_steps: int
    drones: List[Drone]
    success: bool


class PathSolver:
    """Greedy collision-avoiding path planner."""

    def __init__(
        self,
        initials: List[Vec3],
        targets: List[Vec3],
        step_size: float = 1.0,
        seed: Optional[int] = None,
        on_step: Optional[Callable[[StepRecord], None]] = None,
        min_z: float = 0.0,
    ) -> None:
        assert len(initials) == len(targets), "initial and target counts must match"
        self.step_size = step_size
        self.on_step = on_step
        self.min_z = float(min_z)

        if seed is not None:
            random.seed(seed)

        self.drones: List[Drone] = []
        for i, (ini, tgt) in enumerate(zip(initials, targets)):
            self.drones.append(Drone(drone_id=i, initial=tuple(ini), target=tuple(tgt)))

        self.history: List[StepRecord] = []
        self._consecutive_holds: Dict[int, int] = dict.fromkeys(range(len(initials)), 0)
        self._stagnation_steps: Dict[int, int] = dict.fromkeys(range(len(initials)), 0)
        self._guided_waypoint: Dict[int, Vec3] = {}

    # ── Voronoi + GJK guidance ───────────────────────────────────────────

    @staticmethod
    def _collision_radii() -> Vec3:
        """Ellipsoid semi-axes derived from the axis-aligned collision box."""
        return (COLLISION_X / 2, COLLISION_Y / 2, COLLISION_Z / 2)

    def _nearest_neighbor_positions(self, drone: Drone) -> list[Vec3]:
        pos = drone.position
        ranked: list[tuple[float, Vec3]] = []
        for other in self.drones:
            if other.drone_id == drone.drone_id:
                continue
            other_pos = (
                other.position[0],
                other.position[1],
                other.position[2],
            )
            ranked.append((Drone.distance(pos, other_pos), other_pos))
        ranked.sort(key=lambda item: item[0])
        return [p for _, p in ranked[:MAX_VORONOI_NEIGHBORS]]

    def _bvc_halfspaces(self, drone: Drone) -> list[HalfSpace]:
        pos: Vec3 = (drone.position[0], drone.position[1], drone.position[2])
        neighbors = self._nearest_neighbor_positions(drone)
        halfspaces = build_bvc_halfspaces(
            pos,
            neighbors,
            radii=self._collision_radii(),
            use_ellipsoid=True,
        )
        reach = (
            Drone.distance(pos, drone.target)
            + max(COLLISION_X, COLLISION_Y, COLLISION_Z) * len(self.drones)
            + self.step_size * 4
        )
        return add_bounding_box(halfspaces, pos, max(reach, self.step_size * 8))

    @staticmethod
    def _vec3_from_list(position: List[float]) -> Vec3:
        return (position[0], position[1], position[2])

    @staticmethod
    def _normalize(vec: Vec3) -> Vec3:
        length = math.sqrt(sum(c * c for c in vec))
        if length < 1e-12:
            return (0.0, 0.0, 0.0)
        return (vec[0] / length, vec[1] / length, vec[2] / length)

    @staticmethod
    def _lerp_vec3(a: Vec3, b: Vec3, t: float) -> Vec3:
        return (
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        )

    def _voronoi_waypoint(self, drone: Drone, halfspaces: list[HalfSpace]) -> Vec3:
        """Closest point in the buffered Voronoi cell to the drone goal."""
        pos = self._vec3_from_list(drone.position)
        goal: Vec3 = drone.target

        if not halfspaces or contains_point(halfspaces, goal):
            return goal

        raw = closest_point_in_polytope(
            halfspaces,
            goal,
            interior_hint=pos,
        )
        prev = self._guided_waypoint.get(drone.drone_id)
        if prev is None:
            smoothed = raw
        else:
            smoothed = self._lerp_vec3(prev, raw, WAYPOINT_SMOOTHING)
        self._guided_waypoint[drone.drone_id] = smoothed
        return smoothed

    def _step_along_blended_direction(
        self, drone: Drone, goal: Vec3, boundary_hint: Vec3
    ) -> List[float]:
        pos = self._vec3_from_list(drone.position)
        to_goal = self._normalize(
            (goal[0] - pos[0], goal[1] - pos[1], goal[2] - pos[2])
        )
        to_hint = self._normalize(
            (
                boundary_hint[0] - pos[0],
                boundary_hint[1] - pos[1],
                boundary_hint[2] - pos[2],
            )
        )
        direction = self._normalize(
            (
                GOAL_BLEND * to_goal[0] + (1.0 - GOAL_BLEND) * to_hint[0],
                GOAL_BLEND * to_goal[1] + (1.0 - GOAL_BLEND) * to_hint[1],
                GOAL_BLEND * to_goal[2] + (1.0 - GOAL_BLEND) * to_hint[2],
            )
        )
        if sum(abs(c) for c in direction) < 1e-12:
            return drone.peek_next_position(self.step_size)
        return [
            drone.position[0] + direction[0] * self.step_size,
            drone.position[1] + direction[1] * self.step_size,
            drone.position[2] + direction[2] * self.step_size,
        ]

    def _propose_move(self, drone: Drone) -> List[float]:
        if drone.arrived:
            return list(drone.position)

        greedy = self._clamp_position(drone.peek_next_position(self.step_size))

        if self._stagnation_steps[drone.drone_id] >= STAGNATION_LIMIT:
            return greedy

        halfspaces = self._bvc_halfspaces(drone)
        goal: Vec3 = drone.target

        if not halfspaces:
            return greedy

        if contains_point(halfspaces, goal) or contains_point(
            halfspaces, self._vec3_from_list(greedy)
        ):
            return greedy

        boundary_hint = self._voronoi_waypoint(drone, halfspaces)
        return self._clamp_position(
            self._step_along_blended_direction(drone, goal, boundary_hint)
        )

    # ── collision detection ──────────────────────────────────────────────

    @staticmethod
    def _is_colliding(a: List[float], b: List[float]) -> bool:
        return (
            abs(a[0] - b[0]) < COLLISION_X
            and abs(a[1] - b[1]) < COLLISION_Y
            and abs(a[2] - b[2]) < COLLISION_Z
        )

    def _find_collisions(
        self, proposed: Dict[int, List[float]]
    ) -> List[Tuple[int, int]]:
        ids = list(proposed.keys())
        collisions: List[Tuple[int, int]] = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a_id, b_id = ids[i], ids[j]
                if self._is_colliding(proposed[a_id], proposed[b_id]):
                    collisions.append((a_id, b_id))
        return collisions

    @staticmethod
    def _axis_open_interval_on_unit_segment(
        d0: float, v: float, limit: float
    ) -> tuple[float, float] | None:
        """``{ t in [0,1] : |d0 + v*t| < limit }`` as ``(lo, hi)``, or empty."""
        if abs(v) <= _SWEPT_TIME_EPS:
            if abs(d0) < limit:
                return (0.0, 1.0)
            return None
        t_lo = min((limit - d0) / v, (-limit - d0) / v)
        t_hi = max((limit - d0) / v, (-limit - d0) / v)
        lo = max(0.0, t_lo)
        hi = min(1.0, t_hi)
        if hi <= lo + _SWEPT_TIME_EPS:
            return None
        return (lo, hi)

    @classmethod
    def _swept_colliding(
        cls,
        a0: List[float],
        a1: List[float],
        b0: List[float],
        b1: List[float],
    ) -> bool:
        """True if some ``t in [0,1]`` has both drones inside the collision box.

        Each drone moves linearly ``p(t) = p0 + t*(p1-p0)`` with the same ``t``.
        """
        d0 = [a0[k] - b0[k] for k in range(3)]
        vrel = [(a1[k] - a0[k]) - (b1[k] - b0[k]) for k in range(3)]
        limits = (COLLISION_X, COLLISION_Y, COLLISION_Z)
        lo, hi = 0.0, 1.0
        for k in range(3):
            seg = cls._axis_open_interval_on_unit_segment(d0[k], vrel[k], limits[k])
            if seg is None:
                return False
            lo = max(lo, seg[0])
            hi = min(hi, seg[1])
            if hi <= lo + _SWEPT_TIME_EPS:
                return False
        return True

    def _pair_step_conflict(
        self,
        prev_a: List[float],
        next_a: List[float],
        prev_b: List[float],
        next_b: List[float],
    ) -> bool:
        """Endpoint or simultaneous linear motion violates the collision box."""
        if self._is_colliding(prev_a, prev_b):
            return True
        if self._is_colliding(next_a, next_b):
            return True
        return self._swept_colliding(prev_a, next_a, prev_b, next_b)

    def _find_step_collisions(
        self, prev: Dict[int, List[float]], proposed: Dict[int, List[float]]
    ) -> List[Tuple[int, int]]:
        ids = list(proposed.keys())
        collisions: List[Tuple[int, int]] = []
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a_id, b_id = ids[i], ids[j]
                if self._pair_step_conflict(
                    prev[a_id], proposed[a_id], prev[b_id], proposed[b_id]
                ):
                    collisions.append((a_id, b_id))
        return collisions

    # ── detour candidates ────────────────────────────────────────────────

    def _safe_slide(
        self,
        drone_id: int,
        prev_positions: Dict[int, List[float]],
        proposed: Dict[int, List[float]],
    ) -> List[float]:
        """Try a small sidestep instead of freezing after a collision."""
        drone = next(d for d in self.drones if d.drone_id == drone_id)
        base = list(prev_positions[drone_id])
        offsets = [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
            (1, 1, 0),
            (1, -1, 0),
            (-1, 1, 0),
            (-1, -1, 0),
        ]
        to_goal = self._normalize(
            (
                drone.target[0] - base[0],
                drone.target[1] - base[1],
                drone.target[2] - base[2],
            )
        )

        def score_offset(ox: int, oy: int, oz: int) -> float:
            mag = math.sqrt(ox * ox + oy * oy + oz * oz)
            direction = (ox / mag, oy / mag, oz / mag)
            return sum(direction[k] * to_goal[k] for k in range(3))

        ranked = sorted(offsets, key=lambda off: score_offset(*off), reverse=True)
        for ox, oy, oz in ranked:
            mag = math.sqrt(ox * ox + oy * oy + oz * oz)
            candidate = self._clamp_position(
                [
                    base[0] + ox / mag * self.step_size,
                    base[1] + oy / mag * self.step_size,
                    base[2] + oz / mag * self.step_size,
                ]
            )
            test = dict(proposed)
            test[drone_id] = candidate
            if not self._find_step_collisions(prev_positions, test):
                return candidate
        return base

    def _detour_candidates(self, drone: Drone, step_size: float) -> List[List[float]]:
        candidates: List[List[float]] = []
        offsets = [
            (1, 0, 0),
            (-1, 0, 0),
            (0, 1, 0),
            (0, -1, 0),
            (0, 0, 1),
            (0, 0, -1),
            (1, 1, 0),
            (1, -1, 0),
            (-1, 1, 0),
            (-1, -1, 0),
            (1, 0, 1),
            (1, 0, -1),
            (-1, 0, 1),
            (-1, 0, -1),
            (0, 1, 1),
            (0, 1, -1),
            (0, -1, 1),
            (0, -1, -1),
            (1, 1, 1),
            (1, 1, -1),
            (1, -1, 1),
            (1, -1, -1),
            (-1, 1, 1),
            (-1, 1, -1),
            (-1, -1, 1),
            (-1, -1, -1),
        ]
        for ox, oy, oz in offsets:
            mag = math.sqrt(ox * ox + oy * oy + oz * oz)
            nx = drone.position[0] + ox / mag * step_size
            ny = drone.position[1] + oy / mag * step_size
            nz = max(self.min_z, drone.position[2] + oz / mag * step_size)
            candidates.append([nx, ny, nz])
        random.shuffle(candidates)
        return candidates

    def _step_budget(self) -> int:
        longest = 0.0
        for d in self.drones:
            longest = max(longest, Drone.distance(d.position, d.target))
        travel_steps = int(math.ceil(longest / max(self.step_size, 1e-6)))
        budget = travel_steps * 5 + STEP_BUDGET_PER_DRONE * len(self.drones)
        return min(MAX_STEPS, max(budget, len(self.drones) * 10))

    # ── main loop ────────────────────────────────────────────────────────

    def solve(self) -> SolverResult:
        # Step 0: initial positions
        init_record = StepRecord(
            step=0,
            positions={d.drone_id: list(d.position) for d in self.drones},
            collisions=[],
            reverted_drones=[],
            verified=True,
        )
        self.history.append(init_record)
        if self.on_step:
            self.on_step(init_record)

        step_num = 0
        step_budget = self._step_budget()
        global_stagnation = 0
        while step_num < step_budget:
            if all(d.arrived for d in self.drones):
                break

            step_num += 1

            prev_positions: Dict[int, List[float]] = {
                d.drone_id: list(d.position) for d in self.drones
            }

            # Phase 1 — Voronoi/GJK-guided moves toward goal
            proposed: Dict[int, List[float]] = {}
            for d in self.drones:
                proposed[d.drone_id] = self._propose_move(d)

            # Phase 2 — collision resolution (endpoints + swept motion this step)
            collisions = self._find_step_collisions(prev_positions, proposed)
            reverted: List[int] = []

            if collisions:
                colliding_ids: Set[int] = set()
                for a_id, b_id in collisions:
                    colliding_ids.add(a_id)
                    colliding_ids.add(b_id)

                arrived_ids: Set[int] = {d.drone_id for d in self.drones if d.arrived}

                def priority(did: int, arrived_ids=arrived_ids) -> float:
                    if did in arrived_ids:
                        return float("inf")
                    d = next(dr for dr in self.drones if dr.drone_id == did)
                    return d.remaining_distance()

                sorted_colliders = sorted(
                    colliding_ids, key=lambda did: (-priority(did), did)
                )
                survivor = sorted_colliders[0]
                reverted = [
                    did
                    for did in sorted_colliders
                    if did != survivor and did not in arrived_ids
                ]

                for did in reverted:
                    proposed[did] = self._safe_slide(did, prev_positions, proposed)

                # Phase 2.5 — iterative resolution
                max_resolve_iter = len(self.drones) + 5
                for _ in range(max_resolve_iter):
                    post_collisions = self._find_step_collisions(
                        prev_positions, proposed
                    )
                    if not post_collisions:
                        break
                    for a_id, b_id in post_collisions:
                        for cand in (a_id, b_id):
                            if cand not in reverted and cand not in arrived_ids:
                                reverted.append(cand)
                                proposed[cand] = self._safe_slide(
                                    cand, prev_positions, proposed
                                )

            # Phase 2.7 — deadlocked drone detour attempts
            for did in list(reverted):
                drone = next(dr for dr in self.drones if dr.drone_id == did)
                if drone.arrived:
                    continue
                if self._consecutive_holds[did] >= DEADLOCK_THRESHOLD:
                    candidates = self._detour_candidates(drone, self.step_size)
                    for cand_pos in candidates:
                        test_proposed = dict(proposed)
                        test_proposed[did] = cand_pos
                        if not self._find_step_collisions(
                            prev_positions, test_proposed
                        ):
                            proposed[did] = cand_pos
                            reverted.remove(did)
                            break

            # Phase 3 — apply moves
            for d in self.drones:
                d.apply_move(proposed[d.drone_id])

            any_moved = False
            for d in self.drones:
                if d.drone_id in reverted:
                    self._consecutive_holds[d.drone_id] += 1
                else:
                    self._consecutive_holds[d.drone_id] = 0

                prev = prev_positions[d.drone_id]
                moved = math.sqrt(sum((d.position[k] - prev[k]) ** 2 for k in range(3)))
                if moved < self.step_size * 0.05:
                    self._stagnation_steps[d.drone_id] += 1
                else:
                    self._stagnation_steps[d.drone_id] = 0
                    any_moved = True

            if not any_moved:
                global_stagnation += 1
                if global_stagnation >= 30:
                    break
            else:
                global_stagnation = 0

            # Phase 4 — verify
            final_collisions = self._find_collisions(
                {d.drone_id: list(d.position) for d in self.drones}
            )
            verified = len(final_collisions) == 0

            record = StepRecord(
                step=step_num,
                positions={d.drone_id: list(d.position) for d in self.drones},
                collisions=collisions,
                reverted_drones=reverted,
                verified=verified,
            )
            self.history.append(record)

            if self.on_step:
                self.on_step(record)

        success = all(d.arrived for d in self.drones)
        return SolverResult(
            steps=self.history,
            total_steps=step_num,
            drones=self.drones,
            success=success,
        )

    def _clamp_position(self, position: List[float]) -> List[float]:
        """Keep generated paths inside the allowed altitude envelope."""
        return [position[0], position[1], max(self.min_z, position[2])]
