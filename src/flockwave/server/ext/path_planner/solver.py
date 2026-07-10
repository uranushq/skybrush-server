"""Greedy path-planning solver with collision avoidance.

Algorithm
---------
1. Each step, every drone proposes one greedy step toward its target.
2. Collision check: yaw-invariant bounding envelopes inflated by
   ``PLANNING_MARGIN``; endpoints and the simultaneous linear motion are
   checked **exactly** (closed-form swept AABB test, no sampling). A spatial
   hash broad-phase keeps the check near-linear in the number of drones.
3. Colliding drones are grouped into connected clusters. Within each cluster
   the drone with the largest remaining distance keeps moving; the others
   hold at their previous position. If conflicts remain, every non-arrived
   participant holds, which provably restores the previous (collision-free)
   state — so **every accepted step is collision-free by construction**.
4. Drones that held for ``DEADLOCK_THRESHOLD`` consecutive steps try detour
   moves in 26 unit directions (random order, altitude-clamped).
5. The loop **fails fast** instead of returning a partial path: if the fleet
   makes no net progress for ``STAGNATION_WINDOW`` consecutive steps, or
   ``MAX_STEPS`` is reached, the solver aborts with a machine-readable
   failure reason and the list of stuck drones.

Randomness comes from a private ``random.Random`` instance so the solver
never touches the process-global RNG and concurrent requests stay
reproducible when a seed is given.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from math import floor
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

from .collision_volume import (
    ENVELOPE_XY_HALF,
    ENVELOPE_Z_HEIGHT,
    PLANNING_MARGIN,
    envelope_overlap,
    envelope_overlap_swept,
)
from .drone import Drone, Vec3

MAX_STEPS = 50_000
DEADLOCK_THRESHOLD = 2

# Abort when the fleet's best-ever total remaining distance has not improved
# for this many consecutive steps. Detour maneuvers legitimately move away
# from targets for a while; the window is sized so only true deadlocks and
# livelocks trip it.
STAGNATION_WINDOW = 200

# 26 unit directions (faces, edges and corners of a cube), deduplicated by
# construction. Vertical escapes are covered by the (0, 0, ±1) entries; a
# larger raw offset would be normalized back to the same step anyway.
_DETOUR_OFFSETS: Tuple[Tuple[int, int, int], ...] = tuple(
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
)


@dataclass
class StepRecord:
    step: int
    positions: Dict[int, List[float]]
    collisions: List[Tuple[int, int]]
    reverted_drones: List[int]
    verified: bool
    yaws: Dict[int, float] | None = None


@dataclass
class SolverResult:
    steps: List[StepRecord]
    total_steps: int
    drones: List[Drone]
    success: bool
    failure_reason: Optional[str] = None
    stuck_drones: List[int] = field(default_factory=list)


class PathSolver:
    """Greedy collision-avoiding path planner.

    Collision checks use the yaw-invariant bounding envelope inflated by
    ``margin`` (default :data:`PLANNING_MARGIN`) so that the bounded position
    error introduced later by velocity smoothing cannot consume the
    clearance proven here.
    """

    def __init__(
        self,
        initials: List[Vec3],
        targets: List[Vec3],
        step_size: float = 1.0,
        seed: Optional[int] = None,
        on_step: Optional[Callable[[StepRecord], None]] = None,
        min_z: float = 0.0,
        margin: float = PLANNING_MARGIN,
    ) -> None:
        assert len(initials) == len(targets), "initial and target counts must match"
        self.step_size = step_size
        self.on_step = on_step
        self.min_z = float(min_z)
        self.margin = float(margin)
        self._rng = random.Random(seed)

        self.drones: List[Drone] = []
        for i, (ini, tgt) in enumerate(zip(initials, targets)):
            self.drones.append(Drone(drone_id=i, initial=tuple(ini), target=tuple(tgt)))
        self._drones_by_id: Dict[int, Drone] = {d.drone_id: d for d in self.drones}

        self.history: List[StepRecord] = []
        self._consecutive_holds: Dict[int, int] = dict.fromkeys(range(len(initials)), 0)

        # Broad-phase cell size: two drones can only interact within one
        # envelope reach plus one step of motion on each side.
        pair_reach = max(
            2.0 * (ENVELOPE_XY_HALF + self.margin),
            ENVELOPE_Z_HEIGHT + 2.0 * self.margin,
        )
        self._cell_size = pair_reach + 2.0 * step_size

    # ── collision detection ──────────────────────────────────────────────

    def _overlap(self, a: List[float], b: List[float]) -> bool:
        return envelope_overlap(a, b, margin=self.margin)

    def _swept_overlap(
        self,
        prev_a: List[float],
        next_a: List[float],
        prev_b: List[float],
        next_b: List[float],
    ) -> bool:
        return envelope_overlap_swept(prev_a, next_a, prev_b, next_b, margin=self.margin)

    def _candidate_pairs(
        self, prev: Dict[int, List[float]], proposed: Dict[int, List[float]]
    ) -> Iterable[Tuple[int, int]]:
        """Broad-phase: pairs whose inflated swept boxes share a hash cell.

        Each drone's swept box (from its previous to its proposed position)
        is inflated by *half* the pair interaction reach per axis. If two
        drones interact at any time t, the midpoint of their positions at t
        lies inside both inflated boxes, so both cover the cell containing
        it — interacting pairs therefore always share at least one cell,
        regardless of the cell size.
        """
        cell = self._cell_size
        inflate_xy = ENVELOPE_XY_HALF + self.margin
        inflate_z = ENVELOPE_Z_HEIGHT / 2.0 + self.margin
        buckets: Dict[Tuple[int, int, int], List[int]] = {}
        for did, next_pos in proposed.items():
            prev_pos = prev[did]
            x0 = min(prev_pos[0], next_pos[0]) - inflate_xy
            x1 = max(prev_pos[0], next_pos[0]) + inflate_xy
            y0 = min(prev_pos[1], next_pos[1]) - inflate_xy
            y1 = max(prev_pos[1], next_pos[1]) + inflate_xy
            z0 = min(prev_pos[2], next_pos[2]) - inflate_z
            z1 = max(prev_pos[2], next_pos[2]) + inflate_z
            for cx in range(floor(x0 / cell), floor(x1 / cell) + 1):
                for cy in range(floor(y0 / cell), floor(y1 / cell) + 1):
                    for cz in range(floor(z0 / cell), floor(z1 / cell) + 1):
                        buckets.setdefault((cx, cy, cz), []).append(did)

        seen: Set[Tuple[int, int]] = set()
        for ids in buckets.values():
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    a, b = ids[i], ids[j]
                    seen.add((a, b) if a < b else (b, a))
        return seen

    def _find_collisions(
        self, positions: Dict[int, List[float]]
    ) -> List[Tuple[int, int]]:
        return self._find_step_collisions(positions, positions)

    def _find_step_collisions(
        self, prev: Dict[int, List[float]], proposed: Dict[int, List[float]]
    ) -> List[Tuple[int, int]]:
        collisions: List[Tuple[int, int]] = []
        for a_id, b_id in self._candidate_pairs(prev, proposed):
            if self._swept_overlap(
                prev[a_id], proposed[a_id], prev[b_id], proposed[b_id]
            ):
                collisions.append((a_id, b_id))
        return collisions

    # ── collision resolution ─────────────────────────────────────────────

    @staticmethod
    def _clusters(pairs: List[Tuple[int, int]]) -> List[Set[int]]:
        """Connected components of the collision graph."""
        adjacency: Dict[int, Set[int]] = {}
        for a, b in pairs:
            adjacency.setdefault(a, set()).add(b)
            adjacency.setdefault(b, set()).add(a)
        clusters: List[Set[int]] = []
        unvisited = set(adjacency)
        while unvisited:
            frontier = [unvisited.pop()]
            component: Set[int] = set(frontier)
            while frontier:
                node = frontier.pop()
                for neighbour in adjacency[node]:
                    if neighbour not in component:
                        component.add(neighbour)
                        frontier.append(neighbour)
            unvisited -= component
            clusters.append(component)
        return clusters

    def _resolve_collisions(
        self,
        prev: Dict[int, List[float]],
        proposed: Dict[int, List[float]],
        collisions: List[Tuple[int, int]],
        arrived_ids: Set[int],
    ) -> Optional[List[int]]:
        """Revert drones until the step is collision-free.

        Independent collision clusters are resolved independently: each keeps
        exactly one moving survivor (largest remaining distance first). Falls
        back to holding every non-arrived participant, which restores the
        previous verified state. Returns the reverted ids, or ``None`` if a
        conflict persists even then (the previous state itself must have been
        invalid — an internal invariant violation).
        """
        reverted: List[int] = []
        reverted_set: Set[int] = set()

        def revert(did: int) -> None:
            if did not in reverted_set and did not in arrived_ids:
                reverted_set.add(did)
                reverted.append(did)
                proposed[did] = list(prev[did])

        def priority(did: int) -> float:
            if did in arrived_ids:
                return float("inf")
            return self._drones_by_id[did].remaining_distance()

        for cluster in self._clusters(collisions):
            order = sorted(cluster, key=priority, reverse=True)
            for did in order[1:]:
                revert(did)

        max_resolve_iter = len(self.drones) + 5
        for _ in range(max_resolve_iter):
            post_collisions = self._find_step_collisions(prev, proposed)
            if not post_collisions:
                return reverted
            progressed = False
            for a_id, b_id in post_collisions:
                for cand in (a_id, b_id):
                    if cand not in reverted_set and cand not in arrived_ids:
                        revert(cand)
                        progressed = True
            if not progressed:
                break

        # Fallback: hold every non-arrived drone that was still moving in a
        # conflicting pair. All participants become stationary, so remaining
        # conflicts would mean the previous state already overlapped.
        for a_id, b_id in self._find_step_collisions(prev, proposed):
            revert(a_id)
            revert(b_id)
        if self._find_step_collisions(prev, proposed):
            return None
        return reverted

    # ── detour candidates ────────────────────────────────────────────────

    def _detour_candidates(self, drone: Drone, step_size: float) -> List[List[float]]:
        candidates: List[List[float]] = []
        for ox, oy, oz in _DETOUR_OFFSETS:
            mag = (ox * ox + oy * oy + oz * oz) ** 0.5
            nx = drone.position[0] + ox / mag * step_size
            ny = drone.position[1] + oy / mag * step_size
            nz = max(self.min_z, drone.position[2] + oz / mag * step_size)
            candidates.append([nx, ny, nz])
        self._rng.shuffle(candidates)
        return candidates

    # ── failure helper ───────────────────────────────────────────────────

    def _failure(self, step_num: int, reason: str) -> SolverResult:
        return SolverResult(
            steps=self.history,
            total_steps=step_num,
            drones=self.drones,
            success=False,
            failure_reason=reason,
            stuck_drones=[d.drone_id for d in self.drones if not d.arrived],
        )

    # ── main loop ────────────────────────────────────────────────────────

    def solve(self) -> SolverResult:
        # Fail fast on infeasible inputs: overlapping starts or targets can
        # never produce a collision-free plan.
        initial_positions = {d.drone_id: list(d.position) for d in self.drones}
        conflicts = self._find_collisions(initial_positions)
        if conflicts:
            return self._failure(
                0,
                "initial positions violate the required clearance for pairs "
                f"{sorted(conflicts)}",
            )
        target_positions = {d.drone_id: list(d.target) for d in self.drones}
        conflicts = self._find_collisions(target_positions)
        if conflicts:
            return self._failure(
                0,
                "target positions violate the required clearance for pairs "
                f"{sorted(conflicts)}",
            )

        init_record = StepRecord(
            step=0,
            positions=initial_positions,
            collisions=[],
            reverted_drones=[],
            verified=True,
        )
        self.history.append(init_record)
        if self.on_step:
            self.on_step(init_record)

        best_remaining = sum(d.remaining_distance() for d in self.drones)
        stagnant_steps = 0

        step_num = 0
        while step_num < MAX_STEPS:
            if all(d.arrived for d in self.drones):
                break

            step_num += 1

            prev_positions: Dict[int, List[float]] = {
                d.drone_id: list(d.position) for d in self.drones
            }
            arrived_ids: Set[int] = {d.drone_id for d in self.drones if d.arrived}

            # Phase 1 — propose greedy moves
            proposed: Dict[int, List[float]] = {}
            for d in self.drones:
                if d.arrived:
                    proposed[d.drone_id] = list(d.position)
                else:
                    proposed[d.drone_id] = self._clamp_position(
                        d.peek_next_position(self.step_size)
                    )

            # Phase 2 — collision resolution per cluster
            collisions = self._find_step_collisions(prev_positions, proposed)
            reverted: List[int] = []
            if collisions:
                resolved = self._resolve_collisions(
                    prev_positions, proposed, collisions, arrived_ids
                )
                if resolved is None:
                    return self._failure(
                        step_num,
                        "internal error: collision persists with all drones "
                        "holding — previous step state was already invalid",
                    )
                reverted = resolved

            # Phase 2.7 — deadlocked drone detour attempts
            for did in list(reverted):
                drone = self._drones_by_id[did]
                if drone.arrived:
                    continue
                if self._consecutive_holds[did] >= DEADLOCK_THRESHOLD:
                    for cand_pos in self._detour_candidates(drone, self.step_size):
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

            for d in self.drones:
                if d.drone_id in reverted:
                    self._consecutive_holds[d.drone_id] += 1
                else:
                    self._consecutive_holds[d.drone_id] = 0

            # Every accepted step is collision-free by construction (the
            # resolution fallback restores the previous verified state), so
            # the record is verified without a redundant full re-check.
            record = StepRecord(
                step=step_num,
                positions={d.drone_id: list(d.position) for d in self.drones},
                collisions=collisions,
                reverted_drones=reverted,
                verified=True,
            )
            self.history.append(record)

            if self.on_step:
                self.on_step(record)

            # Phase 4 — stagnation detection (fail fast on deadlock/livelock)
            total_remaining = sum(d.remaining_distance() for d in self.drones)
            if total_remaining < best_remaining - 1e-6:
                best_remaining = total_remaining
                stagnant_steps = 0
            else:
                stagnant_steps += 1
                if stagnant_steps >= STAGNATION_WINDOW:
                    return self._failure(
                        step_num,
                        f"no progress for {STAGNATION_WINDOW} consecutive "
                        "steps — the formation is deadlocked",
                    )

        if not all(d.arrived for d in self.drones):
            return self._failure(
                step_num, f"step limit of {MAX_STEPS} reached before arrival"
            )

        return SolverResult(
            steps=self.history,
            total_steps=step_num,
            drones=self.drones,
            success=True,
        )

    def _clamp_position(self, position: List[float]) -> List[float]:
        """Keep generated paths inside the allowed altitude envelope."""
        return [position[0], position[1], max(self.min_z, position[2])]
