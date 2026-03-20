"""Greedy path-planning solver with collision avoidance.

Algorithm
---------
1. Each step, every drone moves one step toward its target (greedy).
2. Collision check after proposed moves (|dx| < 1, |dy| < 1, |dz| < 4).
3. Colliding drones revert to their previous position (hold), prioritised
   by remaining distance.
4. Deadlocked drones attempt detour moves.
5. Repeats until all drones arrive or MAX_STEPS is reached.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set, Tuple

from .drone import Drone, Vec3

# collision envelope per axis
COLLISION_X = 1.0
COLLISION_Y = 1.0
COLLISION_Z = 4.0

MAX_STEPS = 50_000
DEADLOCK_THRESHOLD = 2


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
    ) -> None:
        assert len(initials) == len(targets), (
            "initial and target counts must match"
        )
        self.step_size = step_size
        self.on_step = on_step

        if seed is not None:
            random.seed(seed)

        self.drones: List[Drone] = []
        for i, (ini, tgt) in enumerate(zip(initials, targets)):
            self.drones.append(
                Drone(drone_id=i, initial=tuple(ini), target=tuple(tgt))
            )

        self.history: List[StepRecord] = []
        self._consecutive_holds: Dict[int, int] = {
            i: 0 for i in range(len(initials))
        }

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

    # ── detour candidates ────────────────────────────────────────────────

    def _detour_candidates(
        self, drone: Drone, step_size: float
    ) -> List[List[float]]:
        candidates: List[List[float]] = []
        offsets = [
            (1, 0, 0), (-1, 0, 0),
            (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1),
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),
            (1, 0, 1), (1, 0, -1), (-1, 0, 1), (-1, 0, -1),
            (0, 1, 1), (0, 1, -1), (0, -1, 1), (0, -1, -1),
            (1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
            (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1),
            (0, 0, 4), (0, 0, -4),
            (1, 0, 4), (-1, 0, 4), (0, 1, 4), (0, -1, 4),
            (1, 0, -4), (-1, 0, -4), (0, 1, -4), (0, -1, -4),
        ]
        for ox, oy, oz in offsets:
            mag = math.sqrt(ox * ox + oy * oy + oz * oz)
            nx = drone.position[0] + ox / mag * step_size
            ny = drone.position[1] + oy / mag * step_size
            nz = drone.position[2] + oz / mag * step_size
            candidates.append([nx, ny, nz])
        random.shuffle(candidates)
        return candidates

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
        while step_num < MAX_STEPS:
            if all(d.arrived for d in self.drones):
                break

            step_num += 1

            prev_positions: Dict[int, List[float]] = {
                d.drone_id: list(d.position) for d in self.drones
            }

            # Phase 1 — propose greedy moves
            proposed: Dict[int, List[float]] = {}
            for d in self.drones:
                if d.arrived:
                    proposed[d.drone_id] = list(d.position)
                else:
                    proposed[d.drone_id] = d.peek_next_position(self.step_size)

            # Phase 2 — collision resolution
            collisions = self._find_collisions(proposed)
            reverted: List[int] = []

            if collisions:
                colliding_ids: Set[int] = set()
                for a_id, b_id in collisions:
                    colliding_ids.add(a_id)
                    colliding_ids.add(b_id)

                arrived_ids: Set[int] = {
                    d.drone_id for d in self.drones if d.arrived
                }

                def priority(did: int) -> float:
                    if did in arrived_ids:
                        return float("inf")
                    d = next(dr for dr in self.drones if dr.drone_id == did)
                    return d.remaining_distance()

                sorted_colliders = sorted(
                    colliding_ids, key=priority, reverse=True
                )
                survivor = sorted_colliders[0]
                reverted = [
                    did
                    for did in sorted_colliders
                    if did != survivor and did not in arrived_ids
                ]

                for did in reverted:
                    proposed[did] = list(prev_positions[did])

                # Phase 2.5 — iterative resolution
                max_resolve_iter = len(self.drones) + 5
                for _ in range(max_resolve_iter):
                    post_collisions = self._find_collisions(proposed)
                    if not post_collisions:
                        break
                    for a_id, b_id in post_collisions:
                        for cand in (a_id, b_id):
                            if cand not in reverted and cand not in arrived_ids:
                                reverted.append(cand)
                                proposed[cand] = list(prev_positions[cand])

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
                        if not self._find_collisions(test_proposed):
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
