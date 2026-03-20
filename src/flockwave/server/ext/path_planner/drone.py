"""Drone model — a single drone in 3D coordinate space."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class Drone:
    """Tracks the state of a single drone."""

    drone_id: int
    initial: Vec3
    target: Vec3

    position: List[float] = field(init=False)
    arrived: bool = field(init=False, default=False)

    def __post_init__(self) -> None:
        self.position = list(self.initial)

    # ── helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def distance(a, b) -> float:
        return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))

    def remaining_distance(self) -> float:
        return self.distance(self.position, self.target)

    # ── movement ─────────────────────────────────────────────────────────

    def compute_step_vector(self, step_size: float = 1.0) -> List[float]:
        dx = self.target[0] - self.position[0]
        dy = self.target[1] - self.position[1]
        dz = self.target[2] - self.position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-9:
            self.arrived = True
            return [0.0, 0.0, 0.0]

        move = min(step_size, dist)
        ratio = move / dist
        return [dx * ratio, dy * ratio, dz * ratio]

    def peek_next_position(self, step_size: float = 1.0) -> List[float]:
        vec = self.compute_step_vector(step_size)
        return [self.position[i] + vec[i] for i in range(3)]

    def apply_move(self, new_pos: List[float]) -> None:
        self.position = list(new_pos)
        if self.remaining_distance() < 1e-9:
            self.arrived = True
