"""Greedy path-planning solver with collision avoidance.

Algorithm
---------
0. **Dispatch gate** (downwash-aware flow control): drones start each
   segment *waiting* and are released in **destination-altitude** order
   (highest target first — upper formation slots fill before anyone flies
   in underneath them). Every step, waiting drones are scanned in priority order and
   released only if (a) at most :data:`MAX_CONCURRENT_MOVERS` drones are
   moving afterwards — a hard cap — and (b) every currently moving drone is
   at least :data:`RELEASE_DISTANCE` away horizontally. Drones far apart in
   the plane therefore fly concurrently even at different altitudes, while
   a drone near (or under) an active mover departs staggered, only after
   the leader has pulled :data:`RELEASE_DISTANCE` ahead. A waiting drone that sits on a released
   drone's target is boosted out of turn (the cap still holds; if the cap is
   full, the blocked mover is parked back to waiting to free a slot).
1. **Routing**: each released drone follows a per-drone route. When the
   straight line to the target is clear of every *parked* drone (waiting or
   arrived — the static obstacles of the moment) the route is that line;
   otherwise a 26-connected lattice **A\*** around the parked drones plans
   one. The downwash column :data:`DOWNWASH_ROUTE_CLEARANCE` below every
   parked drone is treated as blocked (except the short final goal-connect
   edge — targets under a temporarily parked drone stay reachable), and
   route costs discount climbs, penalize descents and surcharge edges
   hugging obstacles — a blocked drone always prefers to go **over** a
   parked formation, never under or through tight lateral gaps. Routes are
   re-planned when a parked drone appears on the next route edge, dropped
   after a dynamic detour, and drones deadlocked among concurrent movers
   re-plan an **escape route** (one drone per step, treating the other
   movers as obstacles) so even symmetric head-on meets resolve
   deterministically.
2. Collision check between the *simultaneously moving* drones (and against
   parked ones): yaw-invariant bounding envelopes inflated by
   ``PLANNING_MARGIN``; endpoints and the simultaneous linear motion are
   checked **exactly** (closed-form swept AABB test, no sampling). A spatial
   hash broad-phase keeps the check near-linear in the number of drones.
3. Colliding drones are grouped into connected clusters. Within each cluster
   the drone with the largest remaining distance keeps moving; the others
   hold at their previous position. If conflicts remain, every non-arrived
   participant holds, which provably restores the previous (collision-free)
   state — so **every accepted step is collision-free by construction**.
4. Drones that held for ``DEADLOCK_THRESHOLD`` consecutive steps take a
   detour step, chosen **deterministically** from 26 unit directions:
   goal-biased (shortest resulting distance to the route head), preferring
   climbs over descents, with a short-lived directional memory so
   consecutive detours commit to one way around instead of oscillating.
5. The loop **fails fast** instead of returning a partial path: if the fleet
   makes no net progress for ``STAGNATION_WINDOW`` consecutive steps, or
   ``MAX_STEPS`` is reached, the solver aborts with a machine-readable
   failure reason and the list of stuck drones.

The solver is fully deterministic; the ``seed`` parameter is kept for API
compatibility but no randomness remains.
"""

from __future__ import annotations

import heapq
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

# Consecutive *unproductive* holds before a drone tries a detour step.
# Yielding to a collision partner that actually moved this step is a
# productive wait (the conflict is clearing on its own) and is not counted:
# holding costs only time, while every detour costs real meters. Tuned via
# the path-efficiency benchmark: reacting quickly to *unproductive* holds
# beats waiting longer, because the productive-yield accounting already
# filters out the conflicts that clear by themselves.
DEADLOCK_THRESHOLD = 2

# Unproductive holds before a drone re-plans an escape route around the
# other stuck movers (one drone per step). Same benchmark result: an early
# escape disperses congestion before it compounds.
ESCAPE_AFTER_HOLDS = 2

# While a drone's goal is occupied by another drone, detouring around the
# goal achieves nothing — the drone politely holds for this many steps to
# give the dispatch gate's boost chain time to move the squatter away.
# After the limit it detours anyway: mutual squats (two drones parked on
# each other's targets) can only be resolved by one of them stepping aside.
SQUAT_WAIT_LIMIT = 10

# Yielding to a moving collision partner is forgiven (not counted as a
# hold) only while the drone made progress recently. Two crossing drones
# can otherwise take turns moving in a closed 2-cycle — each step one of
# them moves, so both look "productive" forever while neither gets closer.
PRODUCTIVE_PATIENCE = 12

# A mover that held this many consecutive steps counts as hovering (not
# translating) for the release-distance rule.
HOVERING_HOLDS = 2

# Dispatch gate: hard cap on simultaneously moving drones. Operational rule —
# never raise this above 5.
MAX_CONCURRENT_MOVERS = 5

# A waiting drone is released only when every currently moving drone is at
# least this far away **horizontally** (meters). Horizontal distance is the
# right measure for downwash: a mover overhead within this radius is exactly
# the situation a lower drone must not depart into, while drones far apart
# in the plane are independent regardless of altitude. Chosen above the 4 m
# downwash-concern range used by the staged stack entry.
RELEASE_DISTANCE = 5.0

# ── route planner (lattice A*) tuning ────────────────────────────────────
# Hard cap on node expansions per plan; beyond this the drone falls back to
# greedy pursuit (and the deterministic detour logic).
ASTAR_MAX_EXPANSIONS = 4000
# Search box: bounding box of start/goal inflated by this many steps
# horizontally, plus generous headroom above (climbing over formations is
# the preferred escape) and a little below.
ASTAR_XY_HEADROOM_STEPS = 8
ASTAR_UP_HEADROOM_STEPS = 8
ASTAR_DOWN_HEADROOM_STEPS = 2
# A node this close to the goal may connect to it directly (if clear).
ASTAR_GOAL_CONNECT_STEPS = 1.5
# Ascending edges are discounted and descending edges surcharged, so a
# blocked drone prefers climbing *over* an obstacle to sidestepping around
# it laterally — and never gains from dive-and-return zigzags
# (0.7 + 1.5 > 2). The A* heuristic is scaled by the discount to stay
# admissible.
ASTAR_ASCENT_DISCOUNT = 0.7
ASTAR_DOWNWARD_PENALTY = 0.5
# Route planning treats the downwash column *under* every parked drone as
# blocked, so routes never pass within this many meters below one (matching
# the 4 m downwash-concern range of the staged stack entry). Checked
# exactly via an asymmetric z window in the swept envelope test.
DOWNWASH_ROUTE_CLEARANCE = 4.0
# Edges passing within PROXIMITY_EXTRA of a parked drone's (margin-inflated)
# envelope cost this fraction extra — wide/over routes beat tight squeezes.
ASTAR_PROXIMITY_SURCHARGE = 0.3
ASTAR_PROXIMITY_EXTRA = 0.25
# After a failed plan attempt, wait this many steps before trying again.
REPLAN_COOLDOWN = 5
# How long (steps) a chosen detour direction is remembered and favoured.
DETOUR_BIAS_STEPS = 3

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
    # Steps flagged constant-speed (e.g. staged vertical stack entries) are
    # exempted from velocity smoothing so their commanded speed is kept
    # exactly — see ``apply_velocity_smoothing``.
    constant_speed: bool = False


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
        # ``seed`` is accepted for API compatibility; the solver is fully
        # deterministic and uses no randomness.
        del seed

        self.drones: List[Drone] = []
        for i, (ini, tgt) in enumerate(zip(initials, targets)):
            self.drones.append(Drone(drone_id=i, initial=tuple(ini), target=tuple(tgt)))
        self._drones_by_id: Dict[int, Drone] = {d.drone_id: d for d in self.drones}

        self.history: List[StepRecord] = []
        self._consecutive_holds: Dict[int, int] = dict.fromkeys(range(len(initials)), 0)

        # Dispatch gate state: highest destination altitude first (drone id
        # breaks ties deterministically). Everyone starts waiting; drones
        # already at their target never occupy a mover slot.
        self._dispatch_order: List[int] = sorted(
            (d.drone_id for d in self.drones),
            key=lambda did: (-self._drones_by_id[did].target[2], did),
        )
        self._released: Set[int] = {d.drone_id for d in self.drones if d.arrived}

        # Route-planner state: per-drone waypoint queue (ending at the
        # target), replan cooldown counters and detour direction memory.
        self._routes: Dict[int, List[Tuple[float, float, float]]] = {}
        self._replan_cooldown: Dict[int, int] = {}
        self._detour_bias: Dict[int, Tuple[float, float, float]] = {}
        self._detour_bias_ttl: Dict[int, int] = dict.fromkeys(
            range(len(initials)), 0
        )
        # Drones that got a fresh escape route this step (skip detours once
        # so the route gets a chance before being torn down again).
        self._escaped_this_step: Set[int] = set()
        # Futile-detour tracking: detours reset the hold counter, so without
        # this a drone can 3-hold/detour-dance forever below the escape
        # threshold. A detour is futile when the drone's best-ever distance
        # to its target has not improved since the previous detour.
        self._best_remaining: Dict[int, float] = {
            d.drone_id: d.remaining_distance() for d in self.drones
        }
        self._futile_detours: Dict[int, int] = dict.fromkeys(
            range(len(initials)), 0
        )
        self._steps_since_progress: Dict[int, int] = dict.fromkeys(
            range(len(initials)), 0
        )

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

    # ── route planning (static obstacles = parked drones) ───────────────

    def _static_positions(self, active_ids: Set[int]) -> List[List[float]]:
        """Positions of every drone that is *not* actively moving."""
        return [
            list(d.position)
            for d in self.drones
            if d.drone_id not in active_ids
        ]

    def _edge_blocked(
        self,
        a,
        b,
        statics: List[List[float]],
        *,
        margin: Optional[float] = None,
        downwash: bool = False,
    ) -> bool:
        """Whether the straight edge a→b hits any parked drone's envelope.

        With ``downwash=True`` the check also covers each obstacle's
        downwash column (:data:`DOWNWASH_ROUTE_CLEARANCE` meters below it),
        so route planning never sends a drone underneath a parked one.
        Dynamic legality checks (the solver's swept collision tests) stay
        envelope-exact — the staged stack entry deliberately climbs through
        this column.
        """
        checked_margin = self.margin if margin is None else margin
        below = DOWNWASH_ROUTE_CLEARANCE if downwash else 0.0
        for obs in statics:
            if envelope_overlap_swept(
                a, b, obs, obs, margin=checked_margin, b_extends_below=below
            ):
                return True
        return False

    def _plan_route(
        self, drone: Drone, statics: List[List[float]]
    ) -> Optional[List[Tuple[float, float, float]]]:
        """Plan a waypoint route around parked drones (see module docstring).

        Returns a waypoint list ending at the target, or ``None`` when no
        plan was found within the expansion budget (the caller falls back to
        greedy pursuit). The straight line is tried first, so the lattice
        A* only runs for actually blocked drones.
        """
        start = tuple(drone.position)
        goal = tuple(drone.target)

        if not self._edge_blocked(start, goal, statics, downwash=True):
            return [goal]

        # Goal entry is exempt from the downwash-column restriction (a
        # legitimate target below a temporarily parked drone must stay
        # reachable), so allow a short plain-checked final connect.
        goal_connect = ASTAR_GOAL_CONNECT_STEPS * self.step_size
        if Drone.distance(start, goal) <= goal_connect and not self._edge_blocked(
            start, goal, statics
        ):
            return [goal]

        # A goal squatted by a parked drone is unreachable until the
        # dispatch gate boosts the squatter away; a start locked inside a
        # downwash column cannot be routed out (every outgoing edge starts
        # blocked — e.g. a staged stack climb, which the greedy pursuit
        # handles). Don't burn the search budget on either.
        for obs in statics:
            if envelope_overlap(obs, list(goal), margin=self.margin):
                return None
            if envelope_overlap(
                list(start),
                obs,
                margin=self.margin,
                b_extends_below=DOWNWASH_ROUTE_CLEARANCE,
            ):
                return None

        step = self.step_size
        xy_pad = ASTAR_XY_HEADROOM_STEPS * step
        lo = (
            min(start[0], goal[0]) - xy_pad,
            min(start[1], goal[1]) - xy_pad,
            max(
                self.min_z,
                min(start[2], goal[2]) - ASTAR_DOWN_HEADROOM_STEPS * step,
            ),
        )
        hi = (
            max(start[0], goal[0]) + xy_pad,
            max(start[1], goal[1]) + xy_pad,
            max(start[2], goal[2]) + ASTAR_UP_HEADROOM_STEPS * step,
        )
        proximity_margin = self.margin + ASTAR_PROXIMITY_EXTRA

        # Only obstacles whose (downwash-extended, margin-inflated) envelope
        # can reach into the search box matter for this plan.
        xy_reach = 2.0 * (ENVELOPE_XY_HALF + proximity_margin)
        z_reach = ENVELOPE_Z_HEIGHT + 2.0 * proximity_margin
        statics = [
            obs
            for obs in statics
            if lo[0] - xy_reach <= obs[0] <= hi[0] + xy_reach
            and lo[1] - xy_reach <= obs[1] <= hi[1] + xy_reach
            and lo[2] - z_reach <= obs[2] <= hi[2] + z_reach + DOWNWASH_ROUTE_CLEARANCE
        ]

        def node_pos(node: Tuple[int, int, int]) -> Tuple[float, float, float]:
            return (
                start[0] + node[0] * step,
                start[1] + node[1] * step,
                start[2] + node[2] * step,
            )

        def heuristic(pos) -> float:
            # Scaled by the cheapest possible cost multiplier (the ascent
            # discount) so the estimate never overshoots the true cost.
            return ASTAR_ASCENT_DISCOUNT * Drone.distance(pos, goal)

        origin = (0, 0, 0)
        g_cost: Dict[Tuple[int, int, int], float] = {origin: 0.0}
        parent: Dict[Tuple[int, int, int], Tuple[int, int, int]] = {}
        counter = 0
        open_heap = [(heuristic(start), 0, origin)]
        expansions = 0

        while open_heap and expansions < ASTAR_MAX_EXPANSIONS:
            f_value, _, node = heapq.heappop(open_heap)
            pos = node_pos(node)
            if f_value > g_cost[node] + heuristic(pos) + 1e-9:
                continue  # stale heap entry

            # Goal connect: plain envelope check (no downwash restriction) —
            # the final approach may legitimately enter a parked drone's
            # downwash column; it is at most goal_connect long.
            if Drone.distance(pos, goal) <= goal_connect and not self._edge_blocked(
                pos, goal, statics
            ):
                waypoints: List[Tuple[float, float, float]] = []
                cursor = node
                while cursor != origin:
                    waypoints.append(node_pos(cursor))
                    cursor = parent[cursor]
                waypoints.reverse()
                waypoints.append(goal)
                return self._compress_route(start, waypoints)

            expansions += 1
            for ox, oy, oz in _DETOUR_OFFSETS:
                neighbour = (node[0] + ox, node[1] + oy, node[2] + oz)
                npos = node_pos(neighbour)
                if not (
                    lo[0] <= npos[0] <= hi[0]
                    and lo[1] <= npos[1] <= hi[1]
                    and lo[2] <= npos[2] <= hi[2]
                ):
                    continue
                length = step * (ox * ox + oy * oy + oz * oz) ** 0.5
                cost = length
                if oz > 0:
                    cost = ASTAR_ASCENT_DISCOUNT * length
                elif oz < 0:
                    cost += ASTAR_DOWNWARD_PENALTY * length
                candidate_g = g_cost[node] + cost
                known = g_cost.get(neighbour)
                if known is not None and known <= candidate_g + 1e-12:
                    continue
                if self._edge_blocked(pos, npos, statics, downwash=True):
                    continue
                if self._edge_blocked(
                    pos, npos, statics, margin=proximity_margin, downwash=True
                ):
                    candidate_g += ASTAR_PROXIMITY_SURCHARGE * length
                    if known is not None and known <= candidate_g + 1e-12:
                        continue
                g_cost[neighbour] = candidate_g
                parent[neighbour] = node
                counter += 1
                heapq.heappush(
                    open_heap,
                    (candidate_g + heuristic(npos), counter, neighbour),
                )

        return None

    @staticmethod
    def _compress_route(start, waypoints):
        """Merge collinear runs so the route holds only direction changes.

        Safe: consecutive collinear lattice edges were each collision-checked
        and their union covers the merged straight segment exactly.
        """
        compressed: List[Tuple[float, float, float]] = []
        prev_point = start
        prev_dir: Optional[Tuple[float, float, float]] = None
        for waypoint in waypoints:
            d = (
                waypoint[0] - prev_point[0],
                waypoint[1] - prev_point[1],
                waypoint[2] - prev_point[2],
            )
            norm = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) ** 0.5
            if norm < 1e-12:
                continue
            direction = (d[0] / norm, d[1] / norm, d[2] / norm)
            if prev_dir is not None and all(
                abs(direction[axis] - prev_dir[axis]) < 1e-9 for axis in range(3)
            ):
                compressed[-1] = waypoint
            else:
                compressed.append(waypoint)
                prev_dir = direction
            prev_point = waypoint
        return compressed

    def _route_head(self, drone: Drone) -> Tuple[float, float, float]:
        """Current pursuit point: next route waypoint, or the target."""
        route = self._routes.get(drone.drone_id)
        return route[0] if route else tuple(drone.target)

    def _propose_move(
        self, drone: Drone, statics: List[List[float]]
    ) -> List[float]:
        """Proposal for one released drone: follow its route (planning or
        re-planning it as needed), stepping at most ``step_size``.

        A drone that has been deadlocked for a while re-plans an **escape
        route** that additionally treats the other active movers' current
        positions as obstacles (lightweight prioritized planning). This
        breaks the symmetric livelocks a deterministic solver would
        otherwise preserve — e.g. several drones meeting head-on at the
        centre of a symmetric formation swap.
        """
        did = drone.drone_id

        route = self._routes.get(did)
        if route is not None and self._edge_blocked(
            drone.position, list(route[0]), statics, downwash=True
        ):
            route = None  # a drone parked on/under the next edge — re-plan

        # Escape planning is limited to ONE drone per step: simultaneous
        # escapes in a symmetric configuration would produce congruent
        # (again-conflicting) plans — sequencing them breaks the symmetry
        # deterministically.
        holds = self._consecutive_holds.get(did, 0)
        if not self._escaped_this_step and (
            (
                holds >= ESCAPE_AFTER_HOLDS
                and (holds - ESCAPE_AFTER_HOLDS) % 3 == 0
            )
            or self._futile_detours.get(did, 0) >= 2
        ):
            # Only movers that are themselves stuck count as escape
            # obstacles — arcing around a drone that is about to move away
            # wastes meters; conflicts with genuinely translating drones
            # resolve by yielding. "Stuck" means holding right now OR not
            # having made progress for a while: two mirror-symmetric drones
            # can dance a closed cycle in which each happens to have moved
            # on the step the other plans its escape.
            movers = [
                list(d.position)
                for d in self.drones
                if d.drone_id != did
                and d.drone_id in self._released
                and not d.arrived
                and (
                    self._consecutive_holds.get(d.drone_id, 0) > 0
                    or self._steps_since_progress.get(d.drone_id, 0)
                    >= PRODUCTIVE_PATIENCE
                )
            ]
            escape = self._plan_route(drone, statics + movers)
            if escape is not None:
                self._routes[did] = escape
                self._escaped_this_step.add(did)
                self._futile_detours[did] = 0
                route = escape

        if route is None:
            cooldown = self._replan_cooldown.get(did, 0)
            if cooldown > 0:
                self._replan_cooldown[did] = cooldown - 1
            else:
                route = self._plan_route(drone, statics)
                if route is None:
                    self._replan_cooldown[did] = REPLAN_COOLDOWN
            if route is not None:
                self._routes[did] = route

        head = self._route_head(drone)
        dx = head[0] - drone.position[0]
        dy = head[1] - drone.position[1]
        dz = head[2] - drone.position[2]
        distance = (dx * dx + dy * dy + dz * dz) ** 0.5
        if distance < 1e-9:
            return list(drone.position)
        move = min(self.step_size, distance)
        ratio = move / distance
        return self._clamp_position(
            [
                drone.position[0] + dx * ratio,
                drone.position[1] + dy * ratio,
                drone.position[2] + dz * ratio,
            ]
        )

    def _advance_routes(self) -> None:
        """Pop reached waypoints after moves were applied."""
        for did, route in list(self._routes.items()):
            drone = self._drones_by_id[did]
            while route and Drone.distance(drone.position, route[0]) < 1e-9:
                route.pop(0)
            if not route:
                del self._routes[did]

    # ── detour candidates ────────────────────────────────────────────────

    def _detour_candidates(
        self, drone: Drone
    ) -> List[Tuple[List[float], Tuple[float, float, float]]]:
        """Detour options, best first — fully deterministic.

        Scoring: closest resulting distance to the drone's pursuit point
        (goal bias), a bonus for climbing and a penalty for descending
        (downwash: escape *over*, never dive under), and a penalty for
        reversing the recently chosen detour direction so consecutive
        detours commit to one way around (memory / hysteresis).
        """
        head = self._route_head(drone)
        step = self.step_size
        bias = self._detour_bias.get(drone.drone_id)
        bias_active = self._detour_bias_ttl.get(drone.drone_id, 0) > 0

        scored: List[tuple] = []
        for index, (ox, oy, oz) in enumerate(_DETOUR_OFFSETS):
            mag = (ox * ox + oy * oy + oz * oz) ** 0.5
            ux, uy, uz = ox / mag, oy / mag, oz / mag
            candidate = [
                drone.position[0] + ux * step,
                drone.position[1] + uy * step,
                max(self.min_z, drone.position[2] + uz * step),
            ]
            score = Drone.distance(candidate, head)
            if oz > 0:
                score -= 0.25 * step
            elif oz < 0:
                score += 0.5 * step
            if bias_active and bias is not None:
                alignment = ux * bias[0] + uy * bias[1] + uz * bias[2]
                if alignment < 0.0:
                    score += 0.5 * step
                else:
                    score -= 0.15 * step * alignment
            scored.append((score, index, candidate, (ux, uy, uz)))

        scored.sort(key=lambda item: (item[0], item[1]))
        return [(candidate, direction) for _, _, candidate, direction in scored]

    # ── dispatch gate ────────────────────────────────────────────────────

    def _update_dispatch(self) -> None:
        """Release waiting drones for this step (see module docstring).

        Scans in priority order (highest destination altitude first). A
        candidate is released when the mover cap allows it and every active
        mover is at least :data:`RELEASE_DISTANCE` away; candidates failing
        the distance rule are skipped, so lower-priority but independent
        (far-away) drones may still be released this step.
        """
        movers: List[Drone] = [
            d
            for d in self.drones
            if d.drone_id in self._released and not d.arrived
        ]

        # Unblock target squatters: a waiting drone parked on a mover's
        # target can never be pushed away by the mover itself, so it gets
        # released out of turn. The cap is absolute — when it is full, the
        # blocked mover is parked back to waiting to make room; it will be
        # re-released by the normal scan once its blocker has pulled away.
        for mover in list(movers):
            if self._consecutive_holds[mover.drone_id] < DEADLOCK_THRESHOLD:
                continue
            blocker: Optional[Drone] = None
            for did in self._dispatch_order:
                if did in self._released:
                    continue
                candidate = self._drones_by_id[did]
                if self._overlap(candidate.position, list(mover.target)):
                    blocker = candidate
                    break
            if blocker is None:
                continue
            if len(movers) >= MAX_CONCURRENT_MOVERS:
                self._released.discard(mover.drone_id)
                self._consecutive_holds[mover.drone_id] = 0
                movers.remove(mover)
            self._released.add(blocker.drone_id)
            movers.append(blocker)

        def horizontal_distance(a: List[float], b: List[float]) -> float:
            return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

        # The release-distance rule guards against departing near an
        # actively *translating* drone; a mover that has been holding for a
        # while is effectively hovering, so it must not block its
        # neighbours' releases (which are often exactly what would unstick
        # it). Stuck movers still occupy a slot under the hard cap.
        translating = [
            m
            for m in movers
            if self._consecutive_holds.get(m.drone_id, 0) < HOVERING_HOLDS
        ]

        if len(movers) >= MAX_CONCURRENT_MOVERS:
            return

        # Release order: altitude first (downwash rule), then drones whose
        # straight line to the target is least obstructed by parked drones.
        # Releasing clear-path drones first turns same-lane formations into
        # convoys (leader first, followers staggered behind) instead of
        # making back drones climb over the still-parked drones ahead —
        # this is what keeps flown paths close to the straight-line length.
        waiting = [did for did in self._dispatch_order if did not in self._released]
        if not waiting:
            return
        parked_positions = [
            (d.drone_id, list(d.position))
            for d in self.drones
            if d.arrived or d.drone_id not in self._released
        ]

        def blocked_count(did: int) -> int:
            drone = self._drones_by_id[did]
            goal = list(drone.target)
            count = 0
            for obs_id, obs in parked_positions:
                if obs_id == did:
                    continue
                if envelope_overlap_swept(
                    drone.position,
                    goal,
                    obs,
                    obs,
                    margin=self.margin,
                    b_extends_below=DOWNWASH_ROUTE_CLEARANCE,
                ):
                    count += 1
            return count

        def goal_squatted(did: int) -> int:
            """1 when a parked drone sits on this drone's target.

            Such a drone would only fly across and then hover-wait at the
            far end; releasing drones with free goals first walks squat
            chains (formation permutations) in topological order instead.
            """
            goal = list(self._drones_by_id[did].target)
            for obs_id, obs in parked_positions:
                if obs_id != did and envelope_overlap(
                    obs, goal, margin=self.margin
                ):
                    return 1
            return 0

        waiting.sort(
            key=lambda did: (
                -self._drones_by_id[did].target[2],
                goal_squatted(did),
                blocked_count(did),
                did,
            )
        )

        for did in waiting:
            if len(movers) >= MAX_CONCURRENT_MOVERS:
                break
            candidate = self._drones_by_id[did]
            if all(
                horizontal_distance(candidate.position, m.position)
                >= RELEASE_DISTANCE
                for m in translating
            ):
                self._released.add(did)
                movers.append(candidate)
                translating.append(candidate)

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

            # Phase 0 — dispatch gate: release waiting drones for this step
            self._update_dispatch()

            prev_positions: Dict[int, List[float]] = {
                d.drone_id: list(d.position) for d in self.drones
            }
            # Waiting (not yet released) drones behave like arrived ones for
            # this step: they hold in place, are never chosen to yield (their
            # hold *is* the safe state) and never attempt detours.
            arrived_ids: Set[int] = {
                d.drone_id
                for d in self.drones
                if d.arrived or d.drone_id not in self._released
            }

            # Phase 1 — propose moves (released drones only): each drone
            # follows its planned route around the currently parked drones,
            # planning or re-planning it lazily.
            self._escaped_this_step = set()
            statics = [
                list(d.position) for d in self.drones if d.drone_id in arrived_ids
            ]
            proposed: Dict[int, List[float]] = {}
            for d in self.drones:
                if d.drone_id in arrived_ids:
                    proposed[d.drone_id] = list(d.position)
                else:
                    proposed[d.drone_id] = self._propose_move(d, statics)

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

            # Phase 2.7 — deadlocked drone detour attempts (dynamic
            # conflicts between concurrent movers; static blockages are
            # already routed around by the planner)
            for did in list(reverted):
                drone = self._drones_by_id[did]
                if drone.arrived or did in self._escaped_this_step:
                    continue
                # While the drone's goal is still occupied by another drone
                # a detour achieves nothing — hold (length-free) so the
                # dispatch gate's boost chain can move the squatter away.
                # Bounded by SQUAT_WAIT_LIMIT: mutual squats need a detour.
                goal = list(drone.target)
                if self._consecutive_holds[did] < SQUAT_WAIT_LIMIT and any(
                    other.drone_id != did
                    and envelope_overlap(
                        other.position, goal, margin=self.margin
                    )
                    for other in self.drones
                ):
                    continue
                if self._consecutive_holds[did] >= DEADLOCK_THRESHOLD:
                    for cand_pos, direction in self._detour_candidates(drone):
                        test_proposed = dict(proposed)
                        test_proposed[did] = cand_pos
                        if not self._find_step_collisions(
                            prev_positions, test_proposed
                        ):
                            proposed[did] = cand_pos
                            reverted.remove(did)
                            # Commit to this way around for a few steps and
                            # re-plan the route from the detoured position.
                            self._detour_bias[did] = direction
                            self._detour_bias_ttl[did] = DETOUR_BIAS_STEPS
                            self._routes.pop(did, None)
                            self._futile_detours[did] += 1
                            break

            # Phase 3 — apply moves
            for d in self.drones:
                d.apply_move(proposed[d.drone_id])
            self._advance_routes()

            # Hold accounting: a yield is *productive* when every collision
            # partner of the yielding drone actually moved this step — the
            # conflict is clearing by itself and waiting costs no path
            # length. Only unproductive holds count toward detours/escapes.
            moved_now = {
                did
                for did, position in proposed.items()
                if position != prev_positions[did]
            }
            partners: Dict[int, Set[int]] = {}
            for a_id, b_id in collisions:
                partners.setdefault(a_id, set()).add(b_id)
                partners.setdefault(b_id, set()).add(a_id)

            for d in self.drones:
                did = d.drone_id
                if did in reverted:
                    blocking = partners.get(did, set())
                    forgiven = (
                        blocking
                        and blocking <= moved_now
                        and self._steps_since_progress[did] < PRODUCTIVE_PATIENCE
                    )
                    if not forgiven:
                        self._consecutive_holds[did] += 1
                else:
                    self._consecutive_holds[did] = 0
                ttl = self._detour_bias_ttl.get(did, 0)
                if ttl > 0:
                    self._detour_bias_ttl[did] = ttl - 1
                if did in self._released and not d.arrived:
                    self._steps_since_progress[did] += 1
                remaining = d.remaining_distance()
                if remaining < self._best_remaining[did] - 1e-6:
                    self._best_remaining[did] = remaining
                    self._futile_detours[did] = 0
                    self._steps_since_progress[did] = 0

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
