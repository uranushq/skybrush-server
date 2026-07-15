"""Path-efficiency benchmark: actual flown distance vs straight-line A->B.

Key metric (user-defined): for every drone and every solver segment,
  overhead % = (flown_length / straight_line_distance - 1) * 100
where straight-line ignores collisions entirely.
"""

import sys
import types

quart = types.ModuleType("quart")


class _BP:
    def __init__(self, *a, **k): ...

    def route(self, *a, **k):
        def deco(fn):
            return fn

        return deco


quart.Blueprint = _BP
quart.Response = object
quart.jsonify = lambda *a, **k: a[0] if a else k
quart.request = None
sys.modules["quart"] = quart
trio = types.ModuleType("trio")


async def _sf(): ...


class _TT:
    @staticmethod
    async def run_sync(fn, *a):
        return fn(*a)


trio.sleep_forever = _sf
trio.to_thread = _TT()
sys.modules["trio"] = trio
eb = types.ModuleType("flockwave.server.ext.base")
eb.Extension = type("Extension", (), {})
sys.modules["flockwave.server.ext.base"] = eb
ut = types.ModuleType("flockwave.server.utils")
ut.overridden = lambda *a, **k: None
sys.modules["flockwave.server.utils"] = ut
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from flockwave.server.ext.path_planner.solver import PathSolver  # noqa: E402


def dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


def analyze(name, initials, targets, min_z=0.0):
    solver = PathSolver(list(initials), list(targets), min_z=min_z)
    result = solver.solve()
    if not result.success:
        print(f"[{name}] FAILED: {result.failure_reason}")
        return None

    rows = []
    total_flown = total_straight = 0.0
    for did in range(len(initials)):
        straight = dist(initials[did], targets[did])
        flown = 0.0
        prev = result.steps[0].positions[did]
        for rec in result.steps[1:]:
            cur = rec.positions[did]
            flown += dist(prev, cur)
            prev = cur
        total_flown += flown
        total_straight += straight
        if straight > 1e-9:
            rows.append((did, straight, flown, (flown / straight - 1.0) * 100.0))
        elif flown > 1e-9:
            rows.append((did, 0.0, flown, float("inf")))

    overheads = [r[3] for r in rows if r[3] != float("inf")]
    fleet = (total_flown / total_straight - 1.0) * 100.0 if total_straight else 0.0
    print(
        f"[{name}] drones moving: {len(rows)}, steps: {result.total_steps}\n"
        f"  fleet overhead: {fleet:+.1f}%  "
        f"(flown {total_flown:.1f} m vs straight {total_straight:.1f} m)\n"
        f"  per-drone overhead: mean {sum(overheads)/len(overheads):+.1f}%  "
        f"median {sorted(overheads)[len(overheads)//2]:+.1f}%  "
        f"max {max(overheads):+.1f}%"
    )
    worst = sorted(rows, key=lambda r: -r[3])[:5]
    for did, straight, flown, pct in worst:
        print(
            f"    worst drone {did}: straight {straight:.1f} m -> flown "
            f"{flown:.1f} m ({pct:+.1f}%)"
        )
    return fleet


print("=" * 70)

# 1. Free space: 25 drones, two well-separated formations (no interference
#    beyond crossing paths). Ideal planner = ~0%.
initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
targets = [(2.5 * (i % 5) + 30.0, 2.5 * (i // 5), 10.0) for i in range(25)]
analyze("1. formation shift +30m x (no obstacles ahead)", initials, targets)

# 2. Formation contraction: box 2.5m -> tighter 2m grid shifted, paths cross.
initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
targets = [(2.0 * (i % 5) + 15.0, 2.0 * (i // 5) + 2.0, 10.0) for i in range(25)]
analyze("2. formation shift+reshape (crossing paths)", initials, targets)

# 3. Formation rotation 180deg around center (every path crosses center).
import math  # noqa: E402

cx, cy = 5.0, 5.0
initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
targets = [
    (2.0 * cx - p[0], 2.0 * cy - p[1], 10.0) for p in initials
]
analyze("3. 180-degree rotation about center", initials, targets)

# 4. Altitude reshuffle: flat grid -> two stacked layers (z changes, downwash
#    columns everywhere).
initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
targets = [
    (2.5 * (i % 5), 2.5 * (i // 5), 8.0 if i % 2 else 12.0) for i in range(25)
]
analyze("4. flat grid -> alternating two layers", initials, targets)

# 5. The smoke scenario's stack segment (box -> ring + 3-stack).
initials = [(2.5 * (i % 5), 2.5 * (i // 5), 10.0) for i in range(25)]
targets = []
for i in range(25):
    if i == 0:
        targets.append((-6.0, 0.0, 12.0))
    elif i == 1:
        targets.append((-6.0, 0.0, 6.0))   # approach point (10-4)
    elif i == 2:
        targets.append((-6.0, 0.0, 4.0))   # approach point (8-4)
    else:
        targets.append((3.0 * (i % 5) + 6.0, 3.0 * (i // 5), 8.0))
analyze("5. smoke 'stack' approach segment", initials, targets, min_z=2.0)

# 6. Single crossing of a parked grid (the earlier repro).
parked = [(2.0 * (i % 5), 2.0 * (i // 5), 10.0) for i in range(25)]
analyze(
    "6. single drone crosses parked grid",
    parked + [(-4.0, 4.0, 10.0)],
    parked + [(12.0, 4.0, 10.0)],
)
