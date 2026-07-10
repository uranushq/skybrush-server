"""Standalone test: run solver + converter, verify .skyb files are created."""
import asyncio
import os
import sys
import tempfile
from pathlib import Path

# Add the source tree to sys.path so we can import the extension modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from flockwave.server.ext.path_planner.solver import PathSolver
from flockwave.server.ext.path_planner.converter import (
    build_show_dicts,
    save_skyb_files,
    solver_result_to_trajectory_dicts,
)
from flockwave.server.ext.path_planner.verify import verify_show_dicts


async def main():
    # 3 drones, simple scenario (kept above the 2.5 m altitude floor)
    initials = [(-5, 0, 5), (0, 0, 5), (5, 0, 5)]
    targets = [(5, 5, 10), (-5, -5, 10), (0, 10, 5)]

    solver = PathSolver(
        initials=initials, targets=targets, step_size=1.0, seed=42, min_z=2.5
    )
    result = solver.solve()

    print(f"Solver: success={result.success}, total_steps={result.total_steps}")
    if not result.success:
        print(f"Failure reason: {result.failure_reason}")
        return
    print(f"Drones: {len(result.drones)}, Steps recorded: {len(result.steps)}")

    ground = [[p[0], p[1], 0.0] for p in initials]

    # 1) Test trajectory dict generation
    traj_dicts = solver_result_to_trajectory_dicts(
        result, duration_ms=5000, ground_positions=ground
    )
    print(f"\nTrajectory dicts generated: {len(traj_dicts)}")
    for i, td in enumerate(traj_dicts):
        pts = td["points"]
        print(
            f"  drone-{i+1}: {len(pts)} keyframes, "
            f"start={pts[0][1]}, end={pts[-1][1]}, "
            f"t_end={pts[-1][0]}s"
        )

    # 2) Test show dict generation + verification gate
    show_dicts = build_show_dicts(
        result, duration_ms=5000, ground_positions=ground
    )
    print(f"\nShow dicts generated: {len(show_dicts)}")
    for i, sd in enumerate(show_dicts):
        print(
            f"  drone-{i+1}: home={sd['home']}, "
            f"has_trajectory={bool(sd['trajectory'])}, "
            f"geofence_max_dist={sd['geofence']['maxDistance']}"
        )
    violations = verify_show_dicts(show_dicts)
    print(f"Verification violations: {len(violations)}")
    if violations:
        print(violations[:3])
        return

    # 3) Test .skyb file generation into a temp directory
    output_dir = tempfile.mkdtemp(prefix="path-planner-test-")
    saved = await save_skyb_files(show_dicts, output_dir=output_dir)

    print("\nSaved files:")
    for key, path in saved.items():
        if os.path.isfile(path):
            size = os.path.getsize(path)
            print(f"  {key}: {path} ({size} bytes)")
        else:
            print(f"  {key}: {path}")

    # 4) Validate .skyb files can be parsed back
    from flockwave.server.show.formats import (
        SkybrushBinaryFormatBlockType,
        SkybrushBinaryShowFile,
    )

    for i in range(len(result.drones)):
        drone_id = f"drone-{i+1}"
        skyb_path = saved[drone_id]
        data = open(skyb_path, "rb").read()

        async with SkybrushBinaryShowFile.from_bytes(data) as f:
            blocks = await f.read_all_blocks()
            block_types = []
            for b in blocks:
                block_types.append(SkybrushBinaryFormatBlockType(b.type).name)
                await b.read()
            print(f"\n  {drone_id}.skyb: {len(data)} bytes, blocks={block_types}")

    print("\n✅ All tests passed!")


if __name__ == "__main__":
    asyncio.run(main())
