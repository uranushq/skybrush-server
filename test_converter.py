"""Standalone test: run solver + converter, verify .skyb files are created."""
import asyncio
import sys
import os
import tempfile

# Add the source tree to sys.path so we can import the extension modules
sys.path.insert(0, r"C:\Users\bjw02\OneDrive\Desktop\DCS\skybrush-server\src")

from flockwave.server.ext.path_planner.solver import PathSolver
from flockwave.server.ext.path_planner.converter import (
    solver_result_to_trajectory_dicts,
    build_show_dicts,
    save_skyb_files,
)


async def main():
    # 3 drones, simple scenario
    initials = [(-5, 0, 0), (0, 0, 0), (5, 0, 0)]
    targets  = [(5, 5, 10), (-5, -5, 10), (0, 10, 5)]

    solver = PathSolver(initials=initials, targets=targets, step_size=1.0, seed=42)
    result = solver.solve()

    print(f"Solver: success={result.success}, total_steps={result.total_steps}")
    print(f"Drones: {len(result.drones)}, Steps recorded: {len(result.steps)}")

    # 1) Test trajectory dict generation
    traj_dicts = solver_result_to_trajectory_dicts(result, duration_ms=300)
    print(f"\nTrajectory dicts generated: {len(traj_dicts)}")
    for i, td in enumerate(traj_dicts):
        pts = td["points"]
        print(f"  drone-{i+1}: {len(pts)} keyframes, "
              f"start={pts[0][1]}, end={pts[-1][1]}, "
              f"t_end={pts[-1][0]}s")

    # 2) Test show dict generation
    show_dicts = build_show_dicts(result, duration_ms=300)
    print(f"\nShow dicts generated: {len(show_dicts)}")
    for i, sd in enumerate(show_dicts):
        print(f"  drone-{i+1}: home={sd['home']}, "
              f"has_trajectory={bool(sd['trajectory'])}, "
              f"has_lights={bool(sd['lights'])}")

    # 3) Test .skyb file generation — save to parent directory (DCS/)
    output_dir = r"C:\Users\bjw02\OneDrive\Desktop\DCS"
    saved = await save_skyb_files(result, output_dir=output_dir, duration_ms=300)

    print(f"\nSaved files:")
    for key, path in saved.items():
        if os.path.isfile(path):
            size = os.path.getsize(path)
            print(f"  {key}: {path} ({size} bytes)")
        else:
            print(f"  {key}: {path}")

    # 4) Validate .skyb files can be parsed back
    from flockwave.server.show.formats import SkybrushBinaryShowFile, SkybrushBinaryFormatBlockType

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
            print(f"\n  {drone_id}.skyb: {len(data)} bytes, "
                  f"blocks={block_types}")

    print("\n✅ All tests passed!")


if __name__ == "__main__":
    asyncio.run(main())
