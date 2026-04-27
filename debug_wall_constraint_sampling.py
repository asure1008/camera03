#!/usr/bin/env python3
import argparse
import random

from isaacsim import SimulationApp

sim_app = SimulationApp({'headless': True})

from pxr import Usd
from diaolan_randomizer import _compute_world_bbox_info, _shrink_world_bbox_with_margin, sample_camera_in_changjing


def main():
    parser = argparse.ArgumentParser(description="Debug wall-constrained camera sampling")
    parser.add_argument("--scene", required=True)
    parser.add_argument("--camera-rig", default="/World/CameraRig")
    parser.add_argument("--changjing-path", default="/World/Changjing_AlgorithmVerification_2026_03")
    parser.add_argument("--wall-prim", required=True)
    parser.add_argument("--xy-margin", type=float, default=0.005)
    parser.add_argument("--z-margin", type=float, default=0.05)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--seed", type=int, default=20260407)
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.scene)
    if stage is None:
        raise RuntimeError(f"failed to open stage: {args.scene}")

    raw_bbox = _compute_world_bbox_info(stage, args.wall_prim)
    effective_bbox = _shrink_world_bbox_with_margin(raw_bbox, args.xy_margin, args.z_margin)
    print("[debug-wall] raw_bbox=" + str(raw_bbox), flush=True)
    print("[debug-wall] effective_bbox=" + str(effective_bbox), flush=True)

    rng = random.Random(args.seed)
    for idx in range(args.count):
        x, y, z, _, meta = sample_camera_in_changjing(
            stage,
            args.changjing_path,
            args.camera_rig,
            rng,
            seed=args.seed + idx,
            wall_prim_path=args.wall_prim,
            wall_constraint_xy_margin=args.xy_margin,
            wall_constraint_z_margin=args.z_margin,
        )
        eff = meta['effective_box']
        inside = eff['x_min'] <= x <= eff['x_max'] and eff['y_min'] <= y <= eff['y_max'] and eff['z_min'] <= z <= eff['z_max']
        print(
            f"[debug-wall-sample] idx={idx + 1} xyz=({x:.4f},{y:.4f},{z:.4f}) "
            f"inside_effective_bbox={inside} mode={meta['mode']} source={meta['constraint_source']}",
            flush=True,
        )


if __name__ == "__main__":
    try:
        main()
    finally:
        sim_app.close()
