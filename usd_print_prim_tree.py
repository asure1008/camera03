#!/usr/bin/env python3
"""
打印 USD Stage 的 Prim 路径树（pxr.Usd），便于手工确认吊篮 / 人物 / 相机路径。

用法：
  CONDA_PREFIX=... python3 usd_print_prim_tree.py /path/to/scene.usd
  python3 usd_print_prim_tree.py /path/to/scene.usd --max-depth 4 --skip-regex '.*/Looks/.*'

与 generate_scene_usd.py 共用同一套 pxr 导入逻辑（见下方）。
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys
from typing import Optional


def _conda_lib_dirs():
    out = []
    for base in (os.environ.get("CONDA_PREFIX"), os.path.expanduser("~/miniconda3/envs/env_isaaclab")):
        if base:
            lib = os.path.join(base, "lib")
            if os.path.isdir(lib):
                out.append(lib)
    return out


def _discover_omni_usd_libs():
    roots = [
        os.path.expanduser("~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache"),
        os.path.expanduser("~/projects/issac/.isaac_sim_unzip/extscache"),
    ]
    found = []
    for root in roots:
        found.extend(p for p in glob.glob(os.path.join(root, "omni.usd.libs-*")) if os.path.isdir(p))
    return sorted(set(found))


def setup_px_import() -> None:
    try:
        from pxr import Usd  # noqa: F401

        return
    except ImportError:
        pass
    pxr_dir = os.environ.get("ISAAC_PXR_DIR")
    candidates = []
    if pxr_dir:
        candidates.append(pxr_dir)
    candidates.extend(_discover_omni_usd_libs())
    conda_libs = _conda_lib_dirs()
    for d in candidates:
        if not d or not os.path.isdir(d):
            continue
        sys.path.insert(0, d)
        px_bin = os.path.join(d, "bin")
        os.environ["LD_LIBRARY_PATH"] = ":".join(
            x for x in [px_bin] + conda_libs + [os.environ.get("LD_LIBRARY_PATH", "")] if x
        )
        try:
            from pxr import Usd  # noqa: F401

            return
        except ImportError:
            sys.path.remove(d)
            continue
    raise SystemExit(
        "无法导入 pxr。请设置 ISAAC_PXR_DIR 或使用 env_isaaclab + isaacsim（omni.usd.libs）。"
    )


setup_px_import()
from pxr import Usd, UsdGeom  # noqa: E402


def print_tree(
    stage: Usd.Stage,
    max_depth: int,
    skip_regex: Optional[re.Pattern],
    show_camera: bool,
    paths_only: bool,
) -> None:
    root = stage.GetPseudoRoot()
    up = UsdGeom.GetStageUpAxis(stage)

    def walk(prim: Usd.Prim, depth: int, prefix: str) -> None:
        if depth > max_depth:
            return
        p = str(prim.GetPath())
        if skip_regex and skip_regex.search(p):
            return
        is_cam = prim.IsA(UsdGeom.Camera)
        flag = " [Camera]" if is_cam else ""
        if paths_only:
            if not show_camera or is_cam:
                print(p)
        else:
            print(f"{prefix}{prim.GetName()}  ({prim.GetTypeName()}){flag}")
        for ch in prim.GetChildren():
            walk(ch, depth + 1, prefix + "  ")

    if paths_only:
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if skip_regex and skip_regex.search(p):
                continue
            if show_camera and not prim.IsA(UsdGeom.Camera):
                continue
            print(p)
        return

    print(f"# upAxis={up}  defaultPrim={stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim().IsValid() else 'N/A'}")
    for ch in root.GetChildren():
        walk(ch, 0, "")


def main() -> int:
    ap = argparse.ArgumentParser(description="打印 USD Prim 树或扁平路径列表")
    ap.add_argument("usd_path", help="USD/USDA/USDC 文件路径")
    ap.add_argument("--max-depth", type=int, default=20, help="树深度上限")
    ap.add_argument("--skip-regex", default="", help="跳过的路径（Python regex），如 '.*/Looks/.*'")
    ap.add_argument("--paths-only", action="store_true", help="只输出每行一个全路径（扁平）")
    ap.add_argument("--camera-only", action="store_true", help="仅输出 UsdGeom.Camera 路径（需配合 --paths-only）")
    args = ap.parse_args()

    skip_re = re.compile(args.skip_regex) if args.skip_regex else None

    stage = Usd.Stage.Open(os.path.abspath(args.usd_path))
    if not stage:
        print("无法打开:", args.usd_path, file=sys.stderr)
        return 1

    print_tree(
        stage,
        max_depth=args.max_depth,
        skip_regex=skip_re,
        show_camera=args.camera_only,
        paths_only=args.paths_only,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
