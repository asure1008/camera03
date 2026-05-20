#!/usr/bin/env python3
"""30x camera-only randomize: aggregate wall pool + mount + camera xyz (HTTP against ptz_stream)."""
from __future__ import annotations

import json
import math
import sys
import urllib.request
from collections import Counter

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8081"
N = int(sys.argv[2]) if len(sys.argv) > 2 else 30


def get_json(path: str, data: dict | None = None) -> dict:
    url = BASE.rstrip("/") + path
    if data is None:
        req = urllib.request.Request(url, method="GET")
    else:
        body = json.dumps(data).encode("utf-8")
        req = urllib.request.Request(
            url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
    with urllib.request.urlopen(req, timeout=180) as resp:
        return json.loads(resp.read().decode("utf-8"))


def main() -> None:
    st = get_json("/scene/state")
    # /scene/state 返回 _scene_state_snapshot() 扁平 JSON（无 ok/state 包裹）
    wall_cfg = st.get("wall_sampling_config") or {}
    print("=== wall_sampling_config (runtime) ===")
    print(json.dumps(wall_cfg, ensure_ascii=False, indent=2))

    mounts: list[str] = []
    cand_counts: list[int] = []
    roots: list[str] = []
    seeds: list[bool] = []
    xyzs: list[tuple[float, float, float]] = []
    group_hints: list[str] = []
    notes_samples: list[str] = []
    constraint_modes: list[str] = []
    within_constraints: list[bool] = []
    inset_valids: list[bool] = []
    inset_distances: list[float] = []
    mounted_on_wall_values: list[bool] = []

    payload = {
        "random_config": {
            "random_camera": True,
            "random_gondola": False,
            "random_workers": False,
            "random_hdri": False,
        }
    }

    for i in range(N):
        r = get_json("/scene/randomize", payload)
        if not r.get("ok"):
            print("FAIL randomize", i, r, file=sys.stderr)
            sys.exit(1)
        res = r.get("result") or {}
        cam = res.get("camera_xyz") or []
        if len(cam) >= 3:
            xyzs.append((float(cam[0]), float(cam[1]), float(cam[2])))
        meta = res.get("camera_meta") or {}
        wh = meta.get("wall_height_constraint") or {}
        m = str(wh.get("selected_mount_prim") or "")
        mounts.append(m)
        cand_counts.append(int(wh.get("wall_candidate_pool_size") or 0))
        roots.append(str(wh.get("wall_collection_root") or ""))
        seeds.append(bool(wh.get("seed_filter_applied")))
        gh = str(wh.get("selected_mount_group_hint") or "")
        group_hints.append(gh)
        constraint_modes.append(str(wh.get("constraint_mode") or ""))
        within_constraints.append(bool(wh.get("within_wall_constraint_box")))
        inset_valids.append(bool(wh.get("inset_wall_mount_valid")))
        mounted_on_wall_values.append(bool(wh.get("mounted_on_wall")))
        try:
            d = wh.get("inset_actual_distance_to_wall_surface")
            if d is not None:
                inset_distances.append(float(d))
        except (TypeError, ValueError):
            pass
        ab = meta.get("aabb_clip_notes")
        if isinstance(ab, list) and ab:
            notes_samples.append(";".join(str(x) for x in ab[:6]))

    print(f"\n=== {N} runs summary ===")
    if cand_counts:
        print("candidate_count: min", min(cand_counts), "max", max(cand_counts), "mean", round(sum(cand_counts) / len(cand_counts), 2))
    print("collection_root unique:", sorted(set(roots)))
    print("seed_filter_applied unique:", sorted(set(seeds)))
    mc = Counter(mounts)
    print("unique selected_mount_prim:", len(mc))
    print("top mounts (count, path):")
    for path, c in mc.most_common(12):
        print(f"  {c}\t{path}")
    gc = Counter(group_hints)
    print("selected_mount_group_hint distribution:", dict(gc))
    print("constraint_mode distribution:", dict(Counter(constraint_modes)))
    print("within_wall_constraint_box:", dict(Counter(within_constraints)))
    print("inset_wall_mount_valid:", dict(Counter(inset_valids)))
    print("mounted_on_wall:", dict(Counter(mounted_on_wall_values)))
    if inset_distances:
        print(
            "inset_actual_distance_to_wall_surface: min",
            round(min(inset_distances), 3),
            "max",
            round(max(inset_distances), 3),
            "mean",
            round(sum(inset_distances) / len(inset_distances), 3),
        )

    if xyzs:
        mx = sum(x[0] for x in xyzs) / len(xyzs)
        my = sum(x[1] for x in xyzs) / len(xyzs)
        mz = sum(x[2] for x in xyzs) / len(xyzs)
        sx = math.sqrt(sum((x[0] - mx) ** 2 for x in xyzs) / len(xyzs))
        sy = math.sqrt(sum((x[1] - my) ** 2 for x in xyzs) / len(xyzs))
        sz = math.sqrt(sum((x[2] - mz) ** 2 for x in xyzs) / len(xyzs))
        xr = max(x[0] for x in xyzs) - min(x[0] for x in xyzs)
        yr = max(x[1] for x in xyzs) - min(x[1] for x in xyzs)
        zr = max(x[2] for x in xyzs) - min(x[2] for x in xyzs)
        print("camera_xyz mean:", round(mx, 3), round(my, 3), round(mz, 3))
        print("camera_xyz std:", round(sx, 3), round(sy, 3), round(sz, 3))
        print("camera_xyz range (max-min):", round(xr, 3), round(yr, 3), round(zr, 3))

    if notes_samples:
        print("\nSample aabb_clip_notes (first lines of a few runs):")
        for line in notes_samples[:5]:
            print(" ", line[:200])


if __name__ == "__main__":
    main()
