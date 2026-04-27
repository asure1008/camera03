#!/usr/bin/env python3
"""
Phase 2.5A: 真实 stage 遍历——发现吊篮 prim 路径
=================================================
不依赖任何硬编码路径，通过 pxr/Usd 对 scene_4diaolan_ptz.usda
做全量 PrimRange 遍历，输出：

  1. 所有名字含 Diaolan/diaolan/Gondola/gondola 的 prim 完整路径
  2. 每个候选的父路径 + 类型 + 直接子节点名
  3. 每个吊篮下是否存在 Model / Group8761 / Group1 / workers 特征
  4. 推荐的 target prim
"""

import json
import os
import sys
from pathlib import Path

# ── 优先用 Isaac Sim 环境内的 pxr；也可在 standalone pxr python 中运行 ──
try:
    from pxr import Usd, UsdGeom, Sdf
except ImportError:
    print("[ERROR] 无法 import pxr. 请在 Isaac Sim python.sh 或安装 usd-core 的 venv 中运行.")
    sys.exit(1)

SCENE_PATH = str(Path(__file__).parent / "scene_4diaolan_ptz.usda")
KEYWORDS   = ("diaolan", "gondola")   # 大小写不敏感

# ─────────────────────────────────────────────────────────────────────────────
# 工具
# ─────────────────────────────────────────────────────────────────────────────

def _name_match(name: str) -> bool:
    low = name.lower()
    return any(kw in low for kw in KEYWORDS)


def _child_names(prim) -> list[str]:
    return [c.GetName() for c in prim.GetChildren()]


def _has_desc_of_type(prim, type_class) -> bool:
    for desc in Usd.PrimRange(prim):
        if desc.IsA(type_class):
            return True
    return False


def _count_meshes(prim) -> int:
    return sum(1 for desc in Usd.PrimRange(prim) if desc.IsA(UsdGeom.Mesh))


def _feature_flags(prim) -> dict:
    child_names_set = set(c.GetName() for c in prim.GetChildren())
    flags = {
        "has_Model":    "Model"    in child_names_set,
        "has_Group8761": "Group8761" in child_names_set,
        "has_Group1":   "Group1"   in child_names_set,
        "has_workers":  any("worker" in n.lower() or "people" in n.lower() or "person" in n.lower()
                           for n in child_names_set),
    }
    # 子孙中查特征
    all_desc_names = set(d.GetName() for d in Usd.PrimRange(prim))
    flags["desc_has_Group8761"] = "Group8761" in all_desc_names
    flags["desc_has_Group1"]    = "Group1"    in all_desc_names
    flags["desc_has_workers"]   = any(
        "worker" in n.lower() or "people" in n.lower() or "person" in n.lower()
        for n in all_desc_names
    )
    flags["mesh_count"] = _count_meshes(prim)
    return flags


def _suggest_target(prim) -> str | None:
    """
    按优先级在 prim 子孙中找 target prim:
      1. Model/Group8761
      2. Model/Group1
      3. Model/*  (第一个有 mesh 的直接子)
      4. prim 自身（兜底）
    """
    stage = prim.GetStage()
    path  = prim.GetPath().pathString

    # 1. Model/Group8761
    p = stage.GetPrimAtPath(f"{path}/Model/Group8761")
    if p.IsValid() and _count_meshes(p) > 0:
        return p.GetPath().pathString

    # 2. Model/Group1
    p = stage.GetPrimAtPath(f"{path}/Model/Group1")
    if p.IsValid() and _count_meshes(p) > 0:
        return p.GetPath().pathString

    # 3. 任意含 mesh 的 Model 直接子
    model = stage.GetPrimAtPath(f"{path}/Model")
    if model.IsValid():
        for child in model.GetChildren():
            if _count_meshes(child) > 0:
                return child.GetPath().pathString

    # 4. 兜底
    return path


# ─────────────────────────────────────────────────────────────────────────────
# 主遍历
# ─────────────────────────────────────────────────────────────────────────────

def discover(scene_path: str) -> dict:
    print(f"[discover] Opening stage: {scene_path}")
    stage = Usd.Stage.Open(scene_path)
    if not stage:
        raise RuntimeError(f"无法打开 stage: {scene_path}")

    print("[discover] Traversing all prims …")

    # 候选：名字命中关键字的 prim
    candidates_raw: list[dict] = []

    for prim in stage.Traverse():
        name = prim.GetName()
        if not _name_match(name):
            continue
        path     = prim.GetPath().pathString
        parent   = prim.GetParent()
        par_path = parent.GetPath().pathString if parent and parent.IsValid() else ""
        type_name = prim.GetTypeName() or "unknown"
        children  = _child_names(prim)
        flags     = _feature_flags(prim)

        candidates_raw.append({
            "path":        path,
            "name":        name,
            "type":        type_name,
            "parent_path": par_path,
            "children":    children,
            "features":    flags,
        })

    print(f"[discover] Total keyword-matched prims: {len(candidates_raw)}")

    # ── 过滤：只保留"顶层吊篮容器"──────────────────────────────────────────
    # 策略：若 prim A 是 prim B 的祖先，则保留 A（最靠上的节点）
    all_paths = [c["path"] for c in candidates_raw]

    def is_top_level(path: str) -> bool:
        for other in all_paths:
            if other != path and path.startswith(other + "/"):
                return False
        return True

    top_level = [c for c in candidates_raw if is_top_level(c["path"])]

    # ── 对每个顶层候选补充 target prim 推断 ────────────────────────────────
    results: list[dict] = []
    for c in top_level:
        prim   = stage.GetPrimAtPath(c["path"])
        target = _suggest_target(prim) if prim.IsValid() else c["path"]
        c["suggested_target_prim"] = target
        # 验证 target 是否有 mesh
        t_prim = stage.GetPrimAtPath(target)
        c["target_mesh_count"] = _count_meshes(t_prim) if t_prim.IsValid() else 0
        results.append(c)

    return {
        "scene_path":           scene_path,
        "all_keyword_matches":  candidates_raw,
        "top_level_diaolans":   results,
        "total_top_level":      len(results),
    }


# ─────────────────────────────────────────────────────────────────────────────
# 入口
# ─────────────────────────────────────────────────────────────────────────────

def main():
    report = discover(SCENE_PATH)

    out_path = Path(SCENE_PATH).parent / "_discover_diaolan_report.json"
    out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n[discover] Report saved → {out_path}")

    # 打印摘要
    print("\n" + "=" * 72)
    print(f"  顶层吊篮数量: {report['total_top_level']}")
    print("=" * 72)
    for d in report["top_level_diaolans"]:
        print(f"\n  PATH         : {d['path']}")
        print(f"  TYPE         : {d['type']}")
        print(f"  PARENT       : {d['parent_path']}")
        print(f"  CHILDREN     : {d['children']}")
        print(f"  has_Model    : {d['features']['has_Model']}")
        print(f"  has_Group8761: {d['features']['has_Group8761']}  (desc: {d['features']['desc_has_Group8761']})")
        print(f"  has_Group1   : {d['features']['has_Group1']}  (desc: {d['features']['desc_has_Group1']})")
        print(f"  has_workers  : {d['features']['desc_has_workers']}")
        print(f"  mesh_count   : {d['features']['mesh_count']}")
        print(f"  SUGGESTED TARGET: {d['suggested_target_prim']}  (meshes={d['target_mesh_count']})")

    # 还原所有关键字命中（不只顶层）
    print(f"\n[discover] All keyword-matched paths ({len(report['all_keyword_matches'])}):")
    for c in report["all_keyword_matches"]:
        print(f"  {c['path']}")

    return report


if __name__ == "__main__":
    main()
