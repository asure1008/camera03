import json
import random
import math
import re
import yaml
import copy
from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf

_STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD = 0.15
_STARTUP_VIEW_NEAR_MISS_PIXEL_MARGIN = 80.0

def compute_changjing_aabb(stage, changjing_path="/World/JiKeng_ChangJing01"):
    prim = stage.GetPrimAtPath(changjing_path)
    if not prim.IsValid():
        # Fallback if not found
        return {
            "xmin": 41.10, "xmax": 161.10,
            "ymin": -92.60, "ymax": 93.40,
            "zmin": -0.16, "zmax": 36.42,
            "center": (101.1, 0.4, 18.13),
            "size": (120.0, 186.0, 36.58)
        }
        
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render'])
    bbox = bbox_cache.ComputeWorldBound(prim)
    box_range = bbox.ComputeAlignedRange()
    
    min_pt = box_range.GetMin()
    max_pt = box_range.GetMax()
    
    aabb = {
        "xmin": float(min_pt[0]), "xmax": float(max_pt[0]),
        "ymin": float(min_pt[1]), "ymax": float(max_pt[1]),
        "zmin": float(min_pt[2]), "zmax": float(max_pt[2]),
        "center": (float((min_pt[0]+max_pt[0])/2), float((min_pt[1]+max_pt[1])/2), float((min_pt[2]+max_pt[2])/2)),
        "size": (float(max_pt[0]-min_pt[0]), float(max_pt[1]-min_pt[1]), float(max_pt[2]-min_pt[2]))
    }
    
    with open("changjing_aabb_report.yaml", "w") as f:
        yaml.dump(aabb, f)
        
    return aabb

def find_best_target_prim(stage, model_path, changjing_aabb):
    model_prim = stage.GetPrimAtPath(model_path)
    if not model_prim.IsValid():
        return None
        
    candidates = []
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render'])
    
    print(f"\n[TargetSelection] Scanning candidates under {model_path}")
    
    # Iterate over children of Model
    for child in model_prim.GetChildren():
        child_path = child.GetPath().pathString
        child_name = child.GetName()
        
        # Skip known person nodes
        if "node______" in child_name:
            continue
            
        # Check if it has meshes
        has_mesh = False
        for desc in Usd.PrimRange(child):
            if desc.IsA(UsdGeom.Mesh):
                has_mesh = True
                break
                
        # Compute bbox
        bbox = bbox_cache.ComputeWorldBound(child)
        box_range = bbox.ComputeAlignedRange()
        if box_range.IsEmpty():
            continue
            
        min_pt = box_range.GetMin()
        max_pt = box_range.GetMax()
        z_center = (min_pt[2] + max_pt[2]) / 2.0
        z_min = min_pt[2]
        z_max = max_pt[2]
        
        candidates.append({
            "path": child_path,
            "name": child_name,
            "has_mesh": has_mesh,
            "z_center": z_center,
            "z_min": z_min,
            "z_max": z_max
        })
        
    if not candidates:
        print(f"[TargetSelection] No valid candidates found under {model_path}")
        return None
        
    # Print candidates
    for c in candidates:
        print(f"  - Candidate: {c['path']} | has_mesh={c['has_mesh']} | z_center={c['z_center']:.2f} | z_range=[{c['z_min']:.2f}, {c['z_max']:.2f}]")
        
    # Filter by realistic Z (e.g. within changjing_aabb zmax + some margin)
    valid_z_max = changjing_aabb["zmax"] + 150.0 # generous margin, but avoids 1500+
    
    best_candidate = None
    
    # 1. Prefer Group8761 if it has mesh and valid Z
    for c in candidates:
        if c["name"] == "Group8761" and c["has_mesh"] and c["z_center"] < valid_z_max:
            best_candidate = c
            break
            
    # 2. Prefer any with mesh and valid Z
    if not best_candidate:
        valid_candidates = [c for c in candidates if c["has_mesh"] and c["z_center"] < valid_z_max]
        if valid_candidates:
            # Sort by Z center closest to 0 or just pick first
            best_candidate = sorted(valid_candidates, key=lambda x: abs(x["z_center"]))[0]
            
    # 3. Fallback to any valid Z
    if not best_candidate:
        valid_candidates = [c for c in candidates if c["z_center"] < valid_z_max]
        if valid_candidates:
            best_candidate = sorted(valid_candidates, key=lambda x: abs(x["z_center"]))[0]
            
    # 4. Absolute fallback
    if not best_candidate:
        best_candidate = candidates[0]
        
    print(f"[TargetSelection] Chosen target prim for {model_path}: {best_candidate['path']} (z_center={best_candidate['z_center']:.2f})\n")
    return best_candidate["path"]

def _is_diaolan_name(name: str) -> bool:
    """名字含 diaolan/gondola（大小写不敏感）即视为吊篮候选。"""
    low = name.lower()
    return "diaolan" in low or "gondola" in low


def _count_meshes_under(prim) -> int:
    if prim is None or not prim.IsValid():
        return 0
    return sum(1 for d in Usd.PrimRange(prim) if d.IsA(UsdGeom.Mesh))


# 名称上更像「绳/顶架/悬挂」而非可升降篮体的直接子节点（用于在装配体内收窄高度目标）
_GONDOLA_SUSPENSION_NAME_TOKENS = (
    "rope",
    "cable",
    "wire",
    "suspension",
    "hook",
    "roof",
    "mount",
    "bracket",
    "topframe",
    "ceiling",
    "绳",
    "索",
    "悬挂",
    "吊点",
    "支架",
    "楼顶",
)


def _max_mesh_child_path_under_model(stage, gondola_root_path: str) -> str | None:
    """{root}/Model 下网格数最多的直接子节点路径（装配体根，如 Diaolan_01）。"""
    model = stage.GetPrimAtPath(f"{gondola_root_path.rstrip('/')}/Model")
    if not model.IsValid():
        return None
    best_path, best_count = None, -1
    for child in model.GetChildren():
        cnt = _count_meshes_under(child)
        if cnt > best_count:
            best_count = cnt
            best_path = child.GetPath().pathString
    return best_path


def _pick_height_target_under_assembly(stage, assembly_path: str) -> str:
    """
    在装配体（如 Diaolan_01）下选择应写竖直 translate 的子 Xform：
    优先 Group1/Group8761，其次常见篮体根名，再排除绳/顶架等后取网格最多者。
    若无法收窄则退回装配体自身。
    """
    prim = stage.GetPrimAtPath(assembly_path)
    if not prim.IsValid():
        return assembly_path
    ap = assembly_path.rstrip("/")
    for sub in ("Group1", "Group8761"):
        p = stage.GetPrimAtPath(f"{ap}/{sub}")
        if p.IsValid() and _count_meshes_under(p) > 0:
            return p.GetPath().pathString
    for name in ("Diaolan", "Basket", "Platform", "basket", "platform", "diaolan"):
        p = stage.GetPrimAtPath(f"{ap}/{name}")
        if p.IsValid() and _count_meshes_under(p) > 0:
            return p.GetPath().pathString
    best_path, best_c = None, -1
    for ch in prim.GetChildren():
        nm = ch.GetName()
        low = nm.lower()
        c = _count_meshes_under(ch)
        if c <= 0:
            continue
        if any(t in low for t in _GONDOLA_SUSPENSION_NAME_TOKENS):
            continue
        if any(t in low for t in ("people", "person", "worker")):
            continue
        if c > best_c:
            best_c = c
            best_path = ch.GetPath().pathString
    return best_path or assembly_path


def resolve_diaolan_height_and_assembly(stage, gondola_root_path: str) -> tuple[str | None, str | None]:
    """
    解析单台吊篮的高度写入目标与工人搜索根。

    返回 (height_target_path, assembly_path)：
    - height_target_path：应对外作为 group1 键参与 _set_prim_translate_height（篮体/作业单元）。
    - assembly_path：Model 下主装配体（如 .../Model/Diaolan_01），用于 _collect_worker_prims。

    与 discover_diaolan_prims._suggest_target 一致：优先 Model/Group8761、Model/Group1；
    否则在装配体内再收窄，避免写到 Diaolan_01 根导致 Rope/顶架等兄弟一起动。
    """
    base = gondola_root_path.rstrip("/")
    model = stage.GetPrimAtPath(f"{base}/Model")
    if not model.IsValid():
        return None, None

    for suffix in ("Group8761", "Group1"):
        p = stage.GetPrimAtPath(f"{base}/Model/{suffix}")
        if p.IsValid() and _count_meshes_under(p) > 0:
            asm = _max_mesh_child_path_under_model(stage, base)
            if not asm:
                asm = p.GetPath().pathString
            return p.GetPath().pathString, asm

    assembly = _max_mesh_child_path_under_model(stage, base)
    if not assembly:
        return None, None
    height = _pick_height_target_under_assembly(stage, assembly)
    return height, assembly


def _find_target_prim_under_model(stage, gondola_root_path: str) -> str | None:
    """兼容旧调用：返回 Model 下网格最多的装配体路径（高度目标请用 resolve_diaolan_height_and_assembly）。"""
    return _max_mesh_child_path_under_model(stage, gondola_root_path)


def infer_world_diaolan_instance_root(prim_path: str) -> str | None:
    """从任意位于 /World/<吊篮集群>/<实例>/... 下的路径推断吊篮实例根路径。"""
    s = str(prim_path or "").strip()
    parts = [x for x in s.strip("/").split("/") if x]
    if len(parts) < 3 or parts[0] != "World":
        return None
    cluster, inst = parts[1].strip(), parts[2].strip()
    if not _is_diaolan_name(cluster) or not inst:
        return None
    return f"/World/{cluster}/{inst}".rstrip("/")


def _segment_is_logical_worker_anchor(seg: str) -> bool:
    """用于判定路径段是否为「角色根」：取从左到右第一个锚点，避免 .../People_01/.../People 误拆成两个逻辑人。"""
    sl = str(seg or "").lower()
    if sl in ("person", "persons"):
        return True
    if sl.startswith("worker"):
        return True
    if sl.startswith("people"):
        # 资产里常见子节点名 People（无编号）挂在 People_xx 下，不作为独立逻辑人
        if sl == "people":
            return False
        return True
    return False


def logical_worker_root_path(prim_path: str) -> str:
    """
    将任意工人相关 USD 路径映射到「逻辑工人」稳定键：路径前缀止于**从左到右第一个**工人锚点段。
    同一物理角色下多子 prim 共享同一键；不同吊篮实例下路径不同，故键在实例内唯一。
    """
    raw = str(prim_path or "").strip().rstrip("/")
    if not raw:
        return ""
    parts = [x for x in raw.split("/") if x]
    for i, seg in enumerate(parts):
        if _segment_is_logical_worker_anchor(seg):
            return "/" + "/".join(parts[: i + 1])
    return raw


def resolve_worker_visibility_root_path(prim_path: str) -> str:
    """
    工人 USD 显隐应作用的根路径：与 logical_worker_root_path 对齐，
    保证同一逻辑角色下所有可渲染子树随根一起显隐（计数口径与画面一致）。
    """
    return logical_worker_root_path(prim_path)


def apply_worker_logical_branch_visibility(stage, anchor_path: str, visible: bool) -> None:
    """
    在逻辑工人根 prim 上统一显隐（resolve_worker_visibility_root_path），整棵角色子树随 USD 继承生效。
    不在每个子 mesh 上单独改 visibility，避免 randomize/二次显示时子 prim 被卡在 invisible。
    """
    raw = str(anchor_path or "").strip().rstrip("/")
    if not raw:
        return
    root_path = resolve_worker_visibility_root_path(raw).rstrip("/") or raw
    prim = stage.GetPrimAtPath(root_path)
    if not prim.IsValid():
        prim = stage.GetPrimAtPath(raw)
        if prim.IsValid():
            root_path = prim.GetPath().pathString.rstrip("/")
        else:
            return
    img_root = UsdGeom.Imageable(prim)
    if not img_root:
        return
    if visible:
        img_root.MakeVisible()
    else:
        img_root.MakeInvisible()


def collect_worker_prim_candidates_unmerged(stage, diaolan01_path: str) -> list[str]:
    """与 _collect_worker_prims 相同的收集范围，但不按 logical_worker_root_path 合并，供渲染修复排除前缀。"""
    acc: set[str] = set(_workers_under_assembly_like_root(stage, diaolan01_path))
    inst_root = infer_world_diaolan_instance_root(diaolan01_path)
    if inst_root:
        gao = f"{inst_root}/Model/Gaochuzuoye_Diaolan"
        if stage.GetPrimAtPath(gao).IsValid():
            acc.update(_workers_under_assembly_like_root(stage, gao))
        model = stage.GetPrimAtPath(f"{inst_root.rstrip('/')}/Model")
        if model.IsValid():
            seen = {diaolan01_path.rstrip("/"), gao.rstrip("/")}
            for ch in model.GetChildren():
                sub = ch.GetPath().pathString
                if sub.rstrip("/") in seen:
                    continue
                if _count_meshes_under(ch) <= 0:
                    continue
                acc.update(_workers_under_assembly_like_root(stage, sub))
    return sorted(acc, key=lambda s: s.lower())


def worker_render_exclusion_prefixes(stage, diaolan_entry: dict) -> list[str]:
    """
    当前吊篮下所有「工人相关」USD 路径前缀，用于 gondola renderable 修复链：
    凡落在此前缀下的 Gprim 不得触发祖先 MakeVisible，否则会抵消工人隐藏。
    """
    prefixes: set[str] = set()
    asm = str(diaolan_entry.get("assembly") or "").strip().rstrip("/")
    if asm:
        for p in collect_worker_prim_candidates_unmerged(stage, asm):
            ps = str(p).strip().rstrip("/")
            if not ps:
                continue
            prefixes.add(ps)
            r = resolve_worker_visibility_root_path(ps).rstrip("/")
            if r:
                prefixes.add(r)
    for p in diaolan_entry.get("persons") or []:
        ps = str(p).strip().rstrip("/")
        if not ps:
            continue
        prefixes.add(ps)
        r = resolve_worker_visibility_root_path(ps).rstrip("/")
        if r:
            prefixes.add(r)
    return sorted(prefixes, key=lambda s: (-len(s), s.lower()))


def dedupe_worker_prim_paths_ordered(paths) -> list[str]:
    """
    按逻辑工人去重，保留首次出现顺序；同一键多条路径时取更短路径作代表（更接近角色根 prim）。
    """
    key_to_path: dict[str, str] = {}
    key_order: list[str] = []
    for raw in paths or []:
        p = str(raw or "").strip()
        if not p:
            continue
        key = logical_worker_root_path(p)
        if key not in key_to_path:
            key_to_path[key] = p
            key_order.append(key)
        elif len(p) < len(key_to_path[key]):
            key_to_path[key] = p
    return [key_to_path[k] for k in key_order]


def dedupe_worker_prim_paths_sorted(paths) -> list[str]:
    """去重后按路径字典序排序，供 _collect_worker_prims 等需要稳定输出的场景。"""
    uniq = dedupe_worker_prim_paths_ordered(paths)
    return sorted(uniq, key=lambda s: s.lower())


def count_logical_workers_from_paths(paths) -> int:
    """路径列表上的逻辑工人数（按 logical_worker_root_path 去重计数）。"""
    keys: set[str] = set()
    for raw in paths or []:
        p = str(raw or "").strip()
        if not p:
            continue
        keys.add(logical_worker_root_path(p))
    return len(keys)


def _workers_under_assembly_like_root(stage, asm_path: str) -> list[str]:
    """在单个装配/载荷根下收集作业人员 prim（关键词 + Zuoyuerenyuan），供多根路径合并。"""
    WORKER_KEYWORDS = ("people", "worker", "person")
    out: list[str] = []
    prim = stage.GetPrimAtPath(asm_path)
    if not prim.IsValid():
        return out
    for child in prim.GetChildren():
        low = child.GetName().lower()
        if any(kw in low for kw in WORKER_KEYWORDS):
            has_mesh = any(True for d in Usd.PrimRange(child) if d.IsA(UsdGeom.Mesh))
            if has_mesh:
                out.append(child.GetPath().pathString)
    base = asm_path.rstrip("/")
    zy = stage.GetPrimAtPath(f"{base}/Zuoyuerenyuan")
    if zy.IsValid():
        for child in zy.GetChildren():
            low = child.GetName().lower()
            if any(kw in low for kw in WORKER_KEYWORDS):
                has_mesh = any(True for d in Usd.PrimRange(child) if d.IsA(UsdGeom.Mesh))
                if has_mesh:
                    out.append(child.GetPath().pathString)
    for desc in Usd.PrimRange(prim):
        nm = desc.GetName().lower()
        if nm.startswith("people") or "people_" in nm:
            if any(True for d in Usd.PrimRange(desc) if d.IsA(UsdGeom.Mesh)):
                out.append(desc.GetPath().pathString)
    return out


def _collect_worker_prims(stage, diaolan01_path: str) -> list[str]:
    """
    在装配体根（如 .../Model/Gaochuzuoye_Diaolan）下找工人节点：
      - 直接子：名字含 People/Worker/Person 且有 Mesh 后代
      - 新场景：`.../Model/Gaochuzuoye_Diaolan/Zuoyuerenyuan/People_XX`
    新模型适配：除显式 Gaochuzuoye_Diaolan 外，在实例 /Model 下对其它含网格大子树做同样扫描，
    避免装配目录改名后工人列表为空。
    """
    acc: set[str] = set(_workers_under_assembly_like_root(stage, diaolan01_path))
    inst_root = infer_world_diaolan_instance_root(diaolan01_path)
    if inst_root:
        gao = f"{inst_root}/Model/Gaochuzuoye_Diaolan"
        if stage.GetPrimAtPath(gao).IsValid():
            acc.update(_workers_under_assembly_like_root(stage, gao))
        model = stage.GetPrimAtPath(f"{inst_root.rstrip('/')}/Model")
        if model.IsValid():
            seen = {diaolan01_path.rstrip("/"), gao.rstrip("/")}
            for ch in model.GetChildren():
                sub = ch.GetPath().pathString
                if sub.rstrip("/") in seen:
                    continue
                if _count_meshes_under(ch) <= 0:
                    continue
                acc.update(_workers_under_assembly_like_root(stage, sub))
    return dedupe_worker_prim_paths_sorted(acc)


def resolve_active_diaolan_root(stage, active_diaolan_path: str) -> str | None:
    """解析当前活动吊篮实例根（/World/Diaolan 或 /World/Diaolan/Diaolan_01 等）。"""
    ap = str(active_diaolan_path or "").strip().rstrip("/")
    if not ap:
        return None
    if stage.GetPrimAtPath(ap).IsValid():
        return ap
    alt = infer_world_diaolan_instance_root(ap)
    if alt and stage.GetPrimAtPath(alt).IsValid():
        return alt.rstrip("/")
    return None


def is_prim_effectively_enabled_and_visible(stage, prim_path: str) -> tuple[bool, str]:
    """新模型场景态：prim 是否存在、启用、未被 USD 可见性判为 invisible。"""
    prim = stage.GetPrimAtPath(str(prim_path or "").strip())
    if not prim.IsValid():
        return False, "missing"
    if not prim.IsActive():
        return False, "inactive"
    imageable = UsdGeom.Imageable(prim)
    if imageable:
        vis = imageable.ComputeVisibility(Usd.TimeCode.Default())
        if vis == UsdGeom.Tokens.invisible:
            return False, "invisible"
    return True, "ok"


def _gaochuzuoye_scan_roots(stage, instance_root: str) -> list[str]:
    """当前吊篮实例下用于护栏/安全绳/限位扫描的载荷根（优先 Gaochuzuoye_Diaolan，其次 Model 下含关键词子树）。"""
    roots: list[str] = []
    inst = str(instance_root or "").strip().rstrip("/")
    if not inst:
        return roots
    model = stage.GetPrimAtPath(f"{inst}/Model")
    if not model.IsValid():
        return roots
    gao = f"{inst}/Model/Gaochuzuoye_Diaolan"
    if stage.GetPrimAtPath(gao).IsValid():
        roots.append(gao)
    for ch in model.GetChildren():
        p = ch.GetPath().pathString
        low = ch.GetName().lower()
        if p.rstrip("/") in {x.rstrip("/") for x in roots}:
            continue
        if _count_meshes_under(ch) <= 0:
            continue
        if any(t in low for t in ("diaolan", "gaochuzuoye", "gondola", "basket", "platform")):
            roots.append(p)
    if not roots:
        for ch in model.GetChildren():
            if _count_meshes_under(ch) > 0:
                roots.append(ch.GetPath().pathString)
                break
    return roots


def _pick_deepest_mesh_path(stage, candidate_paths: list[str]) -> str | None:
    meshes = [p for p in candidate_paths if stage.GetPrimAtPath(p).IsA(UsdGeom.Mesh)]
    if meshes:
        return max(meshes, key=lambda s: s.count("/"))
    if candidate_paths:
        return max(candidate_paths, key=lambda s: s.count("/"))
    return None


def evaluate_scene_rule_guardrails(stage, instance_root: str) -> dict:
    """
    新模型适配 — 场景状态判定（非相机视觉）：护栏 / 挡脚板 Front_01 + Front_02。
    在实例载荷根下按路径关键词 Fanghulangan + Front_01/02 解析代表性 Mesh。
    """
    inst = resolve_active_diaolan_root(stage, instance_root)
    ev: dict = {
        "diaolan_instance_root": inst,
        "slots": {},
        "evaluation": "scene_state",
        "adapter_note": "new_model_guardrails_scene_state_not_camera",
    }
    if not inst:
        return {
            "supported": True,
            "has_hazard": None,
            "reason": "path_resolve_failed",
            "evidence": ev,
        }
    slots_found: dict[str, list[str]] = {"Front_01": [], "Front_02": []}
    for gao in _gaochuzuoye_scan_roots(stage, inst):
        root_prim = stage.GetPrimAtPath(gao)
        if not root_prim.IsValid():
            continue
        for d in Usd.PrimRange(root_prim):
            ps = d.GetPath().pathString
            low = ps.lower()
            if "fanghulangan" not in low:
                continue
            if "front_01" in low:
                slots_found["Front_01"].append(ps)
            if "front_02" in low:
                slots_found["Front_02"].append(ps)
    priority = ("missing", "inactive", "invisible", "ok")
    worst = "ok"
    has_hazard = False
    for slot in ("Front_01", "Front_02"):
        cands = slots_found[slot]
        picked = _pick_deepest_mesh_path(stage, cands)
        if not picked:
            has_hazard = True
            st = "missing"
        else:
            _ok, st = is_prim_effectively_enabled_and_visible(stage, picked)
            if not _ok:
                has_hazard = True
        ev["slots"][slot] = {
            "picked_path": picked,
            "status": st if picked else "missing",
            "candidate_count": len(cands),
        }
        if priority.index(st if picked else "missing") < priority.index(worst):
            worst = st if picked else "missing"
    reason = "ok" if not has_hazard else worst
    return {
        "supported": True,
        "has_hazard": bool(has_hazard),
        "reason": reason,
        "evidence": ev,
    }


def _safety_rope_anchor_paths(stage, instance_root: str) -> list[str]:
    anchors: set[str] = set()
    inst = resolve_active_diaolan_root(stage, instance_root)
    if not inst:
        return []
    pat = re.compile(r"(?i)^SafetyRope_Fixed_\d+$")
    for gao in _gaochuzuoye_scan_roots(stage, inst):
        rp = stage.GetPrimAtPath(gao)
        if not rp.IsValid():
            continue
        for d in Usd.PrimRange(rp):
            if pat.match(d.GetName()):
                anchors.add(d.GetPath().pathString)
    return sorted(anchors)


def _rope_anchor_effective(stage, anchor_path: str) -> tuple[bool, str]:
    """锚点（SafetyRope_Fixed_XX）子树内是否存在启用且可见的 Mesh。"""
    prim = stage.GetPrimAtPath(anchor_path)
    if not prim.IsValid():
        return False, "missing"
    for d in Usd.PrimRange(prim):
        if not d.IsA(UsdGeom.Mesh):
            continue
        ps = d.GetPath().pathString
        ok, st = is_prim_effectively_enabled_and_visible(stage, ps)
        if ok:
            return True, "ok"
    return False, "missing"


def evaluate_scene_rule_safety_ropes(stage, instance_root: str, workers_count: int | None) -> dict:
    """
    新模型适配 — 场景状态判定（非相机视觉）：安全绳数量 vs 作业人数。
    本版按「数量满足单人单绳」做场景态判定，不做人物-绳索一一绑定跟踪。
    """
    inst = resolve_active_diaolan_root(stage, instance_root)
    anchors = _safety_rope_anchor_paths(stage, instance_root or "")
    effective = 0
    rope_debug: list[dict] = []
    for a in anchors:
        ok, st = _rope_anchor_effective(stage, a)
        if ok:
            effective += 1
        rope_debug.append({"anchor": a, "effective": ok, "status": st})
    ev = {
        "diaolan_instance_root": inst,
        "safety_rope_anchors": anchors,
        "safety_rope_count": effective,
        "workers_count": workers_count,
        "evaluation": "scene_state",
        "adapter_note": "new_model_safety_rope_scene_state_not_camera",
    }
    if workers_count is None:
        return {
            "supported": True,
            "has_hazard": None,
            "reason": "workers_count_unavailable",
            "evidence": {**ev, "rope_anchors_debug": rope_debug},
        }
    wc = int(workers_count)
    ev["workers_count"] = wc
    ev["rope_anchors_debug"] = rope_debug
    if wc <= 0:
        return {
            "supported": True,
            "has_hazard": False,
            "reason": "no_workers",
            "evidence": ev,
        }
    hz = effective < wc
    reason = (
        "ropes_sufficient"
        if not hz
        else "insufficient_ropes_for_workers"
    )
    return {
        "supported": True,
        "has_hazard": hz,
        "reason": reason,
        "evidence": ev,
    }


def _steel_rope_root_paths(stage, gao_path: str) -> list[str]:
    """
    仅取载荷根（如 Gaochuzuoye_Diaolan）下**直接子** Steel_Rope_XX，避免 PrimRange 扫到
    Xuandiaojigou/Steel_Rope_XX 等同名节点（通常无数值限位 Mesh）造成误判。
    """
    out: list[str] = []
    prim = stage.GetPrimAtPath(gao_path)
    if not prim.IsValid():
        return out
    pat = re.compile(r"(?i)^Steel_Rope_\d+$")
    for ch in prim.GetChildren():
        if pat.match(ch.GetName()):
            out.append(ch.GetPath().pathString)
    return sorted(set(out))


def _limitstop_mesh_paths_under(stage, base_path: str) -> list[str]:
    out: list[str] = []
    prim = stage.GetPrimAtPath(base_path)
    if not prim.IsValid():
        return out
    for d in Usd.PrimRange(prim):
        ps = d.GetPath().pathString
        if "limitstop" not in ps.lower():
            continue
        if d.IsA(UsdGeom.Mesh):
            out.append(ps)
    return out


def _hazard_status_rank(st: str) -> int:
    order = {"missing": 0, "inactive": 1, "invisible": 2, "ok": 3}
    return int(order.get(str(st), 0))


def evaluate_scene_rule_limitstops(stage, instance_root: str) -> dict:
    """
    新模型适配 — 场景状态判定（非相机视觉）：各钢丝绳分支下 Limitstop 几何是否有效。
    若存在 Steel_Rope_XX 命名，则逐根钢丝绳检查其下 Limitstop；否则回退为载荷根下全部 Limitstop。
    """
    inst = resolve_active_diaolan_root(stage, instance_root)
    ev: dict = {
        "diaolan_instance_root": inst,
        "evaluation": "scene_state",
        "adapter_note": "new_model_limitstop_scene_state_not_camera",
    }
    if not inst:
        return {
            "supported": True,
            "has_hazard": None,
            "reason": "path_resolve_failed",
            "evidence": ev,
        }
    per_rope: list[dict] = []
    all_hit: list[str] = []
    has_hazard = False
    worst = "ok"
    gao_roots = _gaochuzuoye_scan_roots(stage, inst)
    for gao in gao_roots:
        steel = _steel_rope_root_paths(stage, gao)
        if steel:
            for sr in steel:
                meshes = _limitstop_mesh_paths_under(stage, sr)
                all_hit.extend(meshes)
                any_ok = False
                best_st = "missing"
                for m in meshes:
                    ok, st = is_prim_effectively_enabled_and_visible(stage, m)
                    if ok:
                        any_ok = True
                        best_st = "ok"
                        break
                    if _hazard_status_rank(st) < _hazard_status_rank(best_st):
                        best_st = st
                if not meshes or not any_ok:
                    has_hazard = True
                per_rope.append(
                    {
                        "steel_rope": sr,
                        "limitstop_mesh_count": len(meshes),
                        "effective": any_ok,
                        "status": best_st if not any_ok else "ok",
                    }
                )
                if not any_ok:
                    st_bad = "missing" if not meshes else str(best_st)
                    if _hazard_status_rank(st_bad) < _hazard_status_rank(worst):
                        worst = st_bad
        else:
            meshes = _limitstop_mesh_paths_under(stage, gao)
            all_hit.extend(meshes)
            any_ok = False
            best_st = "missing"
            for m in meshes:
                ok, st = is_prim_effectively_enabled_and_visible(stage, m)
                if ok:
                    any_ok = True
                    best_st = "ok"
                    break
                if _hazard_status_rank(st) < _hazard_status_rank(best_st):
                    best_st = st
            if not meshes or not any_ok:
                has_hazard = True
            per_rope.append(
                {
                    "steel_rope": None,
                    "gao_scan_root": gao,
                    "limitstop_mesh_count": len(meshes),
                    "effective": any_ok,
                    "status": best_st if not any_ok else "ok",
                }
            )
            if not any_ok:
                st_bad = "missing" if not meshes else str(best_st)
                if _hazard_status_rank(st_bad) < _hazard_status_rank(worst):
                    worst = st_bad
    ev["limitstop_hit_count"] = len(all_hit)
    ev["limitstop_mesh_paths_sample"] = all_hit[:24]
    ev["per_steel_rope_or_fallback"] = per_rope
    reason = "ok" if not has_hazard else worst
    return {
        "supported": True,
        "has_hazard": bool(has_hazard),
        "reason": reason,
        "evidence": ev,
    }


def _fallarrestor_mesh_paths_under(stage, base_path: str) -> list[str]:
    out: list[str] = []
    diaolan_prim = stage.GetPrimAtPath(base_path + "/Diaolan")
    if not diaolan_prim.IsValid():
        return out
    
    for ch in diaolan_prim.GetChildren():
        name = ch.GetName().lower()
        if name.startswith("component_"):
            for comp_ch in ch.GetChildren():
                comp_name = comp_ch.GetName().lower()
                if "fallarrestor" in comp_name or "fullarrestor" in comp_name:
                    stack = [comp_ch]
                    while stack:
                        p = stack.pop()
                        if p.IsA(UsdGeom.Mesh):
                            out.append(p.GetPath().pathString)
                        else:
                            stack.extend(p.GetChildren())
    return out


def evaluate_scene_rule_fallarrestors(stage, instance_root: str) -> dict:
    inst = resolve_active_diaolan_root(stage, instance_root)
    ev: dict = {
        "diaolan_instance_root": inst,
        "evaluation": "scene_state",
        "adapter_note": "new_model_fallarrestor_scene_state_not_camera",
    }
    if not inst:
        return {
            "supported": True,
            "has_hazard": None,
            "reason": "path_resolve_failed",
            "evidence": ev,
        }
    
    all_hit: list[str] = []
    gao_roots = _gaochuzuoye_scan_roots(stage, inst)
    for gao in gao_roots:
        meshes = _fallarrestor_mesh_paths_under(stage, gao)
        all_hit.extend(meshes)

    if not all_hit:
        return {
            "supported": True,
            "has_hazard": True,
            "reason": "missing",
            "evidence": {**ev, "fallarrestor_hit_count": 0, "fallarrestor_mesh_paths_sample": []},
        }

    any_hidden = False
    worst = "ok"
    for m in all_hit:
        ok, st = is_prim_effectively_enabled_and_visible(stage, m)
        if not ok:
            any_hidden = True
            if _hazard_status_rank(st) < _hazard_status_rank(worst):
                worst = st

    ev["fallarrestor_hit_count"] = len(all_hit)
    ev["fallarrestor_mesh_paths_sample"] = all_hit[:24]

    reason = "ok" if not any_hidden else worst
    return {
        "supported": True,
        "has_hazard": bool(any_hidden),
        "reason": reason,
        "evidence": ev,
    }


def collect_guardrail_slot_mesh_targets(stage, instance_root: str) -> dict:
    """
    与 evaluate_scene_rule_guardrails 同源扫描：返回 Front_01/Front_02 的代表 Mesh 与候选列表，
    供 Web/随机事件对「当前吊篮根 path」做相对路径控制。
    """
    inst = resolve_active_diaolan_root(stage, instance_root or "")
    slots_found: dict[str, list[str]] = {"Front_01": [], "Front_02": []}
    if not inst:
        return {
            "resolved_instance_root": None,
            "slots": {s: {"picked": None, "candidates": []} for s in slots_found},
        }
    for gao in _gaochuzuoye_scan_roots(stage, inst):
        root_prim = stage.GetPrimAtPath(gao)
        if not root_prim.IsValid():
            continue
        for d in Usd.PrimRange(root_prim):
            ps = d.GetPath().pathString
            low = ps.lower()
            if "fanghulangan" not in low:
                continue
            if "front_01" in low:
                slots_found["Front_01"].append(ps)
            if "front_02" in low:
                slots_found["Front_02"].append(ps)
    slots: dict = {}
    for slot in ("Front_01", "Front_02"):
        cands = slots_found[slot]
        slots[slot] = {"picked": _pick_deepest_mesh_path(stage, cands), "candidates": list(cands)}
    return {"resolved_instance_root": inst, "slots": slots}


def _imageable_make_vis(stage, pth: str, visible: bool) -> bool:
    prim = stage.GetPrimAtPath(str(pth or "").strip())
    if not prim.IsValid():
        return False
    img = UsdGeom.Imageable(prim)
    if visible:
        img.MakeVisible()
    else:
        img.MakeInvisible()
    return True


def apply_diaolan_safety_component(
    stage,
    instance_root: str,
    component: str,
    action: str,
    workers_count: int | None,
) -> dict:
    """
    对 resolve_active_diaolan_root(instance_root) 解析到的实例做护栏/安全绳/限位显隐。
    - guardrail: show=两侧候选 Mesh 尽量 MakeVisible；hide=至少隐藏一侧代表 Mesh（优先 Front_01）。
    - safety_rope: compliant=全部锚点 MakeVisible；non_compliant=使有效绳数 < workers（workers<=0 时不强制隐患）。
    - limitstop: show=载荷下各钢丝绳 Limitstop Mesh MakeVisible；hide=至少一根钢丝绳下 Limitstop 全部 MakeInvisible。
    """
    comp = str(component or "").strip().lower()
    act = str(action or "").strip().lower()
    root = str(instance_root or "").strip()
    changed: list[str] = []
    inst = resolve_active_diaolan_root(stage, root)
    out: dict = {
        "ok": True,
        "active_diaolan_path": root,
        "resolved_instance_root": inst,
        "component": comp,
        "action": act,
        "changed_prims": changed,
    }
    if not inst:
        out["ok"] = False
        out["error"] = "path_resolve_failed"
        return out

    if comp == "guardrail":
        info = collect_guardrail_slot_mesh_targets(stage, root)
        slots = info.get("slots") or {}
        if act == "show":
            for slot in ("Front_01", "Front_02"):
                for p in (slots.get(slot) or {}).get("candidates") or []:
                    prim = stage.GetPrimAtPath(p)
                    if prim.IsValid() and prim.IsA(UsdGeom.Mesh) and _imageable_make_vis(stage, p, True):
                        changed.append(p)
        elif act == "hide":
            pick01 = (slots.get("Front_01") or {}).get("picked")
            pick02 = (slots.get("Front_02") or {}).get("picked")
            target = pick01 or pick02
            if target and _imageable_make_vis(stage, target, False):
                changed.append(target)
        else:
            out["ok"] = False
            out["error"] = f"unknown_action:{act}"
        return out

    if comp == "safety_rope":
        anchors = _safety_rope_anchor_paths(stage, inst)
        wc = int(workers_count) if workers_count is not None else 0
        if act == "compliant":
            for a in anchors:
                if _imageable_make_vis(stage, a, True):
                    changed.append(a)
            return out
        if act == "non_compliant":
            for a in anchors:
                if _imageable_make_vis(stage, a, True):
                    changed.append(a)
            if wc <= 0:
                out["note"] = "workers_count<=0; ropes left visible (rule 5 no_workers is non-hazard)"
                return out
            eff = sum(1 for a in anchors if _rope_anchor_effective(stage, a)[0])
            idx = len(anchors) - 1
            while eff >= wc and idx >= 0:
                a = anchors[idx]
                if _imageable_make_vis(stage, a, False):
                    changed.append(a)
                eff = sum(1 for x in anchors if _rope_anchor_effective(stage, x)[0])
                idx -= 1
            return out
        out["ok"] = False
        out["error"] = f"unknown_action:{act}"
        return out

    if comp == "limitstop":
        if act == "show":
            for gao in _gaochuzuoye_scan_roots(stage, inst):
                steel = _steel_rope_root_paths(stage, gao)
                if steel:
                    for sr in steel:
                        for m in _limitstop_mesh_paths_under(stage, sr):
                            if _imageable_make_vis(stage, m, True):
                                changed.append(m)
                else:
                    for m in _limitstop_mesh_paths_under(stage, gao):
                        if _imageable_make_vis(stage, m, True):
                            changed.append(m)
            return out
        if act == "hide":
            hid_any = False
            for gao in _gaochuzuoye_scan_roots(stage, inst):
                steel = _steel_rope_root_paths(stage, gao)
                ropes = steel if steel else [gao]
                for base in ropes:
                    meshes = _limitstop_mesh_paths_under(stage, base)
                    if not meshes:
                        continue
                    for m in meshes:
                        if _imageable_make_vis(stage, m, False):
                            changed.append(m)
                    hid_any = True
                    break
                if hid_any:
                    break
            if not hid_any:
                out["note"] = "no_limitstop_meshes_found_to_hide"
            return out
        out["ok"] = False
        out["error"] = f"unknown_action:{act}"
        return out

    if comp == "fallarrestor":
        if act == "compliant":
            for gao in _gaochuzuoye_scan_roots(stage, inst):
                for m in _fallarrestor_mesh_paths_under(stage, gao):
                    if _imageable_make_vis(stage, m, True):
                        changed.append(m)
            return out
        if act == "non_compliant":
            hid_any = False
            for gao in _gaochuzuoye_scan_roots(stage, inst):
                meshes = _fallarrestor_mesh_paths_under(stage, gao)
                if not meshes:
                    continue
                # 只需隐藏其中一侧（Component_01 or Component_02）即可构成不合规
                comps = {}
                for m in meshes:
                    parent_path = stage.GetPrimAtPath(m).GetParent().GetPath().pathString
                    comps.setdefault(parent_path, []).append(m)
                
                if comps:
                    first_group = list(comps.values())[0]
                    for m in first_group:
                        if _imageable_make_vis(stage, m, False):
                            changed.append(m)
                    hid_any = True
                    break
            if not hid_any:
                out["note"] = "no_fallarrestor_meshes_found_to_hide"
            return out
        out["ok"] = False
        out["error"] = f"unknown_action:{act}"
        return out

    out["ok"] = False
    out["error"] = f"unknown_component:{comp}"
    return out


def summarize_diaolan_safety_components(stage, instance_root: str, workers_count: int | None) -> dict:
    """汇总当前实例下护栏/安全绳/限位的 evaluate 结果（供 HTTP / Web 展示）。"""
    root = str(instance_root or "").strip()
    inst = resolve_active_diaolan_root(stage, root)
    g = evaluate_scene_rule_guardrails(stage, root)
    s = evaluate_scene_rule_safety_ropes(stage, root, workers_count)
    l = evaluate_scene_rule_limitstops(stage, root)
    f = evaluate_scene_rule_fallarrestors(stage, root)
    ge = g.get("evidence") if isinstance(g.get("evidence"), dict) else {}
    se = s.get("evidence") if isinstance(s.get("evidence"), dict) else {}
    le = l.get("evidence") if isinstance(l.get("evidence"), dict) else {}
    fe = f.get("evidence") if isinstance(f.get("evidence"), dict) else {}
    return {
        "active_diaolan_path": root,
        "resolved_instance_root": inst,
        "workers_count": workers_count,
        "guardrail": {
            "has_hazard": g.get("has_hazard"),
            "reason": g.get("reason"),
            "evidence": ge,
            "slots": ge.get("slots"),
        },
        "safety_rope": {
            "has_hazard": s.get("has_hazard"),
            "reason": s.get("reason"),
            "evidence": se,
            "safety_rope_count": se.get("safety_rope_count"),
            "safety_rope_anchors": se.get("safety_rope_anchors"),
        },
        "limitstop": {
            "has_hazard": l.get("has_hazard"),
            "reason": l.get("reason"),
            "evidence": le,
            "per_steel_rope_or_fallback": le.get("per_steel_rope_or_fallback"),
        },
        "fallarrestor": {
            "has_hazard": f.get("has_hazard"),
            "reason": f.get("reason"),
            "evidence": fe,
            "fallarrestor_hit_count": fe.get("fallarrestor_hit_count"),
        },
    }


def randomize_active_diaolan_safety_components(
    stage,
    instance_root: str,
    workers_count: int | None,
    rng: random.Random,
    *,
    random_guardrail: bool = False,
    random_safety_rope: bool = False,
    random_limitstop: bool = False,
    random_fallarrestor: bool = False,
    guardrail_mode: str = "random",
    safety_rope_mode: str = "random",
    limitstop_mode: str = "random",
    fallarrestor_mode: str = "random",
) -> dict:
    """仅作用于传入的吊篮根路径（通常为当前随机目标 / 当前选中），与 random_gondola 是否改选无关。"""
    steps: list[dict] = []

    def _pick_mode(mode: str, a: str, b: str) -> str:
        m = str(mode or "random").strip().lower()
        if m in (a, b):
            return m
        return rng.choice([a, b])

    root = str(instance_root or "").strip()
    wc = workers_count
    if random_guardrail:
        m = _pick_mode(guardrail_mode, "intact", "missing")
        act = "show" if m == "intact" else "hide"
        steps.append(
            {
                "component": "guardrail",
                "mode": m,
                "apply": apply_diaolan_safety_component(stage, root, "guardrail", act, wc),
            }
        )
    if random_safety_rope:
        m = _pick_mode(safety_rope_mode, "compliant", "non_compliant")
        act = "compliant" if m == "compliant" else "non_compliant"
        steps.append(
            {
                "component": "safety_rope",
                "mode": m,
                "apply": apply_diaolan_safety_component(stage, root, "safety_rope", act, wc),
            }
        )
    if random_limitstop:
        m = _pick_mode(limitstop_mode, "intact", "missing")
        act = "show" if m == "intact" else "hide"
        steps.append(
            {
                "component": "limitstop",
                "mode": m,
                "apply": apply_diaolan_safety_component(stage, root, "limitstop", act, wc),
            }
        )
    if random_fallarrestor:
        m = _pick_mode(fallarrestor_mode, "compliant", "non_compliant")
        act = "compliant" if m == "compliant" else "non_compliant"
        steps.append(
            {
                "component": "fallarrestor",
                "mode": m,
                "apply": apply_diaolan_safety_component(stage, root, "fallarrestor", act, wc),
            }
        )
    # 不在随机主链路末尾再跑 summarize（三次全量扫描），避免拖慢 randomize；需要时由前端或 status 接口查询。
    return {"steps": steps}


def _standard_diaolan_cluster_instance_roots(stage) -> list[str]:
    """
    新母场景：/World/Diaolan、/World/Diaolan_01 等集群下各 9 台实例 Diaolan + Diaolan_01..08。
    若该结构不存在则返回空列表，回退到旧版 /World 直接子扫描。
    """
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return []
    ordered = ["Diaolan"] + [f"Diaolan_{i:02d}" for i in range(1, 9)]
    out: list[str] = []
    for cluster in world.GetChildren():
        if not _is_diaolan_name(cluster.GetName()):
            continue
        cluster_path = cluster.GetPath().pathString.rstrip("/")
        # 仅把“吊篮实例容器”当集群：子级应包含 Diaolan/Diaolan_XX 且实例下有 Model。
        found_in_cluster = []
        for rel in ordered:
            pth = f"{cluster_path}/{rel}"
            p = stage.GetPrimAtPath(pth)
            if not p.IsValid():
                continue
            model = stage.GetPrimAtPath(f"{pth}/Model")
            if not model.IsValid():
                continue
            if not any(True for d in Usd.PrimRange(model) if d.IsA(UsdGeom.Mesh)):
                continue
            found_in_cluster.append(pth)
        if not found_in_cluster:
            continue
        out.extend(found_in_cluster)
    return out


def scan_diaolan_prims(stage):
    """
    基于 stage 遍历发现吊篮 prim——不依赖任何硬编码路径。

    发现策略：
      1. 遍历 /World 直接子节点（不递归全局，避免性能问题）。
      2. 筛选名字含 "diaolan"/"gondola" 的 Xform prim。
      3. 要求存在 Model 子节点且 Model 下有 Mesh 后代（确保是真实几何节点）。
      4. 用 resolve_diaolan_height_and_assembly 得到高度目标与装配体根（工人搜索根）。
      5. 在装配体（或回退为高度目标）直接子中找工人节点（名含 People/Worker/Person）。

    返回格式（向后兼容）：
      list of {
        "path":     "/World/Diaolan_Ver1_0_2026_XX",
        "group1":   "/World/.../Model/Diaolan_01/Diaolan",  # 竖直位移写入目标（篮体）
        "assembly": "/World/.../Model/Diaolan_01",           # 可选：主装配体
        "persons":  ["/World/.../People_01", ...],
      }
    """
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        # 尝试 stage defaultPrim
        root = stage.GetDefaultPrim()
        if root and root.IsValid():
            world = root
        else:
            print("[scan_diaolan_prims] WARNING: /World not found; falling back to full stage traverse")
            world = None

    candidates = []
    cluster_roots = _standard_diaolan_cluster_instance_roots(stage)
    if cluster_roots:
        candidates = cluster_roots
        print(f"[scan_diaolan_prims] cluster /World/*Diaolan* instances: {candidates}")
    elif world is not None:
        children = list(world.GetChildren())
        for prim in children:
            if not _is_diaolan_name(prim.GetName()):
                continue
            path = prim.GetPath().pathString

            # 必须有 Model 子节点
            model = stage.GetPrimAtPath(f"{path}/Model")
            if not model.IsValid():
                print(f"[scan_diaolan_prims] skip {path}: no Model child")
                continue

            # Model 下必须有 Mesh 后代
            has_mesh = any(True for d in Usd.PrimRange(model) if d.IsA(UsdGeom.Mesh))
            if not has_mesh:
                print(f"[scan_diaolan_prims] skip {path}: Model has no mesh descendants")
                continue

            candidates.append(path)
    else:
        children = [
            p for p in stage.Traverse()
            if p.GetParent() and not p.GetParent().GetParent().IsValid()
        ]
        for prim in children:
            if not _is_diaolan_name(prim.GetName()):
                continue
            path = prim.GetPath().pathString
            model = stage.GetPrimAtPath(f"{path}/Model")
            if not model.IsValid():
                print(f"[scan_diaolan_prims] skip {path}: no Model child")
                continue
            has_mesh = any(True for d in Usd.PrimRange(model) if d.IsA(UsdGeom.Mesh))
            if not has_mesh:
                print(f"[scan_diaolan_prims] skip {path}: Model has no mesh descendants")
                continue
            candidates.append(path)

    print(f"[scan_diaolan_prims] discovered candidates: {candidates}")

    diaolans = []
    for path in candidates:
        height_path, assembly_path = resolve_diaolan_height_and_assembly(stage, path)
        if (
            not height_path
            or not assembly_path
            or not stage.GetPrimAtPath(height_path).IsValid()
            or not stage.GetPrimAtPath(assembly_path).IsValid()
        ):
            print(f"[scan_diaolan_prims] skip {path}: resolve_diaolan_height_and_assembly failed")
            continue

        workers = _collect_worker_prims(stage, assembly_path)
        if not workers:
            workers = _collect_worker_prims(stage, height_path)

        parent_h = (
            stage.GetPrimAtPath(height_path).GetParent().GetPath().pathString
            if stage.GetPrimAtPath(height_path).GetParent().IsValid()
            else ""
        )
        diaolans.append({
            "path": path,
            # 兼容旧键名：group1 = 高度写入目标（篮体/作业单元），非整组装配体根
            "group1": height_path,
            "assembly": assembly_path,
            "persons": workers,
        })
        print(
            "[scan_diaolan_prims] accepted "
            f"path={path} height_target={height_path} parent_of_height={parent_h!r} "
            f"assembly={assembly_path} workers={workers}"
        )

    return diaolans


def _iter_mesh_prims(root_prim):
    for desc in Usd.PrimRange(root_prim):
        if desc.IsA(UsdGeom.Mesh):
            yield desc


def _compute_world_range(bbox_cache, prim):
    bbox = bbox_cache.ComputeWorldBound(prim)
    return bbox.ComputeAlignedRange()


def _get_bound_material(mesh_prim):
    binding = UsdShade.MaterialBindingAPI(mesh_prim).ComputeBoundMaterial()
    if isinstance(binding, tuple):
        return binding[0]
    return binding


def _collect_target_branch_render_stats(stage, target_prim_path):
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not target_prim.IsValid():
        return None

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
    world_range = _compute_world_range(bbox_cache, target_prim)
    root_imageable = UsdGeom.Imageable(target_prim)
    root_visibility = "non-imageable"
    root_purpose = "non-imageable"
    if root_imageable:
        root_visibility = root_imageable.ComputeVisibility(Usd.TimeCode.Default())
        root_purpose = root_imageable.GetPurposeAttr().Get() or "default"

    stats = {
        "path": target_prim_path,
        "world_min": tuple(float(v) for v in world_range.GetMin()),
        "world_max": tuple(float(v) for v in world_range.GetMax()),
        "world_mid": tuple(float(v) for v in world_range.GetMidpoint()),
        "root_visibility": str(root_visibility),
        "root_purpose": str(root_purpose),
        "mesh_count": 0,
        "mesh_invisible_count": 0,
        "mesh_nondefault_purpose_count": 0,
        "mesh_missing_material_count": 0,
        "mesh_preview_surface_count": 0,
        "mesh_mdl_only_count": 0,
        "materials": {},
    }

    for mesh_prim in _iter_mesh_prims(target_prim):
        stats["mesh_count"] += 1
        imageable = UsdGeom.Imageable(mesh_prim)
        mesh_visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
        mesh_purpose = imageable.GetPurposeAttr().Get() or "default"
        if mesh_visibility != UsdGeom.Tokens.inherited:
            stats["mesh_invisible_count"] += 1
        if mesh_purpose != UsdGeom.Tokens.default_:
            stats["mesh_nondefault_purpose_count"] += 1

        material = _get_bound_material(mesh_prim)
        if not material or not material.GetPrim().IsValid():
            stats["mesh_missing_material_count"] += 1
            continue

        material_path = material.GetPath().pathString
        output_names = sorted(out.GetFullName() for out in material.GetOutputs())
        has_preview_surface = "outputs:surface" in output_names
        has_mdl_surface = "outputs:mdl:surface" in output_names
        if has_preview_surface:
            stats["mesh_preview_surface_count"] += 1
        if has_mdl_surface and not has_preview_surface:
            stats["mesh_mdl_only_count"] += 1

        entry = stats["materials"].setdefault(
            material_path,
            {
                "mesh_count": 0,
                "outputs": output_names,
            },
        )
        entry["mesh_count"] += 1

    return stats


def log_target_branch_render_state(stage, target_prim_path, renderer_name=""):
    stats = _collect_target_branch_render_stats(stage, target_prim_path)
    if not stats:
        print(f"[target-branch] invalid target_prim={target_prim_path}")
        return None

    print(
        "[target-branch] "
        f"renderer={renderer_name or 'unknown'} "
        f"chosen={stats['path']} "
        f"world_z={stats['world_mid'][2]:.2f} "
        f"bbox_min={stats['world_min']} "
        f"bbox_max={stats['world_max']}"
    )
    print(
        "[target-branch] "
        f"root_visibility={stats['root_visibility']} "
        f"root_purpose={stats['root_purpose']} "
        f"mesh_count={stats['mesh_count']} "
        f"mesh_invisible={stats['mesh_invisible_count']} "
        f"mesh_nondefault_purpose={stats['mesh_nondefault_purpose_count']} "
        f"mesh_missing_material={stats['mesh_missing_material_count']} "
        f"mesh_preview_surface={stats['mesh_preview_surface_count']} "
        f"mesh_mdl_only={stats['mesh_mdl_only_count']}"
    )

    top_materials = sorted(
        stats["materials"].items(),
        key=lambda item: (-item[1]["mesh_count"], item[0]),
    )[:6]
    for material_path, material_info in top_materials:
        print(
            "[target-branch-material] "
            f"path={material_path} "
            f"mesh_count={material_info['mesh_count']} "
            f"outputs={material_info['outputs']}"
        )

    hydra_mdl_only = (
        renderer_name == "HydraStorm"
        and stats["mesh_count"] > 0
        and stats["mesh_preview_surface_count"] == 0
        and stats["mesh_mdl_only_count"] > 0
    )
    print(f"[target-branch] hydra_mdl_only={hydra_mdl_only}")
    return stats


def _get_direct_binding_targets(prim):
    rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel()
    if not rel:
        return []
    return [str(t) for t in rel.GetTargets()]


def _get_subset_prims(mesh_prim):
    subset_prims = []
    for child in mesh_prim.GetChildren():
        if child.GetTypeName() == "GeomSubset":
            subset_prims.append(child)
    return subset_prims


def _get_shader_inputs_from_material(material):
    result = {}
    if not material or not material.GetPrim().IsValid():
        return result
    for out in material.GetOutputs():
        if out.GetFullName() not in ("outputs:mdl:surface", "outputs:surface"):
            continue
        sources, _ = out.GetConnectedSources()
        if not sources:
            continue
        shader_prim = sources[0].source.GetPrim()
        if not shader_prim or not shader_prim.IsValid():
            continue
        shader = UsdShade.Shader(shader_prim)
        for inp in shader.GetInputs():
            result[inp.GetBaseName()] = inp.Get()
    return result


def _as_color3f(v, default_color):
    if isinstance(v, (Gf.Vec3f, Gf.Vec3d)):
        return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))
    if isinstance(v, (tuple, list)) and len(v) >= 3:
        return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))
    return default_color


def _as_float(v, default_value):
    try:
        return float(v)
    except Exception:
        return float(default_value)


def _build_preview_surface_material(stage, mat_path, base_color, opacity, roughness, metallic, emissive=None):
    mat = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(base_color)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if emissive is not None:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(emissive)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def force_debug_emissive_on_target_branch(stage, target_prim_path):
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not target_prim.IsValid():
        print(f"[force-debug] invalid target_prim={target_prim_path}")
        return {"total": 0, "covered": 0, "uncovered": 0, "subset_mesh_count": 0}

    debug_mat = _build_preview_surface_material(
        stage,
        "/World/Looks/HydraStormDebugEmissive",
        Gf.Vec3f(1.0, 0.0, 0.0),
        1.0,
        1.0,
        0.0,
        emissive=Gf.Vec3f(1.0, 0.0, 0.0),
    )

    total = 0
    covered = 0
    subset_mesh_count = 0
    for mesh_prim in _iter_mesh_prims(target_prim):
        total += 1
        mesh_path = mesh_prim.GetPath().pathString
        bound_mat = _get_bound_material(mesh_prim)
        bound_path = bound_mat.GetPath().pathString if bound_mat and bound_mat.GetPrim().IsValid() else "None"
        direct_targets = _get_direct_binding_targets(mesh_prim)
        subset_prims = _get_subset_prims(mesh_prim)
        has_subset_binding = False
        for subset_prim in subset_prims:
            subset_targets = _get_direct_binding_targets(subset_prim)
            if subset_targets:
                has_subset_binding = True
                break
        if subset_prims:
            subset_mesh_count += 1

        imageable = UsdGeom.Imageable(mesh_prim)
        visibility = str(imageable.ComputeVisibility(Usd.TimeCode.Default()))
        purpose = str(imageable.GetPurposeAttr().Get() or "default")
        mesh = UsdGeom.Mesh(mesh_prim)
        double_sided = mesh.GetDoubleSidedAttr().Get() if mesh.GetDoubleSidedAttr() else None
        normals_attr = mesh.GetNormalsAttr()
        has_normals = bool(normals_attr and normals_attr.HasValue())
        print(
            "[force-debug-before] "
            f"mesh={mesh_path} bound={bound_path} direct={direct_targets} "
            f"subset_binding={has_subset_binding} visibility={visibility} "
            f"purpose={purpose} doubleSided={double_sided} hasNormals={has_normals}"
        )

        UsdShade.MaterialBindingAPI(mesh_prim).Bind(
            debug_mat,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
        mesh.GetDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.0, 0.0)])
        mesh.GetDoubleSidedAttr().Set(True)
        UsdGeom.Imageable(mesh_prim).MakeVisible()

    for mesh_prim in _iter_mesh_prims(target_prim):
        mesh_path = mesh_prim.GetPath().pathString
        bound_mat = _get_bound_material(mesh_prim)
        bound_path = bound_mat.GetPath().pathString if bound_mat and bound_mat.GetPrim().IsValid() else "None"
        is_debug = (bound_path == "/World/Looks/HydraStormDebugEmissive")
        if is_debug:
            covered += 1
        print(f"[force-debug-after] mesh={mesh_path} bound={bound_path} is_debug={is_debug}")

    uncovered = total - covered
    print(
        "[force-debug-summary] "
        f"total={total} covered={covered} uncovered={uncovered} subset_mesh_count={subset_mesh_count}"
    )
    return {"total": total, "covered": covered, "uncovered": uncovered, "subset_mesh_count": subset_mesh_count}


def apply_hydrastorm_formal_materials_on_target_branch(stage, target_prim_path):
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not target_prim.IsValid():
        print(f"[formal-preview] invalid target_prim={target_prim_path}")
        return {"total": 0, "covered": 0, "uncovered": 0}

    mat_cache = {}
    default_color = Gf.Vec3f(0.72, 0.72, 0.72)
    total = 0
    covered = 0

    for mesh_prim in _iter_mesh_prims(target_prim):
        total += 1
        source_mat = _get_bound_material(mesh_prim)
        source_path = source_mat.GetPath().pathString if source_mat and source_mat.GetPrim().IsValid() else "none"
        if source_path in mat_cache:
            compat_mat = mat_cache[source_path]
        else:
            params = _get_shader_inputs_from_material(source_mat)
            base_color = _as_color3f(params.get("diffuseColor", params.get("diffuse_color_constant")), default_color)
            opacity = _as_float(params.get("opacity", params.get("opacity_constant", 1.0)), 1.0)
            roughness = _as_float(params.get("roughness", 0.8), 0.8)
            metallic = _as_float(params.get("metallic", 0.0), 0.0)
            safe_name = source_path.replace("/", "_").replace(":", "_")
            if source_path == "none":
                safe_name = "fallback_none"
            mat_path = f"/World/Looks/HydraStormCompat/{safe_name}"
            compat_mat = _build_preview_surface_material(stage, mat_path, base_color, opacity, roughness, metallic)
            mat_cache[source_path] = compat_mat

        UsdShade.MaterialBindingAPI(mesh_prim).Bind(
            compat_mat,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
        )
        mesh = UsdGeom.Mesh(mesh_prim)
        mesh.GetDoubleSidedAttr().Set(True)
        UsdGeom.Imageable(mesh_prim).MakeVisible()

    for mesh_prim in _iter_mesh_prims(target_prim):
        bound_mat = _get_bound_material(mesh_prim)
        bound_path = bound_mat.GetPath().pathString if bound_mat and bound_mat.GetPrim().IsValid() else "None"
        is_compat = bound_path.startswith("/World/Looks/HydraStormCompat/")
        if is_compat:
            covered += 1
        print(f"[formal-preview-after] mesh={mesh_prim.GetPath().pathString} bound={bound_path} is_compat={is_compat}")

    uncovered = total - covered
    print(f"[formal-preview-summary] total={total} covered={covered} uncovered={uncovered}")
    return {"total": total, "covered": covered, "uncovered": uncovered}


def _clear_direct_material_binding(prim):
    rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel()
    if rel:
        rel.ClearTargets(True)


def prepare_geometry_only_target_branch(stage, target_prim_path):
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not target_prim.IsValid():
        print(f"[geometry-only] invalid target_prim={target_prim_path}")
        return {"mesh_count": 0, "subset_count": 0, "unbound_count": 0}

    subset_count = 0
    unbound_count = 0
    mesh_count = 0
    white = Gf.Vec3f(1.0, 1.0, 1.0)

    root_img = UsdGeom.Imageable(target_prim)
    if root_img:
        root_img.MakeVisible()
        root_img.GetPurposeAttr().Set(UsdGeom.Tokens.default_)

    # Clear direct material bindings for every prim in branch first.
    for prim in Usd.PrimRange(target_prim):
        _clear_direct_material_binding(prim)
        unbound_count += 1

    for mesh_prim in _iter_mesh_prims(target_prim):
        mesh_count += 1
        mesh_path = mesh_prim.GetPath().pathString

        # Remove subset bindings under mesh
        for subset_prim in _get_subset_prims(mesh_prim):
            subset_count += 1
            _clear_direct_material_binding(subset_prim)

        mesh = UsdGeom.Mesh(mesh_prim)
        mesh.GetDisplayColorAttr().Set([white])
        mesh.GetDoubleSidedAttr().Set(True)
        img = UsdGeom.Imageable(mesh_prim)
        img.MakeVisible()
        img.GetPurposeAttr().Set(UsdGeom.Tokens.default_)
        bound_after = _get_bound_material(mesh_prim)
        bound_after_path = bound_after.GetPath().pathString if bound_after and bound_after.GetPrim().IsValid() else "None"
        print(
            "[geometry-only-mesh] "
            f"path={mesh_path} "
            f"bound_after={bound_after_path} "
            "displayColor=(1,1,1) purpose=default visibility=inherited doubleSided=True"
        )

    print(
        "[geometry-only-summary] "
        f"mesh_count={mesh_count} subset_count={subset_count} unbound_count={unbound_count}"
    )
    return {"mesh_count": mesh_count, "subset_count": subset_count, "unbound_count": unbound_count}

def pick_active_diaolan(all_diaolans, rng, forced_path: str | None = None):
    paths = [d.get("path") for d in all_diaolans]
    print(f"[diaolan-pick-debug] candidates={paths}")
    if not all_diaolans:
        print("[diaolan-pick-debug] no candidates")
        return None
    if forced_path:
        for d in all_diaolans:
            if d.get("path") == forced_path:
                print(f"[diaolan-pick-debug] forced={forced_path}")
                return d
        print(f"[diaolan-pick-debug] forced_missing={forced_path} -> fallback_random")
    picked = rng.choice(all_diaolans)
    print(f"[diaolan-pick-debug] random={picked.get('path')}")
    return picked

def apply_diaolan_visibility(stage, active_diaolan, all_diaolans):
    """多吊篮并存：所有吊篮根节点保持可见。active_diaolan 仅保留参数兼容旧调用。"""
    del active_diaolan  # 选中吊篮不再驱动显隐
    for d in all_diaolans:
        prim = stage.GetPrimAtPath(d["path"])
        if not prim.IsValid():
            continue
        UsdGeom.Imageable(prim).MakeVisible()

def randomize_workers(stage, diaolan_info, rng):
    persons = diaolan_info["persons"]
    if not persons:
        return 0
    
    # 0 .. len(persons)（每吊篮槽位数由资产扫描决定，常见 2 或 3）
    count = rng.randint(0, len(persons))
    
    # Shuffle to pick random persons
    person_paths = list(persons)
    rng.shuffle(person_paths)
    
    visible_count = 0
    for i, p_path in enumerate(person_paths):
        prim = stage.GetPrimAtPath(p_path)
        if not prim.IsValid():
            continue
        vis = i < count
        apply_worker_logical_branch_visibility(stage, p_path, vis)
        if vis:
            visible_count += 1
            
    return visible_count

def _stage_height_axis_index(stage) -> int:
    """与 ptz_stream._height_axis_index 一致：Z-up→2，Y-up→1。"""
    up = UsdGeom.GetStageUpAxis(stage)
    if up == UsdGeom.Tokens.z:
        return 2
    if up == UsdGeom.Tokens.y:
        return 1
    return 2


def _set_prim_world_height_axis_match(stage, prim_path: str, world_height: float) -> None:
    """将 prim 的世界平移在场景高度轴上的分量设为 world_height，与 ptz_stream._set_prim_translate_height 同口径。"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    xform = UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()
    h_idx = _stage_height_axis_index(stage)
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_before = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    wx, wy, wz = float(world_before[0]), float(world_before[1]), float(world_before[2])
    if h_idx == 2:
        target_world = Gf.Vec3d(wx, wy, float(world_height))
    else:
        target_world = Gf.Vec3d(wx, float(world_height), wz)
    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_world = xform_cache.GetLocalToWorldTransform(parent)
        target_local = parent_world.GetInverse().TransformAffine(target_world)
    else:
        target_local = target_world
    if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        translate_op.Set(
            Gf.Vec3f(float(target_local[0]), float(target_local[1]), float(target_local[2]))
        )
    else:
        translate_op.Set(
            Gf.Vec3d(float(target_local[0]), float(target_local[1]), float(target_local[2]))
        )


def sync_workers_to_group1(stage, group1_path, person_paths, active_count):
    """把各工人根 prim 的世界「高度轴」分量对齐到 group1_path（篮体高度目标），避免父级层级变化后误读局部 translate。"""
    del active_count  # 保留签名兼容；可见性由 randomize_workers / 其它路径控制
    ref = stage.GetPrimAtPath(group1_path)
    if not ref.IsValid():
        return
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    h_idx = _stage_height_axis_index(stage)
    wt = cache.GetLocalToWorldTransform(ref).ExtractTranslation()
    h_world = float(wt[h_idx])
    for p_path in person_paths:
        _set_prim_world_height_axis_match(stage, p_path, h_world)

def check_safety_hazard(group1_world_z, persons_visible_count, ground_z_baseline=0.12, ground_eps=0.5):
    is_on_ground = group1_world_z < ground_z_baseline + ground_eps
    if not is_on_ground and persons_visible_count == 0:
        return True, "吊篮在空中但人数为0"
    if is_on_ground and persons_visible_count != 0:
        return False, None  # 地面有人=合规
    return False, None


_WALL_HEIGHT_EPS = 1e-5
_WALL_HEIGHT_PRIM_DEFAULT = "/World/JiKeng_ChangJing01/JiKeng_BeiJing/JiKeng_BeiJing/group1/Mesh267/Mesh267"
_WALL_CONSTRAINT_XY_MARGIN_DEFAULT = 0.005
_WALL_CONSTRAINT_Z_MARGIN_DEFAULT = 0.05
_WALL_MOUNT_INSET_M_DEFAULT = 0.0
_WALL_MOUNT_INSET_MODE_INWARD = "inward_from_wall_surface"
_WALL_INSTALL_BAND_MAX_PARENT_STEPS = 6
_WALL_INSTALL_BAND_MIN_HEIGHT = 2.0
_WALL_INSTALL_BAND_MIN_LENGTH = 6.0
_WALL_MOUNT_SURFACE_OFFSET = 0.05
_WALL_MOUNT_SURFACE_DISTANCE_MAX = 0.12
_WALL_MOUNT_TOP_PROXIMITY_MAX = 0.35
_WALL_MOUNT_MIN_BOTTOM_CLEARANCE = 0.35
_WALL_MOUNT_MIN_TOP_CLEARANCE = 0.6
_WALL_MOUNT_NEIGHBOR_RADIUS = 30.0
_WALL_MOUNT_TARGET_RADIUS = 95.0
_WALL_MOUNT_FIRST_LAYER_TOLERANCE = 4.0
_WALL_MOUNT_FIRST_LAYER_COMPONENT_GAP = 2.5
_WALL_MOUNT_COMMON_HEIGHT_MARGIN = 0.12
_WALL_MOUNT_FACE_KEYWORDS = (
    "wall",
    "fence",
    "railing",
    "barrier",
    "guard",
    "metal",
)
_WALL_MOUNT_AGGREGATE_NAMES = ("world", "model")
_WALL_MOUNT_AGGREGATE_PREFIXES = ("group",)

# global_wall_candidates：只在 JiKeng_BeiJing 围墙子树内枚举，再围绕配置 seed 墙做近邻裁剪；
# 不扩大到整场景，避免把楼体本身、机械设备、大门等 wall-like 几何加入相机候选池。
_GLOBAL_WALL_SEMANTIC_SEED_MESH267_PATH = (
    "/World/JiKeng_ChangJing01/JiKeng_BeiJing/JiKeng_BeiJing/group1/Mesh267/Mesh267"
)
_GLOBAL_WALL_COLLECTION_ROOT_PATH = "/World/JiKeng_ChangJing01/JiKeng_BeiJing"
_GLOBAL_WALL_COLLECTION_EXCLUDED_PREFIXES = ()


def _coerce_wall_candidate_region(raw_region):
    if not isinstance(raw_region, dict):
        return None
    if raw_region.get("enabled") is False:
        return None

    try:
        if "min" in raw_region and "max" in raw_region:
            mn = tuple(float(v) for v in raw_region.get("min"))
            mx = tuple(float(v) for v in raw_region.get("max"))
        else:
            mn = (
                float(raw_region.get("x_min")),
                float(raw_region.get("y_min")),
                float(raw_region.get("z_min")),
            )
            mx = (
                float(raw_region.get("x_max")),
                float(raw_region.get("y_max")),
                float(raw_region.get("z_max")),
            )
    except Exception:
        return None

    if len(mn) != 3 or len(mx) != 3:
        return None
    region_min = tuple(min(float(mn[i]), float(mx[i])) for i in range(3))
    region_max = tuple(max(float(mn[i]), float(mx[i])) for i in range(3))
    if not all(math.isfinite(v) for v in (*region_min, *region_max)):
        return None
    if any(region_max[i] - region_min[i] < 0.0 for i in range(3)):
        return None
    return {
        "enabled": True,
        "min": region_min,
        "max": region_max,
        "source": str(raw_region.get("source") or "config_wall_candidate_region"),
    }


def _candidate_intersects_wall_candidate_region(candidate, region):
    if region is None:
        return True
    cmin = candidate["min"]
    cmax = candidate["max"]
    rmin = region["min"]
    rmax = region["max"]
    return all(
        float(cmax[i]) + _WALL_HEIGHT_EPS >= float(rmin[i])
        and float(cmin[i]) - _WALL_HEIGHT_EPS <= float(rmax[i])
        for i in range(3)
    )


def _filter_wall_candidate_region(candidates, raw_region):
    region = _coerce_wall_candidate_region(raw_region)
    diag = {
        "applied": bool(region is not None),
        "mode": "bbox_intersects_config_region",
        "before_count": int(len(candidates or [])),
        "after_count": int(len(candidates or [])),
        "removed_count": 0,
        "fallback_used": False,
        "fallback_reason": None,
        "region_min": None,
        "region_max": None,
        "source": None,
    }
    if region is None:
        diag["fallback_reason"] = "missing_or_invalid_wall_candidate_region"
        return candidates, diag

    filtered = [
        candidate for candidate in (candidates or [])
        if _candidate_intersects_wall_candidate_region(candidate, region)
    ]
    diag.update({
        "after_count": int(len(filtered)),
        "removed_count": int(len(candidates or []) - len(filtered)),
        "region_min": [round(float(v), 4) for v in region["min"]],
        "region_max": [round(float(v), 4) for v in region["max"]],
        "source": region["source"],
    })
    if not filtered:
        diag["fallback_used"] = False
        diag["fallback_reason"] = "wall_candidate_region_empty"
    return filtered, diag


def _prim_path_matches_prefix(path, prefix):
    text = str(path or "")
    pref = str(prefix or "").rstrip("/")
    return bool(pref) and (text == pref or text.startswith(pref + "/"))


def _resolve_global_wall_collection_root_from_mesh267(stage):
    """Mesh267 仅作 parent-chain seed；成功时返回 (Usd.Prim, tag)，失败 (None, reason)。"""
    outer = _GLOBAL_WALL_COLLECTION_ROOT_PATH
    root_prim = stage.GetPrimAtPath(outer)
    if not root_prim or not root_prim.IsValid():
        return None, "missing_stage_prim_at_global_wall_root"

    for seed_path in (_GLOBAL_WALL_SEMANTIC_SEED_MESH267_PATH, _WALL_HEIGHT_PRIM_DEFAULT):
        p = stage.GetPrimAtPath(seed_path)
        if not p or not p.IsValid():
            continue
        pstr = p.GetPath().pathString
        if pstr == outer:
            return root_prim, "seed_prim_is_global_root"
        if not (pstr.startswith(outer + "/")):
            return None, "mesh267_seed_not_under_configured_global_wall_root"
        cur = p
        while cur and cur.IsValid():
            if cur.GetPath().pathString == outer:
                return cur, "parent_chain_from_mesh267_seed"
            cur = cur.GetParent()
        return None, "parent_chain_exhausted_before_global_wall_root"

    return root_prim, "global_wall_root_direct_no_valid_seed_under_tree"


def _vec3_to_tuple(vec):
    return (float(vec[0]), float(vec[1]), float(vec[2]))


def _compute_world_bbox_info(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"wall prim not found: {prim_path}")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render'])
    box_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box_range.IsEmpty():
        raise RuntimeError(f"wall prim has empty world bbox: {prim_path}")
    min_pt = _vec3_to_tuple(box_range.GetMin())
    max_pt = _vec3_to_tuple(box_range.GetMax())
    values = (*min_pt, *max_pt)
    if not all(math.isfinite(v) for v in values):
        raise RuntimeError(f"wall prim bbox contains non-finite values: {prim_path} -> {values}")
    return {
        'prim_path': prim_path,
        'min': min_pt,
        'max': max_pt,
    }


def _normalize_margin_value(raw_value, default_value, label):
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        print(f"[wall-constraint] WARN: invalid {label}={raw_value!r}; fallback={default_value}")
        return float(default_value)
    if value < 0.0:
        print(f"[wall-constraint] WARN: negative {label}={value}; clamp_to=0.0")
        return 0.0
    return value


def _shrink_world_bbox_with_margin(wall_bbox, xy_margin, z_margin):
    xy_margin = _normalize_margin_value(
        xy_margin, _WALL_CONSTRAINT_XY_MARGIN_DEFAULT, "wall_constraint_xy_margin"
    )
    z_margin = _normalize_margin_value(
        z_margin, _WALL_CONSTRAINT_Z_MARGIN_DEFAULT, "wall_constraint_z_margin"
    )

    src_min = wall_bbox['min']
    src_max = wall_bbox['max']
    spans = tuple(float(src_max[i] - src_min[i]) for i in range(3))
    if any(span <= _WALL_HEIGHT_EPS for span in spans):
        raise RuntimeError(
            f"wall prim bbox has degenerate span: path={wall_bbox['prim_path']} spans={spans}"
        )

    effective_min = (
        float(src_min[0] + xy_margin),
        float(src_min[1] + xy_margin),
        float(src_min[2] + z_margin),
    )
    effective_max = (
        float(src_max[0] - xy_margin),
        float(src_max[1] - xy_margin),
        float(src_max[2] - z_margin),
    )
    effective_spans = tuple(float(effective_max[i] - effective_min[i]) for i in range(3))
    if any(span <= _WALL_HEIGHT_EPS for span in effective_spans):
        raise RuntimeError(
            "wall prim bbox becomes invalid after inward margin shrink: "
            f"path={wall_bbox['prim_path']} raw_min={src_min} raw_max={src_max} "
            f"xy_margin={xy_margin} z_margin={z_margin} shrunk_min={effective_min} shrunk_max={effective_max}"
        )

    return {
        'prim_path': wall_bbox['prim_path'],
        'raw_min': src_min,
        'raw_max': src_max,
        'effective_min': effective_min,
        'effective_max': effective_max,
        'xy_margin': float(xy_margin),
        'z_margin': float(z_margin),
        'raw_spans': spans,
        'effective_spans': effective_spans,
    }


def _set_world_translate_on_prim(stage, prim, world_xyz):
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    trans_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            trans_op = op
            break
    if trans_op is None:
        trans_op = xform.AddTranslateOp()

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    parent = prim.GetParent()
    world_vec = Gf.Vec3d(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))
    if parent and parent.IsValid():
        parent_world = xform_cache.GetLocalToWorldTransform(parent)
        local_vec = parent_world.GetInverse().TransformAffine(world_vec)
    else:
        local_vec = world_vec
    trans_op.Set(local_vec)
    return (float(local_vec[0]), float(local_vec[1]), float(local_vec[2]))


def _path_depth(path):
    return max(0, len([token for token in str(path).split("/") if token]))


def _is_mount_aggregate_name(name_lower):
    return (
        name_lower in _WALL_MOUNT_AGGREGATE_NAMES
        or any(name_lower.startswith(prefix) for prefix in _WALL_MOUNT_AGGREGATE_PREFIXES)
    )


def _build_wall_mount_candidate(stage, prim, depth):
    info = _compute_world_bbox_info(stage, prim.GetPath().pathString)
    spans = tuple(float(info['max'][i] - info['min'][i]) for i in range(3))
    thickness_axis = 0 if spans[0] <= spans[1] else 1
    length_axis = 1 - thickness_axis
    thickness_span = float(spans[thickness_axis])
    length_span = float(spans[length_axis])
    height_span = float(spans[2])
    ratio = length_span / max(thickness_span, _WALL_HEIGHT_EPS)
    horizontal_area = float(spans[0] * spans[1])
    name_lower = prim.GetName().lower()
    path_lower = prim.GetPath().pathString.lower()
    keyword_hits = tuple(
        kw for kw in _WALL_MOUNT_FACE_KEYWORDS if kw in name_lower or kw in path_lower
    )
    has_wall_keyword = bool(keyword_hits)
    aggregate_name = _is_mount_aggregate_name(name_lower)
    forbidden_aggregate = (
        aggregate_name
        and not has_wall_keyword
        and (
            depth > 0
            or length_span >= 20.0
            or horizontal_area >= 120.0
            or height_span >= 10.0
        )
    )
    is_mesh = prim.IsA(UsdGeom.Mesh)
    is_wall_like = (
        height_span >= _WALL_INSTALL_BAND_MIN_HEIGHT
        and length_span >= 1.0
        and ratio >= 1.6
        and thickness_span <= max(4.0, length_span * 0.35)
    )
    score = 0.0
    if has_wall_keyword:
        score += 12.0
    if is_mesh:
        score += 7.0
    elif prim.IsA(UsdGeom.Gprim):
        score += 4.0
    score += min(ratio, 20.0) * 0.9
    if 2.0 <= height_span <= 8.0:
        score += 2.5
    elif height_span > 12.0:
        score -= min(6.0, (height_span - 12.0) * 0.35)
    if length_span > 40.0:
        score -= min(8.0, (length_span - 40.0) * 0.12)
    if thickness_span > 2.5:
        score -= min(10.0, (thickness_span - 2.5) * 2.4)
    if aggregate_name and not has_wall_keyword:
        score -= 9.0
    score -= depth * 1.35
    return {
        'prim_path': info['prim_path'],
        'name': prim.GetName(),
        'type_name': prim.GetTypeName(),
        'min': info['min'],
        'max': info['max'],
        'spans': spans,
        'height_span': height_span,
        'length_span': length_span,
        'thickness_span': thickness_span,
        'length_axis': length_axis,
        'thickness_axis': thickness_axis,
        'horizontal_area': horizontal_area,
        'wall_ratio': ratio,
        'depth': int(depth),
        'has_wall_keyword': has_wall_keyword,
        'keyword_hits': keyword_hits,
        'is_mesh': is_mesh,
        'aggregate_name': aggregate_name,
        'forbidden_aggregate': forbidden_aggregate,
        'is_wall_like': is_wall_like,
        'base_score': float(score),
        'score': float(score),
        'path_depth': _path_depth(info['prim_path']),
    }


def _format_mount_candidate(candidate):
    spans = tuple(round(float(v), 4) for v in candidate['spans'])
    return (
        f"path={candidate['prim_path']} type={candidate['type_name']} spans={spans} "
        f"ratio={candidate['wall_ratio']:.3f} score={candidate['score']:.3f} "
        f"depth={candidate['depth']} aggregate={candidate['aggregate_name']} "
        f"forbidden={candidate['forbidden_aggregate']} wall_like={candidate['is_wall_like']} "
        f"keywords={list(candidate['keyword_hits'])}"
    )


def _round_bbox_signature(candidate):
    return (
        tuple(round(float(v), 4) for v in candidate['min']),
        tuple(round(float(v), 4) for v in candidate['max']),
    )


def _candidate_center(candidate):
    return tuple((float(candidate['min'][i]) + float(candidate['max'][i])) * 0.5 for i in range(3))


def _coerce_world_bbox(raw_bbox):
    if not isinstance(raw_bbox, dict):
        return None
    try:
        mn = tuple(float(v) for v in raw_bbox.get("min"))
        mx = tuple(float(v) for v in raw_bbox.get("max"))
    except Exception:
        return None
    if len(mn) != 3 or len(mx) != 3:
        return None
    if not all(math.isfinite(v) for v in (*mn, *mx)):
        return None
    if any(mx[i] - mn[i] <= _WALL_HEIGHT_EPS for i in range(3)):
        return None
    return {
        "min": mn,
        "max": mx,
        "prim_path": str(raw_bbox.get("prim_path") or raw_bbox.get("path") or "").strip() or None,
    }


def _interval_overlap_len(a_min, a_max, b_min, b_max):
    return max(0.0, min(float(a_max), float(b_max)) - max(float(a_min), float(b_min)))


def _interval_gap_len(a_min, a_max, b_min, b_max):
    if float(a_max) < float(b_min):
        return float(b_min) - float(a_max)
    if float(b_max) < float(a_min):
        return float(a_min) - float(b_max)
    return 0.0


def _candidate_xy_gap_to_bbox(candidate, bbox):
    cmin = candidate["min"]
    cmax = candidate["max"]
    bmin = bbox["min"]
    bmax = bbox["max"]
    dx = _interval_gap_len(cmin[0], cmax[0], bmin[0], bmax[0])
    dy = _interval_gap_len(cmin[1], cmax[1], bmin[1], bmax[1])
    return float(math.hypot(dx, dy))


def _candidate_xy_overlap_ratio_to_bbox(candidate, bbox):
    cmin = candidate["min"]
    cmax = candidate["max"]
    bmin = bbox["min"]
    bmax = bbox["max"]
    length_axis = int(candidate.get("length_axis", 0))
    other_axis = 1 - length_axis
    length_span = max(_WALL_HEIGHT_EPS, float(cmax[length_axis] - cmin[length_axis]))
    overlap_len = _interval_overlap_len(
        cmin[length_axis],
        cmax[length_axis],
        bmin[length_axis],
        bmax[length_axis],
    )
    thickness_gap = _interval_gap_len(
        cmin[other_axis],
        cmax[other_axis],
        bmin[other_axis],
        bmax[other_axis],
    )
    return float(overlap_len / length_span), float(thickness_gap), int(length_axis)


def _candidate_xy_gap_between(a, b):
    amin = a["min"]
    amax = a["max"]
    bmin = b["min"]
    bmax = b["max"]
    dx = _interval_gap_len(amin[0], amax[0], bmin[0], bmax[0])
    dy = _interval_gap_len(amin[1], amax[1], bmin[1], bmax[1])
    return float(math.hypot(dx, dy))


def _candidate_z_overlap_sufficient(a, b):
    amin = a["min"]
    amax = a["max"]
    bmin = b["min"]
    bmax = b["max"]
    overlap = _interval_overlap_len(amin[2], amax[2], bmin[2], bmax[2])
    min_h = max(_WALL_HEIGHT_EPS, min(float(amax[2] - amin[2]), float(bmax[2] - bmin[2])))
    required = min(0.35, max(0.12, min_h * 0.2))
    return overlap >= required


def _connected_wall_components(candidates, *, adjacency_gap=_WALL_MOUNT_FIRST_LAYER_COMPONENT_GAP):
    total = len(candidates or [])
    parent = list(range(total))

    def find(idx):
        while parent[idx] != idx:
            parent[idx] = parent[parent[idx]]
            idx = parent[idx]
        return idx

    def union(a, b):
        ra = find(a)
        rb = find(b)
        if ra != rb:
            parent[rb] = ra

    for i in range(total):
        for j in range(i + 1, total):
            if _candidate_xy_gap_between(candidates[i], candidates[j]) <= float(adjacency_gap):
                if _candidate_z_overlap_sufficient(candidates[i], candidates[j]):
                    union(i, j)

    grouped = {}
    for idx in range(total):
        grouped.setdefault(find(idx), []).append(idx)
    return list(grouped.values())


def _filter_first_layer_wall_candidates(candidates, target_context_bbox, components=None):
    bbox = _coerce_world_bbox(target_context_bbox)
    diag = {
        "applied": False,
        "mode": "exclude_connected_first_rectangle",
        "source_prim_path": None,
        "before_count": int(len(candidates or [])),
        "after_count": int(len(candidates or [])),
        "fallback_used": False,
        "fallback_reason": None,
        "component_count": 0,
        "kept_component_count": 0,
        "nearest_distance": None,
        "tolerance": None,
        "component_adjacency_gap": float(_WALL_MOUNT_FIRST_LAYER_COMPONENT_GAP),
    }
    if not candidates or bbox is None:
        diag["fallback_reason"] = "missing_target_context_bbox" if bbox is None else "empty_candidates"
        return candidates, diag

    bmin = bbox["min"]
    bmax = bbox["max"]
    bspan = tuple(float(bmax[i] - bmin[i]) for i in range(3))
    anchor_horizontal_span = max(float(bspan[0]), float(bspan[1]), _WALL_HEIGHT_EPS)
    tolerance = max(6.0, min(14.0, anchor_horizontal_span * 0.15))
    components = components if components is not None else _connected_wall_components(candidates)
    if not components:
        diag["fallback_used"] = True
        diag["fallback_reason"] = "no_connected_components"
        return candidates, diag

    component_rows = []
    for comp_idx, indices in enumerate(components):
        distances = [_candidate_xy_gap_to_bbox(candidates[idx], bbox) for idx in indices]
        nearest = min(distances) if distances else float("inf")
        component_rows.append((float(nearest), comp_idx, indices))

    nearest_distance = min(row[0] for row in component_rows)
    keep_distance_max = nearest_distance + tolerance
    keep_component_ids = {comp_idx for dist, comp_idx, _indices in component_rows if dist <= keep_distance_max}
    first_component_idx = {idx for _dist, comp_idx, indices in component_rows if comp_idx in keep_component_ids for idx in indices}
    first_component_count = len(first_component_idx)
    rectangle_margin = max(1.0, min(4.0, anchor_horizontal_span * 0.08))
    overlap_min_ratio = 0.35
    remove_idx = set()
    kept_in_problem_component = 0
    for idx in first_component_idx:
        candidate = candidates[idx]
        overlap_ratio, thickness_gap, length_axis = _candidate_xy_overlap_ratio_to_bbox(candidate, bbox)
        center = _candidate_center(candidate)
        inside_length_band = (
            float(bmin[length_axis]) - rectangle_margin
            <= float(center[length_axis])
            <= float(bmax[length_axis]) + rectangle_margin
        )
        if overlap_ratio >= overlap_min_ratio or inside_length_band:
            remove_idx.add(idx)
            candidate["first_layer_rectangle_overlap_ratio"] = float(overlap_ratio)
            candidate["first_layer_rectangle_thickness_gap"] = float(thickness_gap)
            candidate["first_layer_rectangle_length_axis"] = int(length_axis)
        else:
            kept_in_problem_component += 1
    filtered_with_idx = [(idx, c) for idx, c in enumerate(candidates) if idx not in remove_idx]
    filtered = [c for _idx, c in filtered_with_idx]
    if not filtered:
        diag["fallback_used"] = True
        diag["fallback_reason"] = "exclude_connected_first_rectangle_empty"
        return candidates, diag

    distance_by_idx = {}
    comp_by_idx = {}
    for dist, comp_idx, indices in component_rows:
        for idx in indices:
            distance_by_idx[idx] = float(dist)
            comp_by_idx[idx] = int(comp_idx)

    for original_idx, candidate in filtered_with_idx:
        candidate["first_layer_wall_component"] = int(comp_by_idx.get(original_idx, -1))
        candidate["first_layer_wall_gap"] = float(distance_by_idx.get(original_idx, 0.0))

    diag.update({
        "applied": True,
        "source_prim_path": bbox.get("prim_path"),
        "anchor_horizontal_span": round(float(anchor_horizontal_span), 4),
        "after_count": int(len(filtered)),
        "component_count": int(len(components)),
        "kept_component_count": int(len(keep_component_ids)),
        "first_component_candidate_count": int(first_component_count),
        "removed_count": int(len(remove_idx)),
        "kept_in_problem_component_count": int(kept_in_problem_component),
        "rectangle_margin": round(float(rectangle_margin), 4),
        "rectangle_overlap_min_ratio": round(float(overlap_min_ratio), 4),
        "rectangle_bbox_min": [round(float(v), 4) for v in bmin],
        "rectangle_bbox_max": [round(float(v), 4) for v in bmax],
        "nearest_distance": round(float(nearest_distance), 4),
        "tolerance": round(float(tolerance), 4),
        "component_distance_cutoff": round(float(keep_distance_max), 4),
    })
    return filtered, diag


def _candidate_has_mesh_descendants(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return False
    for desc in Usd.PrimRange(prim):
        if desc.IsA(UsdGeom.Mesh):
            return True
    return False


def _resolve_wall_semantic_seed(stage, wall_path):
    start_prim = stage.GetPrimAtPath(wall_path)
    if not start_prim.IsValid():
        raise RuntimeError(f"wall prim not found: {wall_path}")
    best = None
    current = start_prim
    depth = 0
    while current and current.IsValid() and depth <= _WALL_INSTALL_BAND_MAX_PARENT_STEPS:
        candidate = _build_wall_mount_candidate(stage, current, depth)
        if candidate['has_wall_keyword'] and not candidate['aggregate_name']:
            best = candidate
        current = current.GetParent()
        depth += 1
    return best or _build_wall_mount_candidate(stage, start_prim, 0)


def _group_hint_from_path(path):
    text = str(path or "")
    m = re.search(r"/Model/(Group\d+)(?:/(Group\d+))?", text)
    if not m:
        return None
    return "/".join([g for g in m.groups() if g])


_WALL_CANDIDATE_POOL_CACHE = {}


def clear_wall_mount_candidate_cache():
    _WALL_CANDIDATE_POOL_CACHE.clear()


def _stage_cache_key(stage):
    root = stage.GetRootLayer() if stage else None
    return str(getattr(root, "identifier", "") or id(stage))


def _wall_pool_cache_key(stage, wall_path, mode, wall_collection_root_path, target_world_xyz=None, target_context_bbox=None, wall_candidate_region=None):
    # The candidate base pool is independent from the current gondola/building
    # anchor. Target scoring and first-rectangle filtering are applied after
    # cache retrieval.
    region = _coerce_wall_candidate_region(wall_candidate_region)
    region_key = None if region is None else (
        tuple(round(float(v), 6) for v in region["min"]),
        tuple(round(float(v), 6) for v in region["max"]),
    )
    return (
        _stage_cache_key(stage),
        str(wall_path or ""),
        str(mode or ""),
        str(wall_collection_root_path or ""),
        region_key,
    )


def _candidate_pool_signature(pool):
    return {
        "root": pool.get("collection_root"),
        "resolved_global_wall_root": pool.get("resolved_global_wall_root"),
        "candidate_count": int(len(pool.get("candidates") or [])),
        "scanned_mesh_count": int(pool.get("scanned_mesh_count") or 0),
    }


def _collection_root_signature(stage, pool):
    path = str(pool.get("collection_root") or pool.get("resolved_global_wall_root") or "")
    prim = stage.GetPrimAtPath(path) if path else None
    if not prim or not prim.IsValid():
        return None
    mesh_count = 0
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            mesh_count += 1
    return (path, mesh_count)


def _wall_pool_cache_valid(stage, pool):
    return bool(pool.get("_stage_cache_key") == _stage_cache_key(stage))


def _apply_first_layer_filter_to_pool(pool, target_context_bbox, target_world_xyz=None):
    out = copy.deepcopy(pool)
    base_candidates = copy.deepcopy(out.get("_base_candidates") or out.get("candidates") or [])
    _update_wall_candidate_target_metrics(
        base_candidates,
        out.get("_sample_center"),
        out.get("_seed_filter_applied_for_base"),
        target_world_xyz,
    )
    if str(out.get("wall_collection_mode") or "").strip().lower() == "global_wall_candidates":
        filtered_raw, first_layer_filter = _filter_first_layer_wall_candidates(
            base_candidates,
            target_context_bbox,
            components=out.get("_base_components"),
        )
        filtered, dedup_count = _dedup_and_sort_wall_candidates(filtered_raw)
        first_layer_filter["after_dedup_count"] = int(dedup_count)
    else:
        filtered = base_candidates
        first_layer_filter = {
            "applied": False,
            "before_count": int(len(base_candidates)),
            "after_count": int(len(base_candidates)),
        }
    out["candidates"] = filtered
    out["first_layer_filter"] = first_layer_filter
    out["final_candidate_count"] = int(len(filtered))
    out["common_height_min"], out["common_height_max"] = _wall_candidate_common_height_range(filtered)
    return out


def _update_wall_candidate_target_metrics(candidates, sample_center, seed_filter_applied, target_world_xyz):
    target_xy = None
    try:
        if target_world_xyz is not None:
            target_xy = (float(target_world_xyz[0]), float(target_world_xyz[1]))
    except Exception:
        target_xy = None
    for candidate in candidates or []:
        center = _candidate_center(candidate)
        horizontal_dist = 0.0
        try:
            if sample_center is not None:
                horizontal_dist = math.hypot(
                    center[0] - float(sample_center[0]),
                    center[1] - float(sample_center[1]),
                )
        except Exception:
            horizontal_dist = 0.0
        target_horizontal_dist = None
        if target_xy is not None:
            target_horizontal_dist = math.hypot(center[0] - target_xy[0], center[1] - target_xy[1])
        candidate['horizontal_distance_to_sample'] = float(horizontal_dist if seed_filter_applied else 0.0)
        candidate['horizontal_distance_to_target'] = None if target_horizontal_dist is None else float(target_horizontal_dist)
        base_score = float(candidate.get('base_score', candidate.get('score', 0.0)))
        if seed_filter_applied:
            candidate['score'] = float(base_score - horizontal_dist * 0.06)
        elif target_horizontal_dist is not None:
            candidate['score'] = float(base_score - target_horizontal_dist * 0.015)
        else:
            candidate['score'] = base_score


def _dedup_and_sort_wall_candidates(candidates):
    dedup = {}
    for candidate in candidates or []:
        signature = _round_bbox_signature(candidate)
        existing = dedup.get(signature)
        if existing is None or (
            candidate['path_depth'] > existing['path_depth']
            or (candidate['path_depth'] == existing['path_depth'] and candidate['score'] > existing['score'])
        ):
            dedup[signature] = candidate
    sorted_candidates = sorted(
        dedup.values(),
        key=lambda c: (
            -c['score'],
            c.get('horizontal_distance_to_target')
                if c.get('horizontal_distance_to_target') is not None
                else c.get('horizontal_distance_to_sample', 0.0),
            -c['path_depth'],
            c['prim_path'],
        ),
    )
    return sorted_candidates, len(dedup)


def _wall_candidate_common_height_range(candidates):
    lows = []
    highs = []
    for candidate in candidates or []:
        spans = candidate['spans']
        bottom_clearance = max(_WALL_CONSTRAINT_Z_MARGIN_DEFAULT, min(_WALL_MOUNT_MIN_BOTTOM_CLEARANCE, spans[2] * 0.15))
        top_clearance = max(_WALL_MOUNT_MIN_TOP_CLEARANCE, min(1.4, spans[2] * 0.28))
        low = float(candidate['min'][2] + bottom_clearance)
        high = float(candidate['max'][2] - top_clearance)
        if high - low > _WALL_HEIGHT_EPS:
            lows.append(low)
            highs.append(high)
    if not lows or not highs:
        return None, None
    common_min = max(lows) - _WALL_MOUNT_COMMON_HEIGHT_MARGIN
    common_max = min(highs) + _WALL_MOUNT_COMMON_HEIGHT_MARGIN
    if common_max - common_min <= _WALL_HEIGHT_EPS:
        common_min = min(lows)
        common_max = max(highs)
    return float(common_min), float(common_max)


def _collect_wall_mount_candidates(stage, wall_path, wall_collection_mode=None, wall_collection_root_path=None, wall_pool_cache=None, target_world_xyz=None, target_context_bbox=None, wall_candidate_region=None):
    mode = str(wall_collection_mode or "semantic_parent").strip().lower() or "semantic_parent"
    request_cache = isinstance(wall_pool_cache, dict)
    cache = wall_pool_cache if request_cache else _WALL_CANDIDATE_POOL_CACHE
    cache_key = _wall_pool_cache_key(
        stage,
        wall_path,
        mode,
        wall_collection_root_path,
        target_world_xyz,
        target_context_bbox,
        wall_candidate_region,
    )
    cached = cache.get(cache_key)
    if isinstance(cached, dict) and (request_cache or _wall_pool_cache_valid(stage, cached)):
        pool = _apply_first_layer_filter_to_pool(cached, target_context_bbox, target_world_xyz=target_world_xyz)
        pool["cache_hit"] = True
        pool["cache_signature"] = _candidate_pool_signature(pool)
        return pool

    sample_candidate = _build_wall_mount_candidate(stage, stage.GetPrimAtPath(wall_path), 0)
    semantic_seed = _resolve_wall_semantic_seed(stage, wall_path)
    semantic_seed_prim = stage.GetPrimAtPath(semantic_seed['prim_path'])
    fallback_reason = None
    global_root_resolve_tag = None

    if mode == "global_wall_candidates":
        collection_root, global_root_resolve_tag = _resolve_global_wall_collection_root_from_mesh267(stage)
        if not collection_root or not collection_root.IsValid():
            diag = (
                f"global_wall_candidates:root_unresolved tag={global_root_resolve_tag} "
                f"expected_root={_GLOBAL_WALL_COLLECTION_ROOT_PATH}"
            )
            print(f"[wall-constraint] ERROR: {diag}")
            return {
                'wall_collection_mode': mode,
                'sample_candidate': sample_candidate,
                'semantic_seed': semantic_seed,
                'seed_filter_applied': False,
                'fallback_reason': diag,
                'fallback_used': False,
                'global_root_resolve_tag': global_root_resolve_tag,
                'resolved_global_wall_root': None,
                'wall_candidates_total_scanned': 0,
                'wall_candidates_after_valid_filter': 0,
                'wall_candidates_after_mode_filter': 0,
                'wall_candidates_excluded_by_prefix': 0,
                'scanned_mesh_count': 0,
                'wall_semantic_candidate_count': 0,
                'valid_wall_candidate_count': 0,
                'final_candidate_count': 0,
                'collection_root': _GLOBAL_WALL_COLLECTION_ROOT_PATH,
                'excluded_wall_candidate_prefixes': list(_GLOBAL_WALL_COLLECTION_EXCLUDED_PREFIXES),
                'candidates': [],
                'common_height_min': None,
                'common_height_max': None,
                'wall_candidate_region_filter': {
                    "applied": False,
                    "fallback_reason": "global_wall_root_unresolved",
                },
            }
        fallback_reason = None
    elif mode == "explicit_root_local":
        requested_root = str(wall_collection_root_path or "").strip()
        if requested_root:
            collection_root = stage.GetPrimAtPath(requested_root)
            if not collection_root or not collection_root.IsValid():
                fallback_reason = f"invalid_explicit_root:{requested_root}"
                collection_root = semantic_seed_prim.GetParent()
        else:
            fallback_reason = "missing_explicit_root_path"
            collection_root = semantic_seed_prim.GetParent()
    else:
        collection_root = semantic_seed_prim.GetParent()

    if not collection_root or not collection_root.IsValid():
        collection_root = semantic_seed_prim

    sample_center = _candidate_center(sample_candidate)
    sample_height = max(sample_candidate['height_span'], _WALL_HEIGHT_EPS)
    sample_length = max(sample_candidate['length_span'], _WALL_HEIGHT_EPS)
    sample_thickness = max(sample_candidate['thickness_span'], _WALL_HEIGHT_EPS)
    target_xy = None
    try:
        if target_world_xyz is not None:
            target_xy = (float(target_world_xyz[0]), float(target_world_xyz[1]))
    except Exception:
        target_xy = None
    has_first_layer_context = mode == "global_wall_candidates" and _coerce_world_bbox(target_context_bbox) is not None
    seed_filter_applied = not (mode == "global_wall_candidates" and (target_xy is not None or has_first_layer_context))
    candidates_raw = []
    total_scanned = 0
    scanned_mesh_count = 0
    valid_filter_count = 0
    mode_filter_count = 0
    excluded_by_prefix_count = 0

    for prim in Usd.PrimRange(collection_root):
        if prim == collection_root:
            continue
        prim_path = prim.GetPath().pathString
        if mode == "global_wall_candidates" and any(
            _prim_path_matches_prefix(prim_path, prefix)
            for prefix in _GLOBAL_WALL_COLLECTION_EXCLUDED_PREFIXES
        ):
            excluded_by_prefix_count += 1
            continue
        total_scanned += 1
        if prim.IsA(UsdGeom.Mesh):
            scanned_mesh_count += 1
        try:
            candidate = _build_wall_mount_candidate(
                stage,
                prim,
                _path_depth(prim.GetPath().pathString) - _path_depth(wall_path),
            )
        except RuntimeError as exc:
            if "empty world bbox" in str(exc):
                continue
            raise
        if any(float(span) <= _WALL_HEIGHT_EPS for span in candidate['spans']):
            continue
        if candidate['aggregate_name'] or candidate['forbidden_aggregate']:
            continue
        if not _candidate_has_mesh_descendants(stage, candidate['prim_path']) and not candidate['is_mesh']:
            continue
        # global：在 JiKeng_BeiJing 子树内用关键词或 wall_like；墙高下限与配置种子墙对齐（模型更新后 bbox 可能 <2m，仍属同语义围墙面）。
        if mode == "global_wall_candidates":
            seed_h = float(sample_candidate['height_span'])
            wall_like_min_h = min(
                float(_WALL_INSTALL_BAND_MIN_HEIGHT),
                max(1.2, seed_h * 0.95),
            )
            relaxed_wall_like = (
                candidate['height_span'] >= wall_like_min_h
                and candidate['length_span'] >= 1.0
                and candidate['wall_ratio'] >= 1.6
                and candidate['thickness_span'] <= max(4.0, candidate['length_span'] * 0.35)
            )
            if not (candidate['has_wall_keyword'] or candidate['is_wall_like'] or relaxed_wall_like):
                continue
        else:
            if not candidate['has_wall_keyword']:
                continue
        valid_filter_count += 1
        center = _candidate_center(candidate)
        horizontal_dist = math.hypot(center[0] - sample_center[0], center[1] - sample_center[1])
        target_horizontal_dist = None
        if target_xy is not None:
            target_horizontal_dist = math.hypot(center[0] - target_xy[0], center[1] - target_xy[1])
        if seed_filter_applied:
            if horizontal_dist > _WALL_MOUNT_NEIGHBOR_RADIUS:
                continue
            if abs(center[2] - sample_center[2]) > max(2.5, sample_height * 2.0):
                continue
            if candidate['height_span'] < sample_height * 0.45 or candidate['height_span'] > sample_height * 2.8:
                continue
            if candidate['length_span'] < sample_length * 0.4 or candidate['length_span'] > max(sample_length * 8.0, 12.0):
                continue
            if candidate['thickness_span'] > max(sample_thickness * 24.0, 0.4):
                continue
        elif target_horizontal_dist is not None and not has_first_layer_context:
            if target_horizontal_dist > _WALL_MOUNT_TARGET_RADIUS:
                continue
        mode_filter_count += 1
        candidate['horizontal_distance_to_sample'] = float(horizontal_dist if seed_filter_applied else 0.0)
        candidate['horizontal_distance_to_target'] = None if target_horizontal_dist is None else float(target_horizontal_dist)
        candidate['collection_root'] = collection_root.GetPath().pathString
        candidate['semantic_seed_path'] = semantic_seed['prim_path']
        candidate['sample_wall_path'] = wall_path
        if seed_filter_applied:
            candidate['score'] = float(candidate['score'] - horizontal_dist * 0.06)
        elif target_horizontal_dist is not None:
            candidate['score'] = float(candidate['score'] - target_horizontal_dist * 0.015)
        candidates_raw.append(candidate)

    candidates_raw, wall_candidate_region_filter = _filter_wall_candidate_region(
        candidates_raw,
        wall_candidate_region,
    )
    candidates, valid_wall_candidate_count = _dedup_and_sort_wall_candidates(candidates_raw)
    first_layer_filter = {
        "applied": False,
        "before_count": int(len(candidates_raw)),
        "after_count": int(len(candidates_raw)),
    }
    base_candidates = copy.deepcopy(candidates_raw if mode == "global_wall_candidates" else candidates)
    base_components = _connected_wall_components(base_candidates) if mode == "global_wall_candidates" else None
    if mode == "global_wall_candidates":
        candidates_raw, first_layer_filter = _filter_first_layer_wall_candidates(
            copy.deepcopy(base_candidates),
            target_context_bbox,
            components=base_components,
        )
        candidates, valid_wall_candidate_count = _dedup_and_sort_wall_candidates(candidates_raw)
        first_layer_filter["after_dedup_count"] = int(valid_wall_candidate_count)

    fallback_used = False
    if not candidates:
        if mode == "global_wall_candidates":
            empty_detail = (
                f"global_wall_candidates_empty:resolved_root={collection_root.GetPath().pathString};"
                f"prim_scanned={total_scanned};mesh_scanned={scanned_mesh_count};"
                f"excluded_prefixes={list(_GLOBAL_WALL_COLLECTION_EXCLUDED_PREFIXES)};"
                f"excluded_by_prefix={excluded_by_prefix_count};"
                f"wall_semantic={valid_filter_count};after_mode_filter={mode_filter_count};"
                f"dedup_unique={valid_wall_candidate_count};no_jikeng_beijing_fallback"
            )
            print(f"[wall-constraint] ERROR: {empty_detail}")
            fallback_reason = empty_detail
            fallback_used = False
        else:
            print(
                f"[wall-constraint] WARN: empty_wall_candidates mode={mode} "
                "-> fallback_candidates=[semantic_seed]"
            )
            candidates = [semantic_seed]
            prefix = f"{fallback_reason};" if fallback_reason else ""
            fallback_reason = prefix + "empty_pool_fallback_semantic_seed"
            fallback_used = True

    common_height_min, common_height_max = _wall_candidate_common_height_range(candidates)

    resolved_global_wall_root = None
    if mode == "global_wall_candidates" and collection_root and collection_root.IsValid():
        resolved_global_wall_root = collection_root.GetPath().pathString

    pool = {
        'wall_collection_mode': mode,
        'sample_candidate': sample_candidate,
        'semantic_seed': semantic_seed,
        'seed_filter_applied': bool(seed_filter_applied),
        'target_filter_applied': bool((not seed_filter_applied) and target_xy is not None),
        'target_filter_radius': float(_WALL_MOUNT_TARGET_RADIUS),
        'wall_candidate_region_filter': wall_candidate_region_filter,
        'first_layer_filter': first_layer_filter,
        'fallback_reason': fallback_reason,
        'fallback_used': bool(fallback_used),
        'global_root_resolve_tag': global_root_resolve_tag,
        'resolved_global_wall_root': resolved_global_wall_root,
        'wall_candidates_total_scanned': int(total_scanned),
        'wall_candidates_after_valid_filter': int(valid_filter_count),
        'wall_candidates_after_mode_filter': int(mode_filter_count),
        'wall_candidates_excluded_by_prefix': int(excluded_by_prefix_count),
        'scanned_mesh_count': int(scanned_mesh_count),
        'wall_semantic_candidate_count': int(valid_filter_count),
        'valid_wall_candidate_count': int(valid_wall_candidate_count),
        'final_candidate_count': int(len(candidates)),
        'collection_root': collection_root.GetPath().pathString,
        'excluded_wall_candidate_prefixes': list(_GLOBAL_WALL_COLLECTION_EXCLUDED_PREFIXES)
            if mode == "global_wall_candidates" else [],
        'candidates': candidates,
        '_base_candidates': base_candidates,
        'common_height_min': None if common_height_min is None else float(common_height_min),
        'common_height_max': None if common_height_max is None else float(common_height_max),
        'cache_hit': False,
        '_stage_cache_key': _stage_cache_key(stage),
        '_sample_center': sample_center,
        '_seed_filter_applied_for_base': bool(seed_filter_applied),
        '_base_components': base_components,
    }
    pool['_collection_root_signature'] = None
    pool['cache_signature'] = _candidate_pool_signature(pool)
    if request_cache or pool.get('_stage_cache_key'):
        cache[cache_key] = copy.deepcopy(pool)
    return pool


def _compute_wall_mount_validation(camera_world_xyz, install_surface):
    raw_min = install_surface['raw_min']
    raw_max = install_surface['raw_max']
    length_axis = int(install_surface['length_axis'])
    thickness_axis = int(install_surface['thickness_axis'])

    surface_distance = abs(float(camera_world_xyz[thickness_axis]) - float(install_surface['surface_plane_coord']))
    inside_wall_bbox = all(
        (raw_min[i] - _WALL_HEIGHT_EPS) <= float(camera_world_xyz[i]) <= (raw_max[i] + _WALL_HEIGHT_EPS)
        for i in range(3)
    )
    within_height_band = (
        install_surface['height_min'] - _WALL_HEIGHT_EPS
        <= float(camera_world_xyz[2])
        <= install_surface['height_max'] + _WALL_HEIGHT_EPS
    )
    within_length_band = (
        install_surface['length_min'] - _WALL_HEIGHT_EPS
        <= float(camera_world_xyz[length_axis])
        <= install_surface['length_max'] + _WALL_HEIGHT_EPS
    )
    near_top_plane = (float(raw_max[2]) - float(camera_world_xyz[2])) <= install_surface['top_proximity_max']
    on_selected_face_side = (
        float(camera_world_xyz[thickness_axis]) >= float(raw_max[thickness_axis]) - _WALL_HEIGHT_EPS
        if install_surface['surface_side_sign'] > 0
        else float(camera_world_xyz[thickness_axis]) <= float(raw_min[thickness_axis]) + _WALL_HEIGHT_EPS
    )
    near_wall_face = (
        surface_distance <= install_surface['surface_distance_max']
        and within_length_band
        and within_height_band
        and on_selected_face_side
    )
    mounted_on_wall = (
        not install_surface['selected_mount_is_aggregate']
        and near_wall_face
        and not inside_wall_bbox
        and not near_top_plane
    )
    return {
        'distance_to_wall_surface': float(surface_distance),
        'inside_wall_bbox': bool(inside_wall_bbox),
        'within_length_band': bool(within_length_band),
        'within_height_band': bool(within_height_band),
        'near_top_plane': bool(near_top_plane),
        'near_wall_face': bool(near_wall_face),
        'on_selected_face_side': bool(on_selected_face_side),
        'selected_mount_is_aggregate': bool(install_surface['selected_mount_is_aggregate']),
        'mounted_on_wall': bool(mounted_on_wall),
    }


def _compute_inset_wall_mount_validation(camera_world_xyz, install_surface, inset_meta):
    if not bool(inset_meta.get("enabled")):
        return False, {
            "inset_actual_distance_to_wall_surface": None,
            "inset_distance_error": None,
            "on_inset_side": False,
        }
    thickness_axis = int(install_surface["thickness_axis"])
    surface_side_sign = int(install_surface.get("surface_side_sign", 1) or 1)
    surface_plane_coord = float(install_surface["surface_plane_coord"])
    actual_coord = float(camera_world_xyz[thickness_axis])
    actual_distance = abs(actual_coord - surface_plane_coord)
    expected_distance = float(inset_meta.get("wall_mount_inset_m") or 0.0)
    tolerance = max(0.02, expected_distance * 0.08)
    on_inset_side = (
        actual_coord >= surface_plane_coord + _WALL_HEIGHT_EPS
        if surface_side_sign > 0
        else actual_coord <= surface_plane_coord - _WALL_HEIGHT_EPS
    )
    distance_ok = abs(actual_distance - expected_distance) <= tolerance
    valid = bool(on_inset_side and distance_ok)
    return valid, {
        "inset_actual_distance_to_wall_surface": float(actual_distance),
        "inset_distance_error": float(actual_distance - expected_distance),
        "inset_distance_tolerance": float(tolerance),
        "on_inset_side": bool(on_inset_side),
    }


def _select_wall_mount_reference(stage, wall_path, rng, wall_collection_mode=None, wall_collection_root_path=None, wall_pool_cache=None, target_world_xyz=None, target_context_bbox=None, wall_candidate_region=None):
    wall_pool = _collect_wall_mount_candidates(
        stage,
        wall_path,
        wall_collection_mode=wall_collection_mode,
        wall_collection_root_path=wall_collection_root_path,
        wall_pool_cache=wall_pool_cache,
        target_world_xyz=target_world_xyz,
        target_context_bbox=target_context_bbox,
        wall_candidate_region=wall_candidate_region,
    )
    candidates = wall_pool['candidates']
    print(
        "[wall-constraint] wall_candidate_pool="
        + ("; ".join(_format_mount_candidate(c) for c in candidates) if candidates else "<empty>")
    )
    ch_range = None
    if wall_pool.get('common_height_min') is not None and wall_pool.get('common_height_max') is not None:
        ch_range = [
            round(float(wall_pool['common_height_min']), 4),
            round(float(wall_pool['common_height_max']), 4),
        ]
    print(
        "[wall-constraint] wall_candidate_pool_meta="
        f"sample_wall_path={wall_path} "
        f"wall_collection_mode={wall_pool.get('wall_collection_mode')} "
        f"resolved_global_wall_root={wall_pool.get('resolved_global_wall_root')} "
        f"global_root_resolve_tag={wall_pool.get('global_root_resolve_tag')} "
        f"semantic_seed={wall_pool['semantic_seed']['prim_path']} "
        f"collection_root={wall_pool['collection_root']} "
        f"seed_filter_applied={wall_pool.get('seed_filter_applied')} "
        f"target_filter_applied={wall_pool.get('target_filter_applied')} "
        f"target_filter_radius={wall_pool.get('target_filter_radius')} "
        f"wall_candidate_region_filter={wall_pool.get('wall_candidate_region_filter')} "
        f"first_layer_filter={wall_pool.get('first_layer_filter')} "
        f"wall_candidates_total_scanned={wall_pool.get('wall_candidates_total_scanned')} "
        f"scanned_mesh_count={wall_pool.get('scanned_mesh_count')} "
        f"wall_semantic_candidate_count={wall_pool.get('wall_semantic_candidate_count')} "
        f"valid_wall_candidate_count={wall_pool.get('valid_wall_candidate_count')} "
        f"final_candidate_count={wall_pool.get('final_candidate_count')} "
        f"wall_candidates_after_valid_filter={wall_pool.get('wall_candidates_after_valid_filter')} "
        f"wall_candidates_after_mode_filter={wall_pool.get('wall_candidates_after_mode_filter')} "
        f"wall_candidates_excluded_by_prefix={wall_pool.get('wall_candidates_excluded_by_prefix')} "
        f"excluded_wall_candidate_prefixes={wall_pool.get('excluded_wall_candidate_prefixes')} "
        f"candidate_count={len(candidates)} "
        f"cache_hit={wall_pool.get('cache_hit')} "
        f"fallback_used={wall_pool.get('fallback_used')} "
        f"fallback_reason={wall_pool.get('fallback_reason')} "
        f"common_height_range={ch_range}"
    )
    if not candidates:
        raise RuntimeError(
            "failed to resolve wall mount reference: "
            f"sample_wall_path={wall_path} wall_collection_mode={wall_pool.get('wall_collection_mode')} "
            f"resolved_global_wall_root={wall_pool.get('resolved_global_wall_root')} "
            f"collection_root={wall_pool.get('collection_root')} "
            f"scanned_mesh_count={wall_pool.get('scanned_mesh_count')} "
            f"wall_semantic_candidate_count={wall_pool.get('wall_semantic_candidate_count')} "
            f"valid_wall_candidate_count={wall_pool.get('valid_wall_candidate_count')} "
            f"final_candidate_count={wall_pool.get('final_candidate_count')} "
            f"wall_candidates_excluded_by_prefix={wall_pool.get('wall_candidates_excluded_by_prefix')} "
            f"excluded_wall_candidate_prefixes={wall_pool.get('excluded_wall_candidate_prefixes')} "
            f"fallback_used={wall_pool.get('fallback_used')} "
            f"fallback_reason={wall_pool.get('fallback_reason')}"
        )
    chosen = rng.choice(candidates)
    print(f"[wall-constraint] chosen_mount_reference={_format_mount_candidate(chosen)}")
    return chosen, wall_pool


def _build_wall_install_band(selected_wall, xy_margin, z_margin, target_world_xyz=None, common_height_range=None):
    xy_margin = _normalize_margin_value(
        xy_margin, _WALL_CONSTRAINT_XY_MARGIN_DEFAULT, "wall_constraint_xy_margin"
    )
    z_margin = _normalize_margin_value(
        z_margin, _WALL_CONSTRAINT_Z_MARGIN_DEFAULT, "wall_constraint_z_margin"
    )

    raw_min = list(selected_wall['min'])
    raw_max = list(selected_wall['max'])
    spans = tuple(float(raw_max[i] - raw_min[i]) for i in range(3))
    if any(span <= _WALL_HEIGHT_EPS for span in spans):
        raise RuntimeError(
            f"selected wall bbox has degenerate span: path={selected_wall['prim_path']} spans={spans}"
        )

    thickness_axis = int(selected_wall['thickness_axis'])
    length_axis = int(selected_wall['length_axis'])
    center = [float((raw_min[i] + raw_max[i]) * 0.5) for i in range(3)]
    if target_world_xyz is not None and all(math.isfinite(float(v)) for v in target_world_xyz):
        surface_side_sign = 1 if float(target_world_xyz[thickness_axis]) >= center[thickness_axis] else -1
    else:
        surface_side_sign = 1
    surface_plane_coord = raw_max[thickness_axis] if surface_side_sign > 0 else raw_min[thickness_axis]
    surface_offset = min(
        _WALL_MOUNT_SURFACE_DISTANCE_MAX * 0.75,
        max(_WALL_MOUNT_SURFACE_OFFSET, xy_margin + 0.02),
    )

    length_margin = min(max(xy_margin, 0.05), max(0.0, spans[length_axis] * 0.12))
    length_min = raw_min[length_axis] + length_margin
    length_max = raw_max[length_axis] - length_margin
    if length_max - length_min <= _WALL_HEIGHT_EPS:
        length_min = raw_min[length_axis]
        length_max = raw_max[length_axis]

    bottom_clearance = max(z_margin, min(_WALL_MOUNT_MIN_BOTTOM_CLEARANCE, spans[2] * 0.15))
    top_clearance = max(_WALL_MOUNT_MIN_TOP_CLEARANCE, min(1.4, spans[2] * 0.28))
    height_min = raw_min[2] + bottom_clearance
    height_max = raw_max[2] - top_clearance
    if common_height_range is not None:
        common_min, common_max = common_height_range
        if common_min is not None:
            height_min = max(height_min, float(common_min))
        if common_max is not None:
            height_max = min(height_max, float(common_max))
    if height_max - height_min <= _WALL_HEIGHT_EPS:
        fallback_bottom = raw_min[2] + z_margin
        fallback_top = raw_max[2] - max(z_margin, min(0.4, spans[2] * 0.12))
        height_min = fallback_bottom
        height_max = fallback_top
    if height_max - height_min <= _WALL_HEIGHT_EPS:
        raise RuntimeError(
            "wall install surface height range invalid after wall-side clamp: "
            f"path={selected_wall['prim_path']} raw_min={raw_min} raw_max={raw_max} "
            f"height_min={height_min} height_max={height_max}"
        )
    eff_min = [raw_min[0], raw_min[1], height_min]
    eff_max = [raw_max[0], raw_max[1], height_max]
    if surface_side_sign > 0:
        eff_min[thickness_axis] = surface_plane_coord
        eff_max[thickness_axis] = surface_plane_coord + surface_offset
    else:
        eff_min[thickness_axis] = surface_plane_coord - surface_offset
        eff_max[thickness_axis] = surface_plane_coord
    eff_min[length_axis] = length_min
    eff_max[length_axis] = length_max
    effective_spans = tuple(float(eff_max[i] - eff_min[i]) for i in range(3))

    return {
        'prim_path': selected_wall['prim_path'],
        'raw_min': tuple(float(v) for v in raw_min),
        'raw_max': tuple(float(v) for v in raw_max),
        'effective_min': tuple(float(v) for v in eff_min),
        'effective_max': tuple(float(v) for v in eff_max),
        'raw_spans': spans,
        'effective_spans': effective_spans,
        'thickness_axis': thickness_axis,
        'length_axis': length_axis,
        'surface_axis': thickness_axis,
        'surface_side_sign': int(surface_side_sign),
        'surface_plane_coord': float(surface_plane_coord),
        'surface_offset': float(surface_offset),
        'length_min': float(length_min),
        'length_max': float(length_max),
        'height_min': float(height_min),
        'height_max': float(height_max),
        'surface_distance_max': float(_WALL_MOUNT_SURFACE_DISTANCE_MAX),
        'top_proximity_max': float(_WALL_MOUNT_TOP_PROXIMITY_MAX),
        'selected_mount_is_aggregate': bool(selected_wall['aggregate_name']),
        'xy_margin': float(xy_margin),
        'z_margin': float(z_margin),
    }


def _sample_point_in_wall_install_band(rng, install_band, target_world_xyz=None):
    box_min = install_band['effective_min']
    box_max = install_band['effective_max']
    length_axis = int(install_band['length_axis'])
    thickness_axis = int(install_band['thickness_axis'])

    coords = [0.0, 0.0, 0.0]
    notes = []
    for axis in range(3):
        if axis == thickness_axis:
            continue
        if axis == 2:
            if target_world_xyz is not None:
                target_z = float(target_world_xyz[axis])
                span = max(_WALL_HEIGHT_EPS, box_max[axis] - box_min[axis])
                edge_span = max(0.18, span * 0.3)
                if target_z >= box_max[axis]:
                    lo = max(box_min[axis], box_max[axis] - edge_span)
                    hi = box_max[axis]
                    coords[axis] = rng.uniform(lo, hi)
                    notes.append(
                        f"target_high_bias_axis={axis} range=[{lo:.4f},{hi:.4f}]"
                    )
                    continue
                if target_z <= box_min[axis]:
                    lo = box_min[axis]
                    hi = min(box_max[axis], box_min[axis] + edge_span)
                    coords[axis] = rng.uniform(lo, hi)
                    notes.append(
                        f"target_low_bias_axis={axis} range=[{lo:.4f},{hi:.4f}]"
                    )
                    continue
            coords[axis] = rng.uniform(box_min[axis], box_max[axis])
            continue
        if axis == length_axis and target_world_xyz is not None:
            target_axis = float(target_world_xyz[axis])
            span = max(_WALL_HEIGHT_EPS, box_max[axis] - box_min[axis])
            # 原 max(0.35, span*0.16) 在「目标在墙面中段」时把采样压到不足一米，连续 randomize 肉眼几乎不变。
            edge_span = max(1.25, span * 0.32)
            if target_axis <= box_min[axis]:
                lo = box_min[axis]
                hi = min(box_max[axis], box_min[axis] + edge_span)
                coords[axis] = rng.uniform(lo, hi)
                notes.append(
                    f"target_edge_bias_axis={axis} side=min range=[{lo:.4f},{hi:.4f}]"
                )
                continue
            if target_axis >= box_max[axis]:
                lo = max(box_min[axis], box_max[axis] - edge_span)
                hi = box_max[axis]
                coords[axis] = rng.uniform(lo, hi)
                notes.append(
                    f"target_edge_bias_axis={axis} side=max range=[{lo:.4f},{hi:.4f}]"
                )
                continue
            center = max(box_min[axis], min(box_max[axis], target_axis))
            half_span = max(2.0, min(span * 0.42, span * 0.5 - _WALL_HEIGHT_EPS))
            lo = max(box_min[axis], center - half_span)
            hi = min(box_max[axis], center + half_span)
            if hi - lo > _WALL_HEIGHT_EPS:
                coords[axis] = rng.uniform(lo, hi)
                notes.append(
                    f"target_bias_axis={axis} center={center:.4f} range=[{lo:.4f},{hi:.4f}]"
                )
                continue
        coords[axis] = rng.uniform(box_min[axis], box_max[axis])

    surface_offset = abs(float(install_band.get('surface_offset', 0.0) or 0.0))
    if surface_offset > _WALL_HEIGHT_EPS:
        min_surface_clearance = min(surface_offset, max(0.03, surface_offset * 0.45))
        if int(install_band.get('surface_side_sign', 1)) > 0:
            lo = min(
                box_max[thickness_axis],
                float(install_band['surface_plane_coord']) + min_surface_clearance,
            )
            hi = box_max[thickness_axis]
        else:
            lo = box_min[thickness_axis]
            hi = max(
                box_min[thickness_axis],
                float(install_band['surface_plane_coord']) - min_surface_clearance,
            )
        if hi - lo > _WALL_HEIGHT_EPS:
            coords[thickness_axis] = rng.uniform(lo, hi)
            notes.append(
                f"surface_clearance_axis={thickness_axis} min={min_surface_clearance:.4f} range=[{lo:.4f},{hi:.4f}]"
            )
        else:
            coords[thickness_axis] = rng.uniform(box_min[thickness_axis], box_max[thickness_axis])
    else:
        coords[thickness_axis] = rng.uniform(box_min[thickness_axis], box_max[thickness_axis])
    return tuple(float(v) for v in coords), notes


def _normalize_wall_mount_inset_mode(raw_mode):
    mode = str(raw_mode or "").strip().lower()
    if mode in ("", "disabled", "off", "strict_wall_surface_attach"):
        return ""
    if mode == _WALL_MOUNT_INSET_MODE_INWARD:
        return mode
    print(
        f"[wall-constraint] WARN: invalid wall_mount_inset_mode={raw_mode!r}; "
        "fallback=strict_wall_surface_attach"
    )
    return ""


def _apply_inward_wall_mount_inset(sampled_world_xyz, install_band, wall_mount_inset_m, wall_mount_inset_mode):
    inset_m = _normalize_margin_value(
        wall_mount_inset_m, _WALL_MOUNT_INSET_M_DEFAULT, "wall_mount_inset_m"
    )
    mode = _normalize_wall_mount_inset_mode(wall_mount_inset_mode)
    meta = {
        "enabled": bool(mode == _WALL_MOUNT_INSET_MODE_INWARD and inset_m > _WALL_HEIGHT_EPS),
        "mode": mode,
        "wall_mount_inset_m": float(inset_m),
        "inset_direction_axis": None,
        "inset_direction_sign": None,
        "inset_wall_mount_valid": False,
        "fallback_used": False,
        "fallback_reason": None,
    }
    if not meta["enabled"]:
        meta["fallback_reason"] = "wall_mount_inset_disabled"
        return sampled_world_xyz, meta

    coords = [float(v) for v in sampled_world_xyz]
    thickness_axis = int(install_band["thickness_axis"])
    raw_min = install_band["raw_min"]
    raw_max = install_band["raw_max"]
    surface_side_sign = int(install_band.get("surface_side_sign", 1) or 1)
    inward_sign = 1 if surface_side_sign > 0 else -1
    surface_plane_coord = float(install_band["surface_plane_coord"])
    inset_coord = surface_plane_coord + inward_sign * inset_m

    inside_wall_thickness = (
        float(raw_min[thickness_axis]) + _WALL_HEIGHT_EPS
        < inset_coord
        < float(raw_max[thickness_axis]) - _WALL_HEIGHT_EPS
    )
    if inside_wall_thickness:
        meta["enabled"] = False
        meta["fallback_used"] = True
        meta["fallback_reason"] = "inset_coord_inside_wall_bbox"
        return sampled_world_xyz, meta

    coords[thickness_axis] = float(inset_coord)
    meta.update({
        "inset_direction_axis": int(thickness_axis),
        "inset_direction_sign": int(inward_sign),
        "inset_wall_mount_valid": True,
    })
    return tuple(float(v) for v in coords), meta


def sample_camera_in_changjing(
    stage,
    changjing_path,
    camera_rig_path,
    rng,
    seed=None,
    cam_z_min=5.0,
    cam_z_max=25.0,
    max_retries=20,
    sample_box=None,
    wall_prim_path=_WALL_HEIGHT_PRIM_DEFAULT,
    wall_constraint_xy_margin=_WALL_CONSTRAINT_XY_MARGIN_DEFAULT,
    wall_constraint_z_margin=_WALL_CONSTRAINT_Z_MARGIN_DEFAULT,
    wall_mount_inset_m=_WALL_MOUNT_INSET_M_DEFAULT,
    wall_mount_inset_mode="",
    target_world_xyz=None,
    wall_collection_mode=None,
    wall_collection_root_path=None,
    wall_pool_cache=None,
    target_context_bbox=None,
    wall_candidate_region=None,
):
    """
    ???????????????????????????
    ?? Mesh ?????????????????????????????????
    ???????????????
    """
    del max_retries, cam_z_min, cam_z_max

    aabb = compute_changjing_aabb(stage, changjing_path)
    wall_path = str(wall_prim_path or _WALL_HEIGHT_PRIM_DEFAULT).strip()

    selected_wall, wall_pool = _select_wall_mount_reference(
        stage,
        wall_path,
        rng,
        wall_collection_mode=wall_collection_mode,
        wall_collection_root_path=wall_collection_root_path,
        wall_pool_cache=wall_pool_cache,
        target_world_xyz=target_world_xyz,
        target_context_bbox=target_context_bbox,
        wall_candidate_region=wall_candidate_region,
    )
    sampled_wall_path = selected_wall['prim_path']
    _swp = stage.GetPrimAtPath(sampled_wall_path)
    sampled_parent_path = (
        _swp.GetParent().GetPath().pathString
        if _swp and _swp.IsValid() and _swp.GetParent() and _swp.GetParent().IsValid()
        else None
    )
    print(
        "[wall-constraint] wall_sample_diag "
        f"wall_collection_mode={wall_pool.get('wall_collection_mode')} "
        f"resolved_global_wall_root={wall_pool.get('resolved_global_wall_root')} "
        f"scanned_mesh_count={wall_pool.get('scanned_mesh_count')} "
        f"wall_semantic_candidate_count={wall_pool.get('wall_semantic_candidate_count')} "
        f"valid_wall_candidate_count={wall_pool.get('valid_wall_candidate_count')} "
        f"final_candidate_count={wall_pool.get('final_candidate_count')} "
        f"wall_candidates_excluded_by_prefix={wall_pool.get('wall_candidates_excluded_by_prefix')} "
        f"excluded_wall_candidate_prefixes={wall_pool.get('excluded_wall_candidate_prefixes')} "
        f"wall_candidate_region_filter={wall_pool.get('wall_candidate_region_filter')} "
        f"first_layer_filter={wall_pool.get('first_layer_filter')} "
        f"cache_hit={wall_pool.get('cache_hit')} "
        f"fallback_used={wall_pool.get('fallback_used')} "
        f"fallback_reason={wall_pool.get('fallback_reason')} "
        f"sampled_wall_path={sampled_wall_path} sampled_parent_path={sampled_parent_path}"
    )
    install_band = _build_wall_install_band(
        selected_wall,
        wall_constraint_xy_margin,
        wall_constraint_z_margin,
        target_world_xyz=target_world_xyz,
        common_height_range=(wall_pool['common_height_min'], wall_pool['common_height_max']),
    )
    box_min = install_band['effective_min']
    box_max = install_band['effective_max']

    print(f"[wall-constraint] requested_prim_path={wall_path}")
    print(
        f"[wall-constraint] selected_mount_prim={selected_wall['prim_path']} "
        f"raw_world_bbox_min={install_band['raw_min']} raw_world_bbox_max={install_band['raw_max']}"
    )
    print(
        f"[wall-constraint] install_surface_bbox_min={box_min} install_surface_bbox_max={box_max} "
        f"surface_axis={install_band['surface_axis']} surface_side_sign={install_band['surface_side_sign']} "
        f"surface_plane_coord={install_band['surface_plane_coord']:.4f} "
        f"surface_offset={install_band['surface_offset']:.4f} "
        f"xy_margin={wall_constraint_xy_margin} z_margin={wall_constraint_z_margin} "
        f"sample_box_ignored={bool(sample_box)}"
    )

    sampled_world_xyz, bias_notes = _sample_point_in_wall_install_band(
        rng,
        install_band,
        target_world_xyz=target_world_xyz,
    )
    sampled_world_xyz, inset_meta = _apply_inward_wall_mount_inset(
        sampled_world_xyz,
        install_band,
        wall_mount_inset_m,
        wall_mount_inset_mode,
    )
    inset_enabled = bool(inset_meta.get("enabled"))
    constraint_source = "wall_surface_inward_inset" if inset_enabled else "wall_surface_attach"
    constraint_mode = "inward_wall_inset_mount" if inset_enabled else "strict_wall_surface_attach"
    if inset_enabled:
        bias_notes.append(
            "inward_wall_inset_axis="
            f"{inset_meta.get('inset_direction_axis')} sign={inset_meta.get('inset_direction_sign')} "
            f"distance={float(inset_meta.get('wall_mount_inset_m') or 0.0):.4f}"
        )
    elif inset_meta.get("fallback_reason") not in (None, "wall_mount_inset_disabled"):
        bias_notes.append(
            f"inward_wall_inset_fallback={inset_meta.get('fallback_reason')}"
        )
    reported_box_min = list(box_min)
    reported_box_max = list(box_max)
    if inset_enabled and inset_meta.get("inset_direction_axis") is not None:
        inset_axis = int(inset_meta["inset_direction_axis"])
        reported_box_min[inset_axis] = float(sampled_world_xyz[inset_axis])
        reported_box_max[inset_axis] = float(sampled_world_xyz[inset_axis])

    box_meta = {
        "mode": constraint_mode,
        "constraint_source": constraint_source,
        "fallback_invalid": False,
        "sample_wall_path": wall_path,
        "sampled_wall_path": sampled_wall_path,
        "sampled_parent_path": sampled_parent_path,
        "semantic_seed_path": wall_pool['semantic_seed']['prim_path'],
        "seed_wall_path": wall_path,
        "wall_collection_mode": wall_pool.get('wall_collection_mode'),
        "wall_collection_root": wall_pool.get('collection_root'),
        "resolved_global_wall_root": wall_pool.get('resolved_global_wall_root'),
        "global_root_resolve_tag": wall_pool.get('global_root_resolve_tag'),
        "seed_filter_applied": wall_pool.get('seed_filter_applied'),
        "target_filter_applied": wall_pool.get('target_filter_applied'),
        "target_filter_radius": wall_pool.get('target_filter_radius'),
        "wall_candidate_region_filter": wall_pool.get('wall_candidate_region_filter'),
        "first_layer_filter": wall_pool.get('first_layer_filter'),
        "wall_candidates_total_scanned": wall_pool.get('wall_candidates_total_scanned'),
        "scanned_mesh_count": wall_pool.get('scanned_mesh_count'),
        "wall_semantic_candidate_count": wall_pool.get('wall_semantic_candidate_count'),
        "valid_wall_candidate_count": wall_pool.get('valid_wall_candidate_count'),
        "final_candidate_count": wall_pool.get('final_candidate_count'),
        "fallback_used": wall_pool.get('fallback_used'),
        "cache_hit": wall_pool.get('cache_hit'),
        "cache_signature": wall_pool.get('cache_signature'),
        "wall_candidates_after_valid_filter": wall_pool.get('wall_candidates_after_valid_filter'),
        "wall_candidates_after_mode_filter": wall_pool.get('wall_candidates_after_mode_filter'),
        "wall_candidates_excluded_by_prefix": wall_pool.get('wall_candidates_excluded_by_prefix'),
        "excluded_wall_candidate_prefixes": wall_pool.get('excluded_wall_candidate_prefixes'),
        "fallback_reason": wall_pool.get('fallback_reason'),
        "collection_root": wall_pool['collection_root'],
        "wall_candidate_pool": [c['prim_path'] for c in wall_pool['candidates']],
        "wall_candidate_pool_size": len(wall_pool['candidates']),
        "wall_common_height_range": None if wall_pool['common_height_min'] is None or wall_pool['common_height_max'] is None else [
            wall_pool['common_height_min'],
            wall_pool['common_height_max'],
        ],
        "effective_box": {
            "x_min": reported_box_min[0],
            "x_max": reported_box_max[0],
            "y_min": reported_box_min[1],
            "y_max": reported_box_max[1],
            "z_min": reported_box_min[2],
            "z_max": reported_box_max[2],
        },
        "config_box": dict(sample_box) if sample_box else None,
        "changjing_aabb": {
            "xmin": aabb["xmin"],
            "xmax": aabb["xmax"],
            "ymin": aabb["ymin"],
            "ymax": aabb["ymax"],
            "zmin": aabb["zmin"],
            "zmax": aabb["zmax"],
        },
        "aabb_clip_notes": [
            f"strict_wall_only=PASS requested_prim={wall_path}",
            f"selected_mount_prim={selected_wall['prim_path']}",
            f"surface_axis={install_band['surface_axis']} surface_side_sign={install_band['surface_side_sign']}",
            "configured_sample_box_ignored_for_strict_wall_semantics"
                if sample_box else
            "configured_sample_box_absent",
            *bias_notes,
        ],
    }

    cam_prim = stage.GetPrimAtPath(camera_rig_path)
    if not cam_prim.IsValid():
        msg = f"camera rig prim invalid: {camera_rig_path}"
        print(f"[camera-sample] ERROR: {msg}")
        raise RuntimeError(msg)

    local_xyz = _set_world_translate_on_prim(stage, cam_prim, sampled_world_xyz)
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    camera_world = xform_cache.GetLocalToWorldTransform(cam_prim).ExtractTranslation()
    camera_world_xyz = _vec3_to_tuple(camera_world)
    validation = _compute_wall_mount_validation(camera_world_xyz, install_band)
    inset_valid, inset_validation = _compute_inset_wall_mount_validation(
        camera_world_xyz,
        install_band,
        inset_meta,
    )
    inset_meta["inset_wall_mount_valid"] = bool(inset_valid)
    if inset_enabled:
        within_constraint_box = bool(
            inset_valid
            and validation["within_length_band"]
            and validation["within_height_band"]
            and not validation["near_top_plane"]
            and not validation["selected_mount_is_aggregate"]
        )
    else:
        within_constraint_box = bool(validation["mounted_on_wall"])
    status = "PASS" if within_constraint_box else "FAIL"

    constraint_meta = {
        "camera_xyz": [round(float(v), 4) for v in camera_world_xyz],
        "camera_local_xyz": [round(float(v), 4) for v in local_xyz],
        "constraint_source": constraint_source,
        "constraint_mode": constraint_mode,
        "wall_path": wall_path,
        "sample_wall_path": wall_path,
        "sampled_wall_path": sampled_wall_path,
        "sampled_parent_path": sampled_parent_path,
        "selected_mount_prim": selected_wall['prim_path'],
        "selected_mount_group_hint": _group_hint_from_path(selected_wall['prim_path']),
        "seed_wall_path": wall_path,
        "wall_collection_mode": wall_pool.get('wall_collection_mode'),
        "wall_collection_root": wall_pool.get('collection_root'),
        "resolved_global_wall_root": wall_pool.get('resolved_global_wall_root'),
        "global_root_resolve_tag": wall_pool.get('global_root_resolve_tag'),
        "seed_filter_applied": wall_pool.get('seed_filter_applied'),
        "target_filter_applied": wall_pool.get('target_filter_applied'),
        "target_filter_radius": wall_pool.get('target_filter_radius'),
        "wall_candidate_region_filter": wall_pool.get('wall_candidate_region_filter'),
        "first_layer_filter": wall_pool.get('first_layer_filter'),
        "wall_candidates_total_scanned": wall_pool.get('wall_candidates_total_scanned'),
        "scanned_mesh_count": wall_pool.get('scanned_mesh_count'),
        "wall_semantic_candidate_count": wall_pool.get('wall_semantic_candidate_count'),
        "valid_wall_candidate_count": wall_pool.get('valid_wall_candidate_count'),
        "final_candidate_count": wall_pool.get('final_candidate_count'),
        "wall_candidates_after_valid_filter": wall_pool.get('wall_candidates_after_valid_filter'),
        "wall_candidates_after_mode_filter": wall_pool.get('wall_candidates_after_mode_filter'),
        "wall_candidates_excluded_by_prefix": wall_pool.get('wall_candidates_excluded_by_prefix'),
        "excluded_wall_candidate_prefixes": wall_pool.get('excluded_wall_candidate_prefixes'),
        "fallback_used": wall_pool.get('fallback_used'),
        "cache_hit": wall_pool.get('cache_hit'),
        "cache_signature": wall_pool.get('cache_signature'),
        "fallback_reason": wall_pool.get('fallback_reason'),
        "selected_mount_in_candidate_pool": bool(selected_wall['prim_path'] in {c['prim_path'] for c in wall_pool['candidates']}),
        "wall_candidate_pool_size": len(wall_pool['candidates']),
        "wall_bbox_min": [round(float(v), 4) for v in install_band['raw_min']],
        "wall_bbox_max": [round(float(v), 4) for v in install_band['raw_max']],
        "effective_bbox_min": [round(float(v), 4) for v in reported_box_min],
        "effective_bbox_max": [round(float(v), 4) for v in reported_box_max],
        "surface_axis": int(install_band['surface_axis']),
        "surface_side_sign": int(install_band['surface_side_sign']),
        "surface_plane_coord": round(float(install_band['surface_plane_coord']), 4),
        "surface_offset": round(float(install_band['surface_offset']), 4),
        "height_range": [round(float(install_band['height_min']), 4), round(float(install_band['height_max']), 4)],
        "wall_common_height_range": None if wall_pool['common_height_min'] is None or wall_pool['common_height_max'] is None else [
            round(float(wall_pool['common_height_min']), 4),
            round(float(wall_pool['common_height_max']), 4),
        ],
        "wall_constraint_xy_margin": round(float(wall_constraint_xy_margin), 4),
        "wall_constraint_z_margin": round(float(wall_constraint_z_margin), 4),
        "wall_mount_inset_m": round(float(inset_meta.get("wall_mount_inset_m") or 0.0), 4),
        "wall_mount_inset_mode": _normalize_wall_mount_inset_mode(wall_mount_inset_mode) or None,
        "inset_direction_axis": inset_meta.get("inset_direction_axis"),
        "inset_direction_sign": inset_meta.get("inset_direction_sign"),
        "inset_wall_mount_valid": bool(inset_meta.get("inset_wall_mount_valid")),
        "inset_actual_distance_to_wall_surface": None if inset_validation.get("inset_actual_distance_to_wall_surface") is None else round(float(inset_validation["inset_actual_distance_to_wall_surface"]), 4),
        "inset_distance_error": None if inset_validation.get("inset_distance_error") is None else round(float(inset_validation["inset_distance_error"]), 4),
        "inset_distance_tolerance": None if inset_validation.get("inset_distance_tolerance") is None else round(float(inset_validation["inset_distance_tolerance"]), 4),
        "on_inset_side": bool(inset_validation.get("on_inset_side")),
        "inset_fallback_used": bool(inset_meta.get("fallback_used")),
        "inset_fallback_reason": inset_meta.get("fallback_reason"),
        "z_sampling_mode": "wall_side_height_band",
        "within_wall_constraint_box": within_constraint_box,
        "distance_to_wall_surface": round(float(validation["distance_to_wall_surface"]), 4),
        "surface_distance_threshold": round(float(install_band['surface_distance_max']), 4),
        "inside_wall_bbox": bool(validation["inside_wall_bbox"]),
        "near_top_plane": bool(validation["near_top_plane"]),
        "near_wall_face": bool(validation["near_wall_face"]),
        "within_length_band": bool(validation["within_length_band"]),
        "within_height_band": bool(validation["within_height_band"]),
        "selected_mount_is_aggregate": bool(validation["selected_mount_is_aggregate"]),
        "mounted_on_wall": bool(validation["mounted_on_wall"]),
        "target_world_xyz": None if target_world_xyz is None else [round(float(v), 4) for v in target_world_xyz],
    }
    box_meta["wall_height_constraint"] = constraint_meta

    print(
        "[camera-sample] "
        f"sampled_world_xyz=({camera_world_xyz[0]:.4f},{camera_world_xyz[1]:.4f},{camera_world_xyz[2]:.4f}) "
        f"selected_mount_prim={selected_wall['prim_path']} result={status}"
    )
    print("[camera-sample] " + json.dumps(constraint_meta, ensure_ascii=False, sort_keys=True))

    if not within_constraint_box:
        raise RuntimeError(
            f"camera world xyz failed {constraint_mode} validation: "
            f"camera={camera_world_xyz} validation={validation} inset_meta={inset_meta}"
        )

    return (camera_world_xyz[0], camera_world_xyz[1], camera_world_xyz[2], seed, box_meta)


def _find_first_camera_prim(prim):
    if not prim or not prim.IsValid():
        return None
    if prim.GetTypeName() == "Camera":
        return prim
    for child in prim.GetChildren():
        found = _find_first_camera_prim(child)
        if found and found.IsValid():
            return found
    return None


def _compute_target_world_midpoint(stage, target_prim_path):
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not (target_prim and target_prim.IsValid()):
        raise RuntimeError(f"target prim invalid: {target_prim_path}")
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
    target_bound = bbox_cache.ComputeWorldBound(target_prim)
    target_range = target_bound.ComputeAlignedRange()
    if target_range.IsEmpty():
        raise RuntimeError(f"target prim world bbox empty: {target_prim_path}")
    mid = target_range.GetMidpoint()
    return (float(mid[0]), float(mid[1]), float(mid[2]))


def _compute_dynamic_lookat_pan_tilt(camera_xyz, target_xyz):
    dx = float(target_xyz[0]) - float(camera_xyz[0])
    dy = float(target_xyz[1]) - float(camera_xyz[1])
    dz = float(target_xyz[2]) - float(camera_xyz[2])
    planar = math.hypot(dx, dy)
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= 1e-6 or planar <= 1e-6:
        raise RuntimeError(
            f"look-at vector degenerate: camera_xyz={camera_xyz} target_xyz={target_xyz}"
        )
    pan_deg = math.degrees(math.atan2(-dx, dy))
    tilt_deg = -math.degrees(math.atan2(dz, planar))
    pan_deg = max(-170.0, min(170.0, pan_deg))
    tilt_deg = max(-90.0, min(30.0, tilt_deg))
    return pan_deg, tilt_deg


def _resolve_ptz_eval_context(stage, camera_rig_path):
    rig_prim = stage.GetPrimAtPath(camera_rig_path)
    camera_prim = _find_first_camera_prim(rig_prim)
    if not (rig_prim and rig_prim.IsValid() and camera_prim and camera_prim.IsValid()):
        return None

    camera_path = camera_prim.GetPath().pathString
    tilt_prim = camera_prim.GetParent()
    if not (tilt_prim and tilt_prim.IsValid()):
        return None

    up_axis = UsdGeom.GetStageUpAxis(stage)
    if up_axis == "Z":
        pan_attr_name = "xformOp:rotateZ"
        tilt_attr_name = "xformOp:rotateX"
        legacy_tilt_attr_name = "xformOp:rotateY"
    else:
        pan_attr_name = "xformOp:rotateY"
        tilt_attr_name = "xformOp:rotateZ"
        legacy_tilt_attr_name = None

    pan_attr = rig_prim.GetAttribute(pan_attr_name)
    if not (pan_attr and pan_attr.IsValid()):
        return None

    tilt_attr = tilt_prim.GetAttribute(tilt_attr_name)
    tilt_attr_name_used = tilt_attr_name
    if not (tilt_attr and tilt_attr.IsValid()) and legacy_tilt_attr_name:
        legacy_attr = tilt_prim.GetAttribute(legacy_tilt_attr_name)
        if legacy_attr and legacy_attr.IsValid():
            tilt_attr = legacy_attr
            tilt_attr_name_used = legacy_tilt_attr_name
    if not (tilt_attr and tilt_attr.IsValid()):
        return None

    pan_base = pan_attr.Get()
    tilt_base = tilt_attr.Get()
    try:
        pan_base = float(pan_base if pan_base is not None else 0.0)
    except Exception:
        pan_base = 0.0
    try:
        tilt_base = float(tilt_base if tilt_base is not None else 0.0)
    except Exception:
        tilt_base = 0.0

    return {
        "rig_prim": rig_prim,
        "camera_prim": camera_prim,
        "camera_path": camera_path,
        "tilt_prim": tilt_prim,
        "up_axis": up_axis,
        "pan_attr_name": pan_attr_name,
        "tilt_attr_name": tilt_attr_name_used,
        "pan_attr": pan_attr,
        "tilt_attr": tilt_attr,
        "pan_base": pan_base,
        "tilt_base": tilt_base,
    }


def _compute_camera_visibility_metrics(stage, camera_prim, target_prim, resolution_wh):
    width, height = int(resolution_wh[0]), int(resolution_wh[1])
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
    target_bound = bbox_cache.ComputeWorldBound(target_prim)
    target_range = target_bound.ComputeAlignedRange()
    if target_range.IsEmpty():
        return {
            "visible": False,
            "frustum_visible": False,
            "center_in_frame": False,
            "intersection_ratio": 0.0,
            "screen_bbox_px": None,
            "center_px": None,
            "intersection_ratio_threshold": _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD,
            "rejection_reason": "target_bound_empty",
            "near_miss": False,
        }

    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    cam_world = xcache.GetLocalToWorldTransform(camera_prim)
    gf_cam = UsdGeom.Camera(camera_prim).GetCamera(Usd.TimeCode.Default())
    frustum = gf_cam.frustum
    frustum.Transform(cam_world)
    frustum_visible = bool(frustum.Intersects(target_bound))

    vm = frustum.ComputeViewMatrix()
    pm = frustum.ComputeProjectionMatrix()

    mn = target_range.GetMin()
    mx = target_range.GetMax()
    pixels = []
    for px in (mn[0], mx[0]):
        for py in (mn[1], mx[1]):
            for pz in (mn[2], mx[2]):
                clip = Gf.Vec4d(px, py, pz, 1.0) * vm * pm
                w = float(clip[3])
                if abs(w) < 1e-9:
                    continue
                ndc_x = float(clip[0]) / w
                ndc_y = float(clip[1]) / w
                sx = (ndc_x * 0.5 + 0.5) * width
                sy = (1.0 - (ndc_y * 0.5 + 0.5)) * height
                pixels.append((sx, sy))

    if not pixels:
        return {
            "visible": False,
            "frustum_visible": frustum_visible,
            "center_in_frame": False,
            "intersection_ratio": 0.0,
            "screen_bbox_px": None,
            "center_px": None,
            "intersection_ratio_threshold": _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD,
            "rejection_reason": "target_projection_empty",
            "near_miss": False,
        }

    min_x = min(v[0] for v in pixels)
    min_y = min(v[1] for v in pixels)
    max_x = max(v[0] for v in pixels)
    max_y = max(v[1] for v in pixels)
    bbox_w = max(1e-6, max_x - min_x)
    bbox_h = max(1e-6, max_y - min_y)
    cx = (min_x + max_x) * 0.5
    cy = (min_y + max_y) * 0.5
    inter_w = max(0.0, min(max_x, width) - max(min_x, 0.0))
    inter_h = max(0.0, min(max_y, height) - max(min_y, 0.0))
    intersection_ratio = (inter_w * inter_h) / (bbox_w * bbox_h)
    center_in_frame = bool(0.0 <= cx <= width and 0.0 <= cy <= height)
    visible = bool(
        frustum_visible
        and center_in_frame
        and intersection_ratio >= _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD
    )
    if not frustum_visible:
        rejection_reason = "target_outside_real_camera_frustum"
    elif not center_in_frame:
        rejection_reason = "target_center_out_of_frame"
    elif intersection_ratio < _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD:
        rejection_reason = (
            "intersection_ratio_below_threshold:"
            f"{intersection_ratio:.4f}<{_STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD:.4f}"
        )
    else:
        rejection_reason = "accepted"
    center_margin_x = min(abs(cx), abs(cx - width))
    center_margin_y = min(abs(cy), abs(cy - height))
    near_miss = bool(
        frustum_visible
        and not visible
        and center_margin_x <= _STARTUP_VIEW_NEAR_MISS_PIXEL_MARGIN
        and center_margin_y <= _STARTUP_VIEW_NEAR_MISS_PIXEL_MARGIN
        and intersection_ratio >= max(0.03, _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD * 0.4)
    )

    return {
        "visible": visible,
        "frustum_visible": frustum_visible,
        "center_in_frame": center_in_frame,
        "intersection_ratio": round(float(intersection_ratio), 4),
        "screen_bbox_px": [round(float(min_x), 2), round(float(min_y), 2), round(float(max_x), 2), round(float(max_y), 2)],
        "center_px": [round(float(cx), 2), round(float(cy), 2)],
        "intersection_ratio_threshold": _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD,
        "rejection_reason": rejection_reason,
        "near_miss": near_miss,
    }


def _evaluate_real_camera_view(stage, camera_rig_path, target_prim_path, pan_deg, tilt_deg, resolution_wh):
    ctx = _resolve_ptz_eval_context(stage, camera_rig_path)
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if ctx is None or not (target_prim and target_prim.IsValid()):
        return {
            "visible": False,
            "frustum_visible": False,
            "center_in_frame": False,
            "intersection_ratio": 0.0,
            "screen_bbox_px": None,
            "center_px": None,
            "intersection_ratio_threshold": _STARTUP_VIEW_INTERSECTION_RATIO_THRESHOLD,
            "rejection_reason": "invalid_camera_or_target",
            "near_miss": False,
            "applied_pan": float(pan_deg),
            "applied_tilt": float(tilt_deg),
            "error": "invalid_camera_or_target",
        }

    pan_attr = ctx["pan_attr"]
    tilt_attr = ctx["tilt_attr"]
    orig_pan = pan_attr.Get()
    orig_tilt = tilt_attr.Get()

    if ctx["up_axis"] == "Z":
        pan_final = (ctx["pan_base"] + 180.0) - float(pan_deg)
    else:
        pan_final = ctx["pan_base"] + float(pan_deg)
    tilt_final = ctx["tilt_base"] + (-float(tilt_deg))

    try:
        pan_attr.Set(float(pan_final))
        tilt_attr.Set(float(tilt_final))
        metrics = _compute_camera_visibility_metrics(
            stage,
            ctx["camera_prim"],
            target_prim,
            resolution_wh,
        )
        metrics.update({
            "applied_pan": round(float(pan_deg), 4),
            "applied_tilt": round(float(tilt_deg), 4),
            "camera_path": ctx["camera_path"],
        })
        return metrics
    finally:
        pan_attr.Set(orig_pan)
        tilt_attr.Set(orig_tilt)


def resolve_dynamic_startup_view_metrics(
    stage,
    camera_rig_path,
    target_prim_path,
    *,
    lookat_target_xyz=None,
    resolution_wh=(960, 540),
    prefer_target_prim_center=True,
    dynamic_startup_pan_offset_deg=0.0,
    dynamic_startup_tilt_offset_deg=0.0,
):
    def _clamp_pan_tilt(pan_deg, tilt_deg):
        return (
            max(-170.0, min(170.0, float(pan_deg))),
            max(-90.0, min(30.0, float(tilt_deg))),
        )

    def _metric_score(metrics):
        center_px = metrics.get("center_px") or [1e9, 1e9]
        center_dx = abs(float(center_px[0]) - float(resolution_wh[0]) * 0.5) if len(center_px) >= 2 else 1e9
        center_dy = abs(float(center_px[1]) - float(resolution_wh[1]) * 0.5) if len(center_px) >= 2 else 1e9
        return (
            1000.0 if metrics.get("visible", False) else 0.0,
            100.0 if metrics.get("frustum_visible", False) else 0.0,
            10.0 if metrics.get("center_in_frame", False) else 0.0,
            float(metrics.get("intersection_ratio") or 0.0),
            -center_dx - center_dy,
        )

    rig_prim = stage.GetPrimAtPath(camera_rig_path)
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not rig_prim.IsValid() or not target_prim.IsValid():
        return {
            "visible": False,
            "mode": "dynamic_lookat",
            "source": "invalid_camera_or_target",
            "rejection_reason": "invalid_camera_or_target",
        }

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    rig_world = xform_cache.GetLocalToWorldTransform(rig_prim).ExtractTranslation()
    camera_xyz = (float(rig_world[0]), float(rig_world[1]), float(rig_world[2]))
    resolved_lookat_target_xyz = None
    resolved_lookat_source = None
    if prefer_target_prim_center:
        try:
            resolved_lookat_target_xyz = _compute_target_world_midpoint(stage, target_prim_path)
            resolved_lookat_source = "target_prim_world_midpoint"
        except Exception as exc:
            print(f"[camera-startup-view] WARN: target midpoint unavailable, fallback_to_config reason={exc}")
    if resolved_lookat_target_xyz is None and lookat_target_xyz is not None:
        resolved_lookat_target_xyz = tuple(float(v) for v in lookat_target_xyz)
        resolved_lookat_source = "config_camera_lookat_target_xyz"

    if resolved_lookat_target_xyz is None:
        return {
            "visible": False,
            "mode": "dynamic_lookat",
            "source": "missing_dynamic_lookat_target",
            "rejection_reason": "missing_dynamic_lookat_target",
        }

    base_pan, base_tilt = _compute_dynamic_lookat_pan_tilt(camera_xyz, resolved_lookat_target_xyz)
    startup_pan, startup_tilt = _clamp_pan_tilt(
        float(base_pan) + float(dynamic_startup_pan_offset_deg),
        float(base_tilt) + float(dynamic_startup_tilt_offset_deg),
    )

    startup_metrics = _evaluate_real_camera_view(
        stage,
        camera_rig_path,
        target_prim_path,
        startup_pan,
        startup_tilt,
        resolution_wh,
    )
    startup_metrics.update({
        "mode": "dynamic_lookat",
        "source": "dynamic_startup_orientation",
        "preset_name": "default_initial",
        "camera_xyz": [round(float(v), 4) for v in camera_xyz],
        "lookat_target_xyz": [round(float(v), 4) for v in resolved_lookat_target_xyz],
        "lookat_target_source": resolved_lookat_source,
        "base_pan": round(float(base_pan), 4),
        "base_tilt": round(float(base_tilt), 4),
        "applied_pan": round(float(startup_pan), 4),
        "applied_tilt": round(float(startup_tilt), 4),
        "dynamic_startup_pan_offset_deg": round(float(dynamic_startup_pan_offset_deg), 4),
        "dynamic_startup_tilt_offset_deg": round(float(dynamic_startup_tilt_offset_deg), 4),
        "startup_fallback_applied": False,
    })
    if startup_metrics.get("visible", False):
        return startup_metrics

    fallback_metrics = _evaluate_real_camera_view(
        stage,
        camera_rig_path,
        target_prim_path,
        base_pan,
        base_tilt,
        resolution_wh,
    )
    fallback_metrics.update({
        "mode": "dynamic_lookat",
        "source": "dynamic_startup_orientation_fallback_base_lookat",
        "preset_name": "default_initial",
        "camera_xyz": [round(float(v), 4) for v in camera_xyz],
        "lookat_target_xyz": [round(float(v), 4) for v in resolved_lookat_target_xyz],
        "lookat_target_source": resolved_lookat_source,
        "base_pan": round(float(base_pan), 4),
        "base_tilt": round(float(base_tilt), 4),
        "applied_pan": round(float(base_pan), 4),
        "applied_tilt": round(float(base_tilt), 4),
        "dynamic_startup_pan_offset_deg": round(float(dynamic_startup_pan_offset_deg), 4),
        "dynamic_startup_tilt_offset_deg": round(float(dynamic_startup_tilt_offset_deg), 4),
        "startup_fallback_applied": True,
        "startup_preferred_pan": round(float(startup_pan), 4),
        "startup_preferred_tilt": round(float(startup_tilt), 4),
        "startup_preferred_visible": bool(startup_metrics.get("visible", False)),
        "startup_preferred_rejection_reason": startup_metrics.get("rejection_reason"),
        "startup_preferred_frustum_visible": startup_metrics.get("frustum_visible"),
        "startup_preferred_center_in_frame": startup_metrics.get("center_in_frame"),
        "startup_preferred_intersection_ratio": startup_metrics.get("intersection_ratio"),
    })
    if fallback_metrics.get("visible", False):
        return fallback_metrics

    best_metrics = startup_metrics if _metric_score(startup_metrics) >= _metric_score(fallback_metrics) else fallback_metrics
    best_source = "dynamic_startup_orientation"
    search_pan_offsets = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0, -120.0, 120.0, -150.0, 150.0]
    search_tilt_values = [startup_tilt, -8.0, -4.0, 0.0, 4.0, base_tilt, 8.0, 16.0, 24.0, -12.0, -20.0, 30.0, -35.0, -60.0]
    tried = {
        (round(float(startup_pan), 4), round(float(startup_tilt), 4)),
        (round(float(base_pan), 4), round(float(base_tilt), 4)),
    }
    for pan_anchor in (startup_pan, base_pan, 0.0):
        for pan_offset in search_pan_offsets:
            pan_candidate, _ = _clamp_pan_tilt(float(pan_anchor) + float(pan_offset), base_tilt)
            for tilt_candidate_raw in search_tilt_values:
                _, tilt_candidate = _clamp_pan_tilt(pan_candidate, tilt_candidate_raw)
                key = (round(float(pan_candidate), 4), round(float(tilt_candidate), 4))
                if key in tried:
                    continue
                tried.add(key)
                search_metrics = _evaluate_real_camera_view(
                    stage,
                    camera_rig_path,
                    target_prim_path,
                    pan_candidate,
                    tilt_candidate,
                    resolution_wh,
                )
                search_metrics.update({
                    "mode": "dynamic_lookat",
                    "source": "dynamic_startup_orientation_search",
                    "preset_name": "default_initial",
                    "camera_xyz": [round(float(v), 4) for v in camera_xyz],
                    "lookat_target_xyz": [round(float(v), 4) for v in resolved_lookat_target_xyz],
                    "lookat_target_source": resolved_lookat_source,
                    "base_pan": round(float(base_pan), 4),
                    "base_tilt": round(float(base_tilt), 4),
                    "applied_pan": round(float(pan_candidate), 4),
                    "applied_tilt": round(float(tilt_candidate), 4),
                    "dynamic_startup_pan_offset_deg": round(float(dynamic_startup_pan_offset_deg), 4),
                    "dynamic_startup_tilt_offset_deg": round(float(dynamic_startup_tilt_offset_deg), 4),
                    "startup_fallback_applied": True,
                    "startup_preferred_pan": round(float(startup_pan), 4),
                    "startup_preferred_tilt": round(float(startup_tilt), 4),
                    "startup_preferred_visible": bool(startup_metrics.get("visible", False)),
                    "startup_preferred_rejection_reason": startup_metrics.get("rejection_reason"),
                    "startup_preferred_frustum_visible": startup_metrics.get("frustum_visible"),
                    "startup_preferred_center_in_frame": startup_metrics.get("center_in_frame"),
                    "startup_preferred_intersection_ratio": startup_metrics.get("intersection_ratio"),
                })
                if search_metrics.get("visible", False):
                    return search_metrics
                if _metric_score(search_metrics) > _metric_score(best_metrics):
                    best_metrics = search_metrics
                    best_source = "dynamic_startup_orientation_search"

    if best_source != "dynamic_startup_orientation":
        best_metrics["startup_fallback_applied"] = True
    return best_metrics


def check_preset_visibility(
    stage,
    camera_rig_path,
    presets_cfg,
    target_prim_path,
    fov_half_deg=60.0,
    *,
    orientation_mode="legacy",
    lookat_target_xyz=None,
    resolution_wh=(960, 540),
    default_preset_token="1",
    return_details=False,
    prefer_target_prim_center=True,
    dynamic_startup_pan_offset_deg=0.0,
    dynamic_startup_tilt_offset_deg=0.0,
):
    del fov_half_deg

    rig_prim = stage.GetPrimAtPath(camera_rig_path)
    target_prim = stage.GetPrimAtPath(target_prim_path)
    if not rig_prim.IsValid() or not target_prim.IsValid():
        empty = {k: False for k in presets_cfg}
        empty["__startup_default__"] = False
        return (empty, {}) if return_details else empty

    results = {}
    details = {}
    for preset_name, preset_data in presets_cfg.items():
        pan = float(preset_data.get("pan", 0.0))
        tilt = float(preset_data.get("tilt", 0.0))
        metrics = _evaluate_real_camera_view(
            stage,
            camera_rig_path,
            target_prim_path,
            pan,
            tilt,
            resolution_wh,
        )
        metrics.update({
            "mode": "legacy",
            "source": "preset",
            "preset_name": str(preset_name),
        })
        results[preset_name] = bool(metrics["visible"])
        details[preset_name] = metrics

    default_key = None
    for candidate in (str(default_preset_token), "1"):
        if candidate in details:
            default_key = candidate
            break
    if default_key is None and details:
        default_key = sorted(details.keys(), key=lambda x: str(x))[0]

    mode = str(orientation_mode or "").strip().lower()
    if mode == "dynamic_lookat":
        startup_metrics = resolve_dynamic_startup_view_metrics(
            stage,
            camera_rig_path,
            target_prim_path,
            lookat_target_xyz=lookat_target_xyz,
            resolution_wh=resolution_wh,
            prefer_target_prim_center=prefer_target_prim_center,
            dynamic_startup_pan_offset_deg=dynamic_startup_pan_offset_deg,
            dynamic_startup_tilt_offset_deg=dynamic_startup_tilt_offset_deg,
        )
    elif default_key is not None:
        startup_metrics = dict(details[default_key])
        startup_metrics.update({
            "mode": "legacy",
            "source": "legacy_default_preset",
            "default_preset_token": str(default_key),
        })
    else:
        startup_metrics = {
            "visible": False,
            "mode": mode or "legacy",
            "source": "missing_default_preset",
        }

    results["__startup_default__"] = bool(startup_metrics.get("visible", False))
    details["__startup_default__"] = startup_metrics

    if return_details:
        return results, details
    return results
