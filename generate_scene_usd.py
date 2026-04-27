#!/usr/bin/env python3
"""
随机化吊篮高度/人数与相机位姿，并导出 USD（纯 pxr.Usd，可在 Isaac Sim 自带 Python 或
已配置 PXR 的环境运行）。

约束：
  - 人数=0 → 吊篮必须在地面（高度轴 = Y）
  - Y 高于地面（空中）→ 人数 >= 1

示例：
  /path/to/python3 generate_scene_usd.py \\
    --input /home/uniubi/xuanyuan/scene.usd \\
    --output /home/uniubi/xuanyuan/scene_generated.usd \\
    --seed 42 \\
    --basket-prims /World/Basket \\
    --person-prims /World/Basket/Person_0,/World/Basket/Person_1,/World/Basket/Person_2 \\
    --camera-prim /World/CameraRig/CamTilt/Camera
"""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import random
import re
import sys
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# pxr 引导：优先当前环境；否则 Isaac 解压目录或 conda isaacsim 的 omni.usd.libs
# （需将 conda env 的 lib 加入 LD_LIBRARY_PATH，否则 _tf 报 libpython 缺失）
# ---------------------------------------------------------------------------


def _conda_lib_dirs() -> List[str]:
    out: List[str] = []
    for base in (
        os.environ.get("CONDA_PREFIX"),
        os.path.expanduser("~/miniconda3/envs/env_isaaclab"),
    ):
        if base:
            lib = os.path.join(base, "lib")
            if os.path.isdir(lib):
                out.append(lib)
    return out


def _discover_omni_usd_libs() -> List[str]:
    """匹配 extscache/omni.usd.libs-* 目录（版本号可变）。"""
    roots = [
        os.path.expanduser("~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/extscache"),
        os.path.expanduser("~/projects/issac/.isaac_sim_unzip/extscache"),
        os.path.expanduser("~/.local/share/ov/data/exts/v2"),
    ]
    found: List[str] = []
    for root in roots:
        pattern = os.path.join(root, "omni.usd.libs-*")
        found.extend(p for p in glob.glob(pattern) if os.path.isdir(p))
    return sorted(set(found))


def _looks_like_package_root_score(parent_dir: str) -> int:
    """
    给 pxr 父目录打分，分数越高越像可直接 import 的 Python 包根目录。
    """
    score = 0
    norm = parent_dir.replace("\\", "/").lower()
    if "site-packages" in norm:
        score += 100
    if "pip_prebundle" in norm:
        score += 80
    if norm.endswith("/python") or "/python/" in norm:
        score += 20
    if "omni.usd.libs" in norm:
        score += 10
    return score


def _is_valid_pxr_namespace_dir(pxr_dir: str) -> Tuple[bool, str]:
    """
    允许 namespace package 形式：
    - 传统包：pxr/__init__.py 存在
    - 命名空间包：无 __init__.py，但存在常见 USD 子模块目录
    """
    init_py = os.path.join(pxr_dir, "__init__.py")
    if os.path.isfile(init_py):
        return True, "__init__.py"
    common_mods = ("Ar", "Gf", "Usd", "Sdf")
    for mod in common_mods:
        if os.path.isdir(os.path.join(pxr_dir, mod)):
            return True, f"namespace:{mod}"
    return False, "missing_init_and_common_modules"


def _collect_pxr_parent_dirs(root_dir: str, searched_paths: List[str]) -> List[str]:
    """
    递归搜索 root_dir 下可用于 import pxr 的父目录。
    """
    out: List[str] = []
    if not root_dir or not os.path.isdir(root_dir):
        return out

    # 常见优先候选：先试这些，减少全量 walk 成本。
    quick_candidates = [
        os.path.join(root_dir, "pxr"),
        os.path.join(root_dir, "site-packages", "pxr"),
        os.path.join(root_dir, "pip_prebundle", "pxr"),
        os.path.join(root_dir, "lib", "python", "pxr"),
    ]
    for pxr_dir in quick_candidates:
        searched_paths.append(os.path.join(pxr_dir, "__init__.py"))
        ok, _ = _is_valid_pxr_namespace_dir(pxr_dir)
        if ok:
            out.append(os.path.dirname(pxr_dir))

    # 深度搜索：匹配 */pxr（支持 namespace package）
    for dirpath, dirnames, filenames in os.walk(root_dir):
        if "pxr" not in dirnames:
            continue
        pxr_dir = os.path.join(dirpath, "pxr")
        searched_paths.append(os.path.join(pxr_dir, "__init__.py"))
        ok, _ = _is_valid_pxr_namespace_dir(pxr_dir)
        if ok:
            out.append(os.path.dirname(pxr_dir))
        # 避免重复在已经命中的 pxr 目录下继续深挖
        try:
            dirnames.remove("pxr")
        except ValueError:
            pass

    return sorted(set(out))


def _debug_pxr_enabled() -> bool:
    return "--debug-pxr-import" in sys.argv or os.environ.get("DEBUG_PXR_IMPORT") == "1"


def _setup_px_import(debug: bool = False) -> None:
    try:
        from pxr import Usd  # noqa: F401

        return
    except ImportError:
        pass

    isaac_pxr_dir = os.environ.get("ISAAC_PXR_DIR", "")
    pxr_pluginpath = os.environ.get("PXR_PLUGINPATH_NAME", "")
    isaac_pxr_has_dir = False
    isaac_pxr_marker = "n/a"
    if isaac_pxr_dir:
        isaac_pxr_has_dir = os.path.isdir(os.path.join(isaac_pxr_dir, "pxr"))
        if isaac_pxr_has_dir:
            _, isaac_pxr_marker = _is_valid_pxr_namespace_dir(
                os.path.join(isaac_pxr_dir, "pxr")
            )

    # 第1层：收集根目录（不能假设 ISAAC_PXR_DIR 本身可直接放 sys.path）。
    root_candidates: List[str] = []
    if isaac_pxr_dir:
        root_candidates.append(isaac_pxr_dir)
    if pxr_pluginpath:
        root_candidates.extend(
            [x.strip() for x in pxr_pluginpath.split(":") if x.strip()]
        )
    root_candidates.extend(_discover_omni_usd_libs())
    root_candidates.append(
        os.path.expanduser(
            "~/projects/issac/.isaac_sim_unzip/extscache/"
            "omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"
        )
    )
    root_candidates = [os.path.abspath(x) for x in root_candidates if x]

    searched_paths: List[str] = []
    parent_dir_candidates: List[str] = []
    # ISAAC_PXR_DIR 直连优先：若其下存在 pxr 目录（含 namespace 子模块），优先注入它本身。
    if isaac_pxr_dir:
        pxr_subdir = os.path.join(os.path.abspath(isaac_pxr_dir), "pxr")
        searched_paths.append(os.path.join(pxr_subdir, "__init__.py"))
        ok_direct, _ = _is_valid_pxr_namespace_dir(pxr_subdir)
        if ok_direct:
            parent_dir_candidates.append(os.path.abspath(isaac_pxr_dir))

    for root in root_candidates:
        parent_dir_candidates.extend(_collect_pxr_parent_dirs(root, searched_paths))
        # 如果 root 本身就是包父目录（支持 namespace package），也加入候选
        root_pxr_dir = os.path.join(root, "pxr")
        searched_paths.append(os.path.join(root_pxr_dir, "__init__.py"))
        ok_root, _ = _is_valid_pxr_namespace_dir(root_pxr_dir)
        if ok_root:
            parent_dir_candidates.append(root)

    parent_dir_candidates = sorted(
        set(parent_dir_candidates),
        key=lambda p: (_looks_like_package_root_score(p), -len(p)),
        reverse=True,
    )

    conda_libs = _conda_lib_dirs()
    attempts: List[str] = []
    import_errors: List[str] = []
    inserted_sys_paths: List[str] = []
    for pkg_root in parent_dir_candidates:
        attempts.append(pkg_root)
        if pkg_root not in sys.path:
            sys.path.insert(0, pkg_root)
            inserted_sys_paths.append(pkg_root)
        px_bin = os.path.join(pkg_root, "bin")
        ld = [px_bin] + conda_libs + [os.environ.get("LD_LIBRARY_PATH", "")]
        os.environ["LD_LIBRARY_PATH"] = ":".join(x for x in ld if x)
        try:
            from pxr import Usd  # noqa: F401

            if debug:
                print(f"[pxr] import success, sys.path 注入目录: {pkg_root}")
            return
        except ImportError as e:
            import_errors.append(f"{pkg_root} -> {repr(e)}")
            if debug:
                print(f"[pxr] import failed with {pkg_root}: {e}")
            # 仅回滚本次注入，避免污染后续优先级
            if sys.path and sys.path[0] == pkg_root:
                sys.path.pop(0)
            elif pkg_root in sys.path:
                sys.path.remove(pkg_root)
            continue

    searched_unique = sorted(set(searched_paths))
    suggestions = [
        "将 ISAAC_PXR_DIR 设置到包含 pxr 目录的上一级目录",
        "若是 namespace package，确认 pxr 下存在 Ar/Gf/Usd/Sdf 之一",
        "可先手动确认：python -c \"import os; print(os.path.isdir('<候选>/pxr'))\"",
    ]
    msg_lines = [
        "无法导入 pxr。",
        f"ISAAC_PXR_DIR={isaac_pxr_dir or '<empty>'}",
        f"ISAAC_PXR_DIR 是否发现 pxr 目录={isaac_pxr_has_dir}",
        f"ISAAC_PXR_DIR pxr 识别标记={isaac_pxr_marker}",
        f"PXR_PLUGINPATH_NAME={pxr_pluginpath or '<empty>'}",
        f"已搜索根目录数量={len(root_candidates)}",
        "最终插入过的 sys.path 目录:",
        *([f"  - {p}" for p in inserted_sys_paths] or ["  - <none>"]),
        "已尝试注入 sys.path 的目录:",
        *([f"  - {p}" for p in attempts] or ["  - <none>"]),
        "捕获到的 import 异常:",
        *([f"  - {e}" for e in import_errors] or ["  - <none>"]),
        "已搜索过的 pxr 候选路径 (__init__.py):",
        *([f"  - {p}" for p in searched_unique] or ["  - <none>"]),
        "建议:",
        *[f"  - {s}" for s in suggestions],
    ]
    raise ImportError("\n".join(msg_lines))


_setup_px_import(debug=_debug_pxr_enabled())
from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

INSPECT_PRIM_PATHS = [
    "/World/DiaoLan/Model/Group1",
    "/World/DiaoLan/Model/node______1",
    "/World/DiaoLan/Model/node______2",
    "/World/CameraRig/CamTilt/Camera",
]

FRESH_OPEN_BASKET_PATHS = [
    "/World/DiaoLan/Model/Group1",
]

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------


@dataclass
class HeightPeopleConstraint:
    """人数与高度约束（参数名便于对照文档）。"""

    ground_z: float = 0.0
    air_z_min: float = 0.05
    air_z_max: float = 40.0
    max_people: int = 5
    ground_people_max: int = 5
    air_prob: float = 0.45
    """随机时选「空中」吊篮的概率；其余为地面。"""


@dataclass
class CameraRegion:
    """相机位置采样 AABB（世界坐标，与场景 upAxis 一致；默认 Z-up）。"""

    xmin: float
    xmax: float
    ymin: float
    ymax: float
    zmin: float
    zmax: float


@dataclass
class CameraLookAtConfig:
    """朝向：在 AABB 内随机选观察目标点，相机 up 为 world_up。"""

    target_xmin: float
    target_xmax: float
    target_ymin: float
    target_ymax: float
    target_zmin: float
    target_zmax: float
    world_up: Tuple[float, float, float] = (0.0, 0.0, 1.0)


@dataclass
class GeneratorConfig:
    input_path: str
    output_path: str
    seed: int = 0
    basket_prim_paths: List[str] = field(default_factory=list)
    basket_name_pattern: Optional[str] = None
    person_prim_paths: List[str] = field(default_factory=list)
    camera_prim_path: str = "/World/Camera"
    height_people: HeightPeopleConstraint = field(default_factory=HeightPeopleConstraint)
    camera_region: Optional[CameraRegion] = None
    camera_look_at: Optional[CameraLookAtConfig] = None
    same_level_threshold: float = 3.0
    target_scene_pattern: str = "random"
    sample_id: str = ""
    sample_index: int = 0
    visibility_min_distance: float = 5.0
    visibility_max_distance: float = 140.0
    visibility_target_radius: float = 35.0
    visibility_max_offaxis_deg: float = 75.0
    visibility_debug: bool = False
    stage_up_axis: str = "Y"
    abnormal_label: str = "normal"
    abnormal_meta: dict = field(default_factory=dict)
    normal_person_count_per_basket: int = 2
    height_abnormal_diff_threshold: float = 3.0
    dual_basket_risk_min_center_gap: float = 2.0


# ---------------------------------------------------------------------------
# 采样逻辑
# ---------------------------------------------------------------------------


def sample_height_and_people(
    rng: random.Random, hc: HeightPeopleConstraint
) -> Tuple[float, int]:
    """
    返回 (吊篮高度Y, 人数)。
    - 地面：Y = ground_z，人数可为 0..ground_people_max
    - 空中：Y ∈ [air_z_min, air_z_max]，人数 ∈ [1, max_people]
    """
    if rng.random() < hc.air_prob:
        z = rng.uniform(hc.air_z_min, hc.air_z_max)
        people = rng.randint(1, hc.max_people)
        return z, people
    z = hc.ground_z
    people = rng.randint(0, hc.ground_people_max)
    return z, people


def assert_constraints(z: float, people: int, hc: HeightPeopleConstraint) -> None:
    eps = 1e-4
    on_ground = abs(z - hc.ground_z) <= eps
    in_air = z >= hc.air_z_min - eps
    if people == 0 and not on_ground:
        raise ValueError(f"约束违反：人数=0 但 Y={z} 非地面 ground_z={hc.ground_z}")
    if in_air and people < 1:
        raise ValueError(f"约束违反：空中 Y={z} 但人数={people}")


# ---------------------------------------------------------------------------
# Prim 查找
# ---------------------------------------------------------------------------


def discover_baskets_by_pattern(stage: Usd.Stage, pattern: str) -> List[str]:
    """pattern 为不区分大小写的正则，匹配 prim 路径或名称。"""
    rx = re.compile(pattern, re.IGNORECASE)
    out: List[str] = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        name = prim.GetName()
        if rx.search(p) or rx.search(name):
            if prim.IsA(UsdGeom.Xformable):
                out.append(p)
    return sorted(set(out))


def resolve_basket_paths(cfg: GeneratorConfig, stage: Usd.Stage) -> List[str]:
    if cfg.basket_prim_paths:
        return list(cfg.basket_prim_paths)
    if cfg.basket_name_pattern:
        return discover_baskets_by_pattern(stage, cfg.basket_name_pattern)
    raise ValueError("请指定 --basket-prims 或 --basket-pattern")


# ---------------------------------------------------------------------------
# Transform / 可见性
# ---------------------------------------------------------------------------


def set_xform_translate_y(prim: Usd.Prim, y: float) -> None:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        raise ValueError(f"Prim 非 Xformable：{prim.GetPath()}")
    local_xf = xf.GetLocalTransformation()
    mat = local_xf[0] if isinstance(local_xf, tuple) else local_xf
    t = mat.ExtractTranslation()
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(t[0], y, t[2]))


def get_local_translate(prim: Usd.Prim) -> Gf.Vec3d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        raise ValueError(f"Prim 非 Xformable：{prim.GetPath()}")
    local_xf = xf.GetLocalTransformation()
    mat = local_xf[0] if isinstance(local_xf, tuple) else local_xf
    t = mat.ExtractTranslation()
    return Gf.Vec3d(float(t[0]), float(t[1]), float(t[2]))


def get_world_translate(stage: Usd.Stage, prim: Usd.Prim) -> Gf.Vec3d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        raise ValueError(f"Prim 非 Xformable：{prim.GetPath()}")
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = mat.ExtractTranslation()
    return Gf.Vec3d(float(t[0]), float(t[1]), float(t[2]))


def _axis_to_index(axis: str) -> int:
    a = str(axis or "Y").upper()
    if a == "X":
        return 0
    if a == "Z":
        return 2
    return 1


def _height_value_from_xyz(xyz: Sequence[float], axis: str) -> float:
    idx = _axis_to_index(axis)
    return float(xyz[idx])


def detect_world_height_semantics(stage: Usd.Stage, basket_prim_path: str) -> Tuple[str, float]:
    """
    估计“local Y authored translate”映射到世界空间的高度轴与量纲：
    - 返回 (world_height_axis, world_units_per_local_y)
    """
    prim = stage.GetPrimAtPath(basket_prim_path)
    if not prim.IsValid():
        return "Y", 1.0
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return "Y", 1.0
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # local +Y 单位向量在 world 的方向与尺度（含缩放）
    v = mat.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    comps = [abs(float(v[0])), abs(float(v[1])), abs(float(v[2]))]
    idx = max(range(3), key=lambda i: comps[i])
    axis = ("X", "Y", "Z")[idx]
    scale = float(comps[idx]) if comps[idx] > 1e-9 else 1.0
    return axis, scale


def detect_authored_height_component(
    stage: Usd.Stage, basket_prim_path: str, world_height_axis: str
) -> int:
    """
    返回 authored translate 的分量索引（0/1/2），该分量对 world_height_axis 贡献最大。
    """
    prim = stage.GetPrimAtPath(basket_prim_path)
    if not prim.IsValid():
        return 1
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return 1
    mat = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    world_idx = _axis_to_index(world_height_axis)
    basis_local = [Gf.Vec3d(1.0, 0.0, 0.0), Gf.Vec3d(0.0, 1.0, 0.0), Gf.Vec3d(0.0, 0.0, 1.0)]
    contrib = []
    for v in basis_local:
        w = mat.TransformDir(v)
        contrib.append(abs(float(w[world_idx])))
    return int(max(range(3), key=lambda i: contrib[i]))


def inspect_usd_translates(usd_path: str, prim_paths: Sequence[str]) -> None:
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        raise FileNotFoundError(f"无法打开 USD：{usd_path}")
    print(f"[inspect] USD: {os.path.abspath(usd_path)}")
    for p in prim_paths:
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            raise RuntimeError(f"[inspect] Invalid prim: {p}")
        t = get_local_translate(prim)
        print(f"[inspect] {p} translate = ({t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f})")


def inspect_usd_translate_map(usd_path: str, prim_paths: Sequence[str]) -> Dict[str, Gf.Vec3d]:
    """读取指定 prim 的 world translate，返回 {prim_path: vec3}。"""
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        raise FileNotFoundError(f"无法打开 USD：{usd_path}")
    out: Dict[str, Gf.Vec3d] = {}
    for p in prim_paths:
        prim = stage.GetPrimAtPath(p)
        if not prim.IsValid():
            raise RuntimeError(f"Invalid prim: {p}")
        out[p] = get_world_translate(stage, prim)
    return out


def set_xform_translate_xyz(prim: Usd.Prim, pos: Gf.Vec3d) -> None:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        raise ValueError(f"Prim 非 Xformable：{prim.GetPath()}")
    op = prim.GetAttribute("xformOp:translate")
    if op:
        op.Set(pos)
        return
    UsdGeom.XformCommonAPI(prim).SetTranslate(pos)


def set_camera_orient_look_at(
    prim: Usd.Prim, eye: Gf.Vec3d, target: Gf.Vec3d, world_up: Gf.Vec3d
) -> None:
    """
    与 Isaac 示例一致：SetLookAt(eye, target, up).GetInverse() 再取旋转四元数。
    适用于 Camera 或外层 Xform（本地 -Z 朝前）。不调用 ClearXformOpOrder，避免破坏 Camera。
    """
    m = Gf.Matrix4d().SetLookAt(eye, target, world_up)
    inv = m.GetInverse()
    rot = inv.ExtractRotation()
    q = rot.GetQuat()
    quat_d = Gf.Quatd(q)

    a = prim.GetAttribute("xformOp:orient")
    if a:
        a.Set(quat_d)
        return

    xf = UsdGeom.Xformable(prim)
    xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat_d)


def set_person_visibility(
    prims: Sequence[Usd.Prim], visible_count: int, warn=None
) -> None:
    """显示前 visible_count 个，其余 invisible。"""
    n = max(0, int(visible_count))
    if n > len(prims) and warn:
        warn(
            f"人数 {visible_count} 超过人员 Prim 数 {len(prims)}，仅显示前 {len(prims)} 个。"
        )
        n = len(prims)
    for i, prim in enumerate(prims):
        img = UsdGeom.Imageable(prim)
        if not img:
            continue
        if i < n:
            img.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited)
        else:
            img.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)


def sample_camera_pose(
    rng: random.Random,
    region: CameraRegion,
    look: CameraLookAtConfig,
) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    eye = Gf.Vec3d(
        rng.uniform(region.xmin, region.xmax),
        rng.uniform(region.ymin, region.ymax),
        rng.uniform(region.zmin, region.zmax),
    )
    target = Gf.Vec3d(
        rng.uniform(look.target_xmin, look.target_xmax),
        rng.uniform(look.target_ymin, look.target_ymax),
        rng.uniform(look.target_zmin, look.target_zmax),
    )
    return eye, target


# ---------------------------------------------------------------------------
# 标签推断
# ---------------------------------------------------------------------------


def _basket_world_height_value(basket: dict) -> float:
    if "world_height_value" in basket:
        return float(basket.get("world_height_value", 0.0))
    if "final_y" in basket:
        return float(basket.get("final_y", 0.0))
    return float(basket.get("basket_world_height", 0.0))


def infer_basket_state(person_count: int, world_height: float, ground_height: float = 0.0) -> str:
    eps = 1e-6
    on_ground = abs(float(world_height) - float(ground_height)) <= eps
    if person_count == 0 and on_ground:
        return "ground_empty"
    if person_count >= 1 and (not on_ground):
        return "air_with_person"
    if person_count >= 1 and on_ground:
        return "ground_with_person"
    return "unknown"


def infer_scene_pattern(
    baskets: Sequence[dict],
    ground_height: float = 0.0,
    ground_eps: float = 1e-6,
) -> str:
    eps = max(1e-6, float(ground_eps))
    airborne = sum(
        1
        for b in baskets
        if abs(_basket_world_height_value(b) - float(ground_height)) > eps
    )
    if airborne == 2:
        return "double_airborne"
    if airborne == 1:
        return "single_airborne"
    return "all_ground"


def infer_height_relation(baskets: Sequence[dict], threshold: float) -> str:
    eps = 1e-6
    if len(baskets) < 2:
        return "not_applicable"
    y_vals = [_basket_world_height_value(b) for b in baskets[:2]]
    if y_vals[0] > eps and y_vals[1] > eps:
        return "same_level" if abs(y_vals[0] - y_vals[1]) < threshold else "different_level"
    return "not_applicable"


def _vec3_dist(a: Sequence[float], b: Sequence[float]) -> float:
    dx = float(a[0]) - float(b[0])
    dy = float(a[1]) - float(b[1])
    dz = float(a[2]) - float(b[2])
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def _vec3_dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def _vec3_norm(v: Sequence[float]) -> float:
    return _vec3_dist(v, (0.0, 0.0, 0.0))


def _horizontal_gap_by_up_axis(
    a: Sequence[float], b: Sequence[float], up_axis: str
) -> float:
    ua = str(up_axis or "Y").upper()
    if ua == "Z":
        dx = float(a[0]) - float(b[0])
        dy = float(a[1]) - float(b[1])
        return (dx * dx + dy * dy) ** 0.5
    if ua == "X":
        dy = float(a[1]) - float(b[1])
        dz = float(a[2]) - float(b[2])
        return (dy * dy + dz * dz) ** 0.5
    dx = float(a[0]) - float(b[0])
    dz = float(a[2]) - float(b[2])
    return (dx * dx + dz * dz) ** 0.5


def evaluate_visibility_check(
    cam_eye: Optional[Sequence[float]],
    cam_target: Optional[Sequence[float]],
    baskets: Sequence[dict],
    min_distance: float,
    max_distance: float,
    target_radius: float,
    max_offaxis_deg: float,
) -> dict:
    """
    最小可落地规则近似可视性检查（非渲染截图）：
    - 距离约束（过近/过远）
    - target 落点是否靠近双吊篮中心
    - 相机前向与篮子方向夹角是否过大
    """
    sample_id = "unknown"
    if not cam_eye or not cam_target or len(baskets) < 2:
        return {
            "status": "skip",
            "visible_ok": False,
            "risk_level": "high",
            "risk_reasons": ["camera_or_basket_data_missing"],
            "failed_rules": ["camera_data_missing"],
            "sample_id": sample_id,
        }

    b1 = baskets[0]
    b2 = baskets[1]
    p1 = b1.get("center_xyz")
    p2 = b2.get("center_xyz")
    if not p1 or not p2:
        return {
            "status": "skip",
            "visible_ok": False,
            "risk_level": "high",
            "risk_reasons": ["basket_center_missing"],
            "failed_rules": ["basket_center_missing"],
            "sample_id": sample_id,
        }

    d1 = _vec3_dist(cam_eye, p1)
    d2 = _vec3_dist(cam_eye, p2)
    center = [
        0.5 * (float(p1[0]) + float(p2[0])),
        0.5 * (float(p1[1]) + float(p2[1])),
        0.5 * (float(p1[2]) + float(p2[2])),
    ]
    target_offset = _vec3_dist(cam_target, center)

    forward = [
        float(cam_target[0]) - float(cam_eye[0]),
        float(cam_target[1]) - float(cam_eye[1]),
        float(cam_target[2]) - float(cam_eye[2]),
    ]
    to_b1 = [
        float(p1[0]) - float(cam_eye[0]),
        float(p1[1]) - float(cam_eye[1]),
        float(p1[2]) - float(cam_eye[2]),
    ]
    to_b2 = [
        float(p2[0]) - float(cam_eye[0]),
        float(p2[1]) - float(cam_eye[1]),
        float(p2[2]) - float(cam_eye[2]),
    ]

    failed_rules: List[str] = []
    risk_reasons: List[str] = []
    for d, tag in ((d1, "basket1"), (d2, "basket2")):
        if d < float(min_distance):
            failed_rules.append(f"{tag}_too_near")
        if d > float(max_distance):
            failed_rules.append(f"{tag}_too_far")

    if target_offset > float(target_radius):
        failed_rules.append("target_far_from_basket_center")

    fn = _vec3_norm(forward)
    offaxis_deg_1 = None
    offaxis_deg_2 = None
    for v, tag in ((to_b1, "basket1"), (to_b2, "basket2")):
        vn = _vec3_norm(v)
        if fn > 1e-6 and vn > 1e-6:
            cosv = max(-1.0, min(1.0, _vec3_dot(forward, v) / (fn * vn)))
            ang = math.degrees(math.acos(cosv))
            if tag == "basket1":
                offaxis_deg_1 = ang
            else:
                offaxis_deg_2 = ang
            if ang > float(max_offaxis_deg):
                failed_rules.append(f"{tag}_offaxis_{ang:.1f}deg")

    if failed_rules:
        risk_reasons.extend(failed_rules)
    visible_ok = len(failed_rules) == 0
    return {
        "status": "ok" if visible_ok else "fail",
        "visible_ok": visible_ok,
        "risk_level": "low" if visible_ok else "high",
        "risk_reasons": risk_reasons,
        "failed_rules": failed_rules,
        "camera_eye": [float(cam_eye[0]), float(cam_eye[1]), float(cam_eye[2])],
        "camera_target": [float(cam_target[0]), float(cam_target[1]), float(cam_target[2])],
        "basket_center": center,
        "distance_to_basket_1": d1,
        "distance_to_basket_2": d2,
        "target_offset_to_basket_center": target_offset,
        "offaxis_deg_1": offaxis_deg_1,
        "offaxis_deg_2": offaxis_deg_2,
        # 坐标语义：全部来自同一 stage local/world 一致语义下的 xyz 欧氏比较。
        # upAxis=Y 仅表示“竖直轴定义”，不影响距离与夹角计算正确性。
        "coordinate_semantics": "same_space_xyz_euclidean",
        "thresholds": {
            "visibility_min_distance": float(min_distance),
            "visibility_max_distance": float(max_distance),
            "visibility_target_radius": float(target_radius),
            "visibility_max_offaxis_deg": float(max_offaxis_deg),
        },
    }


# ---------------------------------------------------------------------------
# 可控场景采样
# ---------------------------------------------------------------------------


def sample_baskets_for_scene_pattern(
    pattern: str,
    basket_paths: Sequence[str],
    rng: random.Random,
    hc: HeightPeopleConstraint,
) -> List[Tuple[str, float, int]]:
    """返回 [(prim, final_y, person_count), ...]。"""
    items = list(basket_paths)
    if pattern == "random":
        out = []
        for bp in items:
            y, people = sample_height_and_people(rng, hc)
            out.append((bp, y, people))
        return out

    if pattern == "double_airborne":
        out = []
        for bp in items:
            y = rng.uniform(hc.air_z_min, hc.air_z_max)
            people = rng.randint(1, hc.max_people)
            out.append((bp, y, people))
        return out

    if pattern == "all_ground":
        return [(bp, hc.ground_z, 0) for bp in items]

    if pattern == "single_airborne":
        if len(items) < 2:
            # 单吊篮场景无法构造 single_airborne，回退为 random（后续校验会拦截）
            y, people = sample_height_and_people(rng, hc)
            return [(items[0], y, people)] if items else []
        idx_air = rng.randrange(len(items))
        out = []
        for i, bp in enumerate(items):
            if i == idx_air:
                y = rng.uniform(hc.air_z_min, hc.air_z_max)
                people = rng.randint(1, hc.max_people)
            else:
                y = hc.ground_z
                people = 0
            out.append((bp, y, people))
        return out

    raise ValueError(f"不支持的 target_scene_pattern: {pattern}")


def apply_abnormal_transform_v1(
    sampled_baskets: Sequence[Tuple[str, float, int]],
    initial_translates: Dict[str, Gf.Vec3d],
    cfg: GeneratorConfig,
    rng: random.Random,
    world_height_axis: str = "Y",
    world_units_per_local_y: float = 1.0,
    authored_height_component_idx: int = 1,
) -> Tuple[List[Tuple[str, float, int]], Dict[str, Gf.Vec3d], dict]:
    out = [(bp, float(y), int(p)) for bp, y, p in sampled_baskets]
    forced_positions: Dict[str, Gf.Vec3d] = {}
    label = str(cfg.abnormal_label or ABNORMAL_LABEL_NORMAL)
    meta = {
        "schema_version": ABNORMAL_SCHEMA_VERSION,
        "implemented": bool(label in ABNORMAL_LABELS_IMPLEMENTED),
        "trigger_type": label,
        "details": {},
    }
    if len(out) < 2:
        return out, forced_positions, meta

    left_prim, left_y, left_people = out[0]
    right_prim, right_y, right_people = out[1]
    if label == ABNORMAL_LABEL_PERSON_COUNT:
        normal_cnt = max(1, int(cfg.normal_person_count_per_basket))
        changed_left = bool(rng.random() < 0.5)
        left_people = 1 if changed_left else normal_cnt
        right_people = normal_cnt if changed_left else 1
        out[0] = (left_prim, left_y, left_people)
        out[1] = (right_prim, right_y, right_people)
        meta["details"] = {
            "normal_person_count_per_basket": normal_cnt,
            "left_basket_person_count": int(left_people),
            "right_basket_person_count": int(right_people),
        }
        return out, forced_positions, meta

    if label == ABNORMAL_LABEL_HEIGHT:
        threshold_world = max(0.0, float(cfg.height_abnormal_diff_threshold))
        local_to_world = max(1e-9, float(world_units_per_local_y))
        threshold_local = threshold_world / local_to_world
        margin_local = max(0.2, abs(threshold_local) + 0.2)
        min_air = float(cfg.height_people.air_z_min)
        max_air = float(cfg.height_people.air_z_max)
        target_pattern = str(cfg.target_scene_pattern or "random")
        ground_z = float(cfg.height_people.ground_z)
        eps = 1e-6

        if target_pattern == "all_ground":
            # 与 all_ground 语义冲突：由上层重映射 label，这里仅兜底降级避免中断。
            meta["implemented"] = False
            meta["trigger_type"] = ABNORMAL_LABEL_NORMAL
            meta["details"] = {
                "skipped_reason": "height_abnormal_incompatible_with_all_ground"
            }
            return out, forced_positions, meta

        if target_pattern == "single_airborne":
            left_is_air = float(left_y) > ground_z + eps
            right_is_air = float(right_y) > ground_z + eps
            if left_is_air == right_is_air:
                left_is_air = bool(rng.random() < 0.5)
                right_is_air = not left_is_air
            air_y = max(min_air, min(max_air, min_air + margin_local))
            if left_is_air:
                left_y = float(air_y)
                right_y = float(ground_z)
                left_people = max(1, int(left_people))
                right_people = int(max(0, right_people))
            else:
                left_y = float(ground_z)
                right_y = float(air_y)
                left_people = int(max(0, left_people))
                right_people = max(1, int(right_people))
            out[0] = (left_prim, left_y, left_people)
            out[1] = (right_prim, right_y, right_people)
        else:
            # double_airborne / random：保持两篮空中并拉大高度差
            left_new = max(min_air, min(max_air, min_air + 0.5))
            right_new = left_new + margin_local
            if right_new > max_air:
                right_new = max_air
                left_new = max(min_air, right_new - margin_local)
            left_y = float(left_new)
            right_y = float(right_new)
            left_people = max(1, int(left_people))
            right_people = max(1, int(right_people))
            out[0] = (left_prim, left_y, left_people)
            out[1] = (right_prim, right_y, right_people)

        meta["details"] = {
            "authored_translate_axis": "Y",
            "authored_left_translate_value": float(left_y),
            "authored_right_translate_value": float(right_y),
            "authored_height_diff": abs(float(left_y) - float(right_y)),
            "world_height_axis": str(world_height_axis or "Y").upper(),
            "world_units_per_authored_local_y": float(local_to_world),
            "threshold": float(threshold_world),
            "target_scene_pattern": target_pattern,
            "unit_note": "threshold/world_height_diff use world-space along world_height_axis",
        }
        return out, forced_positions, meta

    if label == ABNORMAL_LABEL_DUAL_RISK:
        threshold = float(cfg.dual_basket_risk_min_center_gap)
        left_t = initial_translates.get(left_prim, Gf.Vec3d(0.0, left_y, 0.0))
        right_t = initial_translates.get(right_prim, Gf.Vec3d(0.0, right_y, 0.0))
        up_axis = str(world_height_axis or cfg.stage_up_axis or "Y").upper()
        target_gap = max(0.1, threshold * 0.5)
        height_idx = int(authored_height_component_idx)
        if height_idx not in (0, 1, 2):
            height_idx = 1
        horizontal_idx = [i for i in (0, 1, 2) if i != height_idx]
        new_vals = [float(right_t[0]), float(right_t[1]), float(right_t[2])]
        # dual_basket_risk 仅在 authored 水平分量上移动，保留 authored 高度分量。
        new_vals[horizontal_idx[0]] = float(left_t[horizontal_idx[0]]) + target_gap
        new_vals[horizontal_idx[1]] = float(left_t[horizontal_idx[1]])
        # 关键：高度分量保留“当前样本采样结果(out)”而不是 initial translate，
        # 避免 single_airborne 的 airborne 吊篮被回写到初始地面高度。
        new_vals[height_idx] = float(right_y)
        new_right = Gf.Vec3d(new_vals[0], new_vals[1], new_vals[2])
        forced_positions[right_prim] = new_right
        gap = _horizontal_gap_by_up_axis(
            [float(left_t[0]), float(left_t[1]), float(left_t[2])],
            [float(new_right[0]), float(new_right[1]), float(new_right[2])],
            up_axis,
        )
        meta["details"] = {
            "left_basket_center": [float(left_t[0]), float(left_t[1]), float(left_t[2])],
            "right_basket_center": [float(new_right[0]), float(new_right[1]), float(new_right[2])],
            "horizontal_center_gap": float(gap),
            "threshold": float(threshold),
            "horizontal_plane": (
                "XY" if up_axis == "Z" else ("YZ" if up_axis == "X" else "XZ")
            ),
            "up_axis": up_axis,
            "authored_height_component_idx": int(height_idx),
            "authored_height_component_name": ("X", "Y", "Z")[height_idx],
        }
        return out, forced_positions, meta

    return out, forced_positions, meta


def is_abnormal_label_compatible_with_scene_pattern(
    abnormal_label: str, scene_pattern: str
) -> bool:
    label = str(abnormal_label or ABNORMAL_LABEL_NORMAL)
    pattern = str(scene_pattern or "random")
    if label in (ABNORMAL_LABEL_NORMAL,):
        return True
    if label == ABNORMAL_LABEL_RESERVED:
        return False
    if label == ABNORMAL_LABEL_HEIGHT:
        return pattern in ("single_airborne", "double_airborne", "random")
    if label in (ABNORMAL_LABEL_PERSON_COUNT, ABNORMAL_LABEL_DUAL_RISK):
        return True
    return False


def resolve_compatible_abnormal_label(
    requested_label: str, scene_pattern: str, rng: random.Random
) -> str:
    label = str(requested_label or ABNORMAL_LABEL_NORMAL)
    if is_abnormal_label_compatible_with_scene_pattern(label, scene_pattern):
        return label
    candidates = [
        x
        for x in ABNORMAL_LABELS_IMPLEMENTED
        if is_abnormal_label_compatible_with_scene_pattern(x, scene_pattern)
    ]
    if not candidates:
        return ABNORMAL_LABEL_NORMAL
    return candidates[int(rng.randrange(len(candidates)))]


def validate_target_scene_pattern(
    target_pattern: str,
    inferred_pattern: str,
    baskets: Optional[Sequence[dict]] = None,
    ground_height: float = 0.0,
    ground_eps: float = 1e-6,
) -> None:
    if baskets is not None:
        inferred_pattern = infer_scene_pattern(
            baskets=baskets,
            ground_height=ground_height,
            ground_eps=ground_eps,
        )
    if target_pattern != "random" and target_pattern != inferred_pattern:
        raise ValueError(
            f"目标场景模式校验失败: target={target_pattern}, inferred={inferred_pattern}"
        )


def derive_batch_artifact_paths(
    output_path: str, target_scene_pattern: str
) -> Tuple[str, str, str, str]:
    base_dir = "/home/uniubi/xuanyuan/camera05/camera03"
    stem, ext = os.path.splitext(os.path.basename(output_path))
    if not ext:
        ext = ".usd"
    if target_scene_pattern != "random":
        output_stem = f"{stem}_{target_scene_pattern}"
        suffix = f"_{target_scene_pattern}"
    else:
        output_stem = stem
        suffix = ""
    output_pattern = os.path.join(base_dir, f"{output_stem}_seed{{seed}}{ext}")
    summary_path = os.path.join(base_dir, f"batch_summary{suffix}.json")
    manifest_path = os.path.join(base_dir, f"dataset_manifest{suffix}.jsonl")
    stats_path = os.path.join(base_dir, f"dataset_stats{suffix}.json")
    return output_pattern, summary_path, manifest_path, stats_path


# ---------------------------------------------------------------------------
# 配额数据集模式
# ---------------------------------------------------------------------------


def parse_dataset_scene_quotas(text: str) -> dict:
    allowed = {"single_airborne", "double_airborne", "all_ground"}
    quotas = {k: 0 for k in allowed}
    if not text.strip():
        return quotas
    for item in text.split(","):
        kv = item.strip()
        if not kv:
            continue
        if "=" not in kv:
            raise ValueError(f"dataset-scene-quotas 格式错误：{kv}")
        k, v = kv.split("=", 1)
        k = k.strip()
        v = v.strip()
        if k not in allowed:
            raise ValueError(f"dataset-scene-quotas 包含不支持的模式：{k}")
        n = int(v)
        if n < 0:
            raise ValueError(f"dataset-scene-quotas 数量必须为非负整数：{kv}")
        quotas[k] = n
    return quotas


ABNORMAL_SCHEMA_VERSION = "abnormal_v1"
ABNORMAL_LABEL_NORMAL = "normal"
ABNORMAL_LABEL_PERSON_COUNT = "person_count_abnormal"
ABNORMAL_LABEL_HEIGHT = "height_abnormal"
ABNORMAL_LABEL_DUAL_RISK = "dual_basket_risk"
ABNORMAL_LABEL_RESERVED = "safety_violation_reserved"
ABNORMAL_LABELS_ALL = [
    ABNORMAL_LABEL_NORMAL,
    ABNORMAL_LABEL_PERSON_COUNT,
    ABNORMAL_LABEL_HEIGHT,
    ABNORMAL_LABEL_DUAL_RISK,
    ABNORMAL_LABEL_RESERVED,
]
ABNORMAL_LABELS_IMPLEMENTED = [
    ABNORMAL_LABEL_PERSON_COUNT,
    ABNORMAL_LABEL_HEIGHT,
    ABNORMAL_LABEL_DUAL_RISK,
]


def parse_key_value_int_map(text: str, arg_name: str) -> Dict[str, int]:
    out: Dict[str, int] = {}
    if not str(text).strip():
        return out
    for item in str(text).split(","):
        kv = item.strip()
        if not kv:
            continue
        if "=" not in kv:
            raise ValueError(f"{arg_name} 格式错误：{kv}")
        k, v = kv.split("=", 1)
        k = k.strip()
        v = v.strip()
        out[k] = int(v)
    return out


def parse_key_value_float_map(text: str, arg_name: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not str(text).strip():
        return out
    for item in str(text).split(","):
        kv = item.strip()
        if not kv:
            continue
        if "=" not in kv:
            raise ValueError(f"{arg_name} 格式错误：{kv}")
        k, v = kv.split("=", 1)
        k = k.strip()
        v = v.strip()
        out[k] = float(v)
    return out


def _validate_abnormal_ratio_quota_keys(items: Dict[str, float], arg_name: str) -> None:
    for k in items:
        if k not in ABNORMAL_LABELS_ALL:
            raise ValueError(f"{arg_name} 包含不支持的 abnormal_label: {k}")
    if ABNORMAL_LABEL_RESERVED in items:
        raise ValueError(
            f"{arg_name} 显式传入了 {ABNORMAL_LABEL_RESERVED}，但 v1 未实现该异常生成器"
        )


def _distribute_count_by_ratios(total: int, ratios: Dict[str, float]) -> Dict[str, int]:
    if total <= 0:
        return {k: 0 for k in ABNORMAL_LABELS_IMPLEMENTED}
    labels = list(ABNORMAL_LABELS_IMPLEMENTED)
    base_alloc: Dict[str, int] = {k: 0 for k in labels}
    if not ratios:
        ratios = {k: 1.0 / len(labels) for k in labels}
    for k in labels:
        if float(ratios.get(k, 0.0)) < 0:
            raise ValueError(f"abnormal_label_ratios 数值必须为非负: {k}={ratios.get(k)}")
    s = sum(float(ratios.get(k, 0.0)) for k in labels)
    if s <= 0:
        ratios = {k: 1.0 / len(labels) for k in labels}
        s = 1.0
    raw = {k: total * float(ratios.get(k, 0.0)) / s for k in labels}
    for k in labels:
        base_alloc[k] = int(math.floor(raw[k]))
    remain = int(total - sum(base_alloc.values()))
    frac_rank = sorted(
        labels,
        key=lambda x: (raw[x] - math.floor(raw[x]), x),
        reverse=True,
    )
    i = 0
    while remain > 0 and frac_rank:
        base_alloc[frac_rank[i % len(frac_rank)]] += 1
        i += 1
        remain -= 1
    return base_alloc


def build_abnormal_label_plan(
    total_samples: int,
    seed: int,
    abnormal_ratio: float,
    abnormal_target_count: int,
    abnormal_label_ratios_text: str,
    abnormal_label_quotas_text: str,
) -> Tuple[List[str], dict]:
    total = max(0, int(total_samples))
    label_plan = [ABNORMAL_LABEL_NORMAL] * total
    requested: Dict[str, int] = {k: 0 for k in ABNORMAL_LABELS_ALL}

    ratio_map = parse_key_value_float_map(abnormal_label_ratios_text, "abnormal_label_ratios")
    quota_map = parse_key_value_int_map(abnormal_label_quotas_text, "abnormal_label_quotas")
    _validate_abnormal_ratio_quota_keys(ratio_map, "abnormal_label_ratios")
    _validate_abnormal_ratio_quota_keys(quota_map, "abnormal_label_quotas")
    for k, v in quota_map.items():
        if int(v) < 0:
            raise ValueError(f"abnormal_label_quotas 数值必须为非负整数: {k}={v}")
    for k, v in ratio_map.items():
        if float(v) < 0:
            raise ValueError(f"abnormal_label_ratios 数值必须为非负: {k}={v}")

    if quota_map:
        abnormal_total = sum(int(quota_map.get(k, 0)) for k in ABNORMAL_LABELS_IMPLEMENTED)
        if abnormal_total > total:
            raise ValueError(
                f"abnormal_label_quotas 总数({abnormal_total}) 超过 dataset-size({total})"
            )
        for k in ABNORMAL_LABELS_IMPLEMENTED:
            requested[k] = int(quota_map.get(k, 0))
    elif int(abnormal_target_count) > 0:
        abnormal_total = min(int(abnormal_target_count), total)
        alloc = _distribute_count_by_ratios(abnormal_total, ratio_map)
        for k in ABNORMAL_LABELS_IMPLEMENTED:
            requested[k] = int(alloc.get(k, 0))
    elif float(abnormal_ratio) > 0.0:
        if float(abnormal_ratio) < 0.0 or float(abnormal_ratio) > 1.0:
            raise ValueError("abnormal_ratio 必须在 [0,1] 区间")
        abnormal_total = int(round(float(abnormal_ratio) * total))
        alloc = _distribute_count_by_ratios(abnormal_total, ratio_map)
        for k in ABNORMAL_LABELS_IMPLEMENTED:
            requested[k] = int(alloc.get(k, 0))

    abnormal_labels_pool: List[str] = []
    for k in ABNORMAL_LABELS_IMPLEMENTED:
        abnormal_labels_pool.extend([k] * int(requested.get(k, 0)))
    normal_count = max(0, total - len(abnormal_labels_pool))
    requested[ABNORMAL_LABEL_NORMAL] = normal_count
    label_plan = abnormal_labels_pool + [ABNORMAL_LABEL_NORMAL] * normal_count
    rng = random.Random(int(seed) + 104729)
    rng.shuffle(label_plan)
    return label_plan, {
        "schema_version": ABNORMAL_SCHEMA_VERSION,
        "implemented_labels": list(ABNORMAL_LABELS_IMPLEMENTED),
        "requested": requested,
        "plan_size": len(label_plan),
    }


def normalize_abnormal_fields_from_record(rec: dict) -> Tuple[str, bool, dict, bool]:
    raw = rec.get("abnormal_label", None)
    missing = raw in (None, "")
    label = str(raw) if not missing else ABNORMAL_LABEL_NORMAL
    if label not in ABNORMAL_LABELS_ALL:
        label = ABNORMAL_LABEL_NORMAL
    is_abnormal = bool(label != ABNORMAL_LABEL_NORMAL)
    meta = rec.get("abnormal_meta", {}) or {}
    if not isinstance(meta, dict):
        meta = {}
    if "schema_version" not in meta:
        meta["schema_version"] = ABNORMAL_SCHEMA_VERSION
    if "implemented" not in meta:
        meta["implemented"] = bool(label in ABNORMAL_LABELS_IMPLEMENTED)
    if "trigger_type" not in meta:
        meta["trigger_type"] = label
    if "details" not in meta or not isinstance(meta.get("details"), dict):
        meta["details"] = {}
    return label, is_abnormal, meta, missing

def derive_dataset_sample_id(pattern: str, index_within_pattern: int) -> str:
    return f"{pattern}_{index_within_pattern:04d}"


def _default_dataset_run_name() -> str:
    return "dataset_run_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def _parse_dataset_size_to_quotas(total_samples: int) -> dict:
    if total_samples <= 0:
        return {"single_airborne": 0, "double_airborne": 0, "all_ground": 0}
    ordered = ["single_airborne", "double_airborne", "all_ground"]
    base = total_samples // 3
    rem = total_samples % 3
    quotas = {k: base for k in ordered}
    for i in range(rem):
        quotas[ordered[i]] += 1
    return quotas


def derive_dataset_artifact_paths(
    output_path: str,
    dataset_run_dir: str = "",
    dataset_run_name: str = "",
) -> dict:
    base_dir = "/home/uniubi/xuanyuan/camera05/camera03"
    stem, ext = os.path.splitext(os.path.basename(output_path))
    if not ext:
        ext = ".usd"
    if dataset_run_dir.strip():
        run_name = dataset_run_name.strip() or _default_dataset_run_name()
        run_root = os.path.abspath(os.path.join(dataset_run_dir, run_name))
        usd_dir = os.path.join(run_root, "usd")
        manifests_dir = os.path.join(run_root, "manifests")
        stats_dir = os.path.join(run_root, "stats")
        logs_dir = os.path.join(run_root, "logs")
        return {
            "run_root": run_root,
            "usd_dir": usd_dir,
            "manifests_dir": manifests_dir,
            "stats_dir": stats_dir,
            "logs_dir": logs_dir,
            "output_pattern": os.path.join(usd_dir, f"{stem}_dataset_{{sample_id}}{ext}"),
            "summary_path": os.path.join(logs_dir, "dataset_batch_summary.json"),
            "manifest_path": os.path.join(manifests_dir, "dataset_manifest_full.jsonl"),
            "stats_path": os.path.join(stats_dir, "dataset_stats_full.json"),
            "readable_summary_path": os.path.join(logs_dir, "dataset_readable_summary.txt"),
            "check_report_path": os.path.join(logs_dir, "dataset_check_report.json"),
            "spotcheck_path": os.path.join(logs_dir, "spotcheck_samples.jsonl"),
        }
    return {
        "run_root": base_dir,
        "usd_dir": base_dir,
        "manifests_dir": base_dir,
        "stats_dir": base_dir,
        "logs_dir": base_dir,
        "output_pattern": os.path.join(base_dir, f"{stem}_dataset_{{sample_id}}{ext}"),
        "summary_path": os.path.join(base_dir, "dataset_batch_summary.json"),
        "manifest_path": os.path.join(base_dir, "dataset_manifest_full.jsonl"),
        "stats_path": os.path.join(base_dir, "dataset_stats_full.json"),
        "readable_summary_path": os.path.join(base_dir, "dataset_readable_summary.txt"),
        "check_report_path": os.path.join(base_dir, "dataset_check_report.json"),
        "spotcheck_path": os.path.join(base_dir, "spotcheck_samples.jsonl"),
    }


def generate_dataset_by_quotas(
    quotas: dict,
    base_seed: int,
    input_path: str,
    output_path: str,
    baskets: Sequence[str],
    pattern: Optional[str],
    persons: Sequence[str],
    camera_prim: str,
    hc: HeightPeopleConstraint,
    cam_region: Optional[CameraRegion],
    look: Optional[CameraLookAtConfig],
    same_level_threshold: float,
    visibility_min_distance: float,
    visibility_max_distance: float,
    visibility_target_radius: float,
    visibility_max_offaxis_deg: float,
    abnormal_ratio: float,
    abnormal_target_count: int,
    abnormal_label_ratios_text: str,
    abnormal_label_quotas_text: str,
    normal_person_count_per_basket: int,
    height_abnormal_diff_threshold: float,
    dual_basket_risk_min_center_gap: float,
    visibility_debug: bool = False,
    artifacts: Optional[dict] = None,
    dry_run: bool = False,
) -> tuple[list, dict, dict]:
    plan = []
    for scene_pattern in ("single_airborne", "double_airborne", "all_ground"):
        n = int(quotas.get(scene_pattern, 0))
        for i in range(1, n + 1):
            plan.append((scene_pattern, i))

    artifacts = artifacts or derive_dataset_artifact_paths(output_path)
    out_pattern = artifacts["output_pattern"]
    abnormal_plan, abnormal_request = build_abnormal_label_plan(
        total_samples=len(plan),
        seed=int(base_seed),
        abnormal_ratio=float(abnormal_ratio),
        abnormal_target_count=int(abnormal_target_count),
        abnormal_label_ratios_text=abnormal_label_ratios_text,
        abnormal_label_quotas_text=abnormal_label_quotas_text,
    )
    if dry_run:
        print("[dry-run] 数据集生成计划：")
        print(f"[dry-run] total_samples={len(plan)} quotas={quotas} base_seed={base_seed}")
        print(f"[dry-run] output_pattern={out_pattern}")
        print(f"[dry-run] manifest={artifacts['manifest_path']}")
        print(f"[dry-run] stats={artifacts['stats_path']}")
        for idx, (scene_pattern, idx_in_pattern) in enumerate(plan, start=1):
            seed = int(base_seed) + (idx - 1)
            sample_id = derive_dataset_sample_id(scene_pattern, idx_in_pattern)
            requested_abnormal_label = (
                abnormal_plan[idx - 1] if idx - 1 < len(abnormal_plan) else ABNORMAL_LABEL_NORMAL
            )
            abnormal_label = resolve_compatible_abnormal_label(
                requested_abnormal_label,
                scene_pattern,
                random.Random(seed + 17),
            )
            print(
                f"[dry-run] sample_index={idx} sample_id={sample_id} "
                f"seed={seed} target={scene_pattern} abnormal_label={abnormal_label} "
                f"(requested={requested_abnormal_label})"
            )
        return [], artifacts, abnormal_request

    results = []
    for idx, (scene_pattern, idx_in_pattern) in enumerate(plan, start=1):
        seed = int(base_seed) + (idx - 1)
        sample_id = derive_dataset_sample_id(scene_pattern, idx_in_pattern)
        requested_abnormal_label = (
            abnormal_plan[idx - 1] if idx - 1 < len(abnormal_plan) else ABNORMAL_LABEL_NORMAL
        )
        abnormal_label = resolve_compatible_abnormal_label(
            requested_abnormal_label,
            scene_pattern,
            random.Random(seed + 17),
        )
        out_i = out_pattern.format(sample_id=sample_id)
        cfg = GeneratorConfig(
            input_path=input_path,
            output_path=out_i,
            seed=seed,
            basket_prim_paths=list(baskets),
            basket_name_pattern=pattern,
            person_prim_paths=list(persons),
            camera_prim_path=camera_prim,
            height_people=hc,
            camera_region=cam_region,
            camera_look_at=look,
            same_level_threshold=same_level_threshold,
            target_scene_pattern=scene_pattern,
            sample_id=sample_id,
            sample_index=idx,
            visibility_min_distance=float(visibility_min_distance),
            visibility_max_distance=float(visibility_max_distance),
            visibility_target_radius=float(visibility_target_radius),
            visibility_max_offaxis_deg=float(visibility_max_offaxis_deg),
            visibility_debug=bool(visibility_debug),
            abnormal_label=abnormal_label,
            normal_person_count_per_basket=int(normal_person_count_per_basket),
            height_abnormal_diff_threshold=float(height_abnormal_diff_threshold),
            dual_basket_risk_min_center_gap=float(dual_basket_risk_min_center_gap),
        )
        print(
            f"[gen] ===== 配额样本 sample_index={idx} sample_id={sample_id} "
            f"seed={seed} target={scene_pattern} abnormal_label={abnormal_label} "
            f"(requested={requested_abnormal_label}) ====="
        )
        results.append(SceneUsdGenerator(cfg).run())
    return results, artifacts, abnormal_request


# ---------------------------------------------------------------------------
# 数据清单输出
# ---------------------------------------------------------------------------


def write_dataset_manifest_jsonl(results: Sequence[dict], path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for r in sorted(results, key=lambda x: int(x.get("seed", 0))):
            baskets = list(r.get("baskets", []))
            b1 = baskets[0] if len(baskets) > 0 else {}
            b2 = baskets[1] if len(baskets) > 1 else {}
            sample_tags = [str(r.get("scene_pattern", ""))]
            hr = str(r.get("height_relation", "not_applicable"))
            if hr != "not_applicable":
                sample_tags.append(hr)
            rec = {
                "seed": int(r.get("seed", 0)),
                "output_usd": r.get("output_usd"),
                "target_scene_pattern": r.get("target_scene_pattern", "random"),
                "scene_pattern": r.get("scene_pattern"),
                "world_height_axis": r.get("world_height_axis", "Y"),
                "height_relation": hr,
                "camera_eye": r.get("camera_eye"),
                "camera_target": r.get("camera_target"),
                "visibility_check": r.get("visibility_check"),
                "abnormal_label": r.get("abnormal_label", ABNORMAL_LABEL_NORMAL),
                "is_abnormal": bool(r.get("abnormal_label", ABNORMAL_LABEL_NORMAL) != ABNORMAL_LABEL_NORMAL),
                "abnormal_meta": r.get("abnormal_meta", {}),
                "basket_1_prim": b1.get("prim"),
                "basket_1_person_count": b1.get("person_count"),
                "basket_1_world_height": b1.get("world_height_value"),
                "basket_1_final_y": b1.get("final_y"),
                "basket_1_state": b1.get("basket_state"),
                "basket_2_prim": b2.get("prim"),
                "basket_2_person_count": b2.get("person_count"),
                "basket_2_world_height": b2.get("world_height_value"),
                "basket_2_final_y": b2.get("final_y"),
                "basket_2_state": b2.get("basket_state"),
                "sample_tags": sample_tags,
            }
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def write_dataset_full_manifest_jsonl(results: Sequence[dict], path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for r in sorted(results, key=lambda x: int(x.get("sample_index", 0))):
            baskets = list(r.get("baskets", []))
            b1 = baskets[0] if len(baskets) > 0 else {}
            b2 = baskets[1] if len(baskets) > 1 else {}
            sample_tags = [
                str(r.get("target_scene_pattern", "random")),
                str(r.get("scene_pattern", "")),
            ]
            hr = str(r.get("height_relation", "not_applicable"))
            if hr != "not_applicable":
                sample_tags.append(hr)
            rec = {
                "sample_id": r.get("sample_id"),
                "sample_index": int(r.get("sample_index", 0)),
                "seed": int(r.get("seed", 0)),
                "target_scene_pattern": r.get("target_scene_pattern", "random"),
                "scene_pattern": r.get("scene_pattern"),
                "world_height_axis": r.get("world_height_axis", "Y"),
                "output_usd": r.get("output_usd"),
                "camera_eye": r.get("camera_eye"),
                "camera_target": r.get("camera_target"),
                "visibility_check": r.get("visibility_check"),
                "abnormal_label": r.get("abnormal_label", ABNORMAL_LABEL_NORMAL),
                "is_abnormal": bool(r.get("abnormal_label", ABNORMAL_LABEL_NORMAL) != ABNORMAL_LABEL_NORMAL),
                "abnormal_meta": r.get("abnormal_meta", {}),
                "basket_1_prim": b1.get("prim"),
                "basket_1_person_count": b1.get("person_count"),
                "basket_1_world_height": b1.get("world_height_value"),
                "basket_1_final_y": b1.get("final_y"),
                "basket_1_state": b1.get("basket_state"),
                "basket_2_prim": b2.get("prim"),
                "basket_2_person_count": b2.get("person_count"),
                "basket_2_world_height": b2.get("world_height_value"),
                "basket_2_final_y": b2.get("final_y"),
                "basket_2_state": b2.get("basket_state"),
                "sample_tags": sample_tags,
            }
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def write_dataset_stats(
    results: Sequence[dict],
    path: str,
    generated_files: Sequence[str],
    failed_rule_sample_topn: int = 20,
) -> None:
    scene_pattern_counts = {
        "single_airborne": 0,
        "double_airborne": 0,
        "all_ground": 0,
    }
    height_relation_counts = {
        "different_level": 0,
        "same_level": 0,
        "not_applicable": 0,
    }
    basket_state_counts = {
        "ground_empty": 0,
        "air_with_person": 0,
        "ground_with_person": 0,
        "unknown": 0,
    }
    target_scene_pattern_counts = {
        "random": 0,
        "single_airborne": 0,
        "double_airborne": 0,
        "all_ground": 0,
    }
    visibility_status_counts = {"ok": 0, "fail": 0, "skip": 0}
    visibility_failed_samples = []
    abnormal_actual_distribution = {k: 0 for k in ABNORMAL_LABELS_ALL}
    legacy_missing_abnormal_label_count = 0
    abnormal_x_visibility = {
        k: {"ok": 0, "fail": 0, "skip": 0} for k in ABNORMAL_LABELS_ALL
    }

    for r in results:
        sp = str(r.get("scene_pattern", ""))
        if sp in scene_pattern_counts:
            scene_pattern_counts[sp] += 1
        tp = str(r.get("target_scene_pattern", "random"))
        if tp in target_scene_pattern_counts:
            target_scene_pattern_counts[tp] += 1
        hr = str(r.get("height_relation", "not_applicable"))
        if hr in height_relation_counts:
            height_relation_counts[hr] += 1
        for b in r.get("baskets", []):
            bs = str(b.get("basket_state", "unknown"))
            if bs in basket_state_counts:
                basket_state_counts[bs] += 1
            else:
                basket_state_counts["unknown"] += 1
        vc = r.get("visibility_check", {}) or {}
        vs = str(vc.get("status", "skip"))
        if vs in visibility_status_counts:
            visibility_status_counts[vs] += 1
        else:
            visibility_status_counts["skip"] += 1
        if vs == "fail":
            visibility_failed_samples.append(
                {
                    "sample_id": r.get("sample_id"),
                    "seed": r.get("seed"),
                    "output_usd": r.get("output_usd"),
                    "failed_rules": vc.get("failed_rules", []),
                }
            )
        ab_label, _, _, legacy_missing = normalize_abnormal_fields_from_record(r)
        if legacy_missing:
            legacy_missing_abnormal_label_count += 1
        abnormal_actual_distribution[ab_label] = abnormal_actual_distribution.get(ab_label, 0) + 1
        if vs not in ("ok", "fail", "skip"):
            vs = "skip"
        if ab_label not in abnormal_x_visibility:
            abnormal_x_visibility[ab_label] = {"ok": 0, "fail": 0, "skip": 0}
        abnormal_x_visibility[ab_label][vs] += 1

    actual_abnormal_count = int(
        sum(v for k, v in abnormal_actual_distribution.items() if k != ABNORMAL_LABEL_NORMAL)
    )
    total = max(1, len(results))
    stats = {
        "total_samples": len(results),
        "scene_pattern_counts": scene_pattern_counts,
        "target_scene_pattern_counts": target_scene_pattern_counts,
        "height_relation_counts": height_relation_counts,
        "basket_state_counts": basket_state_counts,
        "visibility_status_counts": visibility_status_counts,
        "visibility_failed_samples": visibility_failed_samples,
        "failed_rule_counts": _build_failed_rule_counts(visibility_failed_samples),
        "failed_rule_sample_ids": _build_failed_rule_sample_map(
            visibility_failed_samples, max_samples_per_rule=failed_rule_sample_topn
        ),
        "abnormal_summary": {
            "schema_version": ABNORMAL_SCHEMA_VERSION,
            "implemented_labels": list(ABNORMAL_LABELS_IMPLEMENTED),
            "requested": {},
            "actual_distribution": abnormal_actual_distribution,
            "actual_abnormal_count": actual_abnormal_count,
            "actual_abnormal_ratio": float(actual_abnormal_count) / float(total),
            "legacy_missing_abnormal_label_count": int(legacy_missing_abnormal_label_count),
        },
        "abnormal_x_visibility": abnormal_x_visibility,
        "generated_files": list(generated_files),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)


def write_dataset_full_stats(
    results: Sequence[dict],
    path: str,
    generated_files: Sequence[str],
    abnormal_requested: Optional[dict] = None,
    failed_rule_sample_topn: int = 20,
) -> None:
    scene_pattern_counts = {
        "single_airborne": 0,
        "double_airborne": 0,
        "all_ground": 0,
    }
    target_scene_pattern_counts = {
        "single_airborne": 0,
        "double_airborne": 0,
        "all_ground": 0,
    }
    height_relation_counts = {
        "different_level": 0,
        "same_level": 0,
        "not_applicable": 0,
    }
    basket_state_counts = {
        "ground_empty": 0,
        "air_with_person": 0,
        "ground_with_person": 0,
        "unknown": 0,
    }
    visibility_status_counts = {"ok": 0, "fail": 0, "skip": 0}
    visibility_failed_samples = []
    abnormal_actual_distribution = {k: 0 for k in ABNORMAL_LABELS_ALL}
    legacy_missing_abnormal_label_count = 0
    abnormal_x_visibility = {
        k: {"ok": 0, "fail": 0, "skip": 0} for k in ABNORMAL_LABELS_ALL
    }

    sample_id_list = []
    for r in sorted(results, key=lambda x: int(x.get("sample_index", 0))):
        sample_id_list.append(str(r.get("sample_id", "")))
        sp = str(r.get("scene_pattern", ""))
        if sp in scene_pattern_counts:
            scene_pattern_counts[sp] += 1
        tp = str(r.get("target_scene_pattern", ""))
        if tp in target_scene_pattern_counts:
            target_scene_pattern_counts[tp] += 1
        hr = str(r.get("height_relation", "not_applicable"))
        if hr in height_relation_counts:
            height_relation_counts[hr] += 1
        for b in r.get("baskets", []):
            bs = str(b.get("basket_state", "unknown"))
            if bs in basket_state_counts:
                basket_state_counts[bs] += 1
            else:
                basket_state_counts["unknown"] += 1
        vc = r.get("visibility_check", {}) or {}
        vs = str(vc.get("status", "skip"))
        if vs in visibility_status_counts:
            visibility_status_counts[vs] += 1
        else:
            visibility_status_counts["skip"] += 1
        if vs == "fail":
            visibility_failed_samples.append(
                {
                    "sample_id": r.get("sample_id"),
                    "seed": r.get("seed"),
                    "output_usd": r.get("output_usd"),
                    "failed_rules": vc.get("failed_rules", []),
                }
            )
        ab_label, _, _, legacy_missing = normalize_abnormal_fields_from_record(r)
        if legacy_missing:
            legacy_missing_abnormal_label_count += 1
        abnormal_actual_distribution[ab_label] = abnormal_actual_distribution.get(ab_label, 0) + 1
        if vs not in ("ok", "fail", "skip"):
            vs = "skip"
        if ab_label not in abnormal_x_visibility:
            abnormal_x_visibility[ab_label] = {"ok": 0, "fail": 0, "skip": 0}
        abnormal_x_visibility[ab_label][vs] += 1

    actual_abnormal_count = int(
        sum(v for k, v in abnormal_actual_distribution.items() if k != ABNORMAL_LABEL_NORMAL)
    )
    total = max(1, len(results))
    normalized_requested, normalized_plan_size = _normalize_abnormal_requested_payload(
        abnormal_requested
    )
    stats = {
        "total_samples": len(results),
        "target_scene_pattern_counts": target_scene_pattern_counts,
        "scene_pattern_counts": scene_pattern_counts,
        "height_relation_counts": height_relation_counts,
        "basket_state_counts": basket_state_counts,
        "visibility_status_counts": visibility_status_counts,
        "visibility_failed_samples": visibility_failed_samples,
        "failed_rule_counts": _build_failed_rule_counts(visibility_failed_samples),
        "failed_rule_sample_ids": _build_failed_rule_sample_map(
            visibility_failed_samples, max_samples_per_rule=failed_rule_sample_topn
        ),
        "abnormal_summary": {
            "schema_version": ABNORMAL_SCHEMA_VERSION,
            "implemented_labels": list(ABNORMAL_LABELS_IMPLEMENTED),
            "requested": normalized_requested,
            "plan_size": (
                int(normalized_plan_size)
                if int(normalized_plan_size) > 0
                else int(len(results))
            ),
            "actual_distribution": abnormal_actual_distribution,
            "actual_abnormal_count": actual_abnormal_count,
            "actual_abnormal_ratio": float(actual_abnormal_count) / float(total),
            "legacy_missing_abnormal_label_count": int(legacy_missing_abnormal_label_count),
        },
        "abnormal_x_visibility": abnormal_x_visibility,
        "sample_id_list": sample_id_list,
        "generated_files": list(generated_files),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(stats, f, ensure_ascii=False, indent=2)


def _parse_manifest_jsonl(path: str) -> List[dict]:
    records: List[dict] = []
    with open(path, "r", encoding="utf-8") as f:
        for i, line in enumerate(f, start=1):
            s = line.strip()
            if not s:
                continue
            try:
                records.append(json.loads(s))
            except json.JSONDecodeError as e:
                raise ValueError(f"manifest 第 {i} 行 JSON 解析失败: {e}") from e
    return records


def _derive_stats_path_from_manifest(manifest_path: str) -> str:
    base = os.path.basename(manifest_path)
    stats_base = base.replace("dataset_manifest", "dataset_stats")
    if stats_base == base and base.endswith(".jsonl"):
        stats_base = base[:-6] + ".json"
    return os.path.join(os.path.dirname(manifest_path), stats_base)


def _safe_float(v, default: float = 0.0) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _build_failed_rule_counts(items: Sequence[dict]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for it in items:
        for rule in it.get("failed_rules", []) or []:
            k = str(rule)
            out[k] = out.get(k, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: (-kv[1], kv[0])))


def _build_failed_rule_sample_map(
    items: Sequence[dict], max_samples_per_rule: int = 20
) -> Dict[str, List[str]]:
    out: Dict[str, List[str]] = {}
    n = max(1, int(max_samples_per_rule))
    for it in items:
        sid = str(it.get("sample_id", ""))
        for rule in it.get("failed_rules", []) or []:
            k = str(rule)
            if k not in out:
                out[k] = []
            if sid and len(out[k]) < n:
                out[k].append(sid)
    return dict(sorted(out.items(), key=lambda kv: kv[0]))


def _normalize_abnormal_requested_payload(
    abnormal_requested: Optional[dict],
) -> Tuple[Dict[str, int], int]:
    if not isinstance(abnormal_requested, dict):
        return {}, 0

    requested_raw = abnormal_requested.get("requested", abnormal_requested)
    if not isinstance(requested_raw, dict):
        requested_raw = {}

    requested: Dict[str, int] = {}
    for k, v in requested_raw.items():
        if k in ABNORMAL_LABELS_ALL:
            requested[k] = int(_safe_float(v, 0.0))

    # 兼容历史误嵌套：plan_size 可能在 requested 内层。
    plan_size = int(_safe_float(abnormal_requested.get("plan_size", 0), 0.0))
    if plan_size <= 0:
        plan_size = int(_safe_float(requested_raw.get("plan_size", 0), 0.0))
    return requested, max(0, plan_size)


def build_spotcheck_records(
    results: Sequence[dict],
    count: int,
    mode: str,
    random_seed: int = 0,
) -> List[dict]:
    n = max(0, int(count))
    if n <= 0:
        return []

    rows = []
    for r in sorted(results, key=lambda x: int(x.get("sample_index", 0) or 0)):
        vc = r.get("visibility_check", {}) or {}
        status = vc.get("status", "skip")
        failed_rules = vc.get("failed_rules", [])
        rows.append(
            {
                "sample_id": r.get("sample_id"),
                "scene_pattern": r.get("scene_pattern"),
                "usd_path": r.get("output_usd"),
                # 兼容字段：便于外部脚本直接按 key 读取
                "visibility_status": status,
                "failed_rules": failed_rules,
                # 用户要求口径：visibility_check.status
                "visibility_check.status": status,
                "visibility_check.failed_rules": failed_rules,
                "seed": r.get("seed"),
            }
        )

    if mode == "random":
        rng = random.Random(int(random_seed))
        pool = list(rows)
        rng.shuffle(pool)
        return pool[:n]

    # default fail-first: 先 fail，再 ok/skip 做对照
    fail_rows = [x for x in rows if str(x.get("visibility_status")) == "fail"]
    non_fail_rows = [x for x in rows if str(x.get("visibility_status")) != "fail"]
    return (fail_rows + non_fail_rows)[:n]


def _resolve_usd_path_candidates(manifest_path: str, raw_usd_path: str) -> Tuple[Optional[str], List[str]]:
    """
    针对 archived manifest 路径失配做兜底：
    1) 先用 manifest 原始 output_usd
    2) 再尝试相对 manifest 目录与常见归档目录的候选路径
    """
    raw = raw_usd_path.strip()
    if not raw:
        return None, []

    manifest_dir = os.path.dirname(os.path.abspath(manifest_path))
    basename = os.path.basename(raw)
    candidates: List[str] = []

    def _add(p: str) -> None:
        ap = os.path.abspath(p)
        if ap not in candidates:
            candidates.append(ap)

    # 1) 原始路径（优先）
    _add(raw)

    # 2) 以 manifest 所在目录为基准
    _add(os.path.join(manifest_dir, basename))

    # 3) ../generated_usd/<basename>
    _add(os.path.join(manifest_dir, "..", "generated_usd", basename))

    # 4) _archive/generated_usd/<basename>（针对从项目根或其他目录执行）
    _add(os.path.join(manifest_dir, "_archive", "generated_usd", basename))

    # 5) manifest 在 generated_meta 时，联想 sibling generated_usd
    if os.path.basename(manifest_dir) == "generated_meta":
        parent = os.path.dirname(manifest_dir)
        _add(os.path.join(parent, "generated_usd", basename))
        _add(os.path.join(parent, "_archive", "generated_usd", basename))

    for p in candidates:
        if os.path.isfile(p):
            return p, candidates
    return None, candidates


def _check_scene_pattern_y_rule(
    scene_pattern: str,
    y1: float,
    y2: float,
    ground_z: float,
    ground_eps: float,
    air_min: float,
    air_max: float,
    height_axis: str = "Y",
) -> Tuple[bool, str]:
    axis_name = str(height_axis or "Y").upper()
    inferred = infer_scene_pattern(
        baskets=[
            {"world_height_value": float(y1)},
            {"world_height_value": float(y2)},
        ],
        ground_height=float(ground_z),
        ground_eps=float(ground_eps),
    )
    ok = inferred == scene_pattern
    if scene_pattern == "all_ground":
        msg = (
            f"all_ground 期望两个吊篮都接近 ground_z（高度轴={axis_name}）"
            if not ok
            else "all_ground 校验通过"
        )
        return ok, msg
    if scene_pattern == "single_airborne":
        msg = (
            f"single_airborne 期望一个接近 ground_z，另一个在 air_range（高度轴={axis_name}）"
            if not ok
            else "single_airborne 校验通过"
        )
        return ok, msg
    if scene_pattern == "double_airborne":
        msg = (
            f"double_airborne 期望两个吊篮都在 air_range（高度轴={axis_name}）"
            if not ok
            else "double_airborne 校验通过"
        )
        return ok, msg
    return False, f"不支持的 scene_pattern: {scene_pattern}"


def _infer_height_axis_from_records(records: Sequence[dict]) -> str:
    votes: Dict[str, int] = {"X": 0, "Y": 0, "Z": 0}
    for rec in records:
        axis = str(
            ((rec.get("abnormal_meta") or {}).get("details") or {}).get("world_height_axis", "")
        ).upper()
        if axis in votes:
            votes[axis] += 1
    picked = max(votes.items(), key=lambda kv: kv[1])[0]
    return picked if votes[picked] > 0 else "Y"


def _infer_height_axis_from_record(rec: dict, fallback_axis: str = "Y") -> str:
    axis = str(rec.get("world_height_axis", "")).upper()
    if axis in ("X", "Y", "Z"):
        return axis
    details = ((rec.get("abnormal_meta") or {}).get("details") or {})
    axis = str(details.get("world_height_axis", "")).upper()
    if axis in ("X", "Y", "Z"):
        return axis
    return str(fallback_axis or "Y").upper()


def validate_dataset_manifest(
    manifest_path: str,
    stats_path: str,
    ground_z: float,
    ground_eps: float,
    air_min: float,
    air_max: float,
) -> bool:
    manifest_abs = os.path.abspath(manifest_path)
    stats_abs = os.path.abspath(stats_path)
    records = _parse_manifest_jsonl(manifest_abs)
    if not os.path.isfile(stats_abs):
        raise FileNotFoundError(f"找不到 stats 文件：{stats_abs}")
    with open(stats_abs, "r", encoding="utf-8") as f:
        stats = json.load(f)

    failures: List[str] = []
    pass_count = 0
    default_height_axis = _infer_height_axis_from_records(records)

    # 1) scene_pattern 是否与 target_scene_pattern 一致
    legacy_missing_abnormal_label_count = 0
    for idx, rec in enumerate(records, start=1):
        target = str(rec.get("target_scene_pattern", ""))
        scene = str(rec.get("scene_pattern", ""))
        if scene != target:
            sid = rec.get("sample_id") or f"line_{idx}"
            failures.append(
                f"[F] sample={sid} scene_pattern({scene}) != target_scene_pattern({target})"
            )
        ab_label, _, _, missing_ab = normalize_abnormal_fields_from_record(rec)
        if missing_ab:
            legacy_missing_abnormal_label_count += 1
        elif ab_label not in ABNORMAL_LABELS_ALL:
            sid = rec.get("sample_id") or f"line_{idx}"
            failures.append(f"[F] sample={sid} abnormal_label 不在 schema 枚举内: {ab_label}")
    if not failures:
        pass_count += 1
        print("[PASS] scene_pattern 与 target_scene_pattern 一致")
    else:
        print("[FAIL] scene_pattern 与 target_scene_pattern 存在不一致")

    # 2) 两个吊篮 world-height 符合 scene_pattern 规则（复用 USD inspect 读回）
    basket_rule_failures: List[str] = []
    for idx, rec in enumerate(records, start=1):
        sid = rec.get("sample_id") or f"line_{idx}"
        scene = str(rec.get("scene_pattern", ""))
        usd_path_raw = str(rec.get("output_usd", "")).strip()
        b1 = str(rec.get("basket_1_prim", "")).strip()
        b2 = str(rec.get("basket_2_prim", "")).strip()
        if not usd_path_raw or not b1 or not b2:
            basket_rule_failures.append(
                f"[F] sample={sid} 缺少 output_usd 或 basket prim 字段"
            )
            continue

        resolved_usd_path, tried_paths = _resolve_usd_path_candidates(
            manifest_abs, usd_path_raw
        )
        if not resolved_usd_path:
            basket_rule_failures.append(
                f"[F][missing_file] sample={sid} raw_usd={usd_path_raw} "
                f"tried={tried_paths} hit=<none>"
            )
            continue
        try:
            height_axis = _infer_height_axis_from_record(rec, default_height_axis)
            axis_idx = _axis_to_index(height_axis)
            t_map = inspect_usd_translate_map(resolved_usd_path, [b1, b2])
            y1 = float(t_map[b1][axis_idx])
            y2 = float(t_map[b2][axis_idx])
            ok, reason = _check_scene_pattern_y_rule(
                scene_pattern=scene,
                y1=y1,
                y2=y2,
                ground_z=ground_z,
                ground_eps=ground_eps,
                air_min=air_min,
                air_max=air_max,
                height_axis=height_axis,
            )
            if not ok:
                basket_rule_failures.append(
                    f"[F][y_rule_fail] sample={sid} raw_usd={usd_path_raw} "
                    f"tried={tried_paths} hit={resolved_usd_path} "
                    f"scene={scene} world_height_axis={height_axis} "
                    f"world_height=({y1:.6f}, {y2:.6f}) -> {reason}"
                )
        except FileNotFoundError as e:
            basket_rule_failures.append(
                f"[F][missing_file] sample={sid} raw_usd={usd_path_raw} "
                f"tried={tried_paths} hit={resolved_usd_path} err={e}"
            )
        except Exception as e:  # noqa: BLE001
            basket_rule_failures.append(
                f"[F][usd_open_fail] sample={sid} raw_usd={usd_path_raw} "
                f"tried={tried_paths} hit={resolved_usd_path} err={e}"
            )

    if not basket_rule_failures:
        pass_count += 1
        print("[PASS] 吊篮 world-height 与 scene_pattern 规则一致")
    else:
        print("[FAIL] 吊篮 world-height 与 scene_pattern 规则不一致")
        failures.extend(basket_rule_failures)

    # 3) manifest 与 stats 是否一致（总数 + 场景计数）
    manifest_total = len(records)
    stats_total = int(stats.get("total_samples", -1))
    if manifest_total != stats_total:
        failures.append(
            f"[F] total_samples 不一致: manifest={manifest_total}, stats={stats_total}"
        )

    m_counts = {"single_airborne": 0, "double_airborne": 0, "all_ground": 0}
    for rec in records:
        sp = str(rec.get("scene_pattern", ""))
        if sp in m_counts:
            m_counts[sp] += 1
    s_counts_raw = stats.get("scene_pattern_counts", {})
    s_counts = {k: int(_safe_float(s_counts_raw.get(k, 0), 0.0)) for k in m_counts}
    for k in m_counts:
        if m_counts[k] != s_counts[k]:
            failures.append(
                f"[F] scene_pattern_counts[{k}] 不一致: manifest={m_counts[k]}, stats={s_counts[k]}"
            )

    if not any("total_samples 不一致" in x or "scene_pattern_counts[" in x for x in failures):
        pass_count += 1
        print("[PASS] manifest 与 stats 的总数/场景计数一致")
    else:
        print("[FAIL] manifest 与 stats 的总数/场景计数不一致")

    # 4) 输出清晰 pass/fail；5) 失败时返回非 0（由调用方处理）
    print("\n=== 验收结果 ===")
    print(f"manifest: {manifest_abs}")
    print(f"stats:    {stats_abs}")
    print(f"检查项通过数: {pass_count}/3")
    print(f"legacy_missing_abnormal_label_count: {legacy_missing_abnormal_label_count}")
    if failures:
        print(f"失败条数: {len(failures)}")
        for item in failures:
            print(item)
        return False
    print("全部检查通过。")
    return True


def check_dataset_manifest_integrity(
    manifest_path: str,
    stats_path: str,
    summary_only: bool = False,
    failed_rule_sample_topn: int = 20,
) -> tuple[bool, dict]:
    records = _parse_manifest_jsonl(manifest_path)
    with open(stats_path, "r", encoding="utf-8") as f:
        stats = json.load(f)

    required_keys = [
        "sample_id",
        "sample_index",
        "seed",
        "target_scene_pattern",
        "scene_pattern",
        "output_usd",
        "basket_1_prim",
        "basket_2_prim",
    ]
    scene_counts = {"single_airborne": 0, "double_airborne": 0, "all_ground": 0}
    basket_state_counts = {
        "ground_empty": 0,
        "air_with_person": 0,
        "ground_with_person": 0,
        "unknown": 0,
    }
    visibility_status_counts = {"ok": 0, "fail": 0, "skip": 0}
    visibility_failed_samples = []
    abnormal_actual_distribution = {k: 0 for k in ABNORMAL_LABELS_ALL}
    legacy_missing_abnormal_label_count = 0
    abnormal_x_visibility = {
        k: {"ok": 0, "fail": 0, "skip": 0} for k in ABNORMAL_LABELS_ALL
    }
    failures: List[dict] = []
    sample_indexes: List[int] = []
    per_scene_seq = {"single_airborne": [], "double_airborne": [], "all_ground": []}

    for i, rec in enumerate(records, start=1):
        sid = str(rec.get("sample_id", f"line_{i}"))
        missing = [k for k in required_keys if rec.get(k) in (None, "")]
        if missing:
            failures.append(
                {"sample_id": sid, "type": "manifest_incomplete", "detail": ",".join(missing)}
            )
            continue

        sp = str(rec.get("scene_pattern", ""))
        if sp in scene_counts:
            scene_counts[sp] += 1
        idx = int(rec.get("sample_index", 0))
        sample_indexes.append(idx)

        m = re.match(r"^(single_airborne|double_airborne|all_ground)_(\d{4})$", sid)
        if not m:
            failures.append(
                {
                    "sample_id": sid,
                    "type": "naming_rule_fail",
                    "detail": "sample_id 命名不符合 <scene_pattern>_0001 格式",
                }
            )
        else:
            sp_from_id = m.group(1)
            seq = int(m.group(2))
            if sp_from_id != sp:
                failures.append(
                    {
                        "sample_id": sid,
                        "type": "naming_rule_fail",
                        "detail": f"sample_id 场景({sp_from_id}) 与 scene_pattern({sp}) 不一致",
                    }
                )
            if sp in per_scene_seq:
                per_scene_seq[sp].append(seq)

        out_usd = str(rec.get("output_usd", ""))
        if sid not in out_usd:
            failures.append(
                {
                    "sample_id": sid,
                    "type": "naming_rule_fail",
                    "detail": "output_usd 文件名未包含 sample_id",
                }
            )

        for bk in ("basket_1_state", "basket_2_state"):
            bs = str(rec.get(bk, "unknown"))
            if bs in basket_state_counts:
                basket_state_counts[bs] += 1
            else:
                basket_state_counts["unknown"] += 1
        vc = rec.get("visibility_check", {}) or {}
        vs = str(vc.get("status", "skip"))
        if vs in visibility_status_counts:
            visibility_status_counts[vs] += 1
        else:
            visibility_status_counts["skip"] += 1
        if vs == "fail":
            visibility_failed_samples.append(
                {
                    "sample_id": sid,
                    "failed_rules": vc.get("failed_rules", []),
                }
            )
        ab_label, _, _, legacy_missing = normalize_abnormal_fields_from_record(rec)
        if legacy_missing:
            legacy_missing_abnormal_label_count += 1
        abnormal_actual_distribution[ab_label] = abnormal_actual_distribution.get(ab_label, 0) + 1
        if vs not in ("ok", "fail", "skip"):
            vs = "skip"
        if ab_label not in abnormal_x_visibility:
            abnormal_x_visibility[ab_label] = {"ok": 0, "fail": 0, "skip": 0}
        abnormal_x_visibility[ab_label][vs] += 1

    if records:
        expected_indexes = list(range(1, len(records) + 1))
        if sorted(sample_indexes) != expected_indexes:
            failures.append(
                {
                    "sample_id": "<dataset>",
                    "type": "sample_index_non_contiguous",
                    "detail": f"sample_index 非连续: got={sorted(sample_indexes)}",
                }
            )

    for sp, seqs in per_scene_seq.items():
        if not seqs:
            continue
        seqs_sorted = sorted(seqs)
        expected = list(range(1, len(seqs_sorted) + 1))
        if seqs_sorted != expected:
            failures.append(
                {
                    "sample_id": "<dataset>",
                    "type": "sample_id_non_contiguous",
                    "detail": f"{sp} 编号非连续: got={seqs_sorted}",
                }
            )

    stats_scene = stats.get("scene_pattern_counts", {})
    for k, v in scene_counts.items():
        if int(stats_scene.get(k, -1)) != int(v):
            failures.append(
                {
                    "sample_id": "<dataset>",
                    "type": "stats_mismatch",
                    "detail": f"scene_pattern_counts[{k}] manifest={v}, stats={stats_scene.get(k)}",
                }
            )
    if int(stats.get("total_samples", -1)) != len(records):
        failures.append(
            {
                "sample_id": "<dataset>",
                "type": "stats_mismatch",
                "detail": f"total_samples manifest={len(records)}, stats={stats.get('total_samples')}",
            }
        )

    stats_abnormal_summary = stats.get("abnormal_summary", {}) or {}
    requested_from_stats, plan_size_from_stats = _normalize_abnormal_requested_payload(
        stats_abnormal_summary.get("requested", {})
    )
    if plan_size_from_stats <= 0:
        plan_size_from_stats = int(_safe_float(stats_abnormal_summary.get("plan_size", 0), 0.0))

    report = {
        "manifest_path": os.path.abspath(manifest_path),
        "stats_path": os.path.abspath(stats_path),
        "total_samples": len(records),
        "scene_pattern_counts": scene_counts,
        "basket_state_counts": basket_state_counts,
        "visibility_status_counts": visibility_status_counts,
        "visibility_failed_samples": visibility_failed_samples,
        "failed_rule_counts": _build_failed_rule_counts(visibility_failed_samples),
        "failed_rule_sample_ids": _build_failed_rule_sample_map(
            visibility_failed_samples, max_samples_per_rule=failed_rule_sample_topn
        ),
        "abnormal_summary": {
            "schema_version": ABNORMAL_SCHEMA_VERSION,
            "implemented_labels": list(ABNORMAL_LABELS_IMPLEMENTED),
            "requested": requested_from_stats,
            "plan_size": max(0, int(plan_size_from_stats)),
            "actual_distribution": abnormal_actual_distribution,
            "actual_abnormal_count": int(
                sum(v for k, v in abnormal_actual_distribution.items() if k != ABNORMAL_LABEL_NORMAL)
            ),
            "actual_abnormal_ratio": float(
                sum(v for k, v in abnormal_actual_distribution.items() if k != ABNORMAL_LABEL_NORMAL)
            ) / float(max(1, len(records))),
            "legacy_missing_abnormal_label_count": int(legacy_missing_abnormal_label_count),
        },
        "abnormal_x_visibility": abnormal_x_visibility,
        "failed_samples": len(failures),
        "failures": [] if summary_only else failures,
    }
    ok = len(failures) == 0
    return ok, report


def print_dataset_readable_summary(report: dict, visibility_fail_topn: int = 5) -> None:
    print("\n=== 数据集汇总（易读）===")
    print(f"总样本数: {report.get('total_samples', 0)}")
    print(f"各 scene_pattern 数量: {report.get('scene_pattern_counts', {})}")
    print(f"各 basket_state 数量: {report.get('basket_state_counts', {})}")
    print(f"可视性状态计数: {report.get('visibility_status_counts', {})}")
    vsc = report.get("visibility_status_counts", {}) or {}
    ok_n = int(vsc.get("ok", 0))
    fail_n = int(vsc.get("fail", 0))
    vis_total = max(1, ok_n + fail_n)
    print(f"可视性 ok/fail 比例: {ok_n}/{fail_n} (ok_rate={ok_n / vis_total:.2%})")
    print(f"可视性 failed_rule 计数: {report.get('failed_rule_counts', {})}")
    print(f"可视性 failed_rule 样本列表: {report.get('failed_rule_sample_ids', {})}")
    print(
        f"可视性 fail 样本数: {len(report.get('visibility_failed_samples', []))}"
    )
    topn = max(0, int(visibility_fail_topn))
    top = list(report.get("visibility_failed_samples", []))[:topn]
    if top:
        print(f"可视性失败样本Top{topn}:")
        for it in top:
            print(f"- sample={it.get('sample_id')} rules={it.get('failed_rules', [])}")
    print(f"异常统计: {report.get('abnormal_summary', {})}")
    print(f"异常x可视性交叉统计: {report.get('abnormal_x_visibility', {})}")
    print(f"异常/失败样本数: {report.get('failed_samples', 0)}")


def _resolve_run_manifest_and_stats(target: str) -> Tuple[str, str]:
    p = os.path.abspath(target)
    if os.path.isdir(p):
        manifest = os.path.join(p, "manifests", "dataset_manifest_full.jsonl")
        stats = os.path.join(p, "stats", "dataset_stats_full.json")
        return manifest, stats
    manifest = p
    stats = _derive_stats_path_from_manifest(p)
    return manifest, stats


def dataset_repro_check(target_a: str, target_b: str) -> bool:
    manifest_a, stats_a = _resolve_run_manifest_and_stats(target_a)
    manifest_b, stats_b = _resolve_run_manifest_and_stats(target_b)
    for p in (manifest_a, manifest_b, stats_a, stats_b):
        if not os.path.isfile(p):
            raise FileNotFoundError(f"复现性检查缺少文件：{p}")

    rec_a = _parse_manifest_jsonl(manifest_a)
    rec_b = _parse_manifest_jsonl(manifest_b)
    with open(stats_a, "r", encoding="utf-8") as f:
        st_a = json.load(f)
    with open(stats_b, "r", encoding="utf-8") as f:
        st_b = json.load(f)

    fails: List[str] = []
    pass_count = 0

    if rec_a == rec_b:
        print("[PASS] manifest 内容一致")
        pass_count += 1
    else:
        print("[FAIL] manifest 内容不一致")
        fails.append("manifest 内容不一致")

    ids_a = [str(x.get("sample_id", "")) for x in rec_a]
    ids_b = [str(x.get("sample_id", "")) for x in rec_b]
    if ids_a == ids_b:
        print("[PASS] sample_id 顺序一致")
        pass_count += 1
    else:
        print("[FAIL] sample_id 顺序不一致")
        fails.append(f"sample_id 顺序不一致: A={ids_a[:8]} B={ids_b[:8]}")

    sp_a = st_a.get("scene_pattern_counts", {})
    sp_b = st_b.get("scene_pattern_counts", {})
    if sp_a == sp_b:
        print("[PASS] scene_pattern 分布一致")
        pass_count += 1
    else:
        print("[FAIL] scene_pattern 分布不一致")
        fails.append(f"scene_pattern_counts 不一致: A={sp_a} B={sp_b}")

    usd_set_a = {os.path.basename(str(x.get("output_usd", ""))) for x in rec_a}
    usd_set_b = {os.path.basename(str(x.get("output_usd", ""))) for x in rec_b}
    if usd_set_a == usd_set_b:
        print("[PASS] output USD 文件名集合一致")
        pass_count += 1
    else:
        print("[FAIL] output USD 文件名集合不一致")
        only_a = sorted(list(usd_set_a - usd_set_b))[:8]
        only_b = sorted(list(usd_set_b - usd_set_a))[:8]
        fails.append(f"only_in_A={only_a} only_in_B={only_b}")

    print("\n=== 复现性检查结果 ===")
    print(f"A manifest: {manifest_a}")
    print(f"B manifest: {manifest_b}")
    print(f"检查项通过数: {pass_count}/4")
    if fails:
        print(f"失败条数: {len(fails)}")
        for item in fails:
            print(f"[F] {item}")
        return False
    print("全部检查通过。")
    return True

# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------


class SceneUsdGenerator:
    def __init__(self, cfg: GeneratorConfig):
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)

    def run(self) -> dict:
        cfg = self.cfg
        if not os.path.isfile(cfg.input_path):
            raise FileNotFoundError(f"无法打开 USD：{cfg.input_path}")

        out = cfg.output_path
        os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
        if os.path.exists(out):
            os.remove(out)

        # 输出层采用 subLayer 引用输入场景，所有改动只写到输出 USD override。
        stage = Usd.Stage.CreateNew(out)
        if not stage:
            raise RuntimeError(f"无法创建输出 USD：{out}")
        root_layer = stage.GetRootLayer()
        root_layer.subLayerPaths.append(os.path.abspath(cfg.input_path))
        stage.SetEditTarget(root_layer)

        up = UsdGeom.GetStageUpAxis(stage)
        print(f"[gen] 输入: {cfg.input_path}  upAxis={up}  seed={cfg.seed}")
        cfg.stage_up_axis = str(up)

        baskets = resolve_basket_paths(cfg, stage)
        if not baskets:
            raise RuntimeError("未找到任何吊篮 Prim，请检查 pattern 或路径")

        person_prims: List[Usd.Prim] = []
        for p in cfg.person_prim_paths:
            prim = stage.GetPrimAtPath(p)
            if not prim.IsValid():
                print(f"[gen] 警告：人员 Prim 不存在，跳过：{p}")
                continue
            person_prims.append(prim)

        hc = cfg.height_people

        if len(baskets) > 1 and person_prims:
            print(
                "[gen] 提示：多个吊篮时，人员可见性以最后一次循环为准；"
                "每吊篮独立人数请多次运行或扩展配置。"
            )

        sampled_baskets = sample_baskets_for_scene_pattern(
            cfg.target_scene_pattern, baskets, self.rng, hc
        )
        world_height_axis, world_units_per_local_y = detect_world_height_semantics(
            stage, sampled_baskets[0][0] if sampled_baskets else ""
        )
        authored_height_component_idx = detect_authored_height_component(
            stage,
            sampled_baskets[0][0] if sampled_baskets else "",
            world_height_axis,
        )
        initial_translates: Dict[str, Gf.Vec3d] = {}
        for bp, _, _ in sampled_baskets:
            prim0 = stage.GetPrimAtPath(bp)
            if prim0.IsValid():
                initial_translates[bp] = get_local_translate(prim0)
        sampled_baskets, forced_positions, abnormal_meta = apply_abnormal_transform_v1(
            sampled_baskets=sampled_baskets,
            initial_translates=initial_translates,
            cfg=cfg,
            rng=self.rng,
            world_height_axis=world_height_axis,
            world_units_per_local_y=world_units_per_local_y,
            authored_height_component_idx=authored_height_component_idx,
        )
        basket_results = []
        for bp, y_val, people in sampled_baskets:
            stage.OverridePrim(bp)
            prim = stage.GetPrimAtPath(bp)
            if not prim.IsValid():
                raise RuntimeError(f"Invalid prim: {bp}")
            assert_constraints(y_val, people, hc)
            if bp in forced_positions:
                set_xform_translate_xyz(prim, forced_positions[bp])
            else:
                set_xform_translate_y(prim, y_val)
            t_after = get_local_translate(prim)
            print(
                f"[DEBUG] Set {bp} translate = "
                f"({t_after[0]:.6f}, {t_after[1]:.6f}, {t_after[2]:.6f})"
            )
            t_verify = get_local_translate(stage.GetPrimAtPath(bp))
            t_world = get_world_translate(stage, stage.GetPrimAtPath(bp))
            authored_h = float(t_verify[authored_height_component_idx])
            world_h = _height_value_from_xyz(
                [float(t_world[0]), float(t_world[1]), float(t_world[2])],
                world_height_axis,
            )
            print(
                f"[DEBUG] Verify {bp} translate = "
                f"({t_verify[0]:.6f}, {t_verify[1]:.6f}, {t_verify[2]:.6f})"
            )
            basket_state = infer_basket_state(people, world_h, hc.ground_z)
            print(
                f"[gen] 吊篮 {bp}: "
                f"authored_translate_xyz=({t_verify[0]:.4f}, {t_verify[1]:.4f}, {t_verify[2]:.4f}) "
                f"authored_height_component_value={authored_h:.4f} "
                f"world_height_axis={world_height_axis} world_height_value={world_h:.4f} 人数={people}"
            )
            basket_results.append(
                {
                    "prim": bp,
                    "person_count": int(people),
                    "world_height_value": float(world_h),
                    "final_y": float(world_h),
                    "center_xyz": [float(t_world[0]), float(t_world[1]), float(t_world[2])],
                    "local_translate_xyz": [float(t_verify[0]), float(t_verify[1]), float(t_verify[2])],
                    "world_height_axis": str(world_height_axis),
                    "authored_height_component_idx": int(authored_height_component_idx),
                    "authored_height_component_name": ("X", "Y", "Z")[int(authored_height_component_idx)],
                    "authored_height_component_value": float(authored_h),
                    "basket_state": basket_state,
                }
            )

            if person_prims:
                set_person_visibility(person_prims, people, warn=print)

        cam_path = cfg.camera_prim_path
        cam_prim = stage.GetPrimAtPath(cam_path)
        cam_eye = None
        cam_target = None
        if cam_prim.IsValid() and cfg.camera_region and cfg.camera_look_at:
            stage.OverridePrim(cam_path)
            cam_prim = stage.GetPrimAtPath(cam_path)
            eye, target = sample_camera_pose(
                self.rng, cfg.camera_region, cfg.camera_look_at
            )
            wu = Gf.Vec3d(*cfg.camera_look_at.world_up)
            set_xform_translate_xyz(cam_prim, eye)
            set_camera_orient_look_at(cam_prim, eye, target, wu)
            cam_eye = [float(eye[0]), float(eye[1]), float(eye[2])]
            cam_target = [float(target[0]), float(target[1]), float(target[2])]
            print(
                f"[gen] 相机 {cam_path}: eye={tuple(eye)}  target={tuple(target)}"
            )
        elif cam_prim.IsValid():
            print("[gen] 未设置 camera_region / camera_look_at，跳过相机随机。")

        root_layer.Save()
        print(f"[gen] 已保存：{out}")
        fresh_stage = Usd.Stage.Open(out)
        if not fresh_stage:
            raise RuntimeError(f"fresh reopen 失败：{out}")
        for p in FRESH_OPEN_BASKET_PATHS:
            fp = fresh_stage.GetPrimAtPath(p)
            if not fp.IsValid():
                raise RuntimeError(f"[FRESH-OPEN] Invalid prim: {p}")
            ft = get_local_translate(fp)
            print(
                f"[FRESH-OPEN] {p} translate = "
                f"({ft[0]:.6f}, {ft[1]:.6f}, {ft[2]:.6f})"
            )
        scene_pattern = infer_scene_pattern(
            baskets=basket_results,
            ground_height=float(hc.ground_z),
            ground_eps=1e-4,
        )
        validate_target_scene_pattern(
            cfg.target_scene_pattern,
            scene_pattern,
            baskets=basket_results,
            ground_height=float(hc.ground_z),
            ground_eps=1e-4,
        )
        if len(basket_results) >= 2:
            h1 = _basket_world_height_value(basket_results[0])
            h2 = _basket_world_height_value(basket_results[1])
            print(
                "[gen] "
                f"world_height_axis={world_height_axis} "
                f"basket_1_world_height={h1:.6f} "
                f"basket_2_world_height={h2:.6f} "
                f"inferred_scene_pattern={scene_pattern}"
            )
        height_relation = infer_height_relation(
            basket_results, float(cfg.same_level_threshold)
        )
        visibility_check = evaluate_visibility_check(
            cam_eye=cam_eye,
            cam_target=cam_target,
            baskets=basket_results,
            min_distance=float(cfg.visibility_min_distance),
            max_distance=float(cfg.visibility_max_distance),
            target_radius=float(cfg.visibility_target_radius),
            max_offaxis_deg=float(cfg.visibility_max_offaxis_deg),
        )
        if cfg.visibility_debug:
            th = visibility_check.get("thresholds", {})
            d1 = _safe_float(visibility_check.get("distance_to_basket_1"), -1.0)
            d2 = _safe_float(visibility_check.get("distance_to_basket_2"), -1.0)
            toff = _safe_float(visibility_check.get("target_offset_to_basket_center"), -1.0)
            o1 = visibility_check.get("offaxis_deg_1")
            o2 = visibility_check.get("offaxis_deg_2")
            print(
                "[vis-debug] "
                f"sample_id={cfg.sample_id or '<single>'} status={visibility_check.get('status')} "
                f"d1={d1:.3f} (min={th.get('visibility_min_distance')}, max={th.get('visibility_max_distance')}) "
                f"d2={d2:.3f} (min={th.get('visibility_min_distance')}, max={th.get('visibility_max_distance')}) "
                f"target_offset={toff:.3f} (max={th.get('visibility_target_radius')}) "
                f"offaxis1={o1} offaxis2={o2} (max={th.get('visibility_max_offaxis_deg')}) "
                f"cmp[d1_ok={th.get('visibility_min_distance') <= d1 <= th.get('visibility_max_distance')},"
                f"d2_ok={th.get('visibility_min_distance') <= d2 <= th.get('visibility_max_distance')},"
                f"target_ok={toff <= th.get('visibility_target_radius')},"
                f"off1_ok={(o1 is None) or (o1 <= th.get('visibility_max_offaxis_deg'))},"
                f"off2_ok={(o2 is None) or (o2 <= th.get('visibility_max_offaxis_deg'))}] "
                f"failed_rules={visibility_check.get('failed_rules', [])}"
            )
        if cfg.abnormal_label == ABNORMAL_LABEL_HEIGHT and len(basket_results) >= 2:
            left_local = basket_results[0].get("local_translate_xyz", [0.0, 0.0, 0.0])
            right_local = basket_results[1].get("local_translate_xyz", [0.0, 0.0, 0.0])
            lh = _basket_world_height_value(basket_results[0])
            rh = _basket_world_height_value(basket_results[1])
            abnormal_meta["details"] = {
                "authored_translate_axis": "Y",
                "authored_left_translate_value": float(left_local[1]),
                "authored_right_translate_value": float(right_local[1]),
                "authored_height_diff": abs(float(left_local[1]) - float(right_local[1])),
                "world_height_axis": str(world_height_axis or "Y").upper(),
                "left_basket_world_height": lh,
                "right_basket_world_height": rh,
                "world_height_diff": abs(lh - rh),
                # 兼容旧字段名：语义已统一为 world-space 高度。
                "left_basket_z": lh,
                "right_basket_z": rh,
                "height_diff": abs(lh - rh),
                "threshold": float(cfg.height_abnormal_diff_threshold),
                "unit_note": "threshold/world_height_diff use world-space along world_height_axis",
            }
        if cfg.abnormal_label == ABNORMAL_LABEL_DUAL_RISK and len(basket_results) >= 2:
            lc = basket_results[0].get("center_xyz", [0.0, 0.0, 0.0])
            rc = basket_results[1].get("center_xyz", [0.0, 0.0, 0.0])
            up_axis = str(world_height_axis or cfg.stage_up_axis or "Y").upper()
            gap = _horizontal_gap_by_up_axis(lc, rc, up_axis)
            abnormal_meta["details"] = {
                "left_basket_center": [float(lc[0]), float(lc[1]), float(lc[2])],
                "right_basket_center": [float(rc[0]), float(rc[1]), float(rc[2])],
                "horizontal_center_gap": float(gap),
                "threshold": float(cfg.dual_basket_risk_min_center_gap),
                "horizontal_plane": (
                    "XY" if up_axis == "Z" else ("YZ" if up_axis == "X" else "XZ")
                ),
                "up_axis": up_axis,
            }
        return {
            "sample_id": cfg.sample_id if cfg.sample_id else None,
            "sample_index": int(cfg.sample_index) if cfg.sample_index else None,
            "seed": int(cfg.seed),
            "output_usd": os.path.abspath(out),
            "target_scene_pattern": cfg.target_scene_pattern,
            "scene_pattern": scene_pattern,
            "world_height_axis": str(world_height_axis or "Y").upper(),
            "height_relation": height_relation,
            "baskets": basket_results,
            "camera_eye": cam_eye,
            "camera_target": cam_target,
            "visibility_check": visibility_check,
            "abnormal_label": str(cfg.abnormal_label or ABNORMAL_LABEL_NORMAL),
            "is_abnormal": bool(str(cfg.abnormal_label or ABNORMAL_LABEL_NORMAL) != ABNORMAL_LABEL_NORMAL),
            "abnormal_meta": abnormal_meta if isinstance(abnormal_meta, dict) else {},
        }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_float6(s: str) -> CameraRegion:
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError("camera-region 需要 6 个数：xmin,xmax,ymin,ymax,zmin,zmax")
    return CameraRegion(parts[0], parts[1], parts[2], parts[3], parts[4], parts[5])


def _parse_float6_target(s: str) -> CameraLookAtConfig:
    parts = [float(x) for x in s.split(",")]
    if len(parts) != 6:
        raise argparse.ArgumentTypeError(
            "camera-look-target 需要 6 个数：xmin,xmax,ymin,ymax,zmin,zmax"
        )
    return CameraLookAtConfig(
        parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]
    )


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="USD 吊篮/相机随机生成并导出")
    ap.add_argument(
        "--inspect-usd",
        default="",
        help="仅检查 USD（不生成）：打印固定 prim 的 local translate",
    )
    ap.add_argument(
        "--debug-pxr-import",
        action="store_true",
        help="输出 pxr 导入探测与最终 sys.path 注入目录（启动期生效）",
    )
    ap.add_argument(
        "--validate-dataset",
        default="",
        help="仅执行数据集验收：可传 run 目录或 dataset_manifest*.jsonl 路径",
    )
    ap.add_argument(
        "--validate-stats",
        default="",
        help="与 --validate-dataset 搭配；不传则自动从 manifest 推导 dataset_stats*.json",
    )
    ap.add_argument(
        "--validate-ground-eps",
        type=float,
        default=1e-3,
        help="验收时 ground 判定容差（|Y-ground_z| <= eps）",
    )
    ap.add_argument(
        "--input",
        default="/home/uniubi/xuanyuan/scene.usd",
        help="输入 USD 路径",
    )
    ap.add_argument(
        "--output",
        default="/home/uniubi/xuanyuan/scene_generated.usd",
        help="输出 USD 路径",
    )
    ap.add_argument("--seed", type=int, default=0, help="随机种子")
    ap.add_argument(
        "--batch-seeds",
        default="",
        help="批量种子（逗号分隔）。若提供，则优先于 --seed 执行批量生成。",
    )
    ap.add_argument(
        "--basket-prims",
        default="",
        help="逗号分隔的吊篮 Xform 路径；与 --basket-pattern 二选一",
    )
    ap.add_argument(
        "--basket-pattern",
        default="",
        help="正则：匹配路径或 prim 名称以自动发现吊篮（如 basket|吊篮）",
    )
    ap.add_argument(
        "--person-prims",
        default="",
        help="逗号分隔的人员模型 Prim（顺序对应人数 1..N）",
    )
    ap.add_argument(
        "--camera-prim",
        default="/World/CameraRig/CamTilt/Camera",
        help="相机 Prim 路径（需可设置 translate + orient）",
    )
    ap.add_argument("--ground-z", type=float, default=0.0, help="地面 Z（米）")
    ap.add_argument("--air-z-min", type=float, default=0.05, help="空中最小 Z")
    ap.add_argument("--air-z-max", type=float, default=40.0, help="空中最大 Z")
    ap.add_argument("--max-people", type=int, default=5, help="空中时最大人数")
    ap.add_argument(
        "--ground-people-max",
        type=int,
        default=5,
        help="在地面时人数上限（可为 0）",
    )
    ap.add_argument(
        "--air-prob",
        type=float,
        default=0.45,
        help="每次采样选「空中」吊篮的概率",
    )
    ap.add_argument(
        "--camera-region",
        type=_parse_float6,
        default=None,
        help="相机位置 AABB：xmin,xmax,ymin,ymax,zmin,zmax",
    )
    ap.add_argument(
        "--camera-look-target",
        type=_parse_float6_target,
        default=None,
        help="观察目标点采样 AABB：xmin,xmax,ymin,ymax,zmin,zmax",
    )
    ap.add_argument(
        "--visibility-min-distance",
        type=float,
        default=5.0,
        help="可视性检查：相机到吊篮最小合理距离",
    )
    ap.add_argument(
        "--visibility-max-distance",
        type=float,
        default=140.0,
        help="可视性检查：相机到吊篮最大合理距离",
    )
    ap.add_argument(
        "--visibility-target-radius",
        type=float,
        default=35.0,
        help="可视性检查：target 到双吊篮中心的最大偏移",
    )
    ap.add_argument(
        "--visibility-max-offaxis-deg",
        type=float,
        default=75.0,
        help="可视性检查：相机前向与吊篮方向最大离轴角",
    )
    ap.add_argument(
        "--visibility-debug",
        action="store_true",
        help="打印逐样本可视性调试信息（距离/偏移/离轴角/阈值比较）",
    )
    ap.add_argument(
        "--visibility-fail-topn",
        type=int,
        default=5,
        help="readable_summary 输出前 N 个可视性失败样本",
    )
    ap.add_argument(
        "--visibility-rule-sample-topn",
        type=int,
        default=20,
        help="stats/log 中每种 failed_rule 保留的样本ID条数上限",
    )
    ap.add_argument(
        "--spotcheck-count",
        type=int,
        default=0,
        help="生成后输出 spot-check 抽样条数（0 表示关闭）",
    )
    ap.add_argument(
        "--spotcheck-mode",
        choices=["fail-first", "random"],
        default="fail-first",
        help="spot-check 抽样模式：fail-first 优先失败样本，random 随机抽样",
    )
    ap.add_argument(
        "--same-level-threshold",
        type=float,
        default=3.0,
        help="双吊篮同层高度阈值（仅当两者都在空中时生效）",
    )
    ap.add_argument(
        "--target-scene-pattern",
        choices=["random", "single_airborne", "double_airborne", "all_ground"],
        default="random",
        help="目标场景类型。默认 random（保持独立随机）；非 random 时按目标模式构造并校验。",
    )
    ap.add_argument(
        "--dataset-scene-quotas",
        default="",
        help="组合数据集配额，如 single_airborne=3,double_airborne=3,all_ground=3",
    )
    ap.add_argument(
        "--dataset-size",
        type=int,
        default=0,
        help="按总样本数自动均分三种 scene_pattern（与 --dataset-scene-quotas 二选一）",
    )
    ap.add_argument(
        "--abnormal-ratio",
        type=float,
        default=0.0,
        help="异常样本比例（优先级低于 abnormal_target_count / abnormal_label_quotas）",
    )
    ap.add_argument(
        "--abnormal-target-count",
        type=int,
        default=0,
        help="异常样本目标数量（优先级低于 abnormal_label_quotas）",
    )
    ap.add_argument(
        "--abnormal-label-ratios",
        default="",
        help="异常标签比例，如 person_count_abnormal=0.4,height_abnormal=0.3,dual_basket_risk=0.3",
    )
    ap.add_argument(
        "--abnormal-label-quotas",
        default="",
        help="异常标签配额，如 person_count_abnormal=10,height_abnormal=10,dual_basket_risk=10",
    )
    ap.add_argument(
        "--normal-person-count-per-basket",
        type=int,
        default=2,
        help="person_count_abnormal 参考的每吊篮正常人数（v1 默认 2）",
    )
    ap.add_argument(
        "--height-abnormal-diff-threshold",
        type=float,
        default=3.0,
        help="height_abnormal 判定阈值（左右吊篮高度差）",
    )
    ap.add_argument(
        "--dual-basket-risk-min-center-gap",
        type=float,
        default=2.0,
        help="dual_basket_risk 判定阈值（双吊篮中心最小水平间距）",
    )
    ap.add_argument(
        "--dataset-run-dir",
        default="",
        help="数据集运行根目录；启用后输出 dataset_run_xxx/usd|manifests|stats|logs",
    )
    ap.add_argument(
        "--dataset-run-name",
        default="",
        help="数据集运行目录名（默认自动生成 dataset_run_时间戳）",
    )
    ap.add_argument(
        "--dataset-dry-run",
        action="store_true",
        help="仅输出数据集计划与路径，不实际生成 USD",
    )
    ap.add_argument(
        "--dataset-check-only",
        nargs="?",
        const="__USE_RUN_DIR__",
        default="",
        help=(
            "仅做数据集统计/命名/连续性检查；可传 manifest/run 路径，"
            "也可仅写 --dataset-check-only 并配合 --dataset-run-dir"
        ),
    )
    ap.add_argument(
        "--check-only",
        dest="dataset_check_only",
        nargs="?",
        const="__USE_RUN_DIR__",
        default="",
        help="兼容旧参数名，等价于 --dataset-check-only",
    )
    ap.add_argument(
        "--summary-only",
        action="store_true",
        help="检查模式下仅输出汇总，不打印全部失败明细",
    )
    ap.add_argument(
        "--dataset-repro-check",
        nargs=2,
        metavar=("RUN_A", "RUN_B"),
        default=None,
        help="复现性检查：对比两个 run 目录（或两个 manifest 路径）",
    )
    return ap


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = build_arg_parser()
    args = ap.parse_args(list(argv) if argv is not None else None)

    if args.inspect_usd.strip():
        inspect_usd_translates(args.inspect_usd.strip(), INSPECT_PRIM_PATHS)
        return 0

    if args.validate_dataset.strip():
        target = os.path.abspath(args.validate_dataset.strip())
        manifest_path, inferred_stats_path = _resolve_run_manifest_and_stats(target)
        if not os.path.isfile(manifest_path):
            print(f"错误：manifest 不存在：{manifest_path}", file=sys.stderr)
            return 2
        stats_path = os.path.abspath(args.validate_stats.strip()) if args.validate_stats.strip() else inferred_stats_path
        ok = validate_dataset_manifest(
            manifest_path=manifest_path,
            stats_path=stats_path,
            ground_z=float(args.ground_z),
            ground_eps=float(args.validate_ground_eps),
            air_min=float(args.air_z_min),
            air_max=float(args.air_z_max),
        )
        return 0 if ok else 1

    if args.dataset_repro_check:
        run_a, run_b = args.dataset_repro_check
        ok = dataset_repro_check(run_a, run_b)
        return 0 if ok else 1

    check_only_raw = str(args.dataset_check_only or "").strip()
    run_dir_raw = str(args.dataset_run_dir or "").strip()
    use_check_or_summary = bool(check_only_raw) or (bool(args.summary_only) and bool(run_dir_raw))
    if use_check_or_summary:
        if check_only_raw and check_only_raw != "__USE_RUN_DIR__":
            target = os.path.abspath(check_only_raw)
        elif run_dir_raw:
            target = os.path.abspath(run_dir_raw)
        else:
            print(
                "错误：--dataset-check-only/--check-only 未提供路径时，需要同时提供 --dataset-run-dir",
                file=sys.stderr,
            )
            return 2
        manifest_path = target
        stats_path = ""
        if os.path.isdir(target):
            manifest_path = os.path.join(target, "manifests", "dataset_manifest_full.jsonl")
            stats_path = os.path.join(target, "stats", "dataset_stats_full.json")
        else:
            stats_path = _derive_stats_path_from_manifest(manifest_path)
        if not os.path.isfile(manifest_path):
            print(f"错误：manifest 不存在：{manifest_path}", file=sys.stderr)
            return 2
        if not os.path.isfile(stats_path):
            print(f"错误：stats 不存在：{stats_path}", file=sys.stderr)
            return 2
        ok, report = check_dataset_manifest_integrity(
            manifest_path=manifest_path,
            stats_path=stats_path,
            summary_only=args.summary_only,
            failed_rule_sample_topn=args.visibility_rule_sample_topn,
        )
        print_dataset_readable_summary(report, visibility_fail_topn=args.visibility_fail_topn)
        if (not args.summary_only) and report.get("failures"):
            print("失败明细：")
            for item in report["failures"]:
                print(
                    f"- sample={item.get('sample_id')} type={item.get('type')} detail={item.get('detail')}"
                )
        return 0 if ok else 1

    baskets = [x.strip() for x in args.basket_prims.split(",") if x.strip()]
    pattern = args.basket_pattern.strip() or None
    if not baskets and not pattern:
        print(
            "错误：必须指定 --basket-prims 或 --basket-pattern",
            file=sys.stderr,
        )
        return 2

    persons = [x.strip() for x in args.person_prims.split(",") if x.strip()]

    hc = HeightPeopleConstraint(
        ground_z=args.ground_z,
        air_z_min=args.air_z_min,
        air_z_max=args.air_z_max,
        max_people=args.max_people,
        ground_people_max=args.ground_people_max,
        air_prob=args.air_prob,
    )

    cam_region = args.camera_region
    look = args.camera_look_target
    if (cam_region is None) ^ (look is None):
        print(
            "提示：camera-region 与 camera-look-target 需同时提供才会随机相机。",
            file=sys.stderr,
        )

    input_path = os.path.abspath(args.input)
    output_path = os.path.abspath(args.output)
    quotas = parse_dataset_scene_quotas(args.dataset_scene_quotas)
    if args.dataset_size > 0:
        if sum(quotas.values()) > 0:
            print(
                "错误：--dataset-size 与 --dataset-scene-quotas 不能同时使用",
                file=sys.stderr,
            )
            return 2
        quotas = _parse_dataset_size_to_quotas(int(args.dataset_size))
    if sum(quotas.values()) > 0 and args.target_scene_pattern != "random":
        print(
            "错误：--dataset-scene-quotas 与 --target-scene-pattern 不能同时使用",
            file=sys.stderr,
        )
        return 2

    if sum(quotas.values()) > 0:
        base_seed = int(args.seed) if args.seed is not None else 1
        print(
            f"[gen] 配额数据集模式启用 quotas={quotas} base_seed={base_seed}"
        )
        artifacts = derive_dataset_artifact_paths(
            output_path=output_path,
            dataset_run_dir=args.dataset_run_dir,
            dataset_run_name=args.dataset_run_name,
        )
        for d in (
            artifacts["usd_dir"],
            artifacts["manifests_dir"],
            artifacts["stats_dir"],
            artifacts["logs_dir"],
        ):
            os.makedirs(d, exist_ok=True)
        results, artifacts, abnormal_request = generate_dataset_by_quotas(
            quotas=quotas,
            base_seed=base_seed,
            input_path=input_path,
            output_path=output_path,
            baskets=baskets,
            pattern=pattern,
            persons=persons,
            camera_prim=args.camera_prim,
            hc=hc,
            cam_region=cam_region,
            look=look,
            same_level_threshold=args.same_level_threshold,
            visibility_min_distance=float(args.visibility_min_distance),
            visibility_max_distance=float(args.visibility_max_distance),
            visibility_target_radius=float(args.visibility_target_radius),
            visibility_max_offaxis_deg=float(args.visibility_max_offaxis_deg),
            abnormal_ratio=float(args.abnormal_ratio),
            abnormal_target_count=int(args.abnormal_target_count),
            abnormal_label_ratios_text=str(args.abnormal_label_ratios),
            abnormal_label_quotas_text=str(args.abnormal_label_quotas),
            normal_person_count_per_basket=int(args.normal_person_count_per_basket),
            height_abnormal_diff_threshold=float(args.height_abnormal_diff_threshold),
            dual_basket_risk_min_center_gap=float(args.dual_basket_risk_min_center_gap),
            visibility_debug=bool(args.visibility_debug),
            artifacts=artifacts,
            dry_run=bool(args.dataset_dry_run),
        )
        if args.dataset_dry_run:
            return 0

        summary = {
            "input_usd": input_path,
            "camera_prim": args.camera_prim,
            "basket_prims": baskets,
            "sampling_mode": "dataset_by_scene_quotas",
            "dataset_scene_quotas": quotas,
            "abnormal_summary_requested": abnormal_request,
            "seed_base": base_seed,
            "dataset_run_root": artifacts.get("run_root"),
            "results": sorted(results, key=lambda r: int(r.get("sample_index", 0))),
        }
        with open(artifacts["summary_path"], "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        write_dataset_full_manifest_jsonl(summary["results"], artifacts["manifest_path"])
        write_dataset_full_stats(
            summary["results"],
            artifacts["stats_path"],
            abnormal_requested=abnormal_request,
            generated_files=[
                artifacts["summary_path"],
                artifacts["manifest_path"],
                artifacts["stats_path"],
            ],
            failed_rule_sample_topn=args.visibility_rule_sample_topn,
        )
        ok_report, report = check_dataset_manifest_integrity(
            manifest_path=artifacts["manifest_path"],
            stats_path=artifacts["stats_path"],
            summary_only=False,
            failed_rule_sample_topn=args.visibility_rule_sample_topn,
        )
        with open(artifacts["check_report_path"], "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        lines = [
            f"总样本数: {report.get('total_samples', 0)}",
            f"各 scene_pattern 数量: {report.get('scene_pattern_counts', {})}",
            f"各 basket_state 数量: {report.get('basket_state_counts', {})}",
            f"可视性状态计数: {report.get('visibility_status_counts', {})}",
            (
                "可视性 ok/fail 比例: "
                f"{int((report.get('visibility_status_counts', {}) or {}).get('ok', 0))}/"
                f"{int((report.get('visibility_status_counts', {}) or {}).get('fail', 0))}"
            ),
            f"可视性 failed_rule 计数: {report.get('failed_rule_counts', {})}",
            f"可视性 failed_rule 样本列表: {report.get('failed_rule_sample_ids', {})}",
            f"可视性 fail 样本数: {len(report.get('visibility_failed_samples', []))}",
            f"异常统计: {report.get('abnormal_summary', {})}",
            f"异常x可视性交叉统计: {report.get('abnormal_x_visibility', {})}",
            f"异常/失败样本数: {report.get('failed_samples', 0)}",
            f"manifest: {artifacts['manifest_path']}",
            f"stats: {artifacts['stats_path']}",
            f"check_report: {artifacts['check_report_path']}",
        ]
        top_n = max(0, int(args.visibility_fail_topn))
        top_items = list(report.get("visibility_failed_samples", []))[:top_n]
        if top_items:
            lines.append(f"可视性失败样本Top{top_n}:")
            for it in top_items:
                lines.append(
                    f"- sample={it.get('sample_id')} rules={it.get('failed_rules', [])}"
                )

        spot_n = max(0, int(args.spotcheck_count))
        if spot_n > 0:
            spot_rows = build_spotcheck_records(
                summary["results"],
                count=spot_n,
                mode=str(args.spotcheck_mode),
                random_seed=int(base_seed),
            )
            with open(artifacts["spotcheck_path"], "w", encoding="utf-8") as f:
                for row in spot_rows:
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
            lines.append(f"spotcheck_count: {len(spot_rows)}")
            lines.append(f"spotcheck_mode: {args.spotcheck_mode}")
            lines.append(f"spotcheck_path: {artifacts['spotcheck_path']}")
        with open(artifacts["readable_summary_path"], "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        print_dataset_readable_summary(report, visibility_fail_topn=args.visibility_fail_topn)
        print(f"[gen] 数据集汇总已写入：{artifacts['summary_path']}")
        print(f"[gen] 数据集清单已写入：{artifacts['manifest_path']}")
        print(f"[gen] 数据集统计已写入：{artifacts['stats_path']}")
        print(f"[gen] 可读汇总已写入：{artifacts['readable_summary_path']}")
        print(f"[gen] 一致性检查已写入：{artifacts['check_report_path']}")
        if spot_n > 0:
            print(f"[gen] spot-check 清单已写入：{artifacts['spotcheck_path']}")
        if not ok_report:
            print("[gen] 警告：数据集一致性检查存在失败项", file=sys.stderr)
            return 1
        return 0

    batch_seeds = [int(x.strip()) for x in args.batch_seeds.split(",") if x.strip()]
    if batch_seeds:
        batch_seeds = sorted(batch_seeds)
        print(f"[gen] 批量模式启用 --batch-seeds={batch_seeds}，将忽略 --seed={args.seed}")
        output_pattern, summary_path, manifest_path, stats_path = derive_batch_artifact_paths(
            output_path, args.target_scene_pattern
        )

        results = []
        for sd in batch_seeds:
            out_i = output_pattern.format(seed=sd)
            cfg = GeneratorConfig(
                input_path=input_path,
                output_path=out_i,
                seed=sd,
                basket_prim_paths=baskets,
                basket_name_pattern=pattern,
                person_prim_paths=persons,
                camera_prim_path=args.camera_prim,
                height_people=hc,
                camera_region=cam_region,
                camera_look_at=look,
                same_level_threshold=args.same_level_threshold,
                target_scene_pattern=args.target_scene_pattern,
                visibility_min_distance=float(args.visibility_min_distance),
                visibility_max_distance=float(args.visibility_max_distance),
                visibility_target_radius=float(args.visibility_target_radius),
                visibility_max_offaxis_deg=float(args.visibility_max_offaxis_deg),
                visibility_debug=bool(args.visibility_debug),
            )
            print(f"[gen] ===== 批量子任务 seed={sd} =====")
            results.append(SceneUsdGenerator(cfg).run())

        summary = {
            "input_usd": input_path,
            "camera_prim": args.camera_prim,
            "basket_prims": baskets,
            "sampling_mode": "independent_per_basket",
            "target_scene_pattern": args.target_scene_pattern,
            "results": sorted(results, key=lambda r: r["seed"]),
        }
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"[gen] 批量汇总已写入：{summary_path}")
        write_dataset_manifest_jsonl(summary["results"], manifest_path)
        write_dataset_stats(
            summary["results"],
            stats_path,
            generated_files=[summary_path, manifest_path, stats_path],
            failed_rule_sample_topn=args.visibility_rule_sample_topn,
        )
        print(f"[gen] 数据清单已写入：{manifest_path}")
        print(f"[gen] 数据统计已写入：{stats_path}")
    else:
        cfg = GeneratorConfig(
            input_path=input_path,
            output_path=output_path,
            seed=args.seed,
            basket_prim_paths=baskets,
            basket_name_pattern=pattern,
            person_prim_paths=persons,
            camera_prim_path=args.camera_prim,
            height_people=hc,
            camera_region=cam_region,
            camera_look_at=look,
            same_level_threshold=args.same_level_threshold,
            target_scene_pattern=args.target_scene_pattern,
            visibility_min_distance=float(args.visibility_min_distance),
            visibility_max_distance=float(args.visibility_max_distance),
            visibility_target_radius=float(args.visibility_target_radius),
            visibility_max_offaxis_deg=float(args.visibility_max_offaxis_deg),
            visibility_debug=bool(args.visibility_debug),
        )
        SceneUsdGenerator(cfg).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
