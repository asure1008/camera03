#!/usr/bin/env python3
"""
PTZ 球机朝向与缩放修复工具
================================
用于修复将 PTZ_SecurityDome.usda（Y-up, cm 坐标）放入
Z-up（upAxis=Z）且以米为单位（metersPerUnit=1.0）场景时出现的
"球机倒地"和"球机过大"问题。

使用方法：
    /home/uniubi/projects/issac/.isaac_sim_unzip/python.sh fix_ptz_orientation.py [options]

Options:
    --scene   场景文件路径，默认 ./V3.0.usd
    --prim    PTZ Xform prim 路径，默认自动检测
    --x       放置 X 坐标（米），默认保持原值
    --y       放置 Y 坐标（米），默认保持原值
    --z       放置 Z 坐标（米），默认 0（地面）

示例：
    # 自动检测并修复 V3.0.usd
    python fix_ptz_orientation.py

    # 指定坐标（将 PTZ 放在工地某个角落）
    python fix_ptz_orientation.py --x 5.0 --y -8.0 --z 0.0
"""

import os, sys, argparse

# ── 环境配置 ──────────────────────────────────────────────
PXR_DIR = (
    "/home/uniubi/projects/issac/.isaac_sim_unzip/"
    "extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311"
)
sys.path.insert(0, PXR_DIR)
os.environ["LD_LIBRARY_PATH"] = (
    f"{PXR_DIR}/bin:"
    "/home/uniubi/projects/issac/.isaac_sim_unzip/omni.usd.libs:"
    "/home/uniubi/projects/issac/.isaac_sim_unzip/kit/python/lib:"
    + os.environ.get("LD_LIBRARY_PATH", "")
)

from pxr import Usd, UsdGeom, Gf  # noqa: E402

# ── 参数 ──────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(script_dir, "V3.0.usd"))
ap.add_argument("--prim",  default=None,  help="PTZ Xform prim 路径（留空自动检测）")
ap.add_argument("--x",     type=float, default=None)
ap.add_argument("--y",     type=float, default=None)
ap.add_argument("--z",     type=float, default=0.0, help="Z 高度（米），默认=0（地面）")
args = ap.parse_args()

# ── 打开场景 ──────────────────────────────────────────────
print(f"[fix] 打开场景：{args.scene}")
stage = Usd.Stage.Open(args.scene)

up_axis   = UsdGeom.GetStageUpAxis(stage)
mpu       = UsdGeom.GetStageMetersPerUnit(stage)
print(f"[fix] upAxis={up_axis}  metersPerUnit={mpu}")

if up_axis != UsdGeom.Tokens.z:
    print("[fix] ⚠️  场景不是 Z-up，请确认是否需要修复。")

# ── 找到 PTZ prim ─────────────────────────────────────────
ptz_path = args.prim
if not ptz_path:
    print("[fix] 自动检测 PTZ 引用...")
    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if refs:
            for r in refs.GetAddedOrExplicitItems():
                if "PTZ" in str(r.assetPath):
                    ptz_path = str(prim.GetPath())
                    print(f"[fix]   发现：{ptz_path}  →  {r.assetPath}")
                    break
        if ptz_path:
            break

if not ptz_path:
    sys.exit("[fix] ✗ 未找到 PTZ 引用，请用 --prim 手动指定。")

ptz_prim = stage.GetPrimAtPath(ptz_path)
if not ptz_prim.IsValid():
    sys.exit(f"[fix] ✗ Prim 不存在：{ptz_path}")

# ── 读取当前位置 ──────────────────────────────────────────
cur_translate = Gf.Vec3d(0, 0, 0)
attr_t = ptz_prim.GetAttribute("xformOp:translate")
if attr_t and attr_t.Get():
    cur_translate = attr_t.Get()

x_new = args.x if args.x is not None else cur_translate[0]
y_new = args.y if args.y is not None else cur_translate[1]
z_new = args.z  # 高度设为 0（地面），用户可通过 --z 覆盖

print(f"[fix] 当前位置：{tuple(round(v,3) for v in cur_translate)}")
print(f"[fix] 新位置：  ({x_new:.3f}, {y_new:.3f}, {z_new:.3f})")

# ── 计算缩放系数 ──────────────────────────────────────────
# PTZ_SecurityDome.usda 坐标单位 = 厘米（metersPerUnit=0.01）
# V3.0.usd 单位 = 米（metersPerUnit=1.0）
# 缩放 = PTZ_mpu / scene_mpu = 0.01 / 1.0 = 0.01
PTZ_MPU = 0.01
scale   = PTZ_MPU / mpu  # = 0.01 when scene is meters
print(f"[fix] 缩放系数：{scale}  （PTZ 单位 cm → 场景单位 m）")

# ── 应用修复 ──────────────────────────────────────────────
# 直接写属性（避免精度类型冲突：scene 使用 double3，不重新 AddXformOp）
def _set_attr(prim, name, value):
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        # 不存在则创建（使用 double3）
        from pxr import Sdf
        type_map = {
            "xformOp:translate": Sdf.ValueTypeNames.Double3,
            "xformOp:rotateXYZ": Sdf.ValueTypeNames.Double3,
            "xformOp:scale":     Sdf.ValueTypeNames.Double3,
        }
        attr = prim.CreateAttribute(name, type_map[name])
    attr.Set(value)

_set_attr(ptz_prim, "xformOp:translate", Gf.Vec3d(x_new, y_new, z_new))
# R_X(-90°)：把 PTZ 内部 +Y（柱顶方向）映射到世界 +Z（朝上）
_set_attr(ptz_prim, "xformOp:rotateXYZ", Gf.Vec3d(-90.0, 0.0, 0.0))
_set_attr(ptz_prim, "xformOp:scale",     Gf.Vec3d(scale, scale, scale))

# 确保 xformOpOrder 正确
from pxr import Vt
ptz_prim.GetAttribute("xformOpOrder").Set(
    Vt.TokenArray(["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"])
)

print(f"[fix] 已设置：translate=({x_new:.3f},{y_new:.3f},{z_new:.3f})  "
      f"rotateX=-90  scale={scale}")

# ── 保存 ─────────────────────────────────────────────────
stage.Save()
print(f"[fix] ✓ 已保存：{args.scene}")

# ── 打印修复后的 camera_prim 路径 ─────────────────────────
# PTZ 内部结构：/World/PTZ/Pan/Tilt/Camera
cam_path = ptz_path + "/Pan/Tilt/Camera"
print(f"\n[fix] 提示：如需更新 ptz_rtsp_config.yaml 的 camera_prim，请改为：")
print(f"      camera_prim: {cam_path}")
