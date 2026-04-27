#!/usr/bin/env python3
"""
为 V4.0.usd 的裸 Camera prim 创建 Pan/Tilt 控制层级
======================================================
执行后层级变为：
  /World/CameraRig          ← translate（位置）+ rotateZ = Pan（水平角）
    /CamTilt                ← rotateY = Tilt（俯仰角）
      /Camera               ← 相机本体，保留原始朝向四元数

原 /World/Camera 被停用（deactivate），不影响场景其余内容。

使用方法：
    python3 setup_camera_v4.py [--scene ./V4.0.usd] [--cam /World/Camera]
"""

import argparse, os, sys

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

from pxr import Usd, UsdGeom, Gf, Vt, Sdf  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="./V4.0.usd")
ap.add_argument("--cam",   default="/World/Camera",
                help="场景中已有 Camera prim 的路径")
args = ap.parse_args()

script_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = args.scene if os.path.isabs(args.scene) else \
    os.path.join(script_dir, args.scene)

# ── 打开场景 ──────────────────────────────────────────────
stage = Usd.Stage.Open(scene_path)
up    = UsdGeom.GetStageUpAxis(stage)
mpu   = UsdGeom.GetStageMetersPerUnit(stage)
print(f"[setup] 场景：{scene_path}  upAxis={up}  mpu={mpu}")

# ── 检查是否已设置 ────────────────────────────────────────
orig_path  = args.cam
parts      = orig_path.split("/")            # ['', 'World', 'Camera']
parent_path = "/".join(parts[:-1]) or "/"   # /World
cam_name    = parts[-1]                      # Camera

rig_path  = parent_path + "/CameraRig"
tilt_path = rig_path   + "/CamTilt"
new_cam_path = tilt_path + "/" + cam_name

if stage.GetPrimAtPath(rig_path).IsValid():
    print(f"[setup] ✓ CameraRig 已存在（{rig_path}），无需重复执行。")
    print(f"[setup]   camera_prim = {new_cam_path}")
    sys.exit(0)

orig_cam = stage.GetPrimAtPath(orig_path)
if not orig_cam.IsValid():
    sys.exit(f"[setup] ✗ 未找到 Camera prim：{orig_path}")

# ── 读取原相机的位置 ──────────────────────────────────────
t_attr = orig_cam.GetAttribute("xformOp:translate")
cam_pos = Gf.Vec3d(t_attr.Get()) if (t_attr and t_attr.Get() is not None) \
    else Gf.Vec3d(0, 0, 0)
print(f"[setup] 原相机位置：{tuple(round(v,3) for v in cam_pos)}")

# ── 创建 CameraRig（位置 + Pan）────────────────────────────
rig = UsdGeom.Xform.Define(stage, rig_path)
rig.AddTranslateOp().Set(cam_pos)
if up == "Z":
    rig.AddRotateZOp().Set(0.0)          # Z-up：水平旋转绕 Z
else:
    rig.AddRotateYOp().Set(0.0)          # Y-up：水平旋转绕 Y
print(f"[setup] 创建 {rig_path}  (Pan={'rotateZ' if up=='Z' else 'rotateY'})")

# ── 创建 CamTilt（Tilt）──────────────────────────────────
tilt = UsdGeom.Xform.Define(stage, tilt_path)
if up == "Z":
    tilt.AddRotateYOp().Set(0.0)         # Z-up：俯仰绕 Y
else:
    tilt.AddRotateZOp().Set(0.0)         # Y-up：俯仰绕 Z
print(f"[setup] 创建 {tilt_path}  (Tilt={'rotateY' if up=='Z' else 'rotateZ'})")

# ── 在新路径创建 Camera，复制原属性 ───────────────────────
new_cam = stage.DefinePrim(new_cam_path, "Camera")

SKIP = {"xformOp:translate", "xformOpOrder"}
for attr in orig_cam.GetAttributes():
    n = attr.GetName()
    if n in SKIP:
        continue
    v = attr.Get()
    if v is None:
        continue
    try:
        new_attr = new_cam.CreateAttribute(n, attr.GetTypeName())
        new_attr.Set(v)
    except Exception:
        pass

# 新相机位置归零（位置已移到 CameraRig），保留旋转和缩放
new_cam.CreateAttribute(
    "xformOp:translate", Sdf.ValueTypeNames.Double3
).Set(Gf.Vec3d(0, 0, 0))

# 重建 xformOpOrder（translate + 原有旋转/缩放 ops）
orig_order = orig_cam.GetAttribute("xformOpOrder")
if orig_order and orig_order.Get():
    new_cam.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray
    ).Set(orig_order.Get())
else:
    new_cam.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray
    ).Set(Vt.TokenArray(["xformOp:translate"]))

print(f"[setup] 创建 {new_cam_path}  (Camera 本体)")

# ── 停用原相机 ────────────────────────────────────────────
orig_cam.SetActive(False)
print(f"[setup] 停用原 {orig_path}")

# ── 保存 ─────────────────────────────────────────────────
stage.Save()
print(f"\n[setup] ✓ 已保存：{scene_path}")
print(f"[setup] 请将 ptz_rtsp_config.yaml 中更新为：")
print(f"          scene_path:  ./V4.0.usd")
print(f"          camera_prim: {new_cam_path}")
