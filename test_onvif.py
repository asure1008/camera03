#!/usr/bin/env python3
"""
验证脚本：用 python-onvif-zeep 连接 Camera03 的 ONVIF 端口，
测试 PTZ 控制（AbsoluteMove、RelativeMove、SetPreset、GetPresets、GotoPreset、RemovePreset）和 GetSnapshotUri。
"""

import sys
import traceback

HOST   = "127.0.0.1"
PORT   = 8080
USER   = "admin"
PASSWD = "admin"

print(f"\n{'='*60}")
print(f"[TEST] 连接 Camera03 ONVIF 服务  {HOST}:{PORT}")
print(f"{'='*60}")

from onvif import ONVIFCamera

# ── 1. 初始化 ────────────────────────────────────────────────────────
print("\n[1] 初始化 ONVIFCamera …")
try:
    cam = ONVIFCamera(HOST, PORT, USER, PASSWD)
    media  = cam.create_media_service()
    ptz    = cam.create_ptz_service()
    print("    ✓ 连接成功")
except Exception as e:
    print(f"    ✗ 连接失败: {e}")
    traceback.print_exc()
    sys.exit(1)

# ── 2. GetProfiles ───────────────────────────────────────────────────
print("\n[2] GetProfiles …")
try:
    profiles = media.GetProfiles()
    p0 = profiles[0]
    profile_token = p0.token
    print(f"    ✓ Profile 数量={len(profiles)}")
    print(f"    ✓ Token={profile_token}")
    if p0.PTZConfiguration:
        ptz_cfg_token = p0.PTZConfiguration.token
        print(f"    ✓ PTZConfiguration token={ptz_cfg_token}")
    else:
        print("    ! 无 PTZConfiguration，尝试 GetNodes")
        nodes = ptz.GetNodes()
        ptz_cfg_token = nodes[0].token
        print(f"    ✓ PTZNode token={ptz_cfg_token}")
except Exception as e:
    print(f"    ✗ GetProfiles 失败: {e}")
    traceback.print_exc()
    sys.exit(1)

# ── 3. GetSnapshotUri ────────────────────────────────────────────────
print("\n[3] GetSnapshotUri …")
try:
    req = media.create_type("GetSnapshotUri")
    req.ProfileToken = profile_token
    resp = media.GetSnapshotUri(req)
    snap_uri = resp.Uri
    print(f"    ✓ Uri={snap_uri}")
except Exception as e:
    print(f"    ✗ GetSnapshotUri 失败: {e}")
    traceback.print_exc()

# ── 4. AbsoluteMove ─────────────────────────────────────────────────
print("\n[4] AbsoluteMove(pan=0.3, tilt=-0.2, zoom=0.1) …")
try:
    req = {
        "ProfileToken": profile_token,
        "Position": {
            "PanTilt": {"x": 0.3, "y": -0.2},
            "Zoom":    {"x": 0.1},
        },
    }
    ptz.AbsoluteMove(req)
    print("    ✓ AbsoluteMove 返回 OK")
    print("    期望 Camera03 收到: pan=51.0°, tilt=-18.0°, zoom=4.1x")
except Exception as e:
    print(f"    ✗ AbsoluteMove 失败: {e}")
    traceback.print_exc()

# ── 5. RelativeMove ──────────────────────────────────────────────────
print("\n[5] RelativeMove(pan=0.05, tilt=0.05) …")
try:
    req = {
        "ProfileToken": profile_token,
        "Translation": {
            "PanTilt": {"x": 0.05, "y": 0.05},
        },
    }
    ptz.RelativeMove(req)
    print("    ✓ RelativeMove 返回 OK")
except Exception as e:
    print(f"    ✗ RelativeMove 失败: {e}")
    traceback.print_exc()

# ── 6. SetPreset / GetPresets / GotoPreset / RemovePreset ───────────
print("\n[6] SetPreset(token='1', name='测试位1') …")
try:
    req = ptz.create_type("SetPreset")
    req.ProfileToken = profile_token
    req.PresetToken  = "1"
    req.PresetName   = "测试位1"
    resp = ptz.SetPreset(req)
    print(f"    ✓ SetPreset 返回 OK  token={getattr(resp, 'PresetToken', '1')}")
except Exception as e:
    print(f"    ✗ SetPreset 失败: {e}")
    traceback.print_exc()

print("\n[7] GetPresets …")
try:
    resp = ptz.GetPresets({"ProfileToken": profile_token})
    presets = resp if isinstance(resp, list) else getattr(resp, "Preset", resp)
    if not isinstance(presets, list):
        presets = [presets]
    print(f"    ✓ GetPresets 返回 {len([p for p in presets if p])} 项")
except Exception as e:
    print(f"    ✗ GetPresets 失败: {e}")
    traceback.print_exc()

print("\n[8] GotoPreset(token='1') …")
try:
    req = ptz.create_type("GotoPreset")
    req.ProfileToken = profile_token
    req.PresetToken  = "1"
    ptz.GotoPreset(req)
    print("    ✓ GotoPreset 返回 OK")
except Exception as e:
    print(f"    ✗ GotoPreset 失败: {e}")
    traceback.print_exc()

print("\n[9] RemovePreset(token='1') …")
try:
    req = ptz.create_type("RemovePreset")
    req.ProfileToken = profile_token
    req.PresetToken  = "1"
    ptz.RemovePreset(req)
    print("    ✓ RemovePreset 返回 OK")
except Exception as e:
    print(f"    ✗ RemovePreset 失败: {e}")
    traceback.print_exc()

print(f"\n{'='*60}")
print("[TEST] 全部验证完成 ✓")
print(f"{'='*60}\n")
