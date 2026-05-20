#!/usr/bin/env python3
"""
PTZ 安防球机 RTSP 流输出组件
==============================
将 Isaac Sim 中 PTZ_SecurityDome.usda 的相机画面以 RTSP 流形式输出。

使用方法：
    /home/uniubi/projects/issac/.isaac_sim_unzip/python.sh ptz_rtsp_stream.py
    /home/uniubi/projects/issac/.isaac_sim_unzip/python.sh ptz_rtsp_stream.py --config ./ptz_rtsp_config.yaml
    /home/uniubi/projects/issac/.isaac_sim_unzip/python.sh ptz_rtsp_stream.py --scene ./MyScene.usd --camera /World/PTZCamera/Pan/Tilt/Camera

客户端查看：
    vlc rtsp://localhost:8554/ptz_cam
    ffplay rtsp://localhost:8554/ptz_cam -rtsp_transport tcp

可移植性：
    将本文件、ptz_rtsp_config.yaml、PTZ_SecurityDome.usda 一起复制到目标工程目录即可。
    在配置文件中修改 scene_path 和 camera_prim 适配目标场景。
"""

# ============================================================
# 第一阶段：SimulationApp 必须在所有 omni.* 导入之前启动
# ============================================================
import argparse
from collections import Counter, deque
import copy
import datetime as _dt
import os
import sys
try:
    from zoneinfo import ZoneInfo as _ZoneInfo
except Exception:
    _ZoneInfo = None

# python.sh 会设置 CARB_APP_PATH=$ISAAC_SIM_ROOT/kit/
# isaacsim 包的 expose_api() 依赖 ISAAC_PATH 指向 Isaac Sim 根目录；
# SimulationApp 依赖 EXP_PATH 指向 apps/ 目录（存放 .kit 配置文件）。
# 两者均未由 setup_python_env.sh 设置，从 CARB_APP_PATH 推断。
def _setup_isaac_env():
    carb_app = os.environ.get("CARB_APP_PATH", "")
    if not carb_app:
        return  # 无法推断，可能已在 Isaac Sim 目录内运行

    # CARB_APP_PATH = $ISAAC_SIM_ROOT/kit/，上一级为根目录
    isaac_root = os.path.dirname(carb_app)

    if "ISAAC_PATH" not in os.environ:
        os.environ["ISAAC_PATH"] = isaac_root

    if "EXP_PATH" not in os.environ:
        exp_path = os.path.join(isaac_root, "apps")
        if os.path.isdir(exp_path):
            os.environ["EXP_PATH"] = exp_path

_setup_isaac_env()

# 解析命令行参数（在 SimulationApp 之前，避免 Kit 解析器干扰）
parser = argparse.ArgumentParser(description="PTZ Camera RTSP Streamer for Isaac Sim")
parser.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "ptz_rtsp_config.yaml"),
                    help="YAML 配置文件路径（默认：脚本同目录的 ptz_rtsp_config.yaml）")
parser.add_argument("--scene",     default=None, help="覆盖 scene_path 配置")
parser.add_argument("--camera",    default=None, help="覆盖 camera_prim 配置")
parser.add_argument("--rtsp",      default=None, help="覆盖 rtsp_url 配置")
parser.add_argument("--fps",       type=int, default=None, help="覆盖 fps 配置")
parser.add_argument("--ctrl-port", type=int, default=None, dest="ctrl_port",
                    help="HTTP 控制 API 监听端口（默认读取配置文件 ctrl_port，回退 8080）")
args, unknown = parser.parse_known_args()

# 读取配置文件
import yaml  # Isaac Sim kit python 内置 PyYAML

# 感知迁移：事件注册与标准化 JSON（无 Isaac 依赖，与脚本同目录）
from scene_perception import (
    CAMERA_PERCEPTION_SUPPORTED_RULE_IDS,
    analyze_jpeg_overexposure_metrics,
    attach_perception_to_randomize_result,
    rule11_overexposure_thresholds_meta,
    _safe_plain_dict,
)

script_dir = os.path.dirname(os.path.abspath(__file__))

config_path = args.config
with open(config_path, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

# 命令行参数覆盖配置文件
def _resolve_path(p):
    """将相对路径解析为相对于脚本目录的绝对路径。"""
    if p and not os.path.isabs(p):
        return os.path.join(script_dir, p)
    return p

scene_path      = _resolve_path(args.scene   or cfg["scene_path"])
camera_prim     = args.camera  or cfg["camera_prim"]
rtsp_url        = args.rtsp    or cfg["rtsp_url"]
fps             = args.fps     or cfg.get("fps", 25)
resolution      = tuple(cfg.get("resolution", [1920, 1080]))
bitrate         = cfg.get("bitrate", "4M")
sim_hz          = cfg.get("sim_hz", 60)
mediamtx_cfg    = cfg.get("mediamtx", {})
rtsp_enabled    = cfg.get("rtsp_enabled",    True)
mjpeg_quality   = cfg.get("mjpeg_quality",   80)
FOCAL_LENGTH_1X = float(cfg.get("focal_length_1x", 18.14756))
# 新增配置项
preview_enabled = cfg.get("preview_enabled", True)   # false=关闭 MJPEG/ws-flv，仅 RTSP
snapshot_interval_s = max(0.1, float(cfg.get("snapshot_interval_s", 0.5)))
_RTSP_LOW_LATENCY_MODE = bool(cfg.get("rtsp_low_latency_mode", True))
_RTSP_GOP_SECONDS = max(0.2, float(cfg.get("rtsp_gop_seconds", 1.0)))
_RTSP_REPEAT_LATEST_FRAME = bool(cfg.get("rtsp_repeat_latest_frame", True))
_RTSP_PUBLISH_TRANSPORT = str(cfg.get("rtsp_publish_transport", "tcp") or "tcp").strip().lower()
if _RTSP_PUBLISH_TRANSPORT not in ("tcp", "udp"):
    _RTSP_PUBLISH_TRANSPORT = "tcp"
_RTSP_VIEWPORT_PRIMARY_PROBE_INTERVAL_S = max(
    1.0, float(cfg.get("rtsp_viewport_primary_probe_interval_s", 5.0))
)
_RTSP_VIEWPORT_CAPTURE_TIMEOUT_MS = max(
    20.0, float(cfg.get("rtsp_viewport_capture_timeout_ms", 150.0))
)
_RTSP_STALE_WARN_MS = max(
    100.0, float(cfg.get("rtsp_stale_warn_ms", 800.0))
)
_RTSP_STALE_RECOVERY_MS = max(
    _RTSP_STALE_WARN_MS, float(cfg.get("rtsp_stale_recovery_ms", 1500.0))
)
_RTSP_STALE_RECOVERY_HOLD_S = max(
    1.0, float(cfg.get("rtsp_stale_recovery_hold_s", 10.0))
)
_STATUS_SCENE_REFRESH_INTERVAL_S = max(
    0.5, float(cfg.get("status_scene_refresh_interval_s", 5.0))
)
_STATUS_SCENE_REFRESH_FULL_SCAN = bool(
    cfg.get("status_scene_refresh_full_scan", not _RTSP_LOW_LATENCY_MODE)
)
_NEAR_BLACK_RECOVER_CONSECUTIVE = max(
    2, int(cfg.get("near_black_recover_consecutive", 4))
)
_NEAR_BLACK_RECOVER_COOLDOWN_S = max(
    1.0, float(cfg.get("near_black_recover_cooldown_s", 15.0))
)
_RANDOMIZE_FAST_RESPONSE = bool(cfg.get("randomize_fast_response", True))
_RANDOMIZE_RENDER_STABILIZE_WINDOW_S = max(
    0.0, float(cfg.get("randomize_render_stabilize_window_s", 0.5))
)
_RANDOMIZE_RENDER_SETTLE_MIN_GOOD_FRAMES = max(
    1, int(cfg.get("randomize_render_settle_min_good_frames", 1))
)
_RANDOMIZE_RENDER_SETTLE_MAX_FRAMES = max(
    _RANDOMIZE_RENDER_SETTLE_MIN_GOOD_FRAMES,
    int(cfg.get("randomize_render_settle_max_frames", 2)),
)
_RANDOMIZE_CONTEXT_ORIENTATION_MAX_CANDIDATES = max(
    1, int(cfg.get("randomize_context_orientation_max_candidates", 24))
)
_RANDOMIZE_CONTEXT_DOWN_TILT_WINDOW = max(
    1, int(cfg.get("randomize_context_down_tilt_window", 10))
)
_RANDOMIZE_CONTEXT_DOWN_TILT_MAX_IN_WINDOW = max(
    0,
    min(
        _RANDOMIZE_CONTEXT_DOWN_TILT_WINDOW,
        int(cfg.get("randomize_context_down_tilt_max_in_window", 4)),
    ),
)
_RANDOMIZE_CONTEXT_DOWN_TILT_PROBABILITY = max(
    0.0, min(1.0, float(cfg.get("randomize_context_down_tilt_probability", 0.45)))
)
_RANDOMIZE_FREEZE_STREAM_DURING_APPLY = bool(
    cfg.get("randomize_freeze_stream_during_apply", True)
)
_RANDOMIZE_STABLE_MIN_GOOD_FRAMES = max(
    1, int(cfg.get("randomize_stable_min_good_frames", 3))
)
_RANDOMIZE_STABLE_MAX_WAIT_S = max(
    1.0, float(cfg.get("randomize_stable_max_wait_s", 12.0))
)
_RANDOMIZE_CANDIDATE_INTERVAL_MS = max(
    20.0, float(cfg.get("randomize_candidate_interval_ms", 100.0))
)
_RANDOMIZE_FORCE_VIEWPORT_PRIMARY_ON_BLACK = bool(
    cfg.get("randomize_force_viewport_primary_on_black", True)
)
_POST_RECOVER_SNAPSHOT_GATE_S = max(
    0.0, float(cfg.get("post_recover_snapshot_gate_s", 2.5))
)
_RENDERER_TARGET_MODE = "RTXRealTime"
_WALL_CONSTRAINT_PRIM_DEFAULT = "/World/JiKeng_ChangJing01/JiKeng_BeiJing/JiKeng_BeiJing/group1/Mesh267/Mesh267"


def _cfg_str(raw_value, default_value):
    value = str(raw_value or "").strip()
    return value or str(default_value)


def _cfg_float(raw_value, default_value, label):
    try:
        value = float(raw_value)
    except (TypeError, ValueError):
        print(f"[PTZ-RTSP] WARN: invalid {label}={raw_value!r}; fallback={default_value}")
        return float(default_value)
    if value < 0.0:
        print(f"[PTZ-RTSP] WARN: negative {label}={value}; clamp_to=0.0")
        return 0.0
    return value


def _cfg_float_signed(raw_value, default_value, label):
    try:
        return float(raw_value)
    except (TypeError, ValueError):
        print(f"[PTZ-RTSP] WARN: invalid {label}={raw_value!r}; fallback={default_value}")
        return float(default_value)


_CAMERA_WALL_CONSTRAINT_PRIM = _cfg_str(
    cfg.get("wall_constraint_prim_path", cfg.get("camera_height_wall_prim_path", _WALL_CONSTRAINT_PRIM_DEFAULT)),
    _WALL_CONSTRAINT_PRIM_DEFAULT,
)
_CAMERA_WALL_CONSTRAINT_XY_MARGIN = _cfg_float(
    cfg.get("wall_constraint_xy_margin", 0.005), 0.005, "wall_constraint_xy_margin"
)
_CAMERA_WALL_CONSTRAINT_Z_MARGIN = _cfg_float(
    cfg.get("wall_constraint_z_margin", 0.05), 0.05, "wall_constraint_z_margin"
)
_CAMERA_WALL_MOUNT_INSET_M = _cfg_float(
    cfg.get("wall_mount_inset_m", 0.0), 0.0, "wall_mount_inset_m"
)
_CAMERA_WALL_MOUNT_INSET_MODE = _cfg_str(
    cfg.get("wall_mount_inset_mode", ""),
    "",
).strip().lower()
_WALL_COLLECTION_MODE = _cfg_str(
    cfg.get("wall_collection_mode", "semantic_parent"),
    "semantic_parent",
).lower()
_WALL_COLLECTION_ROOT_PATH = _cfg_str(
    cfg.get("wall_collection_root_path", ""),
    "",
)
_WALL_CANDIDATE_REGION = (
    copy.deepcopy(cfg.get("wall_candidate_region"))
    if isinstance(cfg.get("wall_candidate_region"), dict)
    else None
)
_SCENE_RANDOMIZE_WAIT_TIMEOUT_S = max(
    30.0,
    _cfg_float(
        cfg.get("scene_randomize_wait_timeout_s", cfg.get("randomize_wait_timeout_s", 360.0)),
        360.0,
        "scene_randomize_wait_timeout_s",
    ),
)
try:
    _GONDOLA_RENDERABLE_DETAIL_LIMIT = max(0, int(cfg.get("gondola_renderable_detail_limit", 24)))
except (TypeError, ValueError):
    _GONDOLA_RENDERABLE_DETAIL_LIMIT = 24
_GONDOLA_RENDERABLE_VERBOSE_LOG = bool(cfg.get("gondola_renderable_verbose_log", False))

_JI_KENG_CHANGJING_DEFAULT = "/World/JiKeng_ChangJing01"
_DEFAULT_LOOKAT_TARGET_BUILDING_PRIM = "/World/JiKeng_ChangJing01/Architecture_High"
_CHANGJING_PRIM_PATH = _cfg_str(cfg.get("changjing_prim_path", ""), _JI_KENG_CHANGJING_DEFAULT)
_LOOKAT_TARGET_PRIM_PATH = _cfg_str(
    cfg.get("lookat_target_prim_path", ""),
    _DEFAULT_LOOKAT_TARGET_BUILDING_PRIM,
)


_CAMERA_ORIENTATION_MODE = _cfg_str(
    cfg.get("camera_orientation_mode", "dynamic_lookat"),
    "dynamic_lookat",
)


def _cfg_vec3(raw_value, default_value, label):
    if isinstance(raw_value, (list, tuple)) and len(raw_value) == 3:
        try:
            return (float(raw_value[0]), float(raw_value[1]), float(raw_value[2]))
        except (TypeError, ValueError):
            pass
    print(f"[PTZ-RTSP] WARN: invalid {label}={raw_value!r}; fallback={default_value}")
    return (float(default_value[0]), float(default_value[1]), float(default_value[2]))


_CAMERA_LOOKAT_TARGET_XYZ = _cfg_vec3(
    cfg.get("camera_lookat_target_xyz", [101.1, 0.4, 18.13]),
    (101.1, 0.4, 18.13),
    "camera_lookat_target_xyz",
)


_PRESET_LEFT_PAN_OFFSET_DEG = _cfg_float(
    cfg.get("preset_left_pan_offset_deg", 90.0), 90.0, "preset_left_pan_offset_deg"
)
_PRESET_RIGHT_PAN_OFFSET_DEG = _cfg_float(
    cfg.get("preset_right_pan_offset_deg", -90.0), -90.0, "preset_right_pan_offset_deg"
)
_PRESET_OVERLOOK_TILT_DEG = _cfg_float(
    cfg.get("preset_overlook_tilt_deg", 30.0), 30.0, "preset_overlook_tilt_deg"
)

_DYNAMIC_STARTUP_PAN_OFFSET_DEG = _cfg_float_signed(
    cfg.get("dynamic_startup_pan_offset_deg", 0.0), 0.0, "dynamic_startup_pan_offset_deg"
)
_DYNAMIC_STARTUP_TILT_OFFSET_DEG = _cfg_float_signed(
    cfg.get("dynamic_startup_tilt_offset_deg", 0.0), 0.0, "dynamic_startup_tilt_offset_deg"
)


def _normalize_renderer_mode(raw_renderer) -> str:
    raw = str(raw_renderer or "").strip()
    low = raw.lower()
    if low in ("rtxrealtime", "rtx_real_time", "rt", "raytracedlighting"):
        return "RTXRealTime"
    if raw != _RENDERER_TARGET_MODE:
        print(
            f"[renderer-fix] requested_renderer={raw or '<empty>'} -> forced={_RENDERER_TARGET_MODE}"
        )
    return _RENDERER_TARGET_MODE

def _simulation_renderer_name(mode: str) -> str:
    # SimulationApp后端映射: RTXRealTime -> RaytracedLighting (5.1 实时 token)
    if mode == "RTXRealTime":
        return "RaytracedLighting"
    return mode

renderer_mode   = _normalize_renderer_mode(cfg.get("renderer", _RENDERER_TARGET_MODE))
cfg["renderer"] = renderer_mode
sim_renderer_name = _simulation_renderer_name(renderer_mode)
_osd_cfg        = cfg.get("osd_time", {})
osd_enabled     = bool(_osd_cfg.get("enabled", False))
_OSD_STAGE = str(_osd_cfg.get("stage", "frame_capture") or "frame_capture").strip().lower()
_OSD_FRAME_CAPTURE_ENABLED = bool(osd_enabled and _OSD_STAGE in ("frame_capture", "capture", "python"))
_OSD_TIMEZONE_NAME = str(_osd_cfg.get("timezone", "Asia/Shanghai") or "Asia/Shanghai").strip()
_OSD_FMT = str(_osd_cfg.get("fmt", "%Y-%m-%d %H:%M:%S") or "%Y-%m-%d %H:%M:%S")
_OSD_X = max(0, int(_osd_cfg.get("x", 10) or 0))
_OSD_Y = max(0, int(_osd_cfg.get("y", 10) or 0))
_OSD_SIZE = max(8, int(_osd_cfg.get("size", 28) or 28))
_OSD_FONT = str(_osd_cfg.get("font", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf") or "")
_OSD_COLOR_MODE = str(_osd_cfg.get("color_mode", "auto") or "auto").strip().lower()
_OSD_BG_LUMA_THRESHOLD = float(_osd_cfg.get("bg_luma_threshold", 145.0) or 145.0)
_OSD_STROKE_WIDTH = max(0, int(_osd_cfg.get("stroke_width", 1) or 0))
_OSD_STROKE_ALPHA = max(0, min(255, int(_osd_cfg.get("stroke_alpha", 150) or 150)))
_OSD_BOX_ENABLED = bool(_osd_cfg.get("box", False))
_CAPTURE_SOURCE_PREFER_RTSP_LATEST_FOR_SNAPSHOT = bool(
    cfg.get("capture_source_prefer_rtsp_latest_for_snapshot", True)
)


def _resolve_osd_tzinfo():
    if _ZoneInfo is not None and _OSD_TIMEZONE_NAME:
        try:
            return _ZoneInfo(_OSD_TIMEZONE_NAME)
        except Exception:
            pass
    try:
        return _dt.datetime.now().astimezone().tzinfo
    except Exception:
        return None


_OSD_TZINFO = _resolve_osd_tzinfo()


def _camera_rig_translate_from_cfg(c: dict) -> tuple[float, float, float]:
    """camera_rig_translate_xyz: [x,y,z] → 启动时写入 rig；无效或缺省 → (82,-72,28)。"""
    raw = c.get("camera_rig_translate_xyz")
    if isinstance(raw, (list, tuple)) and len(raw) == 3:
        try:
            return (float(raw[0]), float(raw[1]), float(raw[2]))
        except (TypeError, ValueError):
            pass
    return (82.0, -72.0, 28.0)


_CAMERA_RIG_TRANSLATE_XYZ = _camera_rig_translate_from_cfg(cfg)

W, H = resolution
skip_frames = max(1, round(sim_hz / fps))

print(f"[PTZ-RTSP] 配置：场景={scene_path}")
print(f"[PTZ-RTSP]       相机Prim={camera_prim}")
print(f"[PTZ-RTSP]       RTSP URL={rtsp_url}  分辨率={W}x{H}  fps={fps}")
print(f"[PTZ-RTSP]       renderer={renderer_mode}  preview_enabled={preview_enabled}")
print(
    f"[PTZ-RTSP]       wall_constraint_prim={_CAMERA_WALL_CONSTRAINT_PRIM} "
    f"xy_margin={_CAMERA_WALL_CONSTRAINT_XY_MARGIN} z_margin={_CAMERA_WALL_CONSTRAINT_Z_MARGIN} "
    f"wall_mount_inset_m={_CAMERA_WALL_MOUNT_INSET_M} wall_mount_inset_mode={_CAMERA_WALL_MOUNT_INSET_MODE or '-'} "
    f"wall_collection_mode={_WALL_COLLECTION_MODE} "
    f"wall_collection_root_path={_WALL_COLLECTION_ROOT_PATH or '-'}"
)
print(f"[PTZ-RTSP]       wall_candidate_region={_WALL_CANDIDATE_REGION or '-'}")
print(f"[PTZ-RTSP]       orientation_mode={_CAMERA_ORIENTATION_MODE} lookat_target={_CAMERA_LOOKAT_TARGET_XYZ}")
print(f"[PTZ-RTSP]       preset_offsets left={_PRESET_LEFT_PAN_OFFSET_DEG} right={_PRESET_RIGHT_PAN_OFFSET_DEG} overlook_tilt={_PRESET_OVERLOOK_TILT_DEG}")
print(f"[PTZ-RTSP]       sim_hz={sim_hz}  skip_frames={skip_frames}")
print(
    f"[PTZ-RTSP]       CameraRig translate={_CAMERA_RIG_TRANSLATE_XYZ} "
    f"(camera_rig_translate_xyz 或 fallback)"
)

# 启动 SimulationApp（headless 模式）
from isaacsim import SimulationApp

sim_app = SimulationApp({
    "headless": True,
    "renderer": sim_renderer_name,
    "width": W,
    "height": H,
})

# ============================================================
# 第二阶段：所有 omni.* 导入在 SimulationApp 启动后进行
# ============================================================
import asyncio
import io as _io
import json
import queue
import random
import traceback
import signal
import subprocess
import tarfile
import threading
import time
import urllib.request
import uuid
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World

# ============================================================
# PTZ 控制 HTTP API（内嵌在仿真进程中，port 8080）
# ============================================================




def _enforce_renderer_mode(reason: str, reset_launch_config: bool = False) -> None:
    if reset_launch_config:
        try:
            sim_app.reset_render_settings()
        except Exception as exc:
            print(f"[renderer-fix] reset_render_settings skipped reason={reason} error={exc}")
    
    sim_renderer = _simulation_renderer_name(renderer_mode)
    sim_app.set_setting("/rtx/rendermode", sim_renderer)
    
    if renderer_mode == "RTXRealTime":
        sim_app.set_setting("/rtx/directLighting/enabled", True)
        sim_app.set_setting("/rtx/directLighting/domeLight/enabled", True)
        sim_app.set_setting("/rtx/directLighting/sampledLighting/enabled", True)
        sim_app.set_setting("/rtx/indirectDiffuse/enabled", True)
        sim_app.set_setting("/rtx/shadows/enabled", True)
        
    sim_app.update()
    import carb

    settings = carb.settings.get_settings()
    render_mode_live = settings.get("/rtx/rendermode")
    renderer_plugin = settings.get("/app/renderer/plugin")
    print(
        "[renderer-fix] "
        f"reason={reason} target={sim_renderer} "
        f"/app/renderer/plugin={renderer_plugin} "
        f"/rtx/rendermode={render_mode_live}"
    )
    if str(render_mode_live).strip().lower() != sim_renderer.lower():
        raise RuntimeError(
            f"renderer enforcement failed: expected {sim_renderer}, got {render_mode_live}"
        )

_CTRL_PORT = args.ctrl_port or cfg.get("ctrl_port", 8080)

_ptz_state = {
    "pan":  float(cfg.get("initial_pan",  0.0)),   # -170 ~ +170 度
    "tilt": float(cfg.get("initial_tilt", -15.0)), # (-90, 30]：正值=俯视（向下），负值=仰视（向上）
    "zoom": float(cfg.get("initial_zoom", 1.5)),    # 1× ~ 32×
}
print(f"[PTZ-RTSP] 初始云台 pan={_ptz_state['pan']}° tilt={_ptz_state['tilt']}° zoom={_ptz_state['zoom']}×")
_ptz_lock    = threading.Lock()
_ptz_dirty   = threading.Event()
# /ptz_state 在 _ptz_lock 争用时回退的只读快照（成功路径会刷新）
_ptz_state_http_stale_cache: dict[str, float] = {
    "pan": float(_ptz_state["pan"]),
    "tilt": float(_ptz_state["tilt"]),
    "zoom": float(_ptz_state["zoom"]),
}

# RTX Global Volumetric Effects（体积雾）状态（默认关闭，避免影响当前画面）
_VOLUMETRIC_SETTINGS = {
    "enabled": "/rtx/raytracing/globalVolumetricEffects/enabled",
    "densityMult": "/rtx/raytracing/inscattering/densityMult",
    "maxDistance": "/rtx/raytracing/inscattering/maxDistance",
    "atmosphereHeight": "/rtx/raytracing/inscattering/atmosphereHeight",
    "transmittanceMeasurementDistance": "/rtx/raytracing/inscattering/transmittanceMeasurementDistance",
    # 注意：真实 key 为 fogHeightFallOff（O 大写），不要改成 fogHeightFalloff
    "fogHeightFallOff": "/rtx/pathtracing/ptvol/fogHeightFallOff",
    # 可选扩展字段（不影响默认画面：enabled=false 时不生效）
    "transmittanceColor": "/rtx/raytracing/inscattering/transmittanceColor",
    "singleScatteringAlbedo": "/rtx/raytracing/inscattering/singleScatteringAlbedo",
    "anisotropyFactor": "/rtx/raytracing/inscattering/anisotropyFactor",
    "useDetailNoise": "/rtx/raytracing/inscattering/useDetailNoise",
}
_VOLUMETRIC_DEFAULT_STATE: dict = {
    "enabled": False,
    "densityMult": 1.2,
    "maxDistance": 500.0,
    "atmosphereHeight": 50.0,
    "transmittanceMeasurementDistance": 10000.0,
    "fogHeightFallOff": 10.0,
    # optional
    "transmittanceColor": [0.5, 0.5, 0.5],
    "singleScatteringAlbedo": [0.9, 0.9, 0.9],
    "anisotropyFactor": 0.0,
    "useDetailNoise": False,
}
_VOLUMETRIC_FLOAT_RANGES = {
    "densityMult": (0.0, 2.0),
    "maxDistance": (10.0, 1_000_000.0),
    "atmosphereHeight": (-2000.0, 100_000.0),
    "transmittanceMeasurementDistance": (0.0001, 1_000_000.0),
    "fogHeightFallOff": (10.0, 2000.0),
    "anisotropyFactor": (-0.999, 0.999),
}


def _vol_clamp_float(value: object, lo: float, hi: float, default: float) -> float:
    try:
        n = float(value)  # type: ignore[arg-type]
    except Exception:
        n = float(default)
    return max(float(lo), min(float(hi), float(n)))


def _vol_parse_bool(value: object) -> tuple[bool | None, str | None]:
    if isinstance(value, bool):
        return value, None
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value), None
    if isinstance(value, str):
        v = value.strip().lower()
        if v in ("1", "true", "yes", "y", "on", "enabled"):
            return True, None
        if v in ("0", "false", "no", "n", "off", "disabled"):
            return False, None
    return None, "type_error"


def _vol_normalize_color3(value: object, default: list[float] | tuple[float, float, float]) -> tuple[list[float], str | None]:
    base = [float(default[0]), float(default[1]), float(default[2])]
    if isinstance(value, (list, tuple)) and len(value) >= 3:
        out: list[float] = []
        for i in range(3):
            try:
                c = float(value[i])  # type: ignore[index]
            except Exception:
                c = base[i]
            out.append(max(0.0, min(1.0, c)))
        return out, None
    return base, "type_error"


def _vol_safe_read_setting(path: str):
    try:
        import carb

        s = carb.settings.get_settings()
        return s.get(path)
    except Exception:
        return None


def _vol_set_color3(path: str, value: list[float] | tuple[float, float, float]) -> bool:
    try:
        import carb

        s = carb.settings.get_settings()
        color, _ = _vol_normalize_color3(value, (0.5, 0.5, 0.5))
        if hasattr(s, "set_float_array"):
            s.set_float_array(path, color)
        else:
            s.set(path, color)
        return True
    except Exception:
        return False


def _vol_set_scalar(path: str, value: object) -> bool:
    try:
        import carb

        s = carb.settings.get_settings()
        s.set(path, value)
        return True
    except Exception:
        return False


def _vol_init_state_from_config() -> None:
    raw = cfg.get("volumetric", {})
    if not isinstance(raw, dict):
        raw = {}
    st = dict(_VOLUMETRIC_DEFAULT_STATE)
    b_enabled, _ = _vol_parse_bool(raw.get("enabled", st["enabled"]))
    st["enabled"] = bool(b_enabled) if b_enabled is not None else bool(st["enabled"])
    for k, (lo, hi) in _VOLUMETRIC_FLOAT_RANGES.items():
        if k in raw:
            st[k] = _vol_clamp_float(raw.get(k), lo, hi, float(st[k]))
    for k in ("transmittanceColor", "singleScatteringAlbedo"):
        if k in raw:
            st[k], _ = _vol_normalize_color3(raw.get(k), st[k])
    if "useDetailNoise" in raw:
        b, _ = _vol_parse_bool(raw.get("useDetailNoise"))
        if b is not None:
            st["useDetailNoise"] = bool(b)
    global _volumetric_state
    with _volumetric_lock:
        _volumetric_state = st
        _volumetric_diag.update({"applied_setting_keys": [], "last_error": ""})


def _vol_apply_state() -> None:
    with _volumetric_lock:
        st = dict(_volumetric_state)
    applied: list[str] = []
    # bools
    for k in ("enabled", "useDetailNoise"):
        path = _VOLUMETRIC_SETTINGS.get(k)
        if not path:
            continue
        if _vol_set_scalar(path, bool(st.get(k))):
            applied.append(path)
    # floats
    for k, (lo, hi) in _VOLUMETRIC_FLOAT_RANGES.items():
        path = _VOLUMETRIC_SETTINGS.get(k)
        if not path:
            continue
        v = _vol_clamp_float(st.get(k), lo, hi, float(_VOLUMETRIC_DEFAULT_STATE.get(k, 0.0)))
        if _vol_set_scalar(path, float(v)):
            applied.append(path)
    # colors
    for k in ("transmittanceColor", "singleScatteringAlbedo"):
        path = _VOLUMETRIC_SETTINGS.get(k)
        if not path:
            continue
        if _vol_set_color3(path, st.get(k) or _VOLUMETRIC_DEFAULT_STATE[k]):
            applied.append(path)
    last_error = "" if applied else "no volumetric setting key was applied"
    with _volumetric_lock:
        _volumetric_diag.update({"applied_setting_keys": applied, "last_error": last_error})


def _wait_for_volumetric_apply(timeout_s: float = 1.0) -> None:
    deadline = time.time() + float(timeout_s)
    while _volumetric_dirty.is_set() and time.time() < deadline:
        time.sleep(0.01)


def _vol_snapshot() -> dict:
    warnings: list[str] = []
    with _volumetric_lock:
        st = dict(_volumetric_state)
        diag = dict(_volumetric_diag)
    live = {k: _vol_safe_read_setting(path) for k, path in _VOLUMETRIC_SETTINGS.items()}
    for k in ("transmittanceColor", "singleScatteringAlbedo"):
        raw = live.get(k)
        if isinstance(raw, (list, tuple)) and len(raw) >= 3:
            live[k], _ = _vol_normalize_color3(raw, st.get(k) or _VOLUMETRIC_DEFAULT_STATE[k])
    gondola_renderable_paths = list(runtime.get("gondola_renderable_paths") or [])
    gondola_visible_renderable_paths = list(runtime.get("gondola_visible_renderable_paths") or [])
    gondola_hidden_paths = list(runtime.get("gondola_hidden_paths") or [])
    gondola_renderable_debug = list(runtime.get("gondola_renderable_debug") or [])
    return {
        "ok": True,
        "renderer_mode": renderer_mode,
        "configured": {
            "state": st,
            "ranges": dict(_VOLUMETRIC_FLOAT_RANGES),
        },
        "settings": {"live": live},
        "pending": {"dirty": bool(_volumetric_dirty.is_set())},
        "diag": diag,
        "warnings": warnings,
    }


_volumetric_lock = threading.Lock()
_volumetric_dirty = threading.Event()
_volumetric_state: dict = dict(_VOLUMETRIC_DEFAULT_STATE)
_volumetric_diag: dict = {"applied_setting_keys": [], "last_error": ""}
_vol_init_state_from_config()

# 最近一次 /control 输入（用于调试：打印输入与最终写入旋转链）
_ptz_last_cmd: dict | None = None

# 推流/场景运行时诊断（供 /status、/diagnostics 与日志对照，避免“以为换了样本其实还是母场景”）
_stream_diag_lock = threading.Lock()
_STREAM_DIAG: dict = {
    "role": "isaac_ptz_stream",
    "config_file": os.path.abspath(config_path),
    "scene_path": os.path.abspath(scene_path) if scene_path else "",
    "scene_basename": os.path.basename(scene_path) if scene_path else "",
    "camera_prim": camera_prim,
    "stage_open_ok": False,
    "open_stage_detail": "not_loaded_yet",
    "camera_bound_ok": False,
    "render_product": "",
    "rtsp_enabled": rtsp_enabled,
    "rtsp_url": rtsp_url if rtsp_enabled else None,
    "rtsp_low_latency_mode": _RTSP_LOW_LATENCY_MODE,
    "rtsp_publish_transport": _RTSP_PUBLISH_TRANSPORT,
    "rtsp_writer_target_fps": fps,
    "rtsp_writer_push_fps": None,
    "rtsp_writer_repeated_frame": False,
    "rtsp_writer_frame_age_ms": None,
    "rtsp_writer_last_source": None,
    "rtsp_source_new_fps": None,
    "rtsp_source_repeat_ratio": None,
    "rtsp_source_max_gap_ms": None,
    "rtsp_viewport_capture_wait_ms": None,
    "rtsp_stale_recovery_mode": "normal",
    "rtsp_latest_capture_epoch_ms": None,
    "rtsp_latest_capture_iso": None,
    "rtsp_latest_osd_text": None,
    "osd_enabled": osd_enabled,
    "osd_stage": _OSD_STAGE,
    "osd_draw_ms": None,
    "snapshot_prefer_rtsp_latest": _CAPTURE_SOURCE_PREFER_RTSP_LATEST_FOR_SNAPSHOT,
    "rtsp_capture_mode": "replicator_probe",
    "rtsp_capture_mode_reason": None,
    "preview_enabled": preview_enabled,
    "resolution_wh": [W, H],
    "control_base": f"http://127.0.0.1:{_CTRL_PORT}",
    "mjpeg_url": f"http://127.0.0.1:{_CTRL_PORT}/stream.mjpeg",
    "snapshot_url": f"http://127.0.0.1:{_CTRL_PORT}/snapshot.jpg",
    "frame_pipeline": "Replicator rgb annotator → MJPEG buffer / ffmpeg stdin(RTSP)",
    "ffmpeg_alive": False,
    "mediamtx_started": False,
    "render_capture_last_frame_source": None,
    "render_capture_last_fallback_reason": None,
    "render_capture_last_replicator_rgb_mean": None,
    "render_capture_last_viewport_rgb_mean": None,
    "randomize_active": False,
    "randomize_stream_mode": "idle",
    "randomize_frozen_frame_age_ms": None,
    "randomize_candidate_health": None,
    "randomize_last_commit_source": None,
    "randomize_black_frames_blocked_total": 0,
}


def _stream_diag_update(**kwargs) -> None:
    with _stream_diag_lock:
        _STREAM_DIAG.update(kwargs)


_OSD_PIL_STATE: dict = {"init": False, "Image": None, "ImageDraw": None, "ImageFont": None, "font": None}
_OSD_CV2_STATE: dict = {"init": False, "cv2": None}


def _capture_time_meta(epoch_s: float | None = None) -> dict:
    ts = time.time() if epoch_s is None else float(epoch_s)
    dt = _dt.datetime.fromtimestamp(ts, tz=_OSD_TZINFO)
    osd_text = dt.strftime(_OSD_FMT)
    return {
        "capture_epoch_ms": int(round(ts * 1000.0)),
        "capture_iso": dt.isoformat(timespec="milliseconds"),
        "osd_text": osd_text,
    }


def _load_osd_pil_font():
    if not _OSD_PIL_STATE.get("init"):
        _OSD_PIL_STATE["init"] = True
        try:
            from PIL import Image as _PILImage  # noqa
            from PIL import ImageDraw as _PILImageDraw  # noqa
            from PIL import ImageFont as _PILImageFont  # noqa

            font = None
            if _OSD_FONT:
                try:
                    font = _PILImageFont.truetype(_OSD_FONT, _OSD_SIZE)
                except Exception:
                    font = None
            if font is None:
                try:
                    font = _PILImageFont.load_default()
                except Exception:
                    font = None
            _OSD_PIL_STATE.update(
                {"Image": _PILImage, "ImageDraw": _PILImageDraw, "ImageFont": _PILImageFont, "font": font}
            )
        except Exception:
            _OSD_PIL_STATE.update({"Image": None, "ImageDraw": None, "ImageFont": None, "font": None})
    return _OSD_PIL_STATE.get("Image"), _OSD_PIL_STATE.get("ImageDraw"), _OSD_PIL_STATE.get("font")


def _load_osd_cv2():
    if not _OSD_CV2_STATE.get("init"):
        _OSD_CV2_STATE["init"] = True
        try:
            import cv2 as _cv2  # noqa

            _OSD_CV2_STATE["cv2"] = _cv2
        except Exception:
            _OSD_CV2_STATE["cv2"] = None
    return _OSD_CV2_STATE.get("cv2")


def _osd_pick_text_colors(arr: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> tuple[tuple[int, int, int, int], tuple[int, int, int, int], float]:
    luma = 128.0
    try:
        h, w = arr.shape[:2]
        x0c = max(0, min(w, int(x0)))
        x1c = max(0, min(w, int(x1)))
        y0c = max(0, min(h, int(y0)))
        y1c = max(0, min(h, int(y1)))
        if x1c > x0c and y1c > y0c:
            rgb = arr[y0c:y1c, x0c:x1c, :3].astype(np.float32)
            luma_map = rgb[..., 0] * 0.2126 + rgb[..., 1] * 0.7152 + rgb[..., 2] * 0.0722
            luma = float(np.mean(luma_map))
    except Exception:
        pass
    if _OSD_COLOR_MODE == "black":
        text_rgb = (0, 0, 0)
    elif _OSD_COLOR_MODE == "white":
        text_rgb = (255, 255, 255)
    else:
        text_rgb = (0, 0, 0) if luma >= _OSD_BG_LUMA_THRESHOLD else (255, 255, 255)
    stroke_rgb = (255, 255, 255) if text_rgb == (0, 0, 0) else (0, 0, 0)
    return (*text_rgb, 255), (*stroke_rgb, _OSD_STROKE_ALPHA), luma


def _draw_osd_rgba_inplace(rgba_u8: np.ndarray, text: str) -> tuple[np.ndarray, float, str]:
    if not (_OSD_FRAME_CAPTURE_ENABLED and text):
        return rgba_u8, 0.0, "disabled"
    t0 = time.monotonic()
    arr = np.ascontiguousarray(rgba_u8)
    pil_image_mod, pil_draw_mod, pil_font = _load_osd_pil_font()
    if pil_image_mod is not None and pil_draw_mod is not None and pil_font is not None:
        try:
            img = pil_image_mod.fromarray(arr)
            draw = pil_draw_mod.Draw(img, "RGBA")
            try:
                bbox = draw.textbbox((_OSD_X, _OSD_Y), text, font=pil_font)
                tw = int(bbox[2] - bbox[0])
                th = int(bbox[3] - bbox[1])
            except Exception:
                tw = max(1, int(len(text) * _OSD_SIZE * 0.62))
                th = int(_OSD_SIZE * 1.25)
            pad_x = 4
            pad_y = 4
            box = [
                _OSD_X,
                _OSD_Y,
                _OSD_X + tw + pad_x * 2,
                _OSD_Y + th + pad_y * 2,
            ]
            fill, stroke_fill, luma = _osd_pick_text_colors(arr, box[0], box[1], box[2], box[3])
            if _OSD_BOX_ENABLED:
                box_fill = (0, 0, 0, 96) if fill[:3] == (255, 255, 255) else (255, 255, 255, 80)
                draw.rectangle(box, fill=box_fill)
            text_xy = (_OSD_X + pad_x, _OSD_Y + pad_y)
            draw.text(
                text_xy,
                text,
                font=pil_font,
                fill=fill,
                stroke_width=_OSD_STROKE_WIDTH,
                stroke_fill=stroke_fill,
            )
            out = np.ascontiguousarray(np.asarray(img, dtype=np.uint8))
            method = "pillow_auto_black" if fill[:3] == (0, 0, 0) else "pillow_auto_white"
            if _OSD_COLOR_MODE in ("black", "white"):
                method = f"pillow_{_OSD_COLOR_MODE}"
            return out, (time.monotonic() - t0) * 1000.0, f"{method}_luma_{luma:.0f}"
        except Exception:
            pass
    cv2 = _load_osd_cv2()
    if cv2 is not None:
        try:
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = max(0.35, _OSD_SIZE / 30.0)
            thickness = max(1, int(round(_OSD_SIZE / 14.0)))
            (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
            pad_x = 5
            pad_y = 5
            x0 = _OSD_X
            y0 = _OSD_Y
            x1 = min(arr.shape[1], x0 + tw + pad_x * 2)
            y1 = min(arr.shape[0], y0 + th + baseline + pad_y * 2)
            fill, stroke_fill, luma = _osd_pick_text_colors(arr, x0, y0, x1, y1)
            if _OSD_BOX_ENABLED and x1 > x0 and y1 > y0:
                roi = arr[y0:y1, x0:x1, :3]
                overlay = roi.copy()
                overlay[:, :] = (0, 0, 0) if fill[:3] == (255, 255, 255) else (255, 255, 255)
                cv2.addWeighted(overlay, 0.35, roi, 0.65, 0, dst=roi)
            org = (_OSD_X + pad_x, _OSD_Y + pad_y + th)
            if _OSD_STROKE_WIDTH > 0:
                cv2.putText(
                    arr,
                    text,
                    org,
                    font,
                    scale,
                    stroke_fill,
                    thickness + _OSD_STROKE_WIDTH * 2,
                    cv2.LINE_AA,
                )
            cv2.putText(arr, text, org, font, scale, fill, thickness, cv2.LINE_AA)
            method = "opencv_auto_black" if fill[:3] == (0, 0, 0) else "opencv_auto_white"
            if _OSD_COLOR_MODE in ("black", "white"):
                method = f"opencv_{_OSD_COLOR_MODE}"
            return arr, (time.monotonic() - t0) * 1000.0, f"{method}_luma_{luma:.0f}"
        except Exception:
            pass
    return arr, (time.monotonic() - t0) * 1000.0, "unavailable"


def _prepare_rtsp_rgba_frame(rgba_u8, capture_epoch_s: float | None = None) -> tuple[bytes, dict]:
    meta = _capture_time_meta(capture_epoch_s)
    arr = np.ascontiguousarray(rgba_u8)
    draw_ms = 0.0
    method = "disabled"
    if _OSD_FRAME_CAPTURE_ENABLED:
        arr, draw_ms, method = _draw_osd_rgba_inplace(arr, str(meta.get("osd_text") or ""))
    meta["osd_draw_ms"] = round(float(draw_ms), 3)
    meta["osd_draw_method"] = method
    meta["osd_applied"] = bool(_OSD_FRAME_CAPTURE_ENABLED and method != "unavailable")
    return arr.tobytes(), meta


# ----- RenderProduct / RGB annotator 生命周期与 get_data() 原始缓冲诊断（只读观测）-----
_render_capture_meta_lock = threading.Lock()
_RENDER_CAPTURE_BIND_SEQ = 0
_RENDER_CAPTURE_LATEST: dict = {}
_LAST_GPU_OOM_EVENT: dict | None = None
_RENDER_CAPTURE_REBIND_COOLDOWN_S = max(30.0, float(cfg.get("render_capture_rebind_cooldown_s", 30.0)))
_render_capture_rebind_lock = threading.Lock()
_render_capture_rebinding = False
_last_render_capture_rebind_attempt_mono = 0.0
_last_render_capture_rebind_finish_mono = 0.0


def _is_gpu_device_oom_message(text: str) -> bool:
    if not text:
        return False
    u = text.upper()
    if "ERROR_OUT_OF_DEVICE_MEMORY" in u or "OUT_OF_DEVICE_MEMORY" in u:
        return True
    if "CUDA" in u and "OUT OF MEMORY" in u.replace("_", " "):
        return True
    return False


def _note_gpu_oom_from_exception(exc: BaseException, phase: str) -> None:
    global _LAST_GPU_OOM_EVENT
    parts = [f"{type(exc).__name__}: {exc}"]
    try:
        import traceback as _tb

        parts.append(_tb.format_exc())
    except Exception:
        pass
    msg = "\n".join(parts)
    if not _is_gpu_device_oom_message(msg):
        return
    ident = _snapshot_render_capture_identity(f"oom_after_{phase}")
    with _render_capture_meta_lock:
        _LAST_GPU_OOM_EVENT = {
            "phase": phase,
            "exc_type": type(exc).__name__,
            "exc_str": str(exc),
            "wall_time_iso": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "render_capture_identity": ident,
            "render_capture_latest": dict(_RENDER_CAPTURE_LATEST),
        }
    print(
        "[render-capture-diag] GPU_OOM_EXCEPTION "
        f"phase={phase} exc={type(exc).__name__}: {exc} | {ident}",
        flush=True,
    )


def _numpy_buffer_diag_stats(arr) -> dict:
    """annotator.get_data() 原始缓冲的只读统计（黑帧钉点用）。"""
    try:
        if arr is None:
            return {"error": "buffer_is_none"}
        x = np.asarray(arr)
        if x.size == 0:
            return {"error": "buffer_empty", "shape": tuple(x.shape), "dtype": str(x.dtype)}
        flat = x.ravel()
        finite_mask = np.isfinite(flat)
        if not np.any(finite_mask):
            return {
                "shape": tuple(x.shape),
                "dtype": str(x.dtype),
                "min": None,
                "max": None,
                "mean": None,
                "nonzero_ratio": 0.0,
                "all_zero": True,
                "note": "no_finite_values",
            }
        xf = flat[finite_mask]
        if np.issubdtype(x.dtype, np.floating):
            nonzero_ratio = float(np.mean(np.abs(xf) > 1e-12))
            all_zero = bool(np.all(np.abs(x) <= 1e-12))
        else:
            nonzero_ratio = float(np.mean(xf != 0))
            all_zero = bool(np.all(x == 0))
        return {
            "shape": tuple(x.shape),
            "dtype": str(x.dtype),
            "min": float(np.min(xf)),
            "max": float(np.max(xf)),
            "mean": float(np.mean(xf)),
            "nonzero_ratio": nonzero_ratio,
            "all_zero": all_zero,
        }
    except Exception as exc:
        return {"error": f"stats_failed:{type(exc).__name__}:{exc}"}


def _render_capture_near_black_channel_stats(rgba_raw) -> dict:
    """NEAR_BLACK 专用：raw 缓冲 RGB vs Alpha 分通道统计（仅日志，不影响主流程）。"""
    try:
        if rgba_raw is None:
            return {"channel_stats_error": "buffer_is_none"}
        arr = np.asarray(rgba_raw)
        if arr.ndim != 3:
            return {
                "channel_stats_error": f"expected_ndim3_got_ndim{int(arr.ndim)}",
                "shape": tuple(int(x) for x in arr.shape),
            }
        channel_count = int(arr.shape[-1])
        if channel_count < 3:
            return {
                "channel_stats_error": f"channels_lt_3:{channel_count}",
                "channel_count": channel_count,
                "shape": tuple(int(x) for x in arr.shape),
            }
        rgb = arr[..., :3]
        rgb_sz = int(rgb.size)
        if rgb_sz == 0:
            rgb_min = rgb_max = rgb_mean = 0.0
            rgb_nonzero_ratio = 0.0
        else:
            rgb_flat = rgb.reshape(-1)
            rgb_min = float(np.min(rgb_flat))
            rgb_max = float(np.max(rgb_flat))
            rgb_mean = float(np.mean(rgb_flat))
            rgb_nonzero_ratio = float(np.count_nonzero(rgb_flat)) / float(rgb_sz)
        if channel_count == 4:
            a = arr[..., 3]
            a_sz = int(a.size)
            if a_sz == 0:
                alpha_min = alpha_max = alpha_mean = 0.0
                alpha_nonzero_ratio = 0.0
            else:
                a_flat = a.reshape(-1)
                alpha_min = float(np.min(a_flat))
                alpha_max = float(np.max(a_flat))
                alpha_mean = float(np.mean(a_flat))
                alpha_nonzero_ratio = float(np.count_nonzero(a_flat)) / float(a_sz)
            return {
                "channel_count": channel_count,
                "rgb_min": rgb_min,
                "rgb_max": rgb_max,
                "rgb_mean": rgb_mean,
                "rgb_nonzero_ratio": rgb_nonzero_ratio,
                "alpha_min": alpha_min,
                "alpha_max": alpha_max,
                "alpha_mean": alpha_mean,
                "alpha_nonzero_ratio": alpha_nonzero_ratio,
            }
        return {
            "channel_count": channel_count,
            "rgb_min": rgb_min,
            "rgb_max": rgb_max,
            "rgb_mean": rgb_mean,
            "rgb_nonzero_ratio": rgb_nonzero_ratio,
            "alpha_min": None,
            "alpha_max": None,
            "alpha_mean": None,
            "alpha_nonzero_ratio": None,
        }
    except Exception as e:
        return {"channel_stats_error": repr(e)}


def _snapshot_render_capture_identity(note: str = "") -> str:
    with _render_capture_meta_lock:
        d = dict(_RENDER_CAPTURE_LATEST)
    tail = f" note={note}" if note else ""
    return (
        f"bind_seq={d.get('bind_seq')} rp_pyid={d.get('rp_pyid')} "
        f"annotator_pyid={d.get('annotator_pyid')} rp_repr={d.get('rp_repr')}{tail}"
    )


def _render_capture_on_bind(rp, annotator, camera_prim_path: str, width: int, height: int) -> None:
    global _RENDER_CAPTURE_BIND_SEQ, _LAST_RENDER_CAPTURE_BIND_MONO
    with _render_capture_meta_lock:
        _RENDER_CAPTURE_BIND_SEQ += 1
        bseq = _RENDER_CAPTURE_BIND_SEQ
        meta = {
            "bind_seq": bseq,
            "rp_pyid": id(rp),
            "annotator_pyid": id(annotator),
            "rp_repr": str(rp),
            "annotator_repr": str(annotator),
            "camera_prim_path": camera_prim_path,
            "resolution_wh": [int(width), int(height)],
            "created_wall_time_iso": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "created_monotonic": float(time.monotonic()),
        }
        _RENDER_CAPTURE_LATEST.clear()
        _RENDER_CAPTURE_LATEST.update(meta)
    _LAST_RENDER_CAPTURE_BIND_MONO = time.monotonic()
    print(
        "[render-capture-diag] BIND "
        f"bind_seq={bseq} rp_pyid={id(rp)} annotator_pyid={id(annotator)} "
        f"wh=({int(width)},{int(height)}) camera={camera_prim_path} "
        f"created={meta['created_wall_time_iso']}",
        flush=True,
    )
    _stream_diag_update(
        camera_bound_ok=True,
        render_product=str(rp),
        render_capture_bind_seq=bseq,
        render_capture_rp_pyid=id(rp),
        render_capture_annotator_pyid=id(annotator),
        render_capture_rp_repr=str(rp),
        render_capture_created_iso=meta["created_wall_time_iso"],
    )


def _compose_renderer_hydra_observation() -> dict:
    """区分「配置来源」与「运行时 carb / viewport 观测」，不过度依赖单一 /rtx/rendermode。"""
    observed: dict = {}
    try:
        import carb

        s = carb.settings.get_settings()
        keys = (
            "/app/renderer/plugin",
            "/rtx/rendermode",
            "/rtx/hydra/materialSyncMode",
            "/rtx/hydra/geometrySyncMode",
            "/omni/replicator/RTSubframes",
            "/persistent/app/viewport/displayOptions",
            "/renderer/multiGpu/currentGpu",
            "/rtx/post/autoExposure/enabled",
            "/rtx/autoExposure/enabled",
            "/rtx/post/tonemap/op",
        )
        for k in keys:
            try:
                observed[k] = _json_safe_value(s.get(k))
            except Exception:
                observed[k] = "<read_error>"
    except Exception as exc:
        observed = {"carb_read_error": f"{type(exc).__name__}:{exc}"}
    viewport_hint: dict = {}
    try:
        import omni.kit.viewport.utility as vpu  # type: ignore

        avp = None
        for fn_name in ("get_active_viewport", "get_active_viewport_window"):
            fn = getattr(vpu, fn_name, None)
            if callable(fn):
                try:
                    avp = fn()
                except Exception:
                    avp = None
                if avp is not None:
                    break
        if avp is not None:
            viewport_hint["active_viewport_type"] = type(avp).__name__
            for attr in ("hydra_engine", "delegate", "render_product_path"):
                if hasattr(avp, attr):
                    try:
                        viewport_hint[attr] = _json_safe_value(getattr(avp, attr))
                    except Exception as exc:
                        viewport_hint[attr] = f"<err:{exc}>"
    except Exception as exc:
        viewport_hint["viewport_read_error"] = f"{type(exc).__name__}:{exc}"
    return {
        "config_source": {
            "simulation_app_launch_renderer_kwarg": renderer_mode,
            "launch_resolution_wh_from_simulation_app_kwarg": [int(W), int(H)],
            "hydra_c_branch_geometry_only_cfg": bool(cfg.get("hydra_c_branch_geometry_only", False)),
        },
        "carb_settings_observed_at_poll": observed,
        "viewport_delegate_observed": viewport_hint,
    }


def _compose_render_capture_diagnostics_dict() -> dict:
    with _render_capture_meta_lock:
        latest = dict(_RENDER_CAPTURE_LATEST)
        oom = dict(_LAST_GPU_OOM_EVENT) if _LAST_GPU_OOM_EVENT else None
    try:
        renderer_obs = _compose_renderer_hydra_observation()
    except Exception as exc:
        renderer_obs = {"error": f"{type(exc).__name__}:{exc}"}
    with _same_tick_diag_lock:
        same_tick = dict(_same_tick_latest_diag) if isinstance(_same_tick_latest_diag, dict) else None
    return {
        "ok": True,
        "render_capture_latest_bind": latest,
        "last_gpu_oom_correlation": oom,
        "renderer_config_vs_observed": renderer_obs,
        "same_tick_pipeline": same_tick,
    }


# ----- 同帧抓图诊断（capture_seq 贯穿 get_data→normalize→jpeg→snapshot 缓存）-----
_LAST_RENDER_CAPTURE_BIND_MONO = 0.0
_SAME_TICK_SEQ_LOCK = threading.Lock()
_same_tick_capture_seq = 0
_same_tick_diag_lock = threading.Lock()
_same_tick_latest_diag: dict | None = None
_post_recover_first_capture_pending = False
_render_capture_probe_lock = threading.Lock()
_render_capture_probe_dirty = threading.Event()
_render_capture_probe_holder: dict | None = None
_render_capture_ab_probe_lock = threading.Lock()
_render_capture_ab_probe_dirty = threading.Event()
_render_capture_ab_probe_holder: dict | None = None
# GET /snapshot.jpg 在 cfg 开启时由主线程单独抓 viewport（不写 _snapshot_cache，避免动 MJPEG/RTSP 主链路）
_snapshot_http_viewport_lock = threading.Lock()
_snapshot_http_viewport_dirty = threading.Event()
_snapshot_http_viewport_holder: dict | None = None
_snapshot_http_viewport_frame_id = 0
# GET /diagnostics/snapshot-live-once：主线程即时抓图，绕开 snapshot/MJPEG/last_good 缓存（仅诊断）
_diag_live_once_lock = threading.Lock()
_diag_live_once_dirty = threading.Event()
_diag_live_once_holder: dict | None = None
# GET /snapshot.jpg：主线程即时 viewport（独立队列，避免与 diagnostics/snapshot-live-once 互抢 holder）
_snapshot_jpg_live_vp_lock = threading.Lock()
_snapshot_jpg_live_vp_dirty = threading.Event()
_snapshot_jpg_live_vp_holder: dict | None = None
_snapshot_live_viewport_http_frame_id = 0


def _cfg_capture_prefer_viewport_delegate_for_snapshot() -> bool:
    """可选 YAML：capture_source_prefer_viewport_delegate_for_snapshot（默认 false，仅改 ptz_stream 时由用户手工加键）。"""
    try:
        return bool(cfg.get("capture_source_prefer_viewport_delegate_for_snapshot", False))
    except Exception:
        return False


def _cfg_capture_prefer_rtsp_latest_for_snapshot() -> bool:
    try:
        return bool(_CAPTURE_SOURCE_PREFER_RTSP_LATEST_FOR_SNAPSHOT)
    except Exception:
        return True


def _viewport_pixels_preferred(vp_u8, rep_u8) -> bool:
    """同一分辨率下：viewport 像素是否明显优于 Replicator（用于最小来源切换，保守阈值）。"""
    if vp_u8 is None or rep_u8 is None:
        return False
    try:
        vp_rgb = np.asarray(vp_u8)[..., :3].astype(np.float32)
        rp_rgb = np.asarray(rep_u8)[..., :3].astype(np.float32)
        vp_m = float(np.mean(vp_rgb))
        rp_m = float(np.mean(rp_rgb))
        vp_nz = float(np.mean(vp_rgb > 1e-3))
        rp_nz = float(np.mean(rp_rgb > 1e-3))
        if rp_m < 1e-5:
            return vp_m > 0.05
        return (vp_m > rp_m * 2.5 and vp_m - rp_m > 0.04) or (vp_nz > rp_nz * 2.0 and vp_nz > 0.04)
    except Exception:
        return False


def _try_read_viewport_delegate_rgba_uint8(
    _viewport_capture_diag: dict | None = None,
    *,
    timeout_ms: float | None = None,
) -> tuple:
    """
    从当前 Kit active viewport 抓取一帧 LDR 像素（与 diagnostics 中 viewport_delegate 同源）。
    返回 (rgba_uint8_HxWx4 或 None, render_product_path 或 None, error 或 None)。

    可选 _viewport_capture_diag：写入 capture 诊断（capture_method / callback_fired / wait_ms 等），
    供 AB probe 合并进 viewport_delegate_source。
    """
    _d = _viewport_capture_diag
    if _d is not None:
        _d.clear()
        _d["render_product_path"] = None
        _d["viewport_window_title"] = None
        _d["viewport_delegate_id"] = None
        _d["capture_method"] = None
        _d["callback_fired"] = False
        _d["wait_ms"] = None
        _d["capture_epoch_s"] = None

    rp_path = None
    try:
        renderer_obs = _compose_renderer_hydra_observation()
        vpobs = renderer_obs.get("viewport_delegate_observed") or {}
        rp_path = vpobs.get("render_product_path")
        if rp_path is not None and not isinstance(rp_path, str):
            rp_path = str(rp_path)
    except Exception:
        rp_path = None
    if _d is not None:
        _d["render_product_path"] = rp_path

    try:
        import omni.kit.viewport.utility as vpu
    except ImportError as e:
        if _d is not None:
            _d["capture_method"] = "unavailable:omni.kit.viewport.utility"
            _d["wait_ms"] = 0.0
        return None, rp_path, f"ImportError:omni.kit.viewport.utility:{e}"

    vp_obj = None
    last_vp_err = None
    for fn_name in ("get_active_viewport_window", "get_active_viewport"):
        fn = getattr(vpu, fn_name, None)
        if not callable(fn):
            continue
        try:
            vp_obj = fn()
        except Exception as ex:
            last_vp_err = f"{fn_name}:{type(ex).__name__}:{ex}"
            vp_obj = None
        if vp_obj is not None:
            break
    if vp_obj is None:
        if _d is not None:
            _d["capture_method"] = "no_active_viewport"
            _d["wait_ms"] = 0.0
        return None, rp_path, last_vp_err or "no_active_viewport"

    vapi = getattr(vp_obj, "viewport_api", None)
    if vapi is None:
        gva = getattr(vp_obj, "get_viewport_api", None)
        if callable(gva):
            try:
                vapi = gva()
            except Exception as ex:
                vapi = None
                last_vp_err = f"get_viewport_api:{type(ex).__name__}:{ex}"
    if vapi is None:
        if _d is not None:
            _d["capture_method"] = "viewport_api_unresolved"
            _d["wait_ms"] = 0.0
        return None, rp_path, (last_vp_err or "viewport_api_unresolved")

    if _d is not None:
        try:
            _d["viewport_delegate_id"] = getattr(vapi, "id", None)
        except Exception:
            _d["viewport_delegate_id"] = None
        try:
            _fn_gav = getattr(vpu, "get_active_viewport_and_window", None)
            if callable(_fn_gav):
                _v2, _win = _fn_gav()
                if _win is not None and getattr(_win, "title", None) is not None:
                    _d["viewport_window_title"] = str(_win.title)
        except Exception:
            pass

    box: dict = {"err": None, "arr": None, "w": None, "h": None, "fmt": None, "capture_epoch_s": None}
    evt = threading.Event()
    wait_timeout_s = max(0.02, float(timeout_ms) / 1000.0) if timeout_ms is not None else None

    def _cb(*args, **kwargs):
        try:
            box["capture_epoch_s"] = time.time()
            if kwargs and not args:
                args = tuple(kwargs.values())
            if len(args) == 1:
                a0 = args[0]
                if isinstance(a0, np.ndarray):
                    box["arr"] = np.ascontiguousarray(a0)
                    if box["arr"].ndim == 3:
                        box["h"], box["w"] = int(box["arr"].shape[0]), int(box["arr"].shape[1])
                    return
                to_np = getattr(a0, "to_numpy", None)
                if callable(to_np):
                    try:
                        box["arr"] = np.asarray(to_np())
                    except Exception:
                        try:
                            box["arr"] = np.asarray(a0)
                        except Exception as ex2:
                            box["err"] = f"to_numpy:{type(ex2).__name__}:{ex2}"
                    if isinstance(box["arr"], np.ndarray) and box["arr"].ndim == 3:
                        box["h"], box["w"] = int(box["arr"].shape[0]), int(box["arr"].shape[1])
                    return
                try:
                    box["arr"] = np.asarray(a0)
                    if isinstance(box["arr"], np.ndarray) and box["arr"].ndim == 3:
                        box["h"], box["w"] = int(box["arr"].shape[0]), int(box["arr"].shape[1])
                except Exception as ex:
                    box["err"] = f"asarray:{type(ex).__name__}:{ex}"
                return
            if len(args) >= 4:
                buf, bsize, w, h = args[0], int(args[1]), int(args[2]), int(args[3])
                fmt = args[4] if len(args) > 4 else None
                box["fmt"] = str(fmt) if fmt is not None else None
                arr = None
                try:
                    mv = memoryview(buf)[:bsize]
                    arr = np.frombuffer(mv, dtype=np.uint8).copy()
                except TypeError:
                    # Kit 新版 renderer_capture 可能传 PyCapsule，不能直接 memoryview；与 material_preview 一致走 convert。
                    try:
                        import omni.kit.renderer_capture as _rcap

                        _raw = _rcap.convert_raw_bytes_to_list(buf, bsize, w, h, fmt)
                        arr = np.asarray(_raw, dtype=np.uint8).ravel()
                        if arr.size != bsize:
                            box["err"] = (
                                f"convert_raw_bytes_to_list:size_mismatch got={arr.size} expect={bsize}"
                            )
                            return
                    except Exception as cap_ex:
                        box["err"] = f"buffer_convert:{type(cap_ex).__name__}:{cap_ex}"
                        return
                c = (bsize // (w * h)) if w * h else 4
                if c < 3:
                    c = 4
                try:
                    arr = arr.reshape((h, w, c))
                except Exception as rex:
                    box["err"] = f"reshape:{type(rex).__name__}:{rex}"
                    return
                box["arr"] = arr
                box["w"], box["h"] = int(w), int(h)
        except Exception as ex:
            box["err"] = f"callback:{type(ex).__name__}:{ex}"
        finally:
            evt.set()

    t_cap = time.monotonic()
    capture_delegate = None
    try:
        capture_delegate = vpu.capture_viewport_to_buffer(vapi, _cb, False)
    except Exception as ex:
        if _d is not None:
            _d["capture_method"] = "capture_viewport_to_buffer_raise"
            _d["wait_ms"] = round((time.monotonic() - t_cap) * 1000.0, 2)
        return None, rp_path, f"capture_viewport_to_buffer:{type(ex).__name__}:{ex}"

    if capture_delegate is None:
        if _d is not None:
            _d["capture_method"] = "schedule_capture_returned_none"
            _d["wait_ms"] = round((time.monotonic() - t_cap) * 1000.0, 2)
        return None, rp_path, "schedule_capture_returned_none"

    err_wait = None
    used_run_coroutine = False
    try:
        wfr = getattr(capture_delegate, "wait_for_result", None)
        if callable(wfr) and hasattr(sim_app, "run_coroutine"):
            used_run_coroutine = True

            async def _await_capture():
                return await asyncio.wait_for(wfr(2), timeout=wait_timeout_s or 14.0)

            sim_app.run_coroutine(_await_capture())
        else:
            fut = capture_delegate
            if hasattr(fut, "wait"):
                try:
                    fut.wait(wait_timeout_s or 5.0)
                except TypeError:
                    if wait_timeout_s is None:
                        fut.wait()
                    else:
                        err_wait = "viewport_capture_wait_no_timeout_support"
            remaining_wait_s = 6.0
            if wait_timeout_s is not None:
                remaining_wait_s = max(0.001, wait_timeout_s - (time.monotonic() - t_cap))
            if err_wait is None and not evt.wait(remaining_wait_s):
                err_wait = "viewport_capture_callback_timeout"
    except asyncio.TimeoutError:
        err_wait = "viewport_capture_async_timeout"
    except Exception as ex:
        err_wait = f"viewport_capture_wait:{type(ex).__name__}:{ex}"

    if _d is not None:
        _d["capture_method"] = (
            "byte_capture_LdrColor+sim_app.run_coroutine(wait_for_result)"
            if used_run_coroutine
            else "byte_capture_LdrColor+legacy_threading_event_wait"
        )
        _d["callback_fired"] = bool(evt.is_set())
        _d["wait_ms"] = round((time.monotonic() - t_cap) * 1000.0, 2)
        _d["capture_epoch_s"] = box.get("capture_epoch_s")

    if err_wait:
        return None, rp_path, err_wait

    if box.get("err"):
        return None, rp_path, str(box["err"])
    arr = box.get("arr")
    if arr is None or getattr(arr, "size", 0) <= 0:
        return None, rp_path, "viewport_capture_empty_buffer"

    if arr.ndim != 3 or arr.shape[2] < 3:
        return None, rp_path, f"bad_shape:{getattr(arr, 'shape', None)}"

    fmt_u = (box.get("fmt") or "").upper()
    if "BGR" in fmt_u or "B8G8R8" in fmt_u:
        if arr.shape[2] == 4:
            b, g, r, a = arr[..., 0], arr[..., 1], arr[..., 2], arr[..., 3]
            arr = np.stack([r, g, b, a], axis=-1).astype(np.uint8, copy=False)
        else:
            arr = arr[..., ::-1].astype(np.uint8, copy=False)
            if arr.shape[2] == 3:
                al = np.full((arr.shape[0], arr.shape[1], 1), 255, dtype=np.uint8)
                arr = np.concatenate([arr, al], axis=-1)
    elif arr.shape[2] == 3:
        al = np.full((arr.shape[0], arr.shape[1], 1), 255, dtype=np.uint8)
        arr = np.concatenate([arr, al], axis=-1)

    if arr.shape[:2] != (H, W):
        try:
            arr = np.ascontiguousarray(np.resize(arr, (H, W, arr.shape[2] if arr.ndim == 3 else 4)))
        except Exception as ex:
            return None, rp_path, f"resize_viewport_to_cfg:{type(ex).__name__}:{ex}"

    return np.ascontiguousarray(arr), rp_path, None


def _try_rtsp_rgba_from_viewport_delegate_safe(*, return_meta: bool = False):
    """主线程 RTSP 入队：尝试 live viewport_delegate RGBA；成功返回 (H,W,4) uint8，失败返回 None（不抛）。"""
    meta: dict = {}
    try:
        if return_meta:
            vp_np, _rp_path, _vp_err, meta = _try_read_viewport_delegate_rgba_uint8_for_rtsp_camera_aligned(
                timeout_ms=_RTSP_VIEWPORT_CAPTURE_TIMEOUT_MS,
                return_meta=True,
            )
        else:
            vp_np, _rp_path, _vp_err = _try_read_viewport_delegate_rgba_uint8_for_rtsp_camera_aligned()
    except Exception as exc:
        if return_meta:
            return None, {"error": f"{type(exc).__name__}:{exc}"}
        return None
    try:
        wait_ms = meta.get("wait_ms")
        if wait_ms is not None:
            _stream_diag_update(rtsp_viewport_capture_wait_ms=float(wait_ms))
    except Exception:
        pass
    if vp_np is None or getattr(vp_np, "size", 0) <= 0:
        if return_meta:
            return None, meta
        return None
    try:
        vp_u8 = _normalize_replicator_rgba_for_output(vp_np)
    except Exception:
        if return_meta:
            return None, meta
        return None
    if vp_u8 is None or getattr(vp_u8, "size", 0) <= 0:
        if return_meta:
            return None, meta
        return None
    try:
        if vp_u8.shape[:2] != (H, W):
            vp_u8 = np.ascontiguousarray(
                np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4))
            )
        if vp_u8.shape != (H, W, 4):
            if return_meta:
                return None, meta
            return None
        out = np.ascontiguousarray(vp_u8, dtype=np.uint8)
        if return_meta:
            return out, meta
        return out
    except Exception:
        if return_meta:
            return None, meta
        return None


def _snapshot_http_resolve_active_viewport_api() -> tuple:
    """仅 snapshot HTTP：解析当前 Kit active viewport 的 viewport_api。返回 (vapi_or_None, error_or_None)。"""
    try:
        import omni.kit.viewport.utility as vpu
    except ImportError as e:
        return None, f"ImportError:omni.kit.viewport.utility:{e}"
    vp_obj = None
    last_err = None
    for fn_name in ("get_active_viewport_window", "get_active_viewport"):
        fn = getattr(vpu, fn_name, None)
        if not callable(fn):
            continue
        try:
            vp_obj = fn()
        except Exception as ex:
            last_err = f"{fn_name}:{type(ex).__name__}:{ex}"
            vp_obj = None
        if vp_obj is not None:
            break
    if vp_obj is None:
        return None, last_err or "no_active_viewport"
    vapi = getattr(vp_obj, "viewport_api", None)
    if vapi is None:
        gva = getattr(vp_obj, "get_viewport_api", None)
        if callable(gva):
            try:
                vapi = gva()
            except Exception as ex:
                return None, f"get_viewport_api:{type(ex).__name__}:{ex}"
    if vapi is None:
        return None, "viewport_api_unresolved"
    return vapi, None


def _snapshot_viewport_api_read_camera_path(vapi) -> str | None:
    """读取 viewport_api 当前相机 prim 路径（兼容不同 Kit 版本）。"""
    try:
        gac = getattr(vapi, "get_active_camera", None)
        if callable(gac):
            p = gac()
            if p is not None:
                return str(p)
    except Exception:
        pass
    try:
        cp = getattr(vapi, "camera_path", None)
        if cp is not None:
            return str(cp)
    except Exception:
        pass
    return None


def _snapshot_viewport_api_write_camera_path(vapi, path: str) -> bool:
    """将 viewport_api 绑定到 path；成功返回 True。"""
    if not path:
        return False
    try:
        if hasattr(vapi, "camera_path"):
            vapi.camera_path = path
            return True
    except Exception:
        pass
    try:
        sac = getattr(vapi, "set_active_camera", None)
        if callable(sac):
            sac(path)
            return True
    except Exception:
        pass
    return False


# RTSP viewport_delegate：与 HTTP live snapshot 同款「临时绑定 camera_prim → 抓帧 → 恢复」；日志节流避免刷屏
_RTSP_VIEWPORT_CAMERA_BIND_LOG_MONO: float = 0.0
_RTSP_VIEWPORT_PRIMARY = False
_RTSP_VIEWPORT_PRIMARY_LAST_PROBE_MONO = 0.0
_RTSP_VIEWPORT_PRIMARY_LAST_SWITCH_MONO = 0.0
_RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO = 0.0
_RTSP_STALE_RECOVERY_UNTIL_MONO = 0.0


def _rtsp_log_viewport_camera_bind_throttled(msg: str) -> None:
    global _RTSP_VIEWPORT_CAMERA_BIND_LOG_MONO
    now = time.monotonic()
    if now - _RTSP_VIEWPORT_CAMERA_BIND_LOG_MONO < 30.0:
        return
    _RTSP_VIEWPORT_CAMERA_BIND_LOG_MONO = now
    print(f"[PTZ-RTSP][viewport-camera-bind] {msg}", flush=True)


def _rtsp_viewport_primary_enabled() -> bool:
    return bool(_RTSP_LOW_LATENCY_MODE and _RTSP_VIEWPORT_PRIMARY)


def _rtsp_latest_age_ms(now: float | None = None) -> float | None:
    try:
        latest_mono = float(_rtsp_latest_mono or 0.0)
        if latest_mono <= 0.0:
            return None
        return max(0.0, ((time.monotonic() if now is None else float(now)) - latest_mono) * 1000.0)
    except Exception:
        return None


def _rtsp_update_stale_recovery(now: float | None = None) -> tuple[float | None, str]:
    global _RTSP_STALE_RECOVERY_UNTIL_MONO
    now_f = time.monotonic() if now is None else float(now)
    age_ms = _rtsp_latest_age_ms(now_f)
    if age_ms is not None and age_ms >= _RTSP_STALE_RECOVERY_MS:
        _RTSP_STALE_RECOVERY_UNTIL_MONO = max(
            float(_RTSP_STALE_RECOVERY_UNTIL_MONO),
            now_f + _RTSP_STALE_RECOVERY_HOLD_S,
        )
    mode = "viewport_only_recovery" if now_f < float(_RTSP_STALE_RECOVERY_UNTIL_MONO) else "normal"
    _stream_diag_update(rtsp_stale_recovery_mode=mode)
    return age_ms, mode


def _rtsp_set_viewport_primary(enabled: bool, reason: str) -> None:
    global _RTSP_VIEWPORT_PRIMARY, _RTSP_VIEWPORT_PRIMARY_LAST_SWITCH_MONO
    enabled = bool(enabled)
    if _RTSP_VIEWPORT_PRIMARY == enabled:
        return
    _RTSP_VIEWPORT_PRIMARY = enabled
    _RTSP_VIEWPORT_PRIMARY_LAST_SWITCH_MONO = time.monotonic()
    _stream_diag_update(
        rtsp_capture_mode="viewport_delegate_primary" if enabled else "replicator_probe",
        rtsp_capture_mode_reason=str(reason),
    )
    print(
        f"[PTZ-RTSP][capture-mode] mode={'viewport_delegate_primary' if enabled else 'replicator_probe'} "
        f"reason={reason}",
        flush=True,
    )


def _rtsp_should_probe_replicator() -> bool:
    global _RTSP_VIEWPORT_PRIMARY_LAST_PROBE_MONO
    if not _rtsp_viewport_primary_enabled():
        return True
    now = time.monotonic()
    age_ms, mode = _rtsp_update_stale_recovery(now)
    if mode == "viewport_only_recovery" or (age_ms is not None and age_ms > _RTSP_STALE_WARN_MS):
        return False
    if now - _RTSP_VIEWPORT_PRIMARY_LAST_PROBE_MONO >= _RTSP_VIEWPORT_PRIMARY_PROBE_INTERVAL_S:
        _RTSP_VIEWPORT_PRIMARY_LAST_PROBE_MONO = now
        return True
    return False


def _try_read_viewport_delegate_rgba_uint8_for_rtsp_camera_aligned(
    *,
    timeout_ms: float | None = None,
    return_meta: bool = False,
):
    """
    仅 RTSP 路径：在 _try_read_viewport_delegate_rgba_uint8 前临时将 active viewport 相机切到
    camera_prim（与 _run_snapshot_http_viewport_jpeg / _run_diag_snapshot_live_once_pipeline 同源），
    finally 中恢复；写绑定失败则回退为未对齐抓帧，不中断 RTSP。
    返回 (rgba, render_product_path, err) 与 _try_read_viewport_delegate_rgba_uint8 相同。
    """
    target_prim = str(camera_prim).strip()
    diag: dict = {}
    effective_timeout_ms = (
        _RTSP_VIEWPORT_CAPTURE_TIMEOUT_MS
        if timeout_ms is None and _RTSP_LOW_LATENCY_MODE
        else timeout_ms
    )

    def _ret(vp, rp_path, err):
        if return_meta:
            return vp, rp_path, err, dict(diag)
        return vp, rp_path, err

    vapi, _vapi_err = _snapshot_http_resolve_active_viewport_api()
    if not target_prim or vapi is None:
        vp, rp_path, err = _try_read_viewport_delegate_rgba_uint8(
            diag, timeout_ms=effective_timeout_ms
        )
        return _ret(vp, rp_path, err)

    prev_cam: str | None = None
    align_applied = False
    restore_after_capture = not _RTSP_LOW_LATENCY_MODE
    try:
        try:
            prev_cam = _snapshot_viewport_api_read_camera_path(vapi)
        except Exception:
            prev_cam = None

        if prev_cam == target_prim:
            vp, rp_path, err = _try_read_viewport_delegate_rgba_uint8(
                diag, timeout_ms=effective_timeout_ms
            )
            return _ret(vp, rp_path, err)

        if not _snapshot_viewport_api_write_camera_path(vapi, target_prim):
            _rtsp_log_viewport_camera_bind_throttled(
                f"bind_ok=False restore_skipped rtsp_bind_target={target_prim!r} "
                f"restore_path={prev_cam!r} reason=write_camera_failed"
            )
            vp, rp_path, err = _try_read_viewport_delegate_rgba_uint8(
                diag, timeout_ms=effective_timeout_ms
            )
            return _ret(vp, rp_path, err)

        align_applied = True
        try:
            sim_app.update()
        except Exception as ex:
            _rtsp_log_viewport_camera_bind_throttled(
                f"bind_ok=True restore_pending rtsp_bind_target={target_prim!r} restore_path={prev_cam!r} "
                f"reason=post_bind_update:{type(ex).__name__}"
            )
            return _ret(None, None, f"sim_app.update_after_bind:{type(ex).__name__}:{ex}")

        for _ in range(2):
            try:
                sim_app.update()
            except Exception as ex:
                _rtsp_log_viewport_camera_bind_throttled(
                    f"bind_ok=True restore_pending rtsp_bind_target={target_prim!r} restore_path={prev_cam!r} "
                    f"reason=warmup_update:{type(ex).__name__}"
                )
                return _ret(None, None, f"sim_app.update_warmup:{type(ex).__name__}:{ex}")

        vp, rp_path, err = _try_read_viewport_delegate_rgba_uint8(
            diag, timeout_ms=effective_timeout_ms
        )
        return _ret(vp, rp_path, err)
    finally:
        if align_applied and vapi is not None and restore_after_capture:
            restore_ok: bool | None = None
            try:
                if prev_cam is not None:
                    restore_ok = bool(_snapshot_viewport_api_write_camera_path(vapi, prev_cam))
                else:
                    restore_ok = False
                try:
                    sim_app.update()
                except Exception:
                    pass
            except Exception:
                restore_ok = False
            _rtsp_log_viewport_camera_bind_throttled(
                f"bind_ok=True restore_ok={restore_ok} rtsp_bind_target={target_prim!r} restore_path={prev_cam!r}"
            )
        elif align_applied and vapi is not None:
            _rtsp_log_viewport_camera_bind_throttled(
                f"bind_ok=True restore_skipped_low_latency rtsp_bind_target={target_prim!r} previous_path={prev_cam!r}"
            )


def _run_snapshot_http_viewport_jpeg() -> dict:
    """主线程：cfg 开启时专供 GET /snapshot.jpg，viewport→JPEG；不写 snapshot/MJPEG 缓存。"""
    global _snapshot_http_viewport_frame_id
    align_diag: dict = {
        "viewport_camera_before": None,
        "viewport_camera_target": str(camera_prim),
        "viewport_align_applied": False,
        "viewport_align_write_failed": False,
        "viewport_resolve_error": None,
        "viewport_restore_ok": None,
        "viewport_restore_error": None,
        "viewport_restore_skipped_no_previous": False,
    }
    vapi_align, vapi_err = _snapshot_http_resolve_active_viewport_api()
    if vapi_err:
        align_diag["viewport_resolve_error"] = str(vapi_err)
    prev_cam: str | None = None
    align_applied = False
    target_prim = str(camera_prim).strip()
    if vapi_align is not None and target_prim:
        try:
            prev_cam = _snapshot_viewport_api_read_camera_path(vapi_align)
        except Exception as ex:
            align_diag["viewport_resolve_error"] = f"read_camera_path:{type(ex).__name__}:{ex}"
            prev_cam = None
        align_diag["viewport_camera_before"] = prev_cam
        if prev_cam != target_prim:
            if _snapshot_viewport_api_write_camera_path(vapi_align, target_prim):
                align_applied = True
                align_diag["viewport_align_applied"] = True
                try:
                    sim_app.update()
                except Exception as ex:
                    align_diag["viewport_align_post_write_update_error"] = f"{type(ex).__name__}:{ex}"
            else:
                align_diag["viewport_align_write_failed"] = True

    out: dict | None = None
    try:
        for _ in range(2):
            try:
                sim_app.update()
            except Exception as ex:
                out = {"ok": False, "error": f"sim_app.update:{type(ex).__name__}:{ex}", "viewport_align_diag": align_diag}
                return out
        vp_np, vp_path, vp_err = _try_read_viewport_delegate_rgba_uint8()
        if vp_err or vp_np is None:
            out = {
                "ok": False,
                "error": str(vp_err) if vp_err else "viewport_no_pixels",
                "render_product_path": vp_path,
                "viewport_align_diag": align_diag,
            }
            return out
        vp_u8 = _normalize_replicator_rgba_for_output(vp_np)
        if vp_u8 is None or getattr(vp_u8, "size", 0) <= 0:
            out = {"ok": False, "error": "viewport_normalize_empty", "render_product_path": vp_path, "viewport_align_diag": align_diag}
            return out
        if vp_u8.shape[:2] != (H, W):
            try:
                vp_u8 = np.ascontiguousarray(
                    np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4))
                )
            except Exception as ex:
                out = {"ok": False, "error": f"resize:{type(ex).__name__}:{ex}", "viewport_align_diag": align_diag}
                return out
        if _jpeg_encode_fn is None:
            out = {"ok": False, "error": "jpeg_encoder_unavailable", "viewport_align_diag": align_diag}
            return out
        
        try:
            rmax = int(np.max(vp_u8[:, :, :3]))
            if rmax <= 2:
                out = {"ok": False, "error": f"viewport_black_frame_max_{rmax}", "viewport_align_diag": align_diag}
                return out
        except Exception:
            pass

        try:
            jpg = _jpeg_encode_fn(vp_u8)
        except Exception as ex:
            out = {"ok": False, "error": f"jpeg_encode:{type(ex).__name__}:{ex}", "viewport_align_diag": align_diag}
            return out
        if not jpg:
            out = {"ok": False, "error": "jpeg_encode_empty", "viewport_align_diag": align_diag}
            return out
        _snapshot_http_viewport_frame_id += 1
        out = {
            "ok": True,
            "jpg": bytes(jpg),
            "pixel_source": "viewport_delegate",
            "frame_id": int(_snapshot_http_viewport_frame_id),
            "render_product_path": vp_path,
            "viewport_align_diag": align_diag,
        }
        return out
    finally:
        if align_applied and vapi_align is not None:
            try:
                if prev_cam is not None:
                    ok = _snapshot_viewport_api_write_camera_path(vapi_align, prev_cam)
                    align_diag["viewport_restore_ok"] = bool(ok)
                    if not ok:
                        align_diag["viewport_restore_error"] = "write_previous_camera_path_failed"
                else:
                    align_diag["viewport_restore_skipped_no_previous"] = True
                    align_diag["viewport_restore_ok"] = False
                try:
                    sim_app.update()
                except Exception as ex:
                    align_diag["viewport_restore_post_update_error"] = f"{type(ex).__name__}:{ex}"
            except Exception as ex:
                align_diag["viewport_restore_ok"] = False
                align_diag["viewport_restore_error"] = f"{type(ex).__name__}:{ex}"
        try:
            print(
                "[snapshot-viewport-align] "
                f"before={align_diag.get('viewport_camera_before')!r} "
                f"target={align_diag.get('viewport_camera_target')!r} "
                f"applied={bool(align_diag.get('viewport_align_applied'))} "
                f"write_failed={bool(align_diag.get('viewport_align_write_failed'))} "
                f"restore_ok={align_diag.get('viewport_restore_ok')} "
                f"restore_err={align_diag.get('viewport_restore_error')!r}",
                flush=True,
            )
        except Exception:
            pass


def _run_diag_snapshot_live_once_pipeline(
    annotator, camera_prim_path: str, *, replicator_fallback: bool = True
) -> dict:
    """主线程：诊断用即时一帧 JPEG。不写 last_good / _snapshot_cache / MJPEG；不推进 capture_seq。

    优先 viewport_delegate（与 _run_snapshot_http_viewport_jpeg 同源对齐 + 抓帧）。
    replicator_fallback=True（默认）：viewport 失败后再只读 annotator.get_data()（Replicator）。
    replicator_fallback=False：仅 viewport，失败即返回（供 GET /snapshot.jpg 回退 last_good/cache，不把 Replicator 当“实时 viewport”）。
    """
    align_diag: dict = {
        "viewport_camera_before": None,
        "viewport_camera_target": str(camera_prim_path),
        "viewport_align_applied": False,
        "viewport_align_write_failed": False,
        "viewport_resolve_error": None,
        "viewport_restore_ok": None,
        "viewport_restore_error": None,
        "viewport_restore_skipped_no_previous": False,
    }
    vapi_align, vapi_err = _snapshot_http_resolve_active_viewport_api()
    if vapi_err:
        align_diag["viewport_resolve_error"] = str(vapi_err)
    prev_cam: str | None = None
    align_applied = False
    target_prim = str(camera_prim_path).strip()
    if vapi_align is not None and target_prim:
        try:
            prev_cam = _snapshot_viewport_api_read_camera_path(vapi_align)
        except Exception as ex:
            align_diag["viewport_resolve_error"] = f"read_camera_path:{type(ex).__name__}:{ex}"
            prev_cam = None
        align_diag["viewport_camera_before"] = prev_cam
        if prev_cam != target_prim:
            if _snapshot_viewport_api_write_camera_path(vapi_align, target_prim):
                align_applied = True
                align_diag["viewport_align_applied"] = True
                try:
                    sim_app.update()
                except Exception as ex:
                    align_diag["viewport_align_post_write_update_error"] = f"{type(ex).__name__}:{ex}"
            else:
                align_diag["viewport_align_write_failed"] = True

    def _encode_rgba(vp_u8) -> tuple[bytes | None, str | None]:
        if _jpeg_encode_fn is None:
            return None, "jpeg_encoder_unavailable"
        try:
            jpg = _jpeg_encode_fn(vp_u8)
        except Exception as ex:
            return None, f"jpeg_encode:{type(ex).__name__}"
        if not jpg:
            return None, "jpeg_encode_empty"
        return bytes(jpg), None

    try:
        if _jpeg_encode_fn is None:
            return {"ok": False, "jpg": None, "source": "error", "error": "jpeg_encoder_unavailable", "align_diag": align_diag}
        for _ in range(2):
            try:
                sim_app.update()
            except Exception as ex:
                return {"ok": False, "jpg": None, "source": "error", "error": f"sim_app_update:{type(ex).__name__}", "align_diag": align_diag}
        vp_np, vp_path, vp_err = _try_read_viewport_delegate_rgba_uint8()
        if vp_np is not None and not vp_err:
            vp_u8 = _normalize_replicator_rgba_for_output(vp_np)
            if vp_u8 is not None and getattr(vp_u8, "size", 0) > 0:
                if vp_u8.shape[:2] != (H, W):
                    try:
                        vp_u8 = np.ascontiguousarray(
                            np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4))
                        )
                    except Exception as ex:
                        return {
                            "ok": False,
                            "jpg": None,
                            "source": "error",
                            "error": f"resize_viewport:{type(ex).__name__}",
                            "render_product_path": vp_path,
                            "align_diag": align_diag,
                        }
                jpg, enc_err = _encode_rgba(vp_u8)
                if jpg is not None:
                    return {
                        "ok": True,
                        "jpg": jpg,
                        "source": "viewport_delegate",
                        "error": None,
                        "render_product_path": vp_path,
                        "align_diag": align_diag,
                    }
                return {
                    "ok": False,
                    "jpg": None,
                    "source": "error",
                    "error": enc_err or "jpeg_encode_empty",
                    "render_product_path": vp_path,
                    "align_diag": align_diag,
                }
        vp_fail = str(vp_err) if vp_err else "viewport_no_pixels"
        if not replicator_fallback:
            return {"ok": False, "jpg": None, "source": "error", "error": vp_fail, "align_diag": align_diag}
        rgba_raw = None
        rep_read_err = None
        try:
            rgba_raw = annotator.get_data()
        except Exception as ex:
            rep_read_err = f"annotator_get_data:{type(ex).__name__}"
        rgba = _normalize_replicator_rgba_for_output(rgba_raw) if rgba_raw is not None else None
        if rgba is not None and getattr(rgba, "size", 0) > 0:
            if rgba.shape[:2] != (H, W):
                try:
                    rgba = np.ascontiguousarray(np.resize(rgba, (H, W, rgba.shape[2] if rgba.ndim == 3 else 4)))
                except Exception as ex:
                    return {
                        "ok": False,
                        "jpg": None,
                        "source": "error",
                        "error": f"resize_replicator:{type(ex).__name__}",
                        "viewport_error": vp_fail,
                        "align_diag": align_diag,
                    }
            jpg, enc_err = _encode_rgba(rgba)
            if jpg is not None:
                return {
                    "ok": True,
                    "jpg": jpg,
                    "source": "replicator",
                    "error": None,
                    "viewport_error": vp_fail,
                    "rep_read_error": rep_read_err,
                    "align_diag": align_diag,
                }
            return {
                "ok": False,
                "jpg": None,
                "source": "error",
                "error": enc_err or "replicator_jpeg_failed",
                "viewport_error": vp_fail,
                "align_diag": align_diag,
            }
        err_tail = rep_read_err or "replicator_empty"
        return {"ok": False, "jpg": None, "source": "error", "error": f"viewport_and_rep:{vp_fail}|{err_tail}", "align_diag": align_diag}
    finally:
        if align_applied and vapi_align is not None:
            try:
                if prev_cam is not None:
                    ok = _snapshot_viewport_api_write_camera_path(vapi_align, prev_cam)
                    align_diag["viewport_restore_ok"] = bool(ok)
                    if not ok:
                        align_diag["viewport_restore_error"] = "write_previous_camera_path_failed"
                else:
                    align_diag["viewport_restore_skipped_no_previous"] = True
                    align_diag["viewport_restore_ok"] = False
                try:
                    sim_app.update()
                except Exception as ex:
                    align_diag["viewport_restore_post_update_error"] = f"{type(ex).__name__}:{ex}"
            except Exception as ex:
                align_diag["viewport_restore_ok"] = False
                align_diag["viewport_restore_error"] = f"{type(ex).__name__}:{ex}"


def _ab_stats_from_buffers(rgba_raw, rgba_u8) -> tuple[dict, dict, dict]:
    raw_merged = _diag_raw_rgba_stats_merged(rgba_raw)
    if rgba_u8 is None or getattr(rgba_u8, "size", 0) <= 0:
        norm_stats = {"error": "no_normalized_buffer"}
        jpeg_in_stats = {"error": "no_normalized_buffer"}
    else:
        norm_stats = _diag_normalized_rgb_stats(rgba_u8)
        jpeg_in_stats = _diag_jpeg_input_rgb_stats(rgba_u8)
    return raw_merged, norm_stats, jpeg_in_stats


def _ab_pack_source_dict(
    *,
    source_name: str,
    render_product_path: str | None,
    camera_prim_path: str,
    ab_capture_seq: int,
    source_seq: int,
    rgba_raw,
    rgba_u8,
    error: str | None,
) -> dict:
    raw_m, norm_s, jpeg_s = _ab_stats_from_buffers(None if error else rgba_raw, None if error else rgba_u8)
    jpg_b = None
    jpg_err = None
    if error is None and rgba_u8 is not None and getattr(rgba_u8, "size", 0) > 0 and _jpeg_encode_fn is not None:
        try:
            jpg_b = _jpeg_encode_fn(rgba_u8)
        except Exception as ex:
            jpg_err = f"{type(ex).__name__}:{ex}"
    frame_ok = error is None and rgba_u8 is not None and getattr(rgba_u8, "size", 0) > 0
    out = {
        "source_name": source_name,
        "render_product_path": render_product_path,
        "camera_prim": camera_prim_path,
        "raw_rgba_stats": raw_m,
        "normalized_rgb_stats": norm_s,
        "jpeg_input_stats": jpeg_s,
        "capture_seq": int(ab_capture_seq),
        "source_seq": int(source_seq),
        "frame_ok": bool(frame_ok),
        "jpeg_encoded_ok": bool(jpg_b),
        "jpeg_byte_len": int(len(jpg_b)) if isinstance(jpg_b, (bytes, bytearray)) else None,
        "error": error if error else jpg_err,
    }
    return out


def _ab_extract_mean_nz(norm_stats: dict) -> tuple[float | None, float | None]:
    if not isinstance(norm_stats, dict) or "mean" not in norm_stats:
        return None, None
    try:
        m = float(norm_stats.get("mean"))
        nz = float(norm_stats.get("nonzero_ratio")) if norm_stats.get("nonzero_ratio") is not None else None
        return m, nz
    except (TypeError, ValueError):
        return None, None


def _ab_build_comparison(rep_block: dict, vp_block: dict) -> dict:
    rm, rnz = _ab_extract_mean_nz(rep_block.get("normalized_rgb_stats") or {})
    vm, vnz = _ab_extract_mean_nz(vp_block.get("normalized_rgb_stats") or {})
    rep_ok = bool(rep_block.get("frame_ok"))
    vp_ok = bool(vp_block.get("frame_ok"))
    same_cam = True
    which = "undetermined"
    if not vp_ok and vp_block.get("error"):
        which = "undetermined"
    elif vp_ok and rep_ok:
        if rm is not None and vm is not None:
            near_rep = rm < 0.04 and (rnz is None or rnz < 0.025)
            good_vp = vm > 0.12 or (rm > 1e-6 and vm > rm * 4.0 and vm - rm > 0.06)
            good_vp2 = vnz is not None and rnz is not None and vnz > 0.08 and vnz > rnz * 2.5
            if near_rep and (good_vp or good_vp2):
                which = "viewport_delegate"
            elif near_rep and not good_vp and not good_vp2:
                if vm is not None and vm < 0.04 and (vnz is None or vnz < 0.025):
                    which = "neither"
                else:
                    which = "undetermined"
            elif vm is not None and rm is not None and vm > rm * 1.8 and vm - rm > 0.03:
                which = "viewport_delegate"
            elif rm is not None and vm is not None and rm > vm * 1.8 and rm - vm > 0.03:
                which = "replicator"
            else:
                which = "undetermined"
    elif vp_ok and not rep_ok:
        which = "viewport_delegate"
    elif rep_ok and not vp_ok:
        which = "replicator"
    else:
        which = "neither"
    return {
        "same_camera_assumed": same_cam,
        "replicator_rgb_mean": rm,
        "viewport_rgb_mean": vm,
        "replicator_nonzero_ratio": rnz,
        "viewport_nonzero_ratio": vnz,
        "which_source_is_healthier": which,
        "replicator_frame_ok": rep_ok,
        "viewport_delegate_frame_ok": vp_ok,
    }


def _render_capture_alpha_mean_from_block(source_block: dict) -> float | None:
    try:
        channels = ((source_block or {}).get("raw_rgba_stats") or {}).get("channels") or {}
        alpha_mean = channels.get("alpha_mean")
        return float(alpha_mean) if alpha_mean is not None else None
    except (TypeError, ValueError):
        return None


def _run_render_capture_health_check(
    rp,
    annotator,
    camera_prim_path: str,
    *,
    perform_rep_step: bool,
    source_label: str,
    rep_raw=None,
    rep_u8=None,
    capture_seq: int | None = None,
) -> dict:
    seq = int(capture_seq) if capture_seq is not None else _next_same_tick_capture_seq()
    t0 = time.monotonic()
    rep_err = None
    rep_raw_local = rep_raw
    rep_u8_local = rep_u8
    if perform_rep_step:
        try:
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
        except Exception as ex:
            return {
                "ok": False,
                "source_label": source_label,
                "camera_prim": camera_prim_path,
                "error": f"rep_step:{type(ex).__name__}:{ex}",
                "capture_seq": seq,
            }
    if rep_raw_local is None:
        try:
            rep_raw_local = annotator.get_data()
        except Exception as ex:
            rep_err = f"{type(ex).__name__}:{ex}"
    if rep_u8_local is None and rep_raw_local is not None:
        rep_u8_local = _normalize_replicator_rgba_for_output(rep_raw_local)
    if rep_u8_local is not None and rep_u8_local.shape[:2] != (H, W):
        try:
            rep_u8_local = np.ascontiguousarray(
                np.resize(rep_u8_local, (H, W, rep_u8_local.shape[2] if rep_u8_local.ndim == 3 else 4))
            )
        except Exception as ex:
            rep_err = (rep_err or "") + f"|resize_rep:{type(ex).__name__}:{ex}"
            rep_u8_local = None
    rep_rp_path = None
    try:
        rep_rp_path = _diag_render_product_fields(rp).get("path")
    except Exception:
        rep_rp_path = None
    replicator_source = _ab_pack_source_dict(
        source_name="replicator",
        render_product_path=str(rep_rp_path) if rep_rp_path is not None else None,
        camera_prim_path=camera_prim_path,
        ab_capture_seq=seq,
        source_seq=0,
        rgba_raw=rep_raw_local,
        rgba_u8=rep_u8_local,
        error=rep_err,
    )
    vp_diag: dict = {}
    vp_np, vp_path, vp_err = _try_read_viewport_delegate_rgba_uint8(vp_diag)
    vp_u8 = _normalize_replicator_rgba_for_output(vp_np) if vp_np is not None else None
    if vp_u8 is not None and vp_u8.shape[:2] != (H, W):
        try:
            vp_u8 = np.ascontiguousarray(
                np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4))
            )
        except Exception as ex:
            vp_err = f"{vp_err or ''}|resize_vp:{type(ex).__name__}:{ex}"
            vp_u8 = None
    viewport_delegate_source = _ab_pack_source_dict(
        source_name="viewport_delegate",
        render_product_path=str(vp_path) if vp_path is not None else None,
        camera_prim_path=camera_prim_path,
        ab_capture_seq=seq,
        source_seq=1,
        rgba_raw=vp_np,
        rgba_u8=vp_u8,
        error=vp_err,
    )
    viewport_delegate_source["capture_method"] = vp_diag.get("capture_method")
    viewport_delegate_source["callback_fired"] = bool(vp_diag.get("callback_fired"))
    viewport_delegate_source["wait_ms"] = vp_diag.get("wait_ms")
    viewport_delegate_source["viewport_delegate_id"] = vp_diag.get("viewport_delegate_id")
    viewport_delegate_source["viewport_window_title"] = vp_diag.get("viewport_window_title")
    comparison = _ab_build_comparison(replicator_source, viewport_delegate_source)
    print(
        "[render-capture-diag] health-check "
        f"source={source_label} capture_seq={seq} healthier={comparison.get('which_source_is_healthier')} "
        f"rep_mean={comparison.get('replicator_rgb_mean')} vp_mean={comparison.get('viewport_rgb_mean')} "
        f"rep_alpha_mean={_render_capture_alpha_mean_from_block(replicator_source)} "
        f"vp_alpha_mean={_render_capture_alpha_mean_from_block(viewport_delegate_source)} "
        f"dt_ms={(time.monotonic() - t0) * 1000.0:.1f}",
        flush=True,
    )
    return {
        "ok": True,
        "source_label": source_label,
        "camera_prim": camera_prim_path,
        "capture_seq": seq,
        "replicator_source": replicator_source,
        "viewport_delegate_source": viewport_delegate_source,
        "comparison": comparison,
        "elapsed_monotonic_s": float(time.monotonic() - t0),
    }


def _should_rebind_render_capture_from_health_check(check: dict) -> tuple[bool, str]:
    if not isinstance(check, dict) or not check.get("ok"):
        return False, "health_check_unavailable"
    rep_block = check.get("replicator_source") or {}
    vp_block = check.get("viewport_delegate_source") or {}
    comp = check.get("comparison") or {}
    rep_mean, rep_nz = _ab_extract_mean_nz(rep_block.get("normalized_rgb_stats") or {})
    vp_mean, vp_nz = _ab_extract_mean_nz(vp_block.get("normalized_rgb_stats") or {})
    rep_alpha_mean = _render_capture_alpha_mean_from_block(rep_block)
    rep_full_black = (
        rep_mean is not None
        and rep_mean <= 1e-6
        and (rep_nz is None or rep_nz <= 1e-6)
    )
    vp_healthy = bool(vp_block.get("frame_ok")) and (
        (vp_mean is not None and vp_mean > 0.12)
        or (vp_nz is not None and vp_nz > 0.08)
    )
    viewport_wins = comp.get("which_source_is_healthier") == "viewport_delegate"
    if rep_full_black and vp_healthy and viewport_wins and rep_alpha_mean is not None and rep_alpha_mean > 1.0:
        return True, "replicator_full_black_alpha_alive_viewport_healthy"
    if rep_full_black and (rep_alpha_mean is None or rep_alpha_mean <= 1.0):
        return False, "replicator_full_black_alpha_not_alive"
    if rep_full_black and not vp_healthy:
        return False, "viewport_not_healthy_enough_for_rebind"
    if not rep_full_black:
        return False, "replicator_not_fully_black_after_warmup"
    return False, "comparison_not_decisive"


def _detach_render_capture_annotator(annotator, rp) -> dict:
    out = {"attempted": False, "ok": False, "actions": [], "errors": []}
    if annotator is None or rp is None:
        out["errors"].append("annotator_or_rp_missing")
        return out
    payloads = []
    payloads.append(([rp], "annotator.detach([rp])"))
    try:
        rp_path = _diag_render_product_fields(rp).get("path")
        if rp_path:
            payloads.append(([str(rp_path)], "annotator.detach([rp_path])"))
    except Exception:
        pass
    detach_fn = getattr(annotator, "detach", None)
    if callable(detach_fn):
        for payload, label in payloads:
            out["attempted"] = True
            try:
                detach_fn(payload)
                out["ok"] = True
                out["actions"].append(label)
                return out
            except Exception as exc:
                out["errors"].append(f"{label}:{type(exc).__name__}:{exc}")
    registry = getattr(rep, "AnnotatorRegistry", None)
    registry_detach = getattr(registry, "detach", None)
    if callable(registry_detach):
        for payload, label in payloads:
            out["attempted"] = True
            try:
                registry_detach(annotator, payload)
                out["ok"] = True
                out["actions"].append(f"registry.{label}")
                return out
            except Exception as exc:
                out["errors"].append(f"registry.{label}:{type(exc).__name__}:{exc}")
    if not out["attempted"]:
        out["errors"].append("no_detach_api_available")
    return out


def _destroy_render_capture_render_product(rp) -> dict:
    out = {"attempted": False, "ok": False, "actions": [], "errors": []}
    if rp is None:
        out["errors"].append("rp_missing")
        return out
    destroy_fn = getattr(rp, "destroy", None)
    if callable(destroy_fn):
        out["attempted"] = True
        try:
            destroy_fn()
            out["ok"] = True
            out["actions"].append("rp.destroy()")
            return out
        except Exception as exc:
            out["errors"].append(f"rp.destroy():{type(exc).__name__}:{exc}")
    else:
        out["errors"].append("destroy_api_unavailable")
    return out


def _rebind_render_capture(world, rp, annotator, *, reason: str, camera_prim_path: str, width: int, height: int):
    global _render_capture_rebinding, _last_render_capture_rebind_attempt_mono
    global _last_render_capture_rebind_finish_mono, _post_recover_first_capture_pending
    now = time.monotonic()
    with _render_capture_rebind_lock:
        if _render_capture_rebinding:
            return False, rp, annotator, "rebind_in_progress", None
        if _last_render_capture_rebind_attempt_mono > 0.0:
            remaining = _RENDER_CAPTURE_REBIND_COOLDOWN_S - (now - _last_render_capture_rebind_attempt_mono)
            if remaining > 0.0:
                return False, rp, annotator, f"rebind_cooldown_{remaining:.1f}s", None
        _render_capture_rebinding = True
        _last_render_capture_rebind_attempt_mono = now
    old_rp_fields = _diag_render_product_fields(rp)
    with _render_capture_meta_lock:
        old_bind_seq = _RENDER_CAPTURE_LATEST.get("bind_seq")
    detach_info = _detach_render_capture_annotator(annotator, rp)
    destroy_info = {"attempted": False, "ok": False, "actions": [], "errors": []}
    new_rp = None
    new_annotator = None
    post_check = None
    reattach_info = None
    try:
        new_rp, new_annotator = _bind_camera(camera_prim_path, width, height, force_new=True)
        destroy_info = _destroy_render_capture_render_product(rp)
        try:
            sim_app.update()
            sim_app.update()
        except Exception:
            pass
        for _ in range(4):
            world.step(render=True)
        rep.orchestrator.step(rt_subframes=2, delta_time=0.0, pause_timeline=False)
        sim_app.update()
        sim_app.update()
        post_check = _run_render_capture_health_check(
            new_rp,
            new_annotator,
            camera_prim_path,
            perform_rep_step=True,
            source_label="post_rebind",
        )
        post_comp = post_check.get("comparison") or {}
        new_rp_fields = _diag_render_product_fields(new_rp)
        _post_recover_first_capture_pending = True
        _stream_diag_update(
            camera_bound_ok=True,
            render_product=str(new_rp),
            render_capture_last_rebind_reason=reason,
            render_capture_last_rebind_iso=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        )
        print(
            "[render-capture-diag] REBIND "
            f"reason={reason} camera={camera_prim_path} old_bind_seq={old_bind_seq} "
            f"old_rp={old_rp_fields.get('path') or old_rp_fields.get('repr')} "
            f"new_rp={new_rp_fields.get('path') or new_rp_fields.get('repr')} "
            f"detach_ok={detach_info.get('ok')} destroy_ok={destroy_info.get('ok')} "
            f"post_rep_mean={post_comp.get('replicator_rgb_mean')} "
            f"post_vp_mean={post_comp.get('viewport_rgb_mean')} "
            f"healthier={post_comp.get('which_source_is_healthier')}",
            flush=True,
        )
        return True, new_rp, new_annotator, None, post_check
    except Exception as exc:
        err = f"{type(exc).__name__}:{exc}"
        if new_annotator is not None and new_rp is not None:
            try:
                _detach_render_capture_annotator(new_annotator, new_rp)
            except Exception:
                pass
            try:
                _destroy_render_capture_render_product(new_rp)
            except Exception:
                pass
        try:
            attach_fn = getattr(annotator, "attach", None)
            if callable(attach_fn) and rp is not None:
                attach_fn([rp])
                reattach_info = "annotator.attach([old_rp])"
        except Exception as reattach_exc:
            reattach_info = f"reattach_failed:{type(reattach_exc).__name__}:{reattach_exc}"
        print(
            "[render-capture-diag] REBIND_FAILED "
            f"reason={reason} camera={camera_prim_path} err={err} "
            f"old_rp={old_rp_fields.get('path') or old_rp_fields.get('repr')} "
            f"detach_ok={detach_info.get('ok')} destroy_ok={destroy_info.get('ok')} "
            f"reattach={reattach_info}",
            flush=True,
        )
        return False, rp, annotator, err, post_check
    finally:
        _last_render_capture_rebind_finish_mono = time.monotonic()
        with _render_capture_rebind_lock:
            _render_capture_rebinding = False


def _run_render_capture_ab_probe_pipeline(rp, annotator, camera_prim_path: str) -> dict:
    """主线程：同一轮 rep.step 后先读 Replicator annotator，再读 viewport delegate（双源对照）。"""
    ab_capture_seq = _next_same_tick_capture_seq()
    t0 = time.monotonic()
    rep_err = None
    rep_raw = None
    try:
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    except Exception as ex:
        return {
            "ok": False,
            "camera_prim": camera_prim_path,
            "renderer_mode": renderer_mode,
            "error": f"rep_step:{type(ex).__name__}:{ex}",
            "ab_capture_seq": int(ab_capture_seq),
        }
    try:
        rep_raw = annotator.get_data()
    except Exception as ex:
        rep_err = f"{type(ex).__name__}:{ex}"
    rep_u8 = _normalize_replicator_rgba_for_output(rep_raw) if rep_raw is not None else None
    if rep_u8 is not None and rep_u8.shape[:2] != (H, W):
        try:
            rep_u8 = np.ascontiguousarray(np.resize(rep_u8, (H, W, rep_u8.shape[2] if rep_u8.ndim == 3 else 4)))
        except Exception as ex:
            rep_err = (rep_err or "") + f"|resize_rep:{type(ex).__name__}:{ex}"
            rep_u8 = None
    rep_rp_path = None
    try:
        rep_rp_path = _diag_render_product_fields(rp).get("path")
    except Exception:
        rep_rp_path = None
    replicator_source = _ab_pack_source_dict(
        source_name="replicator",
        render_product_path=str(rep_rp_path) if rep_rp_path is not None else None,
        camera_prim_path=camera_prim_path,
        ab_capture_seq=ab_capture_seq,
        source_seq=0,
        rgba_raw=rep_raw,
        rgba_u8=rep_u8,
        error=rep_err,
    )

    _vp_cap_meta: dict = {}
    vp_np = None
    vp_path = None
    vp_err: str | None = None
    _ab_vp_t0 = time.monotonic()
    _ab_vp_updates = 0
    _ab_vp_attempts_used = 0
    for _ab_attempt in range(3):
        _ab_vp_attempts_used = _ab_attempt + 1
        _pump_lo = 2 if _ab_attempt > 0 else 1
        for _ in range(_pump_lo):
            try:
                sim_app.update()
                _ab_vp_updates += 1
            except Exception as _ab_up_exc:
                print(
                    "[render-capture-diag] ab-probe viewport_delegate sim_app.update "
                    f"attempt={_ab_attempt} err={type(_ab_up_exc).__name__}:{_ab_up_exc}",
                    flush=True,
                )
        _attempt_diag: dict = {}
        vp_np, vp_path, vp_err = _try_read_viewport_delegate_rgba_uint8(_attempt_diag)
        _vp_cap_meta = dict(_attempt_diag)
        if vp_np is not None and vp_err is None:
            break
        if _ab_attempt < 2:
            print(
                "[render-capture-diag] ab-probe viewport_delegate_capture retry "
                f"attempt={_ab_attempt} next_attempt={_ab_attempt + 1} err={vp_err!r} "
                f"callback_fired={_attempt_diag.get('callback_fired')} "
                f"method={_attempt_diag.get('capture_method')}",
                flush=True,
            )
    _vp_cap_meta["retry_count"] = max(0, _ab_vp_attempts_used - 1)
    _vp_cap_meta["ab_probe_sim_app_update_count"] = int(_ab_vp_updates)
    _vp_cap_meta["ab_probe_capture_attempts"] = int(_ab_vp_attempts_used)
    _vp_cap_meta["ab_probe_viewport_phase_wait_ms"] = round((time.monotonic() - _ab_vp_t0) * 1000.0, 2)
    print(
        "[render-capture-diag] ab-probe viewport_delegate_capture done "
        f"attempts={_ab_vp_attempts_used} sim_app_updates={_ab_vp_updates} "
        f"capture_method={_vp_cap_meta.get('capture_method')!r} callback_fired={_vp_cap_meta.get('callback_fired')} "
        f"per_capture_wait_ms={_vp_cap_meta.get('wait_ms')} phase_wait_ms={_vp_cap_meta.get('ab_probe_viewport_phase_wait_ms')} "
        f"err={vp_err!r}",
        flush=True,
    )

    vp_u8 = _normalize_replicator_rgba_for_output(vp_np) if vp_np is not None else None
    if vp_u8 is not None and vp_u8.shape[:2] != (H, W):
        try:
            vp_u8 = np.ascontiguousarray(np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4)))
        except Exception as ex:
            vp_err = f"{vp_err or ''}|resize_vp:{type(ex).__name__}:{ex}"
            vp_u8 = None
    viewport_delegate_source = _ab_pack_source_dict(
        source_name="viewport_delegate",
        render_product_path=str(vp_path) if vp_path is not None else None,
        camera_prim_path=camera_prim_path,
        ab_capture_seq=ab_capture_seq,
        source_seq=1,
        rgba_raw=vp_np,
        rgba_u8=vp_u8,
        error=vp_err,
    )
    viewport_delegate_source["capture_method"] = _vp_cap_meta.get("capture_method")
    viewport_delegate_source["callback_fired"] = bool(_vp_cap_meta.get("callback_fired"))
    viewport_delegate_source["wait_ms"] = _vp_cap_meta.get("wait_ms")
    viewport_delegate_source["retry_count"] = _vp_cap_meta.get("retry_count")
    viewport_delegate_source["viewport_delegate_id"] = _vp_cap_meta.get("viewport_delegate_id")
    viewport_delegate_source["viewport_window_title"] = _vp_cap_meta.get("viewport_window_title")
    viewport_delegate_source["ab_probe_sim_app_update_count"] = _vp_cap_meta.get("ab_probe_sim_app_update_count")
    viewport_delegate_source["ab_probe_capture_attempts"] = _vp_cap_meta.get("ab_probe_capture_attempts")
    viewport_delegate_source["ab_probe_viewport_phase_wait_ms"] = _vp_cap_meta.get("ab_probe_viewport_phase_wait_ms")
    if viewport_delegate_source.get("render_product_path") is None and _vp_cap_meta.get("render_product_path"):
        viewport_delegate_source["render_product_path"] = _vp_cap_meta.get("render_product_path")

    comparison = _ab_build_comparison(replicator_source, viewport_delegate_source)
    print(
        "[render-capture-diag] ab-probe "
        f"ab_capture_seq={ab_capture_seq} healthier={comparison.get('which_source_is_healthier')} "
        f"rep_mean={comparison.get('replicator_rgb_mean')} vp_mean={comparison.get('viewport_rgb_mean')} "
        f"rep_ok={comparison.get('replicator_frame_ok')} vp_ok={comparison.get('viewport_delegate_frame_ok')} "
        f"dt_ms={(time.monotonic() - t0) * 1000.0:.1f}",
        flush=True,
    )
    out_ab: dict = {
        "ok": True,
        "camera_prim": camera_prim_path,
        "renderer_mode": renderer_mode,
        "ab_capture_seq": int(ab_capture_seq),
        "replicator_source": replicator_source,
        "viewport_delegate_source": viewport_delegate_source,
        "comparison": comparison,
        "elapsed_monotonic_s": float(time.monotonic() - t0),
    }
    if _cfg_capture_prefer_viewport_delegate_for_snapshot():
        out_ab["capture_source_prefer_viewport_delegate_for_snapshot"] = True
        out_ab["http_service_paths_note"] = (
            "GET /snapshot.jpg 与 POST /diagnostics/render-capture-probe 在 cfg 开启时使用 viewport_delegate；"
            "本 ab-probe 仍并行对照 replicator 与 viewport_delegate，不改变 RTSP/MJPEG 主链路。"
        )
    return out_ab


def _next_same_tick_capture_seq() -> int:
    global _same_tick_capture_seq
    with _SAME_TICK_SEQ_LOCK:
        _same_tick_capture_seq += 1
        return int(_same_tick_capture_seq)


def _diag_uint8_tensor_stats_u8(arr) -> dict:
    try:
        if arr is None or getattr(arr, "size", 0) <= 0:
            return {"error": "empty", "shape": None, "dtype": None}
        x = np.asarray(arr)
        if x.size == 0:
            return {"error": "empty", "shape": tuple(int(v) for v in x.shape), "dtype": str(x.dtype)}
        flat = x.ravel()
        return {
            "shape": tuple(int(v) for v in x.shape),
            "dtype": str(x.dtype),
            "min": int(np.min(flat)),
            "max": int(np.max(flat)),
            "mean": float(np.mean(flat)),
            "nonzero_ratio": float(np.mean(flat != 0)),
        }
    except Exception as exc:
        return {"error": f"{type(exc).__name__}:{exc}"}


def _diag_raw_rgba_stats_merged(rgba_raw) -> dict:
    out: dict = {"shape": None, "dtype": None}
    try:
        if rgba_raw is None:
            return {**out, "error": "raw_none"}
        arr = np.asarray(rgba_raw)
        out["shape"] = tuple(int(x) for x in arr.shape)
        out["dtype"] = str(arr.dtype)
        out["channels"] = _render_capture_near_black_channel_stats(rgba_raw)
        return out
    except Exception as exc:
        out["error"] = f"{type(exc).__name__}:{exc}"
        return out


def _diag_normalized_rgb_stats(rgba_uint8_hwc) -> dict:
    if rgba_uint8_hwc is None or getattr(rgba_uint8_hwc, "size", 0) <= 0:
        return {"error": "no_rgba"}
    rgb = np.asarray(rgba_uint8_hwc)[..., :3]
    return _diag_uint8_tensor_stats_u8(rgb)


def _diag_jpeg_input_rgb_stats(rgba_uint8_hwc) -> dict:
    return _diag_normalized_rgb_stats(rgba_uint8_hwc)


def _render_capture_optional_subject_bbox_px(image_wh: tuple[int, int] | None = None) -> tuple[int, int, int, int] | None:
    try:
        with _stream_diag_lock:
            tgt = dict(_STREAM_DIAG.get("target_projection_metrics") or {})
    except Exception:
        return None
    if not isinstance(tgt, dict) or not tgt:
        return None
    iw, ih = image_wh if image_wh is not None else (int(W), int(H))
    best_box = None
    best_score = -1
    for k, v in tgt.items():
        if not isinstance(v, (list, tuple)) or len(v) != 4:
            continue
        try:
            vals = [float(x) for x in v]
        except (TypeError, ValueError):
            continue
        name = str(k)
        lname = name.lower()
        score = 0
        for token in ("bbox", "bound", "screen", "pixel", "px"):
            if token in lname:
                score += 2
        for token in ("包围框", "围框", "屏幕", "像素"):
            if token in name:
                score += 2
        if score < best_score:
            continue
        x1, y1, x2, y2 = vals
        x1i = max(0, min(iw, int(math.floor(x1))))
        y1i = max(0, min(ih, int(math.floor(y1))))
        x2i = max(0, min(iw, int(math.ceil(x2))))
        y2i = max(0, min(ih, int(math.ceil(y2))))
        if x2i <= x1i or y2i <= y1i:
            continue
        best_box = (x1i, y1i, x2i, y2i)
        best_score = score
    return best_box


def _render_capture_rgb_roi_stats(rgb, box: tuple[int, int, int, int] | None) -> dict:
    if box is None:
        return {"ok": False, "error": "no_roi_box"}
    try:
        x1, y1, x2, y2 = [int(v) for v in box]
    except Exception as exc:
        return {"ok": False, "error": f"invalid_roi_box:{type(exc).__name__}:{exc}"}
    if x2 <= x1 or y2 <= y1:
        return {"ok": False, "error": "roi_box_empty", "box": [x1, y1, x2, y2]}
    region = np.asarray(rgb)[y1:y2, x1:x2, :3]
    if region.size <= 0:
        return {"ok": False, "error": "roi_region_empty", "box": [x1, y1, x2, y2]}
    dark20 = np.all(region < 20, axis=2)
    dark30 = np.all(region < 30, axis=2)
    return {
        "ok": True,
        "box": [x1, y1, x2, y2],
        "mean": float(np.mean(region)),
        "max": int(np.max(region)),
        "dark_ratio_rgb_lt_20": float(np.mean(dark20)),
        "dark_ratio_rgb_lt_30": float(np.mean(dark30)),
    }


def _render_capture_rgb_frame_health(rgba_uint8_hwc) -> dict:
    if rgba_uint8_hwc is None or getattr(rgba_uint8_hwc, "size", 0) <= 0:
        return {
            "frame_ok": False,
            "healthy": False,
            "should_try_viewport_fallback": False,
            "black_reason": "no_rgba",
        }
    arr = np.asarray(rgba_uint8_hwc)
    if arr.ndim != 3 or arr.shape[-1] < 3:
        return {
            "frame_ok": False,
            "healthy": False,
            "should_try_viewport_fallback": False,
            "black_reason": f"bad_shape:{tuple(int(x) for x in arr.shape)}",
        }
    rgb = arr[..., :3]
    ih, iw = int(rgb.shape[0]), int(rgb.shape[1])
    if ih <= 0 or iw <= 0:
        return {
            "frame_ok": False,
            "healthy": False,
            "should_try_viewport_fallback": False,
            "black_reason": "empty_rgb",
        }
    full_mean = float(np.mean(rgb))
    full_max = int(np.max(rgb))
    full_min = int(np.min(rgb))
    full_nonzero_ratio = float(np.count_nonzero(rgb)) / float(rgb.size)
    central_box = (
        int(iw * 0.20),
        int(ih * 0.20),
        int(iw * 0.80),
        int(ih * 0.85),
    )
    lower_box = (
        int(iw * 0.10),
        int(ih * 0.35),
        int(iw * 0.90),
        int(ih * 0.95),
    )
    subject_box = _render_capture_optional_subject_bbox_px((iw, ih))
    central = _render_capture_rgb_roi_stats(rgb, central_box)
    lower = _render_capture_rgb_roi_stats(rgb, lower_box)
    subject = _render_capture_rgb_roi_stats(rgb, subject_box) if subject_box is not None else {
        "ok": False,
        "error": "no_subject_bbox",
    }
    near_black = full_max <= 2 and full_min == 0 and full_mean < 0.35
    roi_black = bool(
        central.get("ok")
        and lower.get("ok")
        and float(central.get("dark_ratio_rgb_lt_20") or 0.0) >= 0.995
        and float(lower.get("dark_ratio_rgb_lt_20") or 0.0) >= 0.995
    )
    subject_black = bool(
        subject.get("ok")
        and float(subject.get("dark_ratio_rgb_lt_20") or 0.0) >= 0.995
    )
    black_reasons: list[str] = []
    if near_black:
        black_reasons.append("near_black_full_frame")
    if roi_black:
        black_reasons.append("central_lower_roi_black")
    if subject_black:
        black_reasons.append("subject_roi_black")
    healthy = bool(
        full_max >= 30
        and full_mean >= 8.0
        and full_nonzero_ratio >= 0.05
        and (
            float(central.get("dark_ratio_rgb_lt_20") or 1.0) < 0.995
            or float(lower.get("dark_ratio_rgb_lt_20") or 1.0) < 0.995
            or (subject.get("ok") and float(subject.get("dark_ratio_rgb_lt_20") or 1.0) < 0.995)
        )
    )
    return {
        "frame_ok": True,
        "healthy": healthy,
        "should_try_viewport_fallback": bool(black_reasons),
        "black_reason": "|".join(black_reasons) if black_reasons else None,
        "full_mean": full_mean,
        "full_min": full_min,
        "full_max": full_max,
        "full_nonzero_ratio": full_nonzero_ratio,
        "central_roi": central,
        "lower_roi": lower,
        "subject_roi": subject,
    }


def _infer_black_origin_stage(raw_merged: dict, norm_stats: dict, jpeg_stats: dict) -> str | None:
    ch = raw_merged.get("channels") if isinstance(raw_merged.get("channels"), dict) else {}
    try:
        rgb_max = float(ch.get("rgb_max", 1.0))
        rgb_mean = float(ch.get("rgb_mean", 1.0))
        alpha_mean = float(ch.get("alpha_mean") or 0.0)
    except (TypeError, ValueError):
        return None
    if rgb_max <= 1e-6 and rgb_mean <= 1e-6 and alpha_mean > 0.1:
        return "raw_before_normalize_and_jpeg"
    if isinstance(norm_stats, dict) and norm_stats.get("max") == 0 and norm_stats.get("mean", 1.0) < 1e-6:
        if raw_merged.get("channels") and rgb_max > 1e-3:
            return "normalized_uint8"
    if (
        isinstance(jpeg_stats, dict)
        and jpeg_stats.get("max") == 0
        and isinstance(norm_stats, dict)
        and norm_stats.get("max", 1) > 2
    ):
        return "jpeg_encoder_input_mismatch_suspected"
    return None


def _diag_camera_world_transform(stage, cam_path: str) -> dict:
    try:
        from pxr import UsdGeom

        if stage is None:
            return {"ok": False, "error": "no_stage"}
        prim = stage.GetPrimAtPath(cam_path)
        if not prim or not prim.IsValid():
            return {"ok": False, "error": "invalid_camera_prim", "path": cam_path}
        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        m = xcache.GetLocalToWorldTransform(prim)
        mat = [[float(m[r][c]) for c in range(4)] for r in range(4)]
        t = m.ExtractTranslation()
        return {
            "ok": True,
            "camera_prim": cam_path,
            "matrix_row_major_4x4": mat,
            "translation_xyz": [float(t[0]), float(t[1]), float(t[2])],
        }
    except Exception as exc:
        return {"ok": False, "error": f"{type(exc).__name__}:{exc}"}


def _diag_render_product_fields(rp) -> dict:
    out: dict = {"repr": str(rp), "pyid": id(rp)}
    if rp is None:
        return out
    for attr in ("path", "prim_path", "product_path", "hydra_render_product_path", "id"):
        if hasattr(rp, attr):
            try:
                out[attr] = _json_safe_value(getattr(rp, attr))
            except Exception as exc:
                out[attr] = f"<err:{exc}>"
    return out


def _same_tick_store_latest(diag: dict) -> None:
    global _same_tick_latest_diag
    with _same_tick_diag_lock:
        _same_tick_latest_diag = dict(diag)


def _compose_same_tick_binding_and_runtime(
    *,
    rp,
    annotator,
    camera_prim_path: str,
    whether_rebind_recently: bool,
    is_post_recover_first_frame: bool,
    recover_attempted_this_tick: bool,
) -> dict:
    stage = omni.usd.get_context().get_stage()
    with _render_capture_meta_lock:
        bind_meta = dict(_RENDER_CAPTURE_LATEST)
    try:
        renderer_obs = _compose_renderer_hydra_observation()
    except Exception as exc:
        renderer_obs = {"error": f"{type(exc).__name__}:{exc}"}
    with _ptz_lock:
        ptz = {"pan": float(_ptz_state["pan"]), "tilt": float(_ptz_state["tilt"]), "zoom": float(_ptz_state["zoom"])}
    ori = dict(_orientation_state)
    look_prim = None
    try:
        look_prim = _effective_lookat_target_prim_path(stage) if stage is not None else None
    except Exception:
        look_prim = None
    tgt = ori.get("target_xyz")
    if tgt is not None and not isinstance(tgt, (list, tuple)):
        tgt = _json_safe_value(tgt)
    return {
        "camera_prim": camera_prim_path,
        "render_product": _diag_render_product_fields(rp),
        "annotator_repr": str(annotator),
        "annotator_pyid": id(annotator) if annotator is not None else None,
        "whether_rebind_recently": bool(whether_rebind_recently),
        "bind_seq": bind_meta.get("bind_seq"),
        "bind_meta": bind_meta,
        "renderer_mode": renderer_mode,
        "renderer_config_vs_observed": renderer_obs,
        "camera_world": _diag_camera_world_transform(stage, camera_prim_path),
        "pan_tilt_zoom": ptz,
        "orientation_state": ori,
        "target_xyz": ori.get("target_xyz"),
        "lookat_target_prim": look_prim,
        "is_post_recover_first_capture_frame": bool(is_post_recover_first_frame),
        "recover_render_capture_attempted_this_tick": bool(recover_attempted_this_tick),
    }


def _build_same_tick_pipeline_diag(
    *,
    capture_seq: int,
    monotonic_start: float,
    rgba_raw,
    rgba_u8,
    jpg_bytes: bytes | None,
    snapshot_cache_written: bool,
    snapshot_frame_id: int | None,
    snapshot_cache_capture_seq: int | None,
    rp,
    annotator,
    camera_prim_path: str,
    whether_rebind_recently: bool,
    is_post_recover_first_frame: bool,
    recover_attempted_this_tick: bool,
    probe: bool,
) -> dict:
    raw_merged = _diag_raw_rgba_stats_merged(rgba_raw)
    norm_stats = _diag_normalized_rgb_stats(rgba_u8) if rgba_u8 is not None else {"error": "no_normalized_buffer"}
    jpeg_in_stats = _diag_jpeg_input_rgb_stats(rgba_u8) if rgba_u8 is not None else {"error": "no_normalized_buffer"}
    black_origin = _infer_black_origin_stage(raw_merged, norm_stats, jpeg_in_stats)
    if black_origin == "raw_before_normalize_and_jpeg":
        print(
            "[render-capture-diag] BLACK_ORIGIN_STAGE=raw_before_normalize_and_jpeg "
            f"capture_seq={capture_seq} raw_channels={raw_merged.get('channels')}",
            flush=True,
        )
    prev_seq = None
    prev_fid = None
    if not snapshot_cache_written:
        with _mjpeg_lock:
            prev_seq = _snapshot_cache.get("capture_seq")
            prev_fid = _snapshot_cache.get("frame_id")
    snap_stats: dict = {
        "snapshot_cache_updated_this_seq": bool(snapshot_cache_written),
        "capture_seq_expected": int(capture_seq),
        "snapshot_cache_capture_seq_after_write": snapshot_cache_capture_seq,
        "service_snapshot_frame_id": snapshot_frame_id,
        "jpeg_byte_len": int(len(jpg_bytes)) if isinstance(jpg_bytes, (bytes, bytearray)) else None,
    }
    if snapshot_cache_written and isinstance(jpg_bytes, (bytes, bytearray)) and len(jpg_bytes) > 0:
        snap_stats["jpeg_input_rgb_stats_at_write"] = dict(jpeg_in_stats) if isinstance(jpeg_in_stats, dict) else jpeg_in_stats
        snap_stats["aligned_with_same_seq"] = snapshot_cache_capture_seq == capture_seq
    else:
        snap_stats["previous_service_cache_capture_seq"] = prev_seq
        snap_stats["previous_service_cache_frame_id"] = prev_fid
        snap_stats["note"] = "snapshot_interval_gate_or_no_jpeg_this_tick"
    lag_hint = None
    if isinstance(raw_merged.get("channels"), dict):
        try:
            rmax = float(raw_merged["channels"].get("rgb_max", 1.0))
        except (TypeError, ValueError):
            rmax = 1.0
        if rmax <= 1e-6 and snapshot_cache_written and isinstance(jpeg_in_stats, dict):
            jm = jpeg_in_stats.get("max")
            if isinstance(jm, int) and jm > 2:
                lag_hint = "raw_all_zero_but_jpeg_input_nonzero_implies_inconsistent_buffers_same_tick"
            elif not snapshot_cache_written and prev_seq is not None and int(prev_seq) != int(capture_seq):
                lag_hint = "raw_zero_snapshot_not_updated_service_cache_may_lag"
    diag = {
        "ok": True,
        "capture_seq": int(capture_seq),
        "probe": bool(probe),
        "timestamps": {
            "wall_time_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            "monotonic_t_start": float(monotonic_start),
            "monotonic_t_end": float(time.monotonic()),
        },
        "raw_rgba_stats": raw_merged,
        "normalized_rgb_stats": norm_stats,
        "jpeg_input_stats": jpeg_in_stats,
        "snapshot_cache_stats": snap_stats,
        "binding_and_runtime": _compose_same_tick_binding_and_runtime(
            rp=rp,
            annotator=annotator,
            camera_prim_path=camera_prim_path,
            whether_rebind_recently=whether_rebind_recently,
            is_post_recover_first_frame=is_post_recover_first_frame,
            recover_attempted_this_tick=recover_attempted_this_tick,
        ),
        "black_origin_stage": black_origin,
        "cache_lag_analysis": lag_hint,
    }
    if probe:
        diag["probe_side_effects"] = (
            "one rep.orchestrator.step + get_data + normalize + in_memory_jpeg_only; "
            "service snapshot/mjpeg/rtsp caches not written"
        )
    return diag


def _run_render_capture_probe_pipeline(rp, annotator, camera_prim_path: str) -> dict:
    """主线程内执行：额外一步 replicator + 抓帧，不写业务缓存。"""
    seq = _next_same_tick_capture_seq()
    t0 = time.monotonic()
    rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
    try:
        rgba_raw = annotator.get_data()
    except Exception as exc:
        return {"ok": False, "capture_seq": seq, "error": f"{type(exc).__name__}:{exc}"}
    rgba = _normalize_replicator_rgba_for_output(rgba_raw)
    if rgba is None or getattr(rgba, "size", 0) <= 0:
        d = _build_same_tick_pipeline_diag(
            capture_seq=seq,
            monotonic_start=t0,
            rgba_raw=rgba_raw,
            rgba_u8=None,
            jpg_bytes=None,
            snapshot_cache_written=False,
            snapshot_frame_id=None,
            snapshot_cache_capture_seq=None,
            rp=rp,
            annotator=annotator,
            camera_prim_path=camera_prim_path,
            whether_rebind_recently=(time.monotonic() - _LAST_RENDER_CAPTURE_BIND_MONO) < 2.0,
            is_post_recover_first_frame=False,
            recover_attempted_this_tick=False,
            probe=True,
        )
        return {"ok": True, "same_tick_pipeline": d}
    if rgba.shape[:2] != (H, W):
        rgba = np.ascontiguousarray(np.resize(rgba, (H, W, rgba.shape[2] if rgba.ndim == 3 else 4)))
    rgba_probe_raw = rgba_raw
    rgba_probe_u8 = rgba
    probe_pixels_source = "replicator"
    if _cfg_capture_prefer_viewport_delegate_for_snapshot():
        vp_np, _vp_path, vp_err = _try_read_viewport_delegate_rgba_uint8()
        vp_u8 = _normalize_replicator_rgba_for_output(vp_np) if vp_np is not None else None
        if vp_u8 is not None and vp_u8.shape[:2] != (H, W):
            vp_u8 = np.ascontiguousarray(np.resize(vp_u8, (H, W, vp_u8.shape[2] if vp_u8.ndim == 3 else 4)))
        if vp_err:
            probe_pixels_source = "replicator_fallback_viewport_unavailable"
        elif vp_u8 is not None and getattr(vp_u8, "size", 0) > 0:
            rgba_probe_u8 = vp_u8
            rgba_probe_raw = vp_np if vp_np is not None else vp_u8
            probe_pixels_source = "viewport_delegate"
        else:
            probe_pixels_source = "replicator_fallback_viewport_empty"
        print(
            "[render-capture-diag] probe_pixels_source="
            f"{probe_pixels_source} prefer_viewport_delegate_cfg=true",
            flush=True,
        )
    jpg = None
    if _jpeg_encode_fn is not None:
        try:
            jpg = _jpeg_encode_fn(rgba_probe_u8)
        except Exception as exc:
            jpg = None
            return {
                "ok": False,
                "capture_seq": seq,
                "error": f"jpeg_encode:{type(exc).__name__}:{exc}",
                "partial": _build_same_tick_pipeline_diag(
                    capture_seq=seq,
                    monotonic_start=t0,
                    rgba_raw=rgba_probe_raw,
                    rgba_u8=rgba_probe_u8,
                    jpg_bytes=None,
                    snapshot_cache_written=False,
                    snapshot_frame_id=None,
                    snapshot_cache_capture_seq=None,
                    rp=rp,
                    annotator=annotator,
                    camera_prim_path=camera_prim_path,
                    whether_rebind_recently=(time.monotonic() - _LAST_RENDER_CAPTURE_BIND_MONO) < 2.0,
                    is_post_recover_first_frame=False,
                    recover_attempted_this_tick=False,
                    probe=True,
                ),
            }
    d = _build_same_tick_pipeline_diag(
        capture_seq=seq,
        monotonic_start=t0,
        rgba_raw=rgba_probe_raw,
        rgba_u8=rgba_probe_u8,
        jpg_bytes=jpg if isinstance(jpg, (bytes, bytearray)) else None,
        snapshot_cache_written=False,
        snapshot_frame_id=None,
        snapshot_cache_capture_seq=None,
        rp=rp,
        annotator=annotator,
        camera_prim_path=camera_prim_path,
        whether_rebind_recently=(time.monotonic() - _LAST_RENDER_CAPTURE_BIND_MONO) < 2.0,
        is_post_recover_first_frame=False,
        recover_attempted_this_tick=False,
        probe=True,
    )
    out: dict = {
        "ok": True,
        "same_tick_pipeline": d,
        "probe_pixels_source": probe_pixels_source,
        "current_probe_pixel_source": probe_pixels_source,
    }
    if _cfg_capture_prefer_viewport_delegate_for_snapshot():
        out["capture_source_prefer_viewport_delegate_for_snapshot"] = True
    return out


def _ctrl_heavy_tasks_probe() -> dict:
    """非阻塞探测重任务队列（供 /api/health、/status.ctrl_plane）；禁止长时间持锁。"""
    out: dict = {
        "snapshot_jpg_live_vp_queued": False,
        "diag_live_once_queued": False,
        "snapshot_http_viewport_queued": False,
        "scene_randomize_lock_busy": False,
    }
    if _snapshot_jpg_live_vp_lock.acquire(blocking=False):
        try:
            out["snapshot_jpg_live_vp_queued"] = _snapshot_jpg_live_vp_holder is not None
        finally:
            _snapshot_jpg_live_vp_lock.release()
    else:
        out["snapshot_jpg_live_vp_queued"] = True
    if _diag_live_once_lock.acquire(blocking=False):
        try:
            out["diag_live_once_queued"] = _diag_live_once_holder is not None
        finally:
            _diag_live_once_lock.release()
    else:
        out["diag_live_once_queued"] = True
    if _snapshot_http_viewport_lock.acquire(blocking=False):
        try:
            out["snapshot_http_viewport_queued"] = _snapshot_http_viewport_holder is not None
        finally:
            _snapshot_http_viewport_lock.release()
    else:
        out["snapshot_http_viewport_queued"] = True
    if _scene_randomize_lock.acquire(blocking=False):
        _scene_randomize_lock.release()
    else:
        out["scene_randomize_lock_busy"] = True
    return out


# /status 快路径：scene 大 JSON 仅在主线程刷新，HTTP worker 只读此缓存（避免 GET /status 在 HTTP 线程里扫 stage）。
_STATUS_HTTP_SCENE_MAIN_LOCK = threading.Lock()
_status_http_scene_main_cache: dict | None = None
_status_http_scene_main_cache_mono: float = 0.0
_LAST_STATUS_SCENE_MAIN_REFRESH_MONO: float = 0.0
_STATUS_SCENE_MAIN_REFRESH_MIN_INTERVAL_S = 0.9

_CTRL_PLANE_MAIN_DEGRADED_LOCK = threading.Lock()
_ctrl_plane_main_thread_degraded: bool = False
_ctrl_plane_main_thread_degraded_reason: str | None = None


def _read_status_http_scene_cache_dict() -> dict:
    with _STATUS_HTTP_SCENE_MAIN_LOCK:
        cached = _status_http_scene_main_cache
        ts_mono = float(_status_http_scene_main_cache_mono)
    now = time.monotonic()
    if isinstance(cached, dict) and cached:
        out = copy.deepcopy(cached)
        if (now - ts_mono) > 3.0:
            out["stale"] = True
        else:
            out.setdefault("stale", False)
        out.setdefault("scene_cache_host", "main_thread")
        return out
    return {
        "stale": True,
        "degraded": True,
        "degraded_reason": "status_scene_cache_empty",
        "note": "awaiting_main_thread_scene_refresh",
    }


def _refresh_status_http_scene_cache_main_thread(*, force: bool = False) -> None:
    """仅在 SimulationApp 主线程调用：写入供 /status 快路径只读的 scene 摘要。"""
    global _status_http_scene_main_cache, _status_http_scene_main_cache_mono, _LAST_STATUS_SCENE_MAIN_REFRESH_MONO
    now = time.monotonic()
    if not force and (now - _LAST_STATUS_SCENE_MAIN_REFRESH_MONO) < _STATUS_SCENE_MAIN_REFRESH_MIN_INTERVAL_S:
        return
    _LAST_STATUS_SCENE_MAIN_REFRESH_MONO = now
    try:
        if _STATUS_SCENE_REFRESH_FULL_SCAN:
            stage = omni.usd.get_context().get_stage()
            snap = _scene_state_snapshot(stage, runtime_lock_timeout=0.5)
        else:
            snap = _scene_state_lightweight_snapshot()
            snap["scene_cache_mode"] = "lightweight_rtsp_low_latency"
    except Exception as exc:
        snap = {
            "error": f"{type(exc).__name__}:{exc}",
            "stale": True,
            "degraded": True,
            "degraded_reason": "status_scene_refresh_failed",
        }
    with _STATUS_HTTP_SCENE_MAIN_LOCK:
        _status_http_scene_main_cache = snap
        _status_http_scene_main_cache_mono = time.monotonic()


def _update_ctrl_plane_degraded_main_thread_hint() -> None:
    """主线程每帧：刷新供 /api/health 只读的 degraded 提示（HTTP 线程不做重队列 probe）。"""
    global _ctrl_plane_main_thread_degraded, _ctrl_plane_main_thread_degraded_reason
    hp = _ctrl_heavy_tasks_probe()
    parts: list[str] = []
    if hp.get("snapshot_jpg_live_vp_queued"):
        parts.append("snapshot_live_vp")
    if hp.get("diag_live_once_queued"):
        parts.append("diag_live_once")
    if hp.get("snapshot_http_viewport_queued"):
        parts.append("snapshot_http_viewport")
    if hp.get("scene_randomize_lock_busy"):
        parts.append("scene_randomize")
    with _CTRL_PLANE_MAIN_DEGRADED_LOCK:
        _ctrl_plane_main_thread_degraded = bool(parts)
        _ctrl_plane_main_thread_degraded_reason = ",".join(parts) if parts else None


def _api_health_fast_path_payload() -> dict:
    """绝对轻量 /api/health：不读 stage、不 probe 重队列、不调用 _compose_status_dict。"""
    global _last_health_fast_path_ts
    _last_health_fast_path_ts = time.time()
    ts = time.time()
    with _CTRL_HTTP_ACTIVE_REQUESTS_LOCK:
        inf = int(_CTRL_HTTP_ACTIVE_REQUESTS)
    max_inf = max(8, int(_CTRL_HTTP_MAX_INFLIGHT))
    pressure = inf >= max(2, max_inf - 1)
    with _CTRL_PLANE_MAIN_DEGRADED_LOCK:
        main_deg = bool(_ctrl_plane_main_thread_degraded)
        main_reason = _ctrl_plane_main_thread_degraded_reason
    degraded = bool(pressure or main_deg)
    reason_parts: list[str] = []
    if pressure:
        reason_parts.append("ctrl_http_near_capacity")
    if main_deg and main_reason:
        reason_parts.append(str(main_reason))
    reason = ",".join(reason_parts) if reason_parts else None
    return {
        "ok": True,
        "process_alive": True,
        "control_http_ready": True,
        "degraded": degraded,
        "reason": reason,
        "ts": ts,
        "service": "ptz_stream",
        "listen": "0.0.0.0",
        "ctrl_port": _CTRL_PORT,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        "stream_init_ready": bool(_STREAM_INIT_READY.is_set()),
        "control_plane_snapshot": {
            "ctrl_http_inflight": inf,
            "ctrl_http_max_inflight": int(_CTRL_HTTP_MAX_INFLIGHT),
            "degraded": degraded,
            "busy_reason": reason,
        },
    }


def _random_config_http_payload_best_effort(*, api_wrap: bool) -> dict:
    """GET random-config：短超时读内存；失败则回退 _scene_runtime_stale_cache，不阻塞。"""
    stale = False
    degraded = False
    snap = _scene_state_runtime_snapshot(lock_timeout=0.08)
    if not isinstance(snap, dict) or not snap:
        snap = dict(_scene_runtime_stale_cache) if isinstance(_scene_runtime_stale_cache, dict) else {}
        stale = True
        degraded = True
    rc = _sanitize_random_config((snap or {}).get("random_config"))
    if api_wrap:
        return {"ok": True, "random_config": rc, "stale": stale, "degraded": degraded}
    out = dict(rc)
    out["stale"] = stale
    out["degraded"] = degraded
    return out


def _compose_status_dict(*, use_main_thread_scene_cache: bool = False) -> dict:
    global _stream_diag_stale_cache
    stage = omni.usd.get_context().get_stage()
    if _ptz_lock.acquire(timeout=_PTZ_STATE_HTTP_LOCK_TIMEOUT_S):
        try:
            out = dict(_ptz_state)
        finally:
            _ptz_lock.release()
    else:
        out = dict(_ptz_state_http_stale_cache)
    if _stream_diag_lock.acquire(timeout=_PTZ_STATE_HTTP_LOCK_TIMEOUT_S):
        try:
            sd = dict(_STREAM_DIAG)
            out["stream"] = sd
            _stream_diag_stale_cache = copy.deepcopy(sd)
        finally:
            _stream_diag_lock.release()
    else:
        out["stream"] = (
            copy.deepcopy(_stream_diag_stale_cache)
            if isinstance(_stream_diag_stale_cache, dict) and _stream_diag_stale_cache
            else {"stream_diag_lock_busy": True}
        )
    try:
        # 走短 TTL + single-flight 缓存，避免 /status 在缓存失效瞬间并发打爆 carb/viewport 读路径（与 8081 假死、Recv-Q 积压强相关）
        if not isinstance(out["stream"], dict):
            out["stream"] = {}
        out["stream"]["renderer_runtime_observation"] = _compose_renderer_hydra_observation_for_status_cached()
    except Exception as exc:
        if not isinstance(out.get("stream"), dict):
            out["stream"] = {}
        out["stream"]["renderer_runtime_observation"] = {"error": f"{type(exc).__name__}:{exc}"}
    if use_main_thread_scene_cache:
        out["scene"] = _read_status_http_scene_cache_dict()
    else:
        out["scene"] = _scene_state_snapshot(stage, runtime_lock_timeout=0.18)
    out["orientation"] = dict(_orientation_state)
    out["startup_view"] = dict(_startup_view_state)
    hp = _ctrl_heavy_tasks_probe()
    busy_reason_parts: list[str] = []
    if hp.get("snapshot_jpg_live_vp_queued"):
        busy_reason_parts.append("snapshot_live_vp")
    if hp.get("diag_live_once_queued"):
        busy_reason_parts.append("diag_live_once")
    if hp.get("snapshot_http_viewport_queued"):
        busy_reason_parts.append("snapshot_http_viewport")
    if hp.get("scene_randomize_lock_busy"):
        busy_reason_parts.append("scene_randomize")
    heavy_active = bool(busy_reason_parts)
    with _CTRL_HTTP_ACTIVE_REQUESTS_LOCK:
        _infl = int(_CTRL_HTTP_ACTIVE_REQUESTS)
    out["ctrl_plane"] = {
        "ctrl_http_inflight": _infl,
        "ctrl_http_max_inflight": int(_CTRL_HTTP_MAX_INFLIGHT),
        "heavy_task_active": heavy_active,
        "heavy_task_probe": hp,
        "last_health_fast_path_ts": float(_last_health_fast_path_ts),
        "last_snapshot_dt_ms": float(_last_snapshot_http_dt_ms),
        "last_rep_orchestrator_step_ms": float(_last_rep_orchestrator_step_ms),
        "busy_reason": ",".join(busy_reason_parts) if busy_reason_parts else None,
    }
    if use_main_thread_scene_cache:
        sc = out.get("scene")
        if isinstance(sc, dict):
            if sc.get("stale"):
                out["stale"] = True
            if sc.get("degraded"):
                out["degraded"] = True
                dr = sc.get("degraded_reason")
                if dr:
                    out["degraded_reason"] = str(dr)
        with _CTRL_PLANE_MAIN_DEGRADED_LOCK:
            if _ctrl_plane_main_thread_degraded:
                out["degraded"] = True
                if not out.get("degraded_reason"):
                    out["degraded_reason"] = _ctrl_plane_main_thread_degraded_reason or "main_thread_heavy_tasks"
    return out
_scene_up_axis = "Y"   # 加载场景后自动更新；Z-up 时改为 "Z"

# CamTilt 上实际用于 tilt 的 rotate 属性名（与 USD 资产是否含对应 op 有关，见 _resolve_or_create_tilt_rotate_attr）
_last_tilt_attr_used: str | None = None
_tilt_axis_diag_printed: bool = False
_ptz_base_initialized: bool = False
_ptz_base_pan_deg: float = 0.0
_ptz_base_tilt_deg: float = 0.0


_orientation_state: dict = {
    "mode": "legacy",
    "camera_xyz": None,
    "target_xyz": None,
    "base_pan": None,
    "base_tilt": None,
    "applied_pan": None,
    "applied_tilt": None,
    "applied_roll": 0.0,
    "last_source": "startup",
    "last_preset_name": "default_initial",
    "fallback": False,
    "fallback_reason": None,
}
_randomize_context_tilt_history = deque(maxlen=_RANDOMIZE_CONTEXT_DOWN_TILT_WINDOW)


_TILT_DIRECTION_THRESHOLD_DEG = 2.0


def _tilt_direction_label(tilt_deg) -> str | None:
    try:
        tilt = float(tilt_deg)
    except (TypeError, ValueError):
        return None
    if tilt > _TILT_DIRECTION_THRESHOLD_DEG:
        return "down"
    if tilt < -_TILT_DIRECTION_THRESHOLD_DEG:
        return "up"
    return "level"


def _is_down_tilt(tilt_deg) -> bool:
    return _tilt_direction_label(tilt_deg) == "down"


_startup_view_state: dict = {
    "token": "startup",
    "name": "StartupView",
    "pan": None,
    "tilt": None,
    "zoom": None,
    "camera_xyz": None,
    "target_xyz": None,
    "base_pan": None,
    "base_tilt": None,
    "source": None,
    "preset_name": "default_initial",
}

# ── 吊篮高度随机范围（与 HTTP POST /scene/gondola 的 clamp 一致，集中配置）────
_GONDOLA_HEIGHT_MIN = 0.0
_GONDOLA_HEIGHT_MAX = 3300.0

# _CAMERA_RIG_TRANSLATE_XYZ 在 cfg 读取段由 camera_rig_translate_xyz 解析（见 _camera_rig_translate_from_cfg）

# 路径候选：优先用户约定路径，其次常见 SceneRoot 前缀；运行时以 stage 上实际存在为准
_GONDOLA_PATH_CANDIDATES = (
    "/World/DiaoLan/Model/Group1",
    "/World/SceneRoot/DiaoLan/Model/Group1",
    "/World/SceneRoot/DiaoLan_01/Model/Group1",
)
# node_1（及 USD 转义名 node______1）
_NODE1_PATH_CANDIDATES = (
    "/World/DiaoLan/Model/node_1",
    "/World/SceneRoot/DiaoLan/Model/node_1",
    "/World/DiaoLan/Model/node______1",
    "/World/SceneRoot/DiaoLan/Model/node______1",
    "/World/SceneRoot/DiaoLan_01/Model/node______1",
)
_WORKER2_PATH_CANDIDATES = (
    "/World/DiaoLan/Model/node_2",
    "/World/SceneRoot/DiaoLan/Model/node_2",
    "/World/DiaoLan/Model/node______2",
    "/World/SceneRoot/DiaoLan/Model/node______2",
    "/World/SceneRoot/DiaoLan_01/Model/node______2",
)

# 与 Group1 同挂在 .../Model 下的工人根 prim 名（USD 转义名）
_NODE1_SIBLING_NAMES = frozenset(("node______1", "node_1"))
_NODE2_SIBLING_NAMES = frozenset(("node______2", "node_2"))
_CAMERA_RIG_PATH_CANDIDATES_EXTRA = (
    "/World/CameraRig",
    "/World/SceneRoot/CameraRig",
)

# ── 场景控制状态（吊篮 + 工人）；prim 路径在 stage 加载后解析写入下列变量 ──
_GONDOLA_PRIM: str = ""
_WORKER1_PRIM: str = ""   # 与 Group1 做高度同步的 node_1（解析失败则为空）
_WORKER2_PRIM: str = ""
_CAMERA_RIG_PRIM: str = ""  # 写入固定 translate 的 rig（由 camera_prim 推断或候选解析）

# 首次应用场景状态时，若 node_1 与 Group1 为兄弟关系，记录二者在「高度轴」上的初始差
_node1_vs_group1_height_offset: float | None = None
# 首次应用场景状态时，若 node_2 与 Group1 为兄弟关系，记录二者在「高度轴」上的初始差
_node2_vs_group1_height_offset: float | None = None

# 启动时随机一次的吊篮高度（仅日志/诊断；与 gondola_y 一致）
_last_gondola_init_sampled_height: float | None = None
_last_gondola_init_synced_node1: bool = False
_last_gondola_init_synced_node2: bool = False
_last_gondola_init_relation: str = ""
_last_gondola_init_group1_source: str = ""  # dynamic_traverse | fallback_list | missing

_GONDOLA_HEIGHT_DIAG_PRINTED: bool = False


def _get_translate_tuple(stage, prim_path: str) -> tuple[float, float, float] | None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    attr = prim.GetAttribute("xformOp:translate")
    if not (attr and attr.IsValid()) or attr.Get() is None:
        return None
    v = attr.Get()
    return (float(v[0]), float(v[1]), float(v[2]))


def _gondola_world_aabb_str(stage, prim_path: str) -> str:
    from pxr import UsdGeom, Usd

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return "invalid"
    try:
        img = UsdGeom.Imageable(prim)
        wb = img.ComputeWorldBound(Usd.TimeCode.Default(), UsdGeom.Tokens.default_)
        box = wb.ComputeAlignedBox()
        mn = box.GetMin()
        mx = box.GetMax()
        return (
            f"world_aabb min=({mn[0]:.3f},{mn[1]:.3f},{mn[2]:.3f}) "
            f"max=({mx[0]:.3f},{mx[1]:.3f},{mx[2]:.3f})"
        )
    except Exception as exc:
        return f"aabb_err={exc!r}"


def _print_gondola_height_line(
    stage,
    label: str,
    prim_path: str,
    world_height_value_set: float,
    before_t: tuple[float, float, float] | None,
) -> None:
    """单行：写高度目标 prim、高度轴分量前后、父级、类型、world AABB。"""
    idx = _height_axis_index()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[gondola-height] {label} INVALID prim_path={prim_path!r}", flush=True)
        return
    parent = prim.GetParent().GetPath().pathString if prim.GetParent().IsValid() else ""
    ptype = prim.GetTypeName()
    authored_comp = before_t[idx] if before_t is not None else None
    after_t = _get_translate_tuple(stage, prim_path)
    after_comp = after_t[idx] if after_t is not None else None
    aabb = _gondola_world_aabb_str(stage, prim_path)
    print(
        "[gondola-height] "
        f"label={label!r} prim={prim_path!r} type={ptype!r} parent={parent!r} "
        f"height_axis_idx={idx} "
        f"authored_height_component={authored_comp} world_height_value_set={world_height_value_set} "
        f"before_translate={before_t} after_translate={after_t} "
        f"height_component_after={after_comp} {aabb}",
        flush=True,
    )


_HDRI_TEXTURE_ATTR_NAME = "inputs:texture:file"
_HDRI_FIXED_GROUP_ID = "A"
_HDRI_FIXED_GROUP_NAME = "固定 HDRI 4 选 1"
_HDRI_FIXED_SPECS = (
    ("A1", 1, "sky_sun", os.path.join("changjing", "sky_sun.hdr"), "固定白名单 HDRI（sky_sun.hdr）"),
    ("A2", 2, "sunny_rose_garden_8k", os.path.join("changjing", "sunny_rose_garden_8k.hdr"), "固定白名单 HDRI（sunny_rose_garden_8k.hdr）"),
    ("A3", 3, "minedump_flats_8k", os.path.join("changjing", "minedump_flats_8k.hdr"), "固定白名单 HDRI（minedump_flats_8k.hdr）"),
    ("A4", 4, "farm_road_8k", os.path.join("changjing", "farm_road_8k.hdr"), "固定白名单 HDRI（farm_road_8k.hdr）"),
)


def _cfg_environment_mode_initial() -> str:
    m = str(cfg.get("environment_mode") or "hdri").strip().lower()
    return m if m in ("hdri", "dynamic_sky") else "hdri"


def _cfg_dynamic_sky_preset_abs() -> str:
    raw = cfg.get("dynamic_sky_preset_path")
    if isinstance(raw, str) and raw.strip():
        rp = raw.strip()
        return os.path.normpath(_resolve_path(rp)) if not os.path.isabs(rp) else os.path.normpath(rp)
    return os.path.normpath(
        os.path.join(
            script_dir,
            "dynamic_sky_pkg",
            "Assets",
            "Skies",
            "2022_1",
            "Skies",
            "Dynamic",
            "ClearSky.usd",
        )
    )


def _cfg_dynamic_sky_root_prim() -> str:
    p = str(cfg.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot").strip()
    return p if p.startswith("/") else "/World/DynamicSkyRoot"


def _dynamic_sky_presets_dir() -> str:
    return os.path.normpath(
        os.path.join(script_dir, "dynamic_sky_pkg", "Assets", "Skies", "2022_1", "Skies", "Dynamic")
    )


def _list_dynamic_sky_web_presets() -> list[dict]:
    """列出 dynamic_sky_pkg 内 Skies/Dynamic 下的 USD 预设（与资产目录一致，当前为 7 个天气类预设）。"""
    d = _dynamic_sky_presets_dir()
    out: list[dict] = []
    if not os.path.isdir(d):
        return out
    for name in sorted(os.listdir(d)):
        low = name.lower()
        if not low.endswith(".usd"):
            continue
        full = os.path.normpath(os.path.join(d, name))
        if not os.path.isfile(full):
            continue
        preset_id = name[:-4] if low.endswith(".usd") else name
        out.append({"id": preset_id, "label": preset_id, "path": full})
    return out


def _dynamic_sky_preset_id_from_path(path_str: str) -> str:
    base = os.path.basename(str(path_str or "").strip())
    if not base:
        return ""
    low = base.lower()
    if low.endswith(".usd"):
        return base[:-4]
    return base


def _http_dynamic_sky_presets_payload(stage=None, *, include_stage_status: bool = True) -> dict:
    presets = _list_dynamic_sky_web_presets()
    if include_stage_status:
        st = stage or omni.usd.get_context().get_stage()
        env = _environment_public_status(st)
    else:
        with _scene_lock:
            mode = str(_scene_state.get("environment_mode") or "hdri")
            dy_en = bool(_scene_state.get("dynamic_sky_enabled"))
            preset = str(_scene_state.get("dynamic_sky_preset_path") or "")
            root = str(_scene_state.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot")
            mount_ok = bool(_scene_state.get("dynamic_sky_mount_ok"))
            mounted_preset = _scene_state.get("dynamic_sky_mounted_preset_path")
            last_err = _scene_state.get("dynamic_sky_last_error")
            last_at = _scene_state.get("dynamic_sky_last_action_at")
        env = {
            "environment_mode": mode,
            "dynamic_sky_effective": bool(str(mode).strip().lower() == "dynamic_sky" and dy_en),
            "dynamic_sky_enabled": dy_en,
            "dynamic_sky_preset_path": preset,
            "dynamic_sky_root_prim": root,
            "dynamic_sky_mount_ok": mount_ok,
            "dynamic_sky_mounted_preset_path": mounted_preset,
            "dynamic_sky_root_exists": None,
            "dynamic_sky_root_active": None,
            "preset_file_exists": bool(preset and os.path.isfile(preset)),
            "hdri_environment_prims_disabled": [],
            "hdri_env_mutually_excluded": None,
            "dynamic_sky_last_error": last_err,
            "dynamic_sky_last_action_at": last_at,
            "stream": {},
            "stage_status_omitted": True,
        }
    mounted = str(env.get("dynamic_sky_mounted_preset_path") or "").strip()
    configured = str(env.get("dynamic_sky_preset_path") or "").strip()
    cur_path = mounted or configured
    cur_id = _dynamic_sky_preset_id_from_path(cur_path)
    return {
        "ok": True,
        "presets": presets,
        "dynamic_sky_presets": presets,
        "current_preset": cur_id,
        "current_dynamic_sky_preset": cur_id,
        "current_preset_path": cur_path,
        "environment_mode": str(env.get("environment_mode") or "hdri"),
        "environment": env,
    }


def _normalize_hdri_path(raw_value) -> str:
    value = str(raw_value or "").strip()
    if not value:
        return ""
    return os.path.abspath(_resolve_path(value))


def _unique_ordered_strings(values) -> list[str]:
    out = []
    seen = set()
    for raw in values:
        value = str(raw or "").strip()
        if not value or value in seen:
            continue
        seen.add(value)
        out.append(value)
    return out


def _build_hdri_entry(entry_id: str, slot: int, name: str, raw_path: str, evidence: str) -> dict:
    normalized_path = _normalize_hdri_path(raw_path) if raw_path else ""
    return {
        "id": str(entry_id),
        "slot": int(slot),
        "name": str(name),
        "path": normalized_path,
        "basename": os.path.basename(normalized_path) if normalized_path else None,
        "exists": bool(normalized_path and os.path.isfile(normalized_path)),
        "evidence": str(evidence or ""),
    }


def _configured_hdri_entries() -> list[dict]:
    configured_paths = cfg.get("hdri_whitelist") if isinstance(cfg.get("hdri_whitelist"), (list, tuple)) else None
    specs = []
    for idx, base_spec in enumerate(_HDRI_FIXED_SPECS):
        entry_id, slot, name, raw_path, evidence = base_spec
        override_path = configured_paths[idx] if isinstance(configured_paths, (list, tuple)) and idx < len(configured_paths) else raw_path
        specs.append(_build_hdri_entry(entry_id, slot, name, override_path, evidence))
    return specs


def _build_hdri_groups() -> dict:
    return {
        _HDRI_FIXED_GROUP_ID: {
            "id": _HDRI_FIXED_GROUP_ID,
            "name": _HDRI_FIXED_GROUP_NAME,
            "entries": _configured_hdri_entries(),
        },
    }


def _hdri_group_entries(group_id: str) -> list[dict]:
    groups = _build_hdri_groups()
    return list((groups.get(group_id) or groups[_HDRI_FIXED_GROUP_ID]).get("entries") or [])


def _default_hdri_candidates() -> list[str]:
    return [entry["path"] for entry in _hdri_group_entries(_HDRI_FIXED_GROUP_ID) if entry.get("exists") and entry.get("path")]


def _sanitize_hdri_candidates(raw_value, fallback=None) -> list[str]:
    allowed = _default_hdri_candidates()
    allowed_set = set(allowed)
    if isinstance(raw_value, (list, tuple, set)):
        raw_items = list(raw_value)
    elif isinstance(raw_value, str):
        normalized_text = raw_value.replace("\r", "\n").replace(",", "\n")
        raw_items = normalized_text.split("\n")
    else:
        raw_items = []

    explicit_nonempty = [item for item in raw_items if str(item or "").strip()]
    normalized = _unique_ordered_strings(
        value for value in (_normalize_hdri_path(item) for item in raw_items) if value in allowed_set
    )
    if normalized:
        return normalized
    # 用户显式填了路径但均不在白名单或文件无效：尊重为「空结果」
    if explicit_nonempty:
        return []
    # []、空串、仅空白行：视为「未指定」，回退到默认白名单/组候选（避免 random_hdri 因 textarea 空而整体失败）
    if fallback is not None:
        return [value for value in fallback if _normalize_hdri_path(value) in allowed_set]
    return allowed



def _default_hdri_control_state() -> dict:
    return {
        "current_group_id": _HDRI_FIXED_GROUP_ID,
        "selected_by_group": {_HDRI_FIXED_GROUP_ID: "A1"},
        "last_switch_time": None,
        "last_apply_ok": None,
        "last_apply_message": "",
        "last_action": None,
    }


def _sanitize_hdri_control_state(raw) -> dict:
    base = _default_hdri_control_state()
    if not isinstance(raw, dict):
        return base
    group_id = _HDRI_FIXED_GROUP_ID
    selected_raw = raw.get("selected_by_group") if isinstance(raw.get("selected_by_group"), dict) else {}
    selected_by_group = {
        _HDRI_FIXED_GROUP_ID: str(
            selected_raw.get(_HDRI_FIXED_GROUP_ID) or base["selected_by_group"][_HDRI_FIXED_GROUP_ID]
        ).strip()
    }
    return {
        "current_group_id": group_id,
        "selected_by_group": selected_by_group,
        "last_switch_time": raw.get("last_switch_time") or base["last_switch_time"],
        "last_apply_ok": raw.get("last_apply_ok"),
        "last_apply_message": str(raw.get("last_apply_message") or ""),
        "last_action": str(raw.get("last_action") or "") or None,
    }


def _available_hdri_entries(group_id: str) -> list[dict]:
    return [entry for entry in _hdri_group_entries(group_id) if entry.get("exists") and entry.get("path")]


def _entry_id_for_slot(group_id: str, slot: int | None) -> str | None:
    if slot is None:
        return None
    for entry in _hdri_group_entries(group_id):
        if int(entry.get("slot") or 0) == int(slot):
            return entry.get("id")
    return None


def _get_hdri_entry(group_id: str, entry_id: str | None = None) -> dict | None:
    entries = _hdri_group_entries(group_id)
    if entry_id:
        for entry in entries:
            if entry.get("id") == entry_id:
                return entry
    return entries[0] if entries else None


def _match_hdri_entry_by_path(path_value: str | None) -> tuple[str | None, dict | None]:
    normalized_path = _normalize_hdri_path(path_value)
    if not normalized_path:
        return None, None
    groups = _build_hdri_groups()
    for group_id, group in groups.items():
        for entry in group["entries"]:
            if entry.get("path") and _normalize_hdri_path(entry.get("path")) == normalized_path:
                return group_id, dict(entry)
    return None, None


def _current_group_hdri_candidates(control_state: dict | None = None) -> list[str]:
    state = _sanitize_hdri_control_state(control_state)
    group_id = state.get("current_group_id") or _HDRI_FIXED_GROUP_ID
    return [entry["path"] for entry in _available_hdri_entries(group_id)]


def _hdri_candidate_file_report() -> list[dict]:
    return [
        {
            "id": entry.get("id"),
            "name": entry.get("name"),
            "path": entry.get("path"),
            "exists": bool(entry.get("exists")),
        }
        for entry in _configured_hdri_entries()
    ]


def _list_hdri_binding_candidates(stage) -> list[dict]:
    out = []
    if stage is None:
        return out
    light_audit = _collect_runtime_light_audit(stage)
    audit_by_path = {
        str(item.get("prim_path")): item
        for item in (light_audit.get("lights") if isinstance(light_audit, dict) else [])
        if isinstance(item, dict) and item.get("prim_path")
    }
    for prim in stage.Traverse():
        if not prim.IsValid() or prim.GetTypeName() != "DomeLight":
            continue
        attr = prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        current = _extract_asset_path(attr.Get()) if attr and attr.IsValid() else ""
        audit = audit_by_path.get(prim.GetPath().pathString, {})
        out.append(
            {
                "prim_path": prim.GetPath().pathString,
                "has_texture_attr": bool(attr and attr.IsValid()),
                "current_hdri": current or None,
                "active": bool(audit.get("active")) if isinstance(audit, dict) else bool(prim.IsActive()),
                "visibility": audit.get("visibility") if isinstance(audit, dict) else _attr_value(prim, "visibility", "inherited"),
                "intensity": audit.get("intensity") if isinstance(audit, dict) else _attr_value(prim, "inputs:intensity"),
                "exposure": audit.get("exposure") if isinstance(audit, dict) else _attr_value(prim, "inputs:exposure"),
                "render_score": audit.get("render_score") if isinstance(audit, dict) else None,
                "is_primary_environment_light": bool(audit.get("is_primary_environment_light")) if isinstance(audit, dict) else False,
            }
        )
    return out


_scene_state = {
    "gondola_y": 0.0,
    "gondola_heights": {},
    "workers": 2,
    "active_diaolan_path": "",
    "selected_diaolan_path": "",
    "all_diaolan_paths": [],
    "all_worker_paths": [],
    "visible_worker_paths": [],
    "gondola_renderable_paths": [],
    "gondola_visible_renderable_paths": [],
    "gondola_hidden_paths": [],
    "gondola_renderable_debug": [],
    "height_debug": {},
    "random_config": {
        "auto_random_on_start": True,
        "auto_random_timer_enabled": True,
        "auto_random_interval_seconds": 600,
        "random_gondola": True,
        "random_workers": True,
        "random_camera": True,
        "random_hdri": True,
        "keep_target_visible": True,
        "keep_wall_install_constraint": True,
        "auto_look_at_target": True,
        "hdri_candidates": _default_hdri_candidates(),
    },
    "hdri_control": _default_hdri_control_state(),
    "hdri_backend_status": {},
    "last_random_result": None,
    "pending_active_diaolan_path": "",
    # 各吊篮根路径 -> 可见工人数（0..len(persons)）；与「当前操控吊篮」解耦
    "workers_visible_count_by_diaolan_path": {},
    # 环境：HDRI 与 Dynamic Sky（USD preset）互斥；默认 HDRI，Dynamic Sky 需显式开启
    "environment_mode": _cfg_environment_mode_initial(),
    "dynamic_sky_enabled": bool(cfg.get("dynamic_sky_enabled", False)),
    "dynamic_sky_preset_path": _cfg_dynamic_sky_preset_abs(),
    "dynamic_sky_root_prim": _cfg_dynamic_sky_root_prim(),
    "dynamic_sky_mount_ok": False,
    "dynamic_sky_mounted_preset_path": None,
    "dynamic_sky_last_error": None,
    "dynamic_sky_last_action_at": None,
    "hdri_env_exclude_snapshot": None,
    # rule11：各次 POST 前先还原到此标称能量，再加亮，避免叠乘导致数值失控/画面变暗
    "rule11_hdri_dome_energy_baseline": None,
    # rule11 显式请求时顺带抬 KeyLight；记录标称强度以便每次先还原
    "rule11_keylight_intensity_baseline": None,
}
_scene_lock = threading.Lock()
_scene_dirty = threading.Event()
_scene_randomize_dirty = threading.Event()
_scene_randomize_lock = threading.Lock()
_scene_randomize_request = None
# RTSP：主线程执行 scene randomize 期间为 True；供 _pipe_writer 放宽 repeat_blocked 停写，避免 ffmpeg stdin 长时间零写入。
_rtsp_randomize_keepalive_active: bool = False
# 主循环内定时随机：下一触发用的 frame_idx（每轮 world.step 后 frame_idx+=1，步长用 sim_hz 换算秒）。
_auto_random_deadline_frame_idx: int | None = None

# POST /scene/randomize 请求体中非 random_config 的元字段（勿写入 random_config 合并）
_RANDOMIZE_REQ_META_KEYS = frozenset(
    {
        "trigger",
        "active_diaolan_path",
        "runtime_random_config",
        "is_auto",
        "source",
        "request_id",
        "rule_id",
        "event_id",
        # 可选：仅当未显式传 rule_id 时作为测试强制归因（默认不传则不影响 {}）
        "debug_force_rule_id",
        # 可选：为本次事件指定稳定实例 id（否则服务端生成 evt_r*）
        "event_instance_id",
    }
)
_scene_hdri_dirty = threading.Event()
_scene_hdri_lock = threading.Lock()
_scene_hdri_request = None
_scene_environment_dirty = threading.Event()
_scene_environment_lock = threading.Lock()
_scene_environment_request = None
_pending_hdri_audits: list[dict] = []
_chosen_worker: int = 1
_scene_experiment_state = {
    "active_hidden_paths": [],
    "original_visibility": {},
}


def _dynamic_sky_effective_unlocked() -> bool:
    if str(_scene_state.get("environment_mode") or "hdri").strip().lower() != "dynamic_sky":
        return False
    return bool(_scene_state.get("dynamic_sky_enabled"))


def _dynamic_sky_effective() -> bool:
    with _scene_lock:
        return _dynamic_sky_effective_unlocked()


def _environment_allows_hdri() -> bool:
    return not _dynamic_sky_effective()


def _default_random_config() -> dict:
    return {
        "auto_random_on_start": True,
        "auto_random_timer_enabled": True,
        "auto_random_interval_seconds": 600,
        "random_gondola": True,
        "random_workers": True,
        "random_camera": True,
        "random_hdri": True,
        "random_guardrail": False,
        "random_safety_rope": False,
        "random_limitstop": False,
        "random_fallarrestor": False,
        "fallarrestor_noncompliant_probability": 0.5,
        "random_overexposure_event": False,
        "random_overexposure_event_probability": 0.2,
        "rule11_overexposure_exposure_delta": 30.0,
        "guardrail_mode": "random",
        "safety_rope_mode": "random",
        "limitstop_mode": "random",
        "fallarrestor_mode": "random",
        "keep_target_visible": True,
        "keep_wall_install_constraint": True,
        "auto_look_at_target": True,
        "hdri_candidates": _default_hdri_candidates(),
    }


def _sanitize_random_config(raw) -> dict:
    cfg_in = raw if isinstance(raw, dict) else {}
    merged = _default_random_config()
    bool_keys = (
        "auto_random_on_start",
        "auto_random_timer_enabled",
        "random_gondola",
        "random_workers",
        "random_camera",
        "random_hdri",
        "random_guardrail",
        "random_safety_rope",
        "random_limitstop",
        "random_fallarrestor",
        "random_overexposure_event",
        "keep_target_visible",
        "keep_wall_install_constraint",
        "auto_look_at_target",
    )
    for key in bool_keys:
        if key in cfg_in:
            merged[key] = bool(cfg_in.get(key))
    if "auto_random_interval_seconds" in cfg_in:
        try:
            iv = int(cfg_in.get("auto_random_interval_seconds"))
        except (TypeError, ValueError):
            iv = merged.get("auto_random_interval_seconds", 600)
        merged["auto_random_interval_seconds"] = max(10, min(86400, iv))
    merged["hdri_candidates"] = _sanitize_hdri_candidates(
        cfg_in.get("hdri_candidates") if "hdri_candidates" in cfg_in else None,
        fallback=merged.get("hdri_candidates") or _default_hdri_candidates(),
    )
    gm = str(cfg_in.get("guardrail_mode", merged.get("guardrail_mode", "random")) or "random").strip().lower()
    if gm not in ("intact", "missing", "random"):
        gm = "random"
    merged["guardrail_mode"] = gm
    sm = str(cfg_in.get("safety_rope_mode", merged.get("safety_rope_mode", "random")) or "random").strip().lower()
    if sm not in ("compliant", "non_compliant", "random"):
        sm = "random"
    merged["safety_rope_mode"] = sm
    lm = str(cfg_in.get("limitstop_mode", merged.get("limitstop_mode", "random")) or "random").strip().lower()
    if lm not in ("intact", "missing", "random"):
        lm = "random"
    merged["limitstop_mode"] = lm
    fm = str(cfg_in.get("fallarrestor_mode", merged.get("fallarrestor_mode", "weighted_random")) or "weighted_random").strip().lower()
    if fm not in ("manual", "force_compliant", "force_noncompliant", "weighted_random"):
        fm = "weighted_random"
    merged["fallarrestor_mode"] = fm
    if "fallarrestor_noncompliant_probability" in cfg_in:
        try:
            fp = float(cfg_in.get("fallarrestor_noncompliant_probability"))
        except (TypeError, ValueError):
            fp = float(merged.get("fallarrestor_noncompliant_probability") or 0.5)
        merged["fallarrestor_noncompliant_probability"] = max(0.0, min(1.0, fp))
    if "random_overexposure_event_probability" in cfg_in:
        try:
            p = float(cfg_in.get("random_overexposure_event_probability"))
        except (TypeError, ValueError):
            p = float(merged.get("random_overexposure_event_probability") or 0.2)
        merged["random_overexposure_event_probability"] = max(0.0, min(1.0, p))
    if "rule11_overexposure_exposure_delta" in cfg_in:
        try:
            d = float(cfg_in.get("rule11_overexposure_exposure_delta"))
        except (TypeError, ValueError):
            d = float(merged.get("rule11_overexposure_exposure_delta") or 30.0)
        merged["rule11_overexposure_exposure_delta"] = max(0.5, min(48.0, d))
    return merged


# 进程内 random_config：代码默认见 _default_random_config()；若 ptz_config.yaml 含 random_config 则再合并
with _scene_lock:
    _rc_from_yaml = cfg.get("random_config")
    if isinstance(_rc_from_yaml, dict):
        _scene_state["random_config"] = _sanitize_random_config(_rc_from_yaml)
    else:
        _scene_state["random_config"] = _sanitize_random_config({})


def _scene_state_runtime_snapshot(*, lock_timeout: float | None = None) -> dict:
    """lock_timeout 非 None 时：拿不到锁则返回最近一次成功快照（可能略旧），避免 /status 被 randomize 长事务拖死。"""
    global _scene_runtime_stale_cache
    if lock_timeout is None:
        acq = _scene_lock.acquire(blocking=True)
    else:
        acq = _scene_lock.acquire(timeout=float(lock_timeout))
    if not acq:
        return dict(_scene_runtime_stale_cache) if isinstance(_scene_runtime_stale_cache, dict) else {}
    try:
        out = {
            "gondola_y": float(_scene_state.get("gondola_y", 0.0)),
            "gondola_heights": dict(_scene_state.get("gondola_heights") or {}),
            "workers": int(_scene_state.get("workers", 2)),
            "active_diaolan_path": str(_scene_state.get("active_diaolan_path", "") or ""),
            "selected_diaolan_path": str(_scene_state.get("selected_diaolan_path", "") or ""),
            "all_diaolan_paths": list(_scene_state.get("all_diaolan_paths") or []),
            "all_worker_paths": list(_scene_state.get("all_worker_paths") or []),
            "visible_worker_paths": list(_scene_state.get("visible_worker_paths") or []),
            "gondola_renderable_paths": list(_scene_state.get("gondola_renderable_paths") or []),
            "gondola_visible_renderable_paths": list(_scene_state.get("gondola_visible_renderable_paths") or []),
            "gondola_hidden_paths": list(_scene_state.get("gondola_hidden_paths") or []),
            "gondola_renderable_debug": list(_scene_state.get("gondola_renderable_debug") or []),
            "height_debug": dict(_scene_state.get("height_debug") or {}),
            "random_config": _sanitize_random_config(_scene_state.get("random_config")),
            "hdri_control": _sanitize_hdri_control_state(_scene_state.get("hdri_control")),
            "hdri_backend_status": _sanitize_hdri_backend_status(_scene_state.get("hdri_backend_status")),
            "last_random_result": _scene_state.get("last_random_result"),
            "pending_active_diaolan_path": str(_scene_state.get("pending_active_diaolan_path", "") or ""),
            "workers_visible_count_by_diaolan_path": dict(
                _scene_state.get("workers_visible_count_by_diaolan_path") or {}
            ),
            "environment_mode": str(_scene_state.get("environment_mode") or "hdri"),
            "dynamic_sky_enabled": bool(_scene_state.get("dynamic_sky_enabled")),
            "dynamic_sky_preset_path": str(_scene_state.get("dynamic_sky_preset_path") or ""),
            "dynamic_sky_root_prim": str(_scene_state.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot"),
            "dynamic_sky_mount_ok": bool(_scene_state.get("dynamic_sky_mount_ok")),
            "dynamic_sky_mounted_preset_path": _scene_state.get("dynamic_sky_mounted_preset_path"),
            "dynamic_sky_last_error": _scene_state.get("dynamic_sky_last_error"),
            "dynamic_sky_last_action_at": _scene_state.get("dynamic_sky_last_action_at"),
            "hdri_env_exclude_snapshot_paths": list(
                (_scene_state.get("hdri_env_exclude_snapshot") or {}).keys()
            )
            if isinstance(_scene_state.get("hdri_env_exclude_snapshot"), dict)
            else [],
        }
        _scene_runtime_stale_cache = copy.deepcopy(out)
        return out
    finally:
        _scene_lock.release()


def _extract_asset_path(value) -> str:
    if value is None:
        return ""
    raw_path = getattr(value, "path", None)
    resolved_path = getattr(value, "resolvedPath", None)
    if resolved_path:
        return str(resolved_path)
    if raw_path:
        return str(raw_path)
    return str(value)


def _looks_like_hdri_asset_path(value) -> bool:
    low = str(value or "").strip().lower()
    if not low:
        return False
    for sep in ("?", "#"):
        if sep in low:
            low = low.split(sep, 1)[0]
    return (
        low.endswith(".hdr")
        or low.endswith(".exr")
        or ".hdr]" in low
        or ".exr]" in low
    )


def _repair_broken_texture_paths(stage) -> None:
    """修复部分构建管线/编辑器中产生的非法资产后缀（如 .png]），导致黑模，并重定向丢失的贴图"""
    from pxr import UsdShade, Sdf
    import os
    if not stage:
        return
    
    fallback_dir = "/home/uniubi/xuanyuan/camera05/camera03/textures"
    repaired_count = 0
    fallback_count = 0
    alias_count = 0
    missing_reported = set()

    texture_aliases = {
        "Ground037_4K-PNG_NormalDX.png": "Ground037_4K-PNG_NormalGL.png",
        "Ground037_4K-PNG_AmbientOcclusion.png": "Ground037_4K-PNG_Color.png",
        "T_Grunge_Concrete_Wall_01_2K_BaseColor.png": "Damaged_Concrete_Wall_vdcnfcd_4K_BaseColor.jpg",
        "T_Grunge_Concrete_Wall_01_2K_Normal.png": "Damaged_Concrete_Wall_vdcnfcd_4K_Normal.jpg",
    }

    def _resolve_texture_fallback(base_name: str) -> tuple[str, str]:
        direct_path = os.path.join(fallback_dir, base_name)
        if os.path.isfile(direct_path):
            return direct_path, "exact"
        alias_name = texture_aliases.get(base_name)
        if alias_name:
            alias_path = os.path.join(fallback_dir, alias_name)
            if os.path.isfile(alias_path):
                return alias_path, f"alias:{alias_name}"
        return "", ""
    
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        for inp in shader.GetInputs():
            if inp.GetTypeName() in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.String, Sdf.ValueTypeNames.Token):
                val = inp.Get()
                if not val:
                    continue
                v_path = str(getattr(val, "path", val))
                if not v_path:
                    continue
                
                new_val = v_path.strip()
                changed = False
                
                if new_val.endswith(']'):
                    new_val = new_val[:-1].strip()
                    changed = True
                    repaired_count += 1
                
                # 若贴图包含 textures/，则重定向到 fallback_dir 中的精确文件；少数 USDZ 漏打包贴图使用明确同族/同通道别名兜底。
                if "textures/" in new_val:
                    base_name = os.path.basename(new_val).strip()
                    fallback_path, fallback_source = _resolve_texture_fallback(base_name)
                    if fallback_path:
                        new_val = fallback_path
                        changed = True
                        if fallback_source == "exact":
                            fallback_count += 1
                        else:
                            alias_count += 1
                    else:
                        if base_name not in missing_reported and (
                            "Ground037" in base_name
                            or "cgaxis_pbr" in base_name
                            or "T_Grunge_Concrete_Wall_01_2K" in base_name
                        ):
                            missing_reported.add(base_name)
                            print(f"[texture-repair-missing] {base_name} not found in {fallback_dir}", flush=True)
                
                if changed:
                    try:
                        if inp.GetTypeName() == Sdf.ValueTypeNames.Asset:
                            inp.Set(Sdf.AssetPath(new_val))
                        else:
                            inp.Set(new_val)
                    except Exception:
                        pass
                        
    if repaired_count > 0 or fallback_count > 0 or alias_count > 0:
        print(
            f"[texture-repair] Successfully repaired {repaired_count} broken texture paths, "
            f"redirected {fallback_count} exact textures, redirected {alias_count} alias textures",
            flush=True,
        )


def _attr_value(prim, attr_name: str, default=None):
    if prim is None or not prim.IsValid():
        return default
    attr = prim.GetAttribute(attr_name)
    if not (attr and attr.IsValid()):
        return default
    try:
        value = attr.Get()
    except Exception:
        return default
    return default if value is None else value


def _light_intensity_score(intensity_value, exposure_value) -> float:
    try:
        intensity = float(intensity_value)
    except (TypeError, ValueError):
        intensity = 0.0
    try:
        exposure = float(exposure_value)
    except (TypeError, ValueError):
        exposure = 0.0
    intensity = max(0.0, intensity)
    # 统一把 light 强度映射到可比较分值，方便找“主导”光源。
    return float(intensity * (2.0 ** exposure))


def _json_safe_value(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(k): _json_safe_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe_value(v) for v in value]
    if hasattr(value, "path") or hasattr(value, "resolvedPath"):
        return _extract_asset_path(value)
    try:
        return [_json_safe_value(v) for v in value]
    except Exception:
        pass
    try:
        return float(value)
    except Exception:
        return str(value)


def _normalize_replicator_rgba_for_output(rgba):
    """将 Replicator rgb 输出规范为 HWC uint8 RGBA。

    HydraStorm 等路径下 annotator 可能返回 float32/float16 线性颜色；若直接交给 Pillow / rawvideo，
    会被当作 uint8 或错误字节序解释，表现为整帧近纯黑或花屏。
    """
    if rgba is None or getattr(rgba, "size", 0) <= 0:
        return rgba
    arr = np.asarray(rgba)
    if arr.ndim != 3 or arr.shape[-1] < 3:
        return rgba
    if arr.dtype == np.uint8:
        return rgba
    rgb = np.asarray(arr[:, :, :3], dtype=np.float32)
    rgb = np.nan_to_num(rgb, nan=0.0, posinf=65504.0, neginf=0.0)
    mx = float(np.max(rgb)) if rgb.size else 0.0
    if mx <= 1.0 + 1e-3:
        out_rgb = np.clip(rgb * 255.0, 0.0, 255.0).astype(np.uint8)
    elif mx <= 255.0 + 1e-3:
        out_rgb = np.clip(rgb, 0.0, 255.0).astype(np.uint8)
    else:
        out_rgb = np.clip((rgb / (mx + 1e-6)) * 255.0, 0.0, 255.0).astype(np.uint8)
    if arr.shape[2] >= 4:
        a = np.asarray(arr[:, :, 3], dtype=np.float32)
        a = np.nan_to_num(a, nan=1.0, posinf=1.0, neginf=0.0)
        amx = float(np.max(a)) if a.size else 1.0
        if amx <= 1.0 + 1e-3:
            out_a = np.clip(a * 255.0, 0.0, 255.0).astype(np.uint8)
        elif amx <= 255.0 + 1e-3:
            out_a = np.clip(a, 0.0, 255.0).astype(np.uint8)
        else:
            out_a = np.clip((a / (amx + 1e-6)) * 255.0, 0.0, 255.0).astype(np.uint8)
    else:
        out_a = np.full((out_rgb.shape[0], out_rgb.shape[1]), 255, dtype=np.uint8)
    return np.ascontiguousarray(np.dstack((out_rgb, out_a)))


def _collect_runtime_light_audit(stage) -> dict:
    from pxr import Usd, UsdGeom, UsdLux

    if stage is None:
        return {"lights": [], "dominant_environment_light": None, "dominant_direct_light": None}

    watched_type_names = {
        "DomeLight",
        "DistantLight",
        "DiskLight",
        "RectLight",
        "SphereLight",
        "CylinderLight",
    }
    watched_name_tokens = ("KeyLight", "FillLight", "SunLight", "SkyLight", "EnvLight")
    lights = []
    for prim in stage.Traverse():
        if not prim.IsValid():
            continue
        type_name = prim.GetTypeName()
        prim_name = prim.GetName() or ""
        is_watched_name = any(token in prim_name for token in watched_name_tokens)
        if type_name not in watched_type_names and not is_watched_name:
            continue
        if not (type_name in watched_type_names or prim.IsA(UsdLux.Light)):
            continue
        intensity = _attr_value(prim, "inputs:intensity")
        exposure = _attr_value(prim, "inputs:exposure")
        color = _attr_value(prim, "inputs:color")
        visibility = _attr_value(prim, "visibility", "inherited")
        computed_visibility = None
        if prim.IsA(UsdGeom.Imageable):
            try:
                computed_visibility = str(UsdGeom.Imageable(prim).ComputeVisibility(Usd.TimeCode.Default()))
            except Exception:
                computed_visibility = None
        normalize = _attr_value(prim, "inputs:normalize")
        enable_color_temp = _attr_value(prim, "inputs:enableColorTemperature")
        color_temp = _attr_value(prim, "inputs:colorTemperature")
        texture_raw = _attr_value(prim, _HDRI_TEXTURE_ATTR_NAME)
        texture_file = _extract_asset_path(texture_raw) if texture_raw is not None else None
        is_active = bool(prim.IsActive())
        render_score = _light_intensity_score(intensity, exposure)
        vis_eff = computed_visibility if computed_visibility is not None else (
            str(visibility) if visibility is not None else "inherited"
        )
        lights.append(
            {
                "prim_path": prim.GetPath().pathString,
                "parent_path": prim.GetPath().GetParentPath().pathString,
                "prim_name": prim_name,
                "type": type_name,
                "active": is_active,
                "visibility": str(visibility) if visibility is not None else None,
                "computed_visibility": computed_visibility,
                "intensity": _json_safe_value(intensity),
                "exposure": _json_safe_value(exposure),
                "color": _json_safe_value(color),
                "normalize": _json_safe_value(normalize),
                "enableColorTemperature": _json_safe_value(enable_color_temp),
                "colorTemperature": _json_safe_value(color_temp),
                "texture_file": texture_file or None,
                "render_score": round(render_score, 3),
                "effective_for_render": bool(is_active and str(vis_eff) != "invisible"),
            }
        )

    env_candidates = [row for row in lights if row.get("type") == "DomeLight" and row.get("effective_for_render")]
    for row in env_candidates:
        if row.get("texture_file"):
            row["render_score"] = round(float(row.get("render_score") or 0.0) + 0.001, 3)
    env_candidates.sort(key=lambda row: float(row.get("render_score") or 0.0), reverse=True)
    dominant_env = env_candidates[0] if env_candidates else None

    direct_candidates = [
        row
        for row in lights
        if row.get("type") in {"DistantLight", "DiskLight", "RectLight", "SphereLight", "CylinderLight"}
        and row.get("effective_for_render")
    ]
    direct_candidates.sort(key=lambda row: float(row.get("render_score") or 0.0), reverse=True)
    dominant_direct = direct_candidates[0] if direct_candidates else None

    dominant_env_path = dominant_env.get("prim_path") if isinstance(dominant_env, dict) else None
    dominant_direct_path = dominant_direct.get("prim_path") if isinstance(dominant_direct, dict) else None
    for row in lights:
        row["is_primary_environment_light"] = bool(row.get("prim_path") == dominant_env_path)
        row["is_primary_direct_light"] = bool(row.get("prim_path") == dominant_direct_path)
    return {
        "lights": lights,
        "dominant_environment_light": dominant_env,
        "dominant_direct_light": dominant_direct,
    }


def _coordinate_dominant_direct_light(stage, light_audit: dict | None = None) -> dict | None:
    if not bool(cfg.get("hdri_direct_light_coordinate_enabled", True)):
        return None
    target_intensity = max(0.0, float(cfg.get("hdri_direct_light_target_intensity", 5000.0)))
    audit = light_audit if isinstance(light_audit, dict) else _collect_runtime_light_audit(stage)
    dominant_direct = audit.get("dominant_direct_light") if isinstance(audit, dict) else None
    if not isinstance(dominant_direct, dict):
        return None
    prim_path = str(dominant_direct.get("prim_path") or "")
    if not prim_path:
        return None
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    intensity_attr = prim.GetAttribute("inputs:intensity")
    if not (intensity_attr and intensity_attr.IsValid()):
        return None
    current_intensity = intensity_attr.Get()
    try:
        current_intensity_f = float(current_intensity)
    except (TypeError, ValueError):
        return None
    if current_intensity_f <= target_intensity:
        return {
            "coordinated": False,
            "reason": "dominant_direct_intensity_already_low",
            "prim_path": prim_path,
            "old_intensity": current_intensity_f,
            "new_intensity": current_intensity_f,
        }
    intensity_attr.Set(float(target_intensity))
    print(
        "[HDRI_LIGHT_COORD] "
        f"dominant_direct={prim_path} type={dominant_direct.get('type')} "
        f"old_intensity={current_intensity_f} new_intensity={target_intensity}",
        flush=True,
    )
    return {
        "coordinated": True,
        "prim_path": prim_path,
        "type": dominant_direct.get("type"),
        "old_intensity": current_intensity_f,
        "new_intensity": float(target_intensity),
    }


def _resolve_hdri_dome_light(stage):
    if stage is None:
        return None, None

    # 根 USD（scene_4diaolan_ptz.usda）在 /World/FillLight 上挂白名单 HDRI；新场景子层可能再带高强度
    # DomeLight（例如 /World/JiKeng_ChangJing01/DomeLight）。若仅按 intensity 选「主导」穹顶，会把贴图写到嵌套
    # Dome 而 FillLight 仍保持旧贴图/极低强度，导致 Web 状态与 RTSP/抓图（常走 FillLight 环境）不一致、背景发黑。
    fill_prim = stage.GetPrimAtPath("/World/FillLight")
    if fill_prim and fill_prim.IsValid() and fill_prim.GetTypeName() == "DomeLight":
        fill_attr = fill_prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        if fill_attr and fill_attr.IsValid():
            return fill_prim, fill_attr
        return fill_prim, None

    audit = _collect_runtime_light_audit(stage)
    dominant_env = audit.get("dominant_environment_light") if isinstance(audit, dict) else None
    if isinstance(dominant_env, dict):
        dominant_prim = stage.GetPrimAtPath(str(dominant_env.get("prim_path") or ""))
        if dominant_prim and dominant_prim.IsValid():
            dominant_attr = dominant_prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
            if dominant_attr and dominant_attr.IsValid():
                return dominant_prim, dominant_attr
            return dominant_prim, None

    candidates = _list_hdri_binding_candidates(stage)
    first_dome = None
    for candidate in candidates:
        prim = stage.GetPrimAtPath(candidate["prim_path"])
        if first_dome is None:
            first_dome = prim
        attr = prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        if attr and attr.IsValid():
            return prim, attr
    if first_dome is not None and first_dome.IsValid():
        attr = first_dome.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        return first_dome, attr if attr and attr.IsValid() else None
    return None, None


def _prim_path_under_dynamic_sky_root(path_str: str, root_prefix: str) -> bool:
    if not path_str or not root_prefix:
        return False
    pp = str(root_prefix).rstrip("/")
    ps = str(path_str).rstrip("/")
    if not pp.startswith("/"):
        pp = "/" + pp
    if not ps.startswith("/"):
        ps = "/" + ps
    return ps == pp or ps.startswith(pp + "/")


def _collect_hdri_environment_dome_paths(stage, dynamic_sky_root: str) -> list[str]:
    """主场景 HDRI 穹顶（与 _apply_hdri_texture 同步策略一致），不包含 DynamicSkyRoot 子树。"""
    out: list[str] = []
    seen: set[str] = set()
    dsr = str(dynamic_sky_root or "").strip() or "/World/DynamicSkyRoot"

    fill_prim = stage.GetPrimAtPath("/World/FillLight")
    if fill_prim and fill_prim.IsValid() and fill_prim.GetTypeName() == "DomeLight":
        p = fill_prim.GetPath().pathString
        if not _prim_path_under_dynamic_sky_root(p, dsr) and p not in seen:
            seen.add(p)
            out.append(p)

    prim0, _ = _resolve_hdri_dome_light(stage)
    primary_path_str = prim0.GetPath().pathString if prim0 and prim0.IsValid() else ""

    for extra in stage.Traverse():
        if not extra.IsValid() or extra.GetTypeName() != "DomeLight":
            continue
        p = extra.GetPath().pathString
        if _prim_path_under_dynamic_sky_root(p, dsr) or p in seen:
            continue
        ex_attr = extra.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        include = False
        if ex_attr and ex_attr.IsValid():
            prev_ex = _extract_asset_path(ex_attr.Get())
            low = str(prev_ex).lower()
            if _looks_like_hdri_asset_path(low):
                include = True
        elif primary_path_str and p == primary_path_str:
            include = True
        if include:
            seen.add(p)
            out.append(p)
    return out


def _read_dome_energy_state(prim) -> dict:
    from pxr import Usd, UsdGeom

    snap: dict = {
        "intensity_attr": None,
        "intensity": None,
        "exposure_attr": None,
        "exposure": None,
        "visibility": None,
    }
    for name in ("inputs:intensity", "intensity"):
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            snap["intensity_attr"] = name
            try:
                snap["intensity"] = float(attr.Get())
            except (TypeError, ValueError):
                snap["intensity"] = None
            break
    for name in ("inputs:exposure", "exposure"):
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            snap["exposure_attr"] = name
            try:
                snap["exposure"] = float(attr.Get())
            except (TypeError, ValueError):
                snap["exposure"] = None
            break
    if prim.IsA(UsdGeom.Imageable):
        img = UsdGeom.Imageable(prim)
        try:
            # 未单独打 visibility 属性时 Get() 常为「空」，但计算结果仍为 inherited；与 _apply_dome_disabled_visual 配对时必须可还原
            snap["visibility"] = str(img.ComputeVisibility(Usd.TimeCode.Default()))
        except Exception:
            snap["visibility"] = None
    return snap


def _apply_dome_disabled_visual(prim) -> None:
    from pxr import UsdGeom

    for name in ("inputs:intensity", "intensity"):
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            attr.Set(0.0)
            break
    for name in ("inputs:exposure", "exposure"):
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            attr.Set(-20.0)
            break
    if prim.IsA(UsdGeom.Imageable):
        va = UsdGeom.Imageable(prim).GetVisibilityAttr()
        if va and va.IsValid():
            va.Set(UsdGeom.Tokens.invisible)


def _restore_dome_energy_state(prim, snap: dict) -> None:
    from pxr import Tf, UsdGeom

    iattr = snap.get("intensity_attr")
    if isinstance(iattr, str) and iattr:
        a = prim.GetAttribute(iattr)
        if a and a.IsValid() and snap.get("intensity") is not None:
            try:
                a.Set(float(snap["intensity"]))
            except (TypeError, ValueError):
                pass
    eattr = snap.get("exposure_attr")
    if isinstance(eattr, str) and eattr:
        a = prim.GetAttribute(eattr)
        if a and a.IsValid() and snap.get("exposure") is not None:
            try:
                a.Set(float(snap["exposure"]))
            except (TypeError, ValueError):
                pass
    vis = snap.get("visibility")
    if prim.IsA(UsdGeom.Imageable):
        va = UsdGeom.Imageable(prim).GetVisibilityAttr()
        if va and va.IsValid():
            if vis is not None:
                try:
                    va.Set(Tf.Token(str(vis)))
                except Exception:
                    pass
            else:
                try:
                    va.Set(UsdGeom.Tokens.inherited)
                except Exception:
                    pass


def _repair_stuck_invisible_environment_domelights(stage) -> None:
    """Dynamic Sky 互斥路径会把 HDRI 穹顶设为 invisible；旧版快照若未记录 visibility，恢复能量后仍会永远不参与渲染，RTSP/快照近纯黑。"""
    from pxr import Usd, UsdGeom

    dsky = str(cfg.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot")
    try:
        paths = _collect_hdri_environment_dome_paths(stage, dsky)
    except Exception:
        return
    for path_str in paths:
        prim = stage.GetPrimAtPath(str(path_str))
        if not prim or not prim.IsValid() or not prim.IsA(UsdGeom.Imageable):
            continue
        img = UsdGeom.Imageable(prim)
        try:
            cv = str(img.ComputeVisibility(Usd.TimeCode.Default()))
        except Exception:
            continue
        if cv != str(UsdGeom.Tokens.invisible):
            continue
        snap = _read_dome_energy_state(prim)
        try:
            in_v = float(snap["intensity"]) if snap.get("intensity") is not None else 0.0
        except (TypeError, ValueError):
            in_v = 0.0
        tex = ""
        ta = prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        if ta and ta.IsValid():
            tex = _extract_asset_path(ta.Get()) or ""
        if not (in_v > 1.0 or tex):
            continue
        va = img.GetVisibilityAttr()
        if va and va.IsValid():
            try:
                va.Set(UsdGeom.Tokens.inherited)
                print(
                    "[hdri-dome-visibility-repair] "
                    f"path={path_str} intensity={snap.get('intensity')} "
                    f"exposure={snap.get('exposure')} had_texture={bool(tex)}",
                    flush=True,
                )
            except Exception:
                pass


def _repair_domelight_energy_stuck_after_env_disable(stage) -> None:
    """Dynamic Sky 互斥写入的「关穹顶视觉」若未完整恢复，会长期 intensity≈0 且 exposure≈-20，抓图/RTSP 近纯黑。"""
    from pxr import UsdGeom

    if str(_scene_state.get("environment_mode") or "hdri").strip().lower() == "dynamic_sky":
        return
    dsky = str(cfg.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot")
    try:
        paths = _collect_hdri_environment_dome_paths(stage, dsky)
    except Exception:
        return
    try:
        fill_nominal_exp = float(cfg.get("hdri_fill_exposure_nominal", 1.0))
    except (TypeError, ValueError):
        fill_nominal_exp = 1.0
    for path_str in paths:
        prim = stage.GetPrimAtPath(str(path_str))
        if not prim or not prim.IsValid() or prim.GetTypeName() != "DomeLight":
            continue
        ta = prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        if not (ta and ta.IsValid()):
            continue
        tex = _extract_asset_path(ta.Get()) or ""
        if not tex:
            continue
        snap = _read_dome_energy_state(prim)
        try:
            ex_v = float(snap["exposure"]) if snap.get("exposure") is not None else None
        except (TypeError, ValueError):
            ex_v = None
        try:
            in_v = float(snap["intensity"]) if snap.get("intensity") is not None else None
        except (TypeError, ValueError):
            in_v = None
        stuck_ex = ex_v is not None and abs(ex_v + 20.0) < 0.05
        stuck_in = in_v is not None and abs(in_v) < 1e-2
        if not (stuck_ex or stuck_in):
            continue
        base = None
        with _scene_lock:
            bl = _scene_state.get("rule11_hdri_dome_energy_baseline")
            if isinstance(bl, dict):
                base = bl.get(path_str)
        if isinstance(base, dict) and base.get("intensity") is not None:
            _restore_dome_energy_state(prim, base)
            print(
                "[hdri-dome-energy-repair] restored_from_rule11_baseline "
                f"path={path_str}",
                flush=True,
            )
            continue
        if path_str == "/World/FillLight":
            for ex_name in ("inputs:exposure", "exposure"):
                ea = prim.GetAttribute(ex_name)
                if ea and ea.IsValid():
                    try:
                        ea.Set(float(fill_nominal_exp))
                    except Exception:
                        pass
                    break
            for in_name in ("inputs:intensity", "intensity"):
                ia = prim.GetAttribute(in_name)
                if ia and ia.IsValid():
                    try:
                        ia.Set(650.0)
                    except Exception:
                        pass
                    break
            print(
                "[hdri-dome-energy-repair] fill_nominal_defaults "
                f"path={path_str} exposure={fill_nominal_exp}",
                flush=True,
            )
            continue
        for ex_name in ("inputs:exposure", "exposure"):
            ea = prim.GetAttribute(ex_name)
            if ea and ea.IsValid():
                try:
                    ea.Set(0.0)
                except Exception:
                    pass
                break
        for in_name in ("inputs:intensity", "intensity"):
            ia = prim.GetAttribute(in_name)
            if ia and ia.IsValid():
                try:
                    ia.Set(1000.0)
                except Exception:
                    pass
                break
        if prim.IsA(UsdGeom.Imageable):
            va = UsdGeom.Imageable(prim).GetVisibilityAttr()
            if va and va.IsValid():
                try:
                    va.Set(UsdGeom.Tokens.inherited)
                except Exception:
                    pass
        print(
            "[hdri-dome-energy-repair] nested_dome_nominal_defaults "
            f"path={path_str}",
            flush=True,
        )


def _sync_hdri_environment_dome_repairs(stage) -> None:
    _repair_stuck_invisible_environment_domelights(stage)
    _repair_domelight_energy_stuck_after_env_disable(stage)


def _replicator_flush_after_lighting_usd_write() -> None:
    """HDRI/环境类 USD 写入后多步进 Replicator，减少「状态已写、首帧仍黑」的时序窗口。"""
    try:
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
        sim_app.update()
    except Exception as exc:
        print(f"[replicator-flush] lighting_write_flush failed: {exc}", flush=True)


def _mount_dynamic_sky_usd(stage, root_path: str, asset_abs: str) -> dict:
    ap = os.path.normpath(str(asset_abs or "").strip())
    if not ap or not os.path.isfile(ap):
        return {"ok": False, "error": f"preset not found: {ap}"}

    rp = str(root_path or "").strip() or "/World/DynamicSkyRoot"
    prim = stage.GetPrimAtPath(rp)
    if not prim.IsValid():
        prim = stage.DefinePrim(rp, "Xform")
    if not prim.IsValid():
        return {"ok": False, "error": f"failed to define prim {rp}"}

    refs = prim.GetReferences()
    refs.ClearReferences()
    refs.AddReference(ap)
    prim.SetActive(True)
    return {"ok": True, "root_prim_path": rp, "preset_path": ap}


def _deactivate_dynamic_sky_root(stage, root_path: str) -> None:
    rp = str(root_path or "").strip() or "/World/DynamicSkyRoot"
    prim = stage.GetPrimAtPath(rp)
    if prim and prim.IsValid():
        prim.SetActive(False)


def _restore_hdri_dome_env_from_snapshot(stage, snap: dict | None) -> None:
    if not isinstance(snap, dict) or not snap:
        return
    for path_str, saved in snap.items():
        if not isinstance(saved, dict):
            continue
        prim = stage.GetPrimAtPath(str(path_str))
        if not prim or not prim.IsValid():
            continue
        _restore_dome_energy_state(prim, saved)


def _environment_public_status(stage=None) -> dict:
    stage = stage or omni.usd.get_context().get_stage()
    with _scene_lock:
        mode = str(_scene_state.get("environment_mode") or "hdri")
        dy_en = bool(_scene_state.get("dynamic_sky_enabled"))
        preset = str(_scene_state.get("dynamic_sky_preset_path") or "")
        root = str(_scene_state.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot")
        mount_ok = bool(_scene_state.get("dynamic_sky_mount_ok"))
        last_err = _scene_state.get("dynamic_sky_last_error")
        last_at = _scene_state.get("dynamic_sky_last_action_at")
        mounted_preset = _scene_state.get("dynamic_sky_mounted_preset_path")
        snap = _scene_state.get("hdri_env_exclude_snapshot")

    prim = stage.GetPrimAtPath(root) if stage else None
    prim_valid = bool(prim and prim.IsValid())
    prim_active = bool(prim_valid and prim.IsActive())
    snap_paths = list(snap.keys()) if isinstance(snap, dict) else []

    stream = {}
    with _stream_diag_lock:
        stream = {
            "rtsp_enabled": bool(_STREAM_DIAG.get("rtsp_enabled")),
            "ffmpeg_alive": bool(_STREAM_DIAG.get("ffmpeg_alive")),
        }

    return {
        "environment_mode": mode,
        "dynamic_sky_effective": bool(str(mode).strip().lower() == "dynamic_sky" and dy_en),
        "dynamic_sky_enabled": dy_en,
        "dynamic_sky_preset_path": preset,
        "dynamic_sky_root_prim": root,
        "dynamic_sky_mount_ok": mount_ok,
        "dynamic_sky_mounted_preset_path": mounted_preset,
        "dynamic_sky_root_exists": prim_valid,
        "dynamic_sky_root_active": prim_active,
        "preset_file_exists": bool(preset and os.path.isfile(preset)),
        "hdri_environment_prims_disabled": snap_paths,
        "hdri_env_mutually_excluded": bool(str(mode).strip().lower() == "dynamic_sky" and dy_en and len(snap_paths) > 0),
        "dynamic_sky_last_error": last_err,
        "dynamic_sky_last_action_at": last_at,
        "stream": stream,
    }


def _apply_environment_request_main_thread(stage, request: dict | None) -> dict:
    """在主线程执行：根据 environment_mode / dynamic_sky_enabled 挂载或卸载 ClearSky preset，并与 HDRI 穹顶互斥。"""
    req = request if isinstance(request, dict) else {}
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    with _scene_lock:
        cur_mode = str(_scene_state.get("environment_mode") or "hdri").strip().lower()
        cur_en = bool(_scene_state.get("dynamic_sky_enabled"))
        preset0 = str(_scene_state.get("dynamic_sky_preset_path") or "").strip()
        root0 = str(_scene_state.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot").strip()

    mode_in = req.get("environment_mode")
    if mode_in is not None:
        m = str(mode_in).strip().lower()
        if m not in ("hdri", "dynamic_sky"):
            return {"ok": False, "error": f"invalid environment_mode: {mode_in!r}", "timestamp": ts}
        new_mode = m
    else:
        new_mode = cur_mode

    if "dynamic_sky_enabled" in req:
        new_en = bool(req.get("dynamic_sky_enabled"))
    else:
        if mode_in is not None and new_mode == "dynamic_sky":
            new_en = True
        elif mode_in is not None and new_mode == "hdri":
            new_en = False
        else:
            new_en = cur_en

    path_override = req.get("dynamic_sky_preset_path")
    if isinstance(path_override, str) and path_override.strip():
        po = path_override.strip()
        preset_use = os.path.normpath(po) if os.path.isabs(po) else os.path.normpath(_resolve_path(po))
        with _scene_lock:
            _scene_state["dynamic_sky_preset_path"] = preset_use
    else:
        preset_use = preset0

    root_override = req.get("dynamic_sky_root_prim")
    if isinstance(root_override, str) and root_override.strip():
        rpv = root_override.strip()
        if not rpv.startswith("/"):
            rpv = "/" + rpv
        with _scene_lock:
            _scene_state["dynamic_sky_root_prim"] = rpv
        root_use = rpv
    else:
        root_use = root0

    desired_dynamic = bool(str(new_mode).strip().lower() == "dynamic_sky" and new_en)
    err: str | None = None
    mount_detail: dict | None = None

    try:
        if desired_dynamic:
            if not preset_use or not os.path.isfile(preset_use):
                err = f"dynamic_sky preset missing on disk: {preset_use}"
            else:
                with _scene_lock:
                    had_snap = isinstance(_scene_state.get("hdri_env_exclude_snapshot"), dict) and bool(
                        _scene_state.get("hdri_env_exclude_snapshot")
                    )
                paths = _collect_hdri_environment_dome_paths(stage, root_use)
                if not had_snap:
                    snap_new: dict[str, dict] = {}
                    for p in paths:
                        prim = stage.GetPrimAtPath(p)
                        if prim and prim.IsValid():
                            snap_new[p] = _read_dome_energy_state(prim)
                    with _scene_lock:
                        _scene_state["hdri_env_exclude_snapshot"] = snap_new
                    snap_for_restore = snap_new
                else:
                    with _scene_lock:
                        snap_for_restore = dict(_scene_state.get("hdri_env_exclude_snapshot") or {})
                for p in paths:
                    prim = stage.GetPrimAtPath(p)
                    if prim and prim.IsValid():
                        _apply_dome_disabled_visual(prim)
                mount_detail = _mount_dynamic_sky_usd(stage, root_use, preset_use)
                if not mount_detail.get("ok"):
                    err = str(mount_detail.get("error") or "mount failed")
                    _restore_hdri_dome_env_from_snapshot(stage, snap_for_restore)
                    _sync_hdri_environment_dome_repairs(stage)
                    _replicator_flush_after_lighting_usd_write()
                    with _scene_lock:
                        _scene_state["hdri_env_exclude_snapshot"] = None
                else:
                    with _scene_lock:
                        _scene_state["environment_mode"] = new_mode
                        _scene_state["dynamic_sky_enabled"] = new_en
                        _scene_state["dynamic_sky_mount_ok"] = True
                        _scene_state["dynamic_sky_mounted_preset_path"] = preset_use
                        _scene_state["dynamic_sky_last_error"] = None
                        _scene_state["dynamic_sky_last_action_at"] = ts
        else:
            _deactivate_dynamic_sky_root(stage, root_use)
            with _scene_lock:
                snap = _scene_state.get("hdri_env_exclude_snapshot")
            _restore_hdri_dome_env_from_snapshot(stage, snap if isinstance(snap, dict) else {})
            _sync_hdri_environment_dome_repairs(stage)
            _replicator_flush_after_lighting_usd_write()
            with _scene_lock:
                _scene_state["hdri_env_exclude_snapshot"] = None
                _scene_state["environment_mode"] = new_mode
                _scene_state["dynamic_sky_enabled"] = new_en
                _scene_state["dynamic_sky_mount_ok"] = False
                _scene_state["dynamic_sky_mounted_preset_path"] = None
                _scene_state["dynamic_sky_last_error"] = None
                _scene_state["dynamic_sky_last_action_at"] = ts
    except Exception as exc:
        err = str(exc)

    if err:
        with _scene_lock:
            _scene_state["dynamic_sky_last_error"] = err
            if desired_dynamic:
                _scene_state["dynamic_sky_mount_ok"] = False
        _refresh_hdri_backend_status_from_stage(stage)
        _invalidate_scene_state_http_cache()
        _invalidate_status_http_cache()
        return {
            "ok": False,
            "error": err,
            "timestamp": ts,
            "environment": _environment_public_status(stage),
            "state": _scene_state_snapshot(stage),
        }

    _refresh_hdri_backend_status_from_stage(stage)
    _invalidate_scene_state_http_cache()
    _invalidate_hdri_state_http_cache()
    _invalidate_status_http_cache()
    return {
        "ok": True,
        "timestamp": ts,
        "environment": _environment_public_status(stage),
        "state": _scene_state_snapshot(stage),
        "mount": mount_detail,
    }


def _describe_hdri_state(stage=None, random_cfg=None) -> dict:
    stage = stage or omni.usd.get_context().get_stage()
    cfg = random_cfg if isinstance(random_cfg, dict) else _default_random_config()
    candidates = _sanitize_hdri_candidates(
        cfg.get("hdri_candidates") if isinstance(cfg, dict) else None,
        fallback=_default_hdri_candidates(),
    )
    existing_candidates = [path for path in candidates if os.path.isfile(path)]
    missing_candidates = [path for path in candidates if path not in existing_candidates]
    info = {
        "hdri_dome_light_path": None,
        "hdri_texture_attr": _HDRI_TEXTURE_ATTR_NAME,
        "current_hdri": None,
        "current_hdri_basename": None,
        "hdri_candidates": candidates,
        "hdri_candidates_existing": existing_candidates,
        "hdri_candidates_missing": missing_candidates,
        "hdri_candidate_file_report": _hdri_candidate_file_report(),
        "hdri_binding_candidates": _list_hdri_binding_candidates(stage),
    }
    prim, attr = _resolve_hdri_dome_light(stage)
    if prim is None or not prim.IsValid():
        return info

    current_hdri = _extract_asset_path(attr.Get()) if attr and attr.IsValid() else ""
    info["hdri_dome_light_path"] = prim.GetPath().pathString
    info["current_hdri"] = current_hdri or None
    info["current_hdri_basename"] = os.path.basename(current_hdri) if current_hdri else None
    return info


def _default_hdri_backend_status() -> dict:
    return {
        "current_hdri_name": None,
        "current_hdri_path": None,
        "current_group": None,
        "current_group_id": None,
        "updated_at": None,
        "confirmed_by_backend": False,
        "hdri_dome_light_path": None,
        "hdri_texture_attr": _HDRI_TEXTURE_ATTR_NAME,
        "actual_binding_target": None,
        "other_light_evidence": [],
        "dominant_environment_light": None,
        "dominant_direct_light": None,
    }


def _sanitize_hdri_backend_status(raw) -> dict:
    base = _default_hdri_backend_status()
    data = raw if isinstance(raw, dict) else {}
    return {
        "current_hdri_name": str(data.get("current_hdri_name") or "") or None,
        "current_hdri_path": _normalize_hdri_path(data.get("current_hdri_path")) or None,
        "current_group": str(data.get("current_group") or "") or None,
        "current_group_id": str(data.get("current_group_id") or "") or None,
        "updated_at": str(data.get("updated_at") or "") or None,
        "confirmed_by_backend": bool(data.get("confirmed_by_backend")),
        "hdri_dome_light_path": str(data.get("hdri_dome_light_path") or "") or None,
        "hdri_texture_attr": str(data.get("hdri_texture_attr") or base["hdri_texture_attr"]) or base["hdri_texture_attr"],
        "actual_binding_target": str(data.get("actual_binding_target") or "") or None,
        "other_light_evidence": list(data.get("other_light_evidence") or []),
        "dominant_environment_light": data.get("dominant_environment_light") if isinstance(data.get("dominant_environment_light"), dict) else None,
        "dominant_direct_light": data.get("dominant_direct_light") if isinstance(data.get("dominant_direct_light"), dict) else None,
    }


def _list_other_light_evidence(stage) -> list[dict]:
    audit = _collect_runtime_light_audit(stage)
    if not isinstance(audit, dict):
        return []
    return list(audit.get("lights") or [])


def _store_hdri_backend_status(status: dict | None = None) -> dict:
    cached = _sanitize_hdri_backend_status(status)
    with _scene_lock:
        _scene_state["hdri_backend_status"] = cached
    _invalidate_hdri_state_http_cache()
    return cached


def _refresh_hdri_backend_status_from_stage(stage, control_state: dict | None = None, updated_at: str | None = None) -> dict:
    hdri_state = _describe_hdri_state(stage)
    light_audit = _collect_runtime_light_audit(stage)
    groups = _build_hdri_groups()
    actual_group_id, actual_entry = _match_hdri_entry_by_path(hdri_state.get("current_hdri"))
    actual_group = groups.get(actual_group_id) if actual_group_id else None
    dome_path = hdri_state.get("hdri_dome_light_path")
    texture_attr = hdri_state.get("hdri_texture_attr") or _HDRI_TEXTURE_ATTR_NAME
    binding_target = f"{dome_path}.{texture_attr}" if dome_path and texture_attr else None
    control = _sanitize_hdri_control_state(control_state)
    if not actual_group and control.get("current_group_id") in groups:
        actual_group = groups.get(control.get("current_group_id"))
        actual_group_id = actual_group.get("id") if isinstance(actual_group, dict) else None
    # 舞台上尚未解析到纹理路径时，用 Web 侧当前选中条目兜底展示（避免 UI 长期「未加载」）
    display_path = hdri_state.get("current_hdri")
    display_name = hdri_state.get("current_hdri_basename")
    if not display_path:
        sel = _get_hdri_entry(
            control.get("current_group_id"),
            (control.get("selected_by_group") or {}).get(control.get("current_group_id")),
        )
        if isinstance(sel, dict) and sel.get("path"):
            display_path = str(sel.get("path") or "").strip() or None
            display_name = sel.get("name") or (os.path.basename(display_path) if display_path else None)
    if not actual_entry and display_path:
        ag, ae = _match_hdri_entry_by_path(display_path)
        if ag and isinstance(ae, dict):
            actual_group_id, actual_entry = ag, ae
            actual_group = groups.get(actual_group_id)
    return _store_hdri_backend_status(
        {
            "current_hdri_name": display_name,
            "current_hdri_path": display_path,
            "current_group": actual_group.get("name") if isinstance(actual_group, dict) else None,
            "current_group_id": actual_group_id,
            "updated_at": updated_at or time.strftime("%Y-%m-%d %H:%M:%S"),
            "confirmed_by_backend": bool(hdri_state.get("current_hdri")),
            "hdri_dome_light_path": dome_path,
            "hdri_texture_attr": texture_attr,
            "actual_binding_target": binding_target,
            "other_light_evidence": list(light_audit.get("lights") or []),
            "dominant_environment_light": light_audit.get("dominant_environment_light"),
            "dominant_direct_light": light_audit.get("dominant_direct_light"),
        }
    )


def _format_timing_ts(ts: float) -> str:
    local = time.localtime(ts)
    return time.strftime("%Y-%m-%d %H:%M:%S", local) + f".{int((ts % 1) * 1000):03d}"


def _log_hdri_timing(
    route: str,
    start_ts: float,
    *,
    end_ts: float | None = None,
    timeout: bool = False,
    exception: Exception | None = None,
    cache_hit: bool | None = None,
    phase: str | None = None,
) -> None:
    finish = end_ts if end_ts is not None else time.time()
    payload = {
        "route": route,
        "phase": phase or "request",
        "start_ts": _format_timing_ts(start_ts),
        "end_ts": _format_timing_ts(finish),
        "elapsed_ms": round((finish - start_ts) * 1000.0, 2),
        "timeout": bool(timeout),
        "exception": str(exception) if exception else None,
        "whether_main_thread": threading.current_thread() is threading.main_thread(),
        "whether_cache_hit": cache_hit,
    }
    print(f"[HDRI_TIMING] {json.dumps(payload, ensure_ascii=False)}", flush=True)


def _apply_hdri_texture(stage, hdri_path: str) -> dict:
    from pxr import Sdf

    normalized_path = _normalize_hdri_path(hdri_path)
    if not normalized_path:
        raise ValueError("hdri_path is required")
    if not os.path.isfile(normalized_path):
        raise FileNotFoundError(f"HDRI file not found: {normalized_path}")

    if not _environment_allows_hdri():
        raise RuntimeError(
            "HDRI apply blocked while environment_mode=dynamic_sky (mutually exclusive with Dynamic Sky)"
        )

    prim, attr = _resolve_hdri_dome_light(stage)
    if prim is None or not prim.IsValid():
        raise RuntimeError("no DomeLight prim found for HDRI texture")

    previous_hdri = _extract_asset_path(attr.Get()) if attr and attr.IsValid() else ""
    if not (attr and attr.IsValid()):
        attr = prim.CreateAttribute(_HDRI_TEXTURE_ATTR_NAME, Sdf.ValueTypeNames.Asset)
    attr.Set(Sdf.AssetPath(normalized_path))
    # 嵌套场景里其它已带 HDR 贴图的 DomeLight 仍可能主导 GI；与主 prim 同步同一张图，避免「接口已切、画面仍像旧环境」。
    primary_path_str = prim.GetPath().pathString
    for extra in stage.Traverse():
        if not extra.IsValid() or extra.GetTypeName() != "DomeLight":
            continue
        if extra.GetPath().pathString == primary_path_str:
            continue
        ex_attr = extra.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
        if not (ex_attr and ex_attr.IsValid()):
            continue
        prev_ex = _extract_asset_path(ex_attr.Get())
        if not prev_ex:
            continue
        low = str(prev_ex).lower()
        if not _looks_like_hdri_asset_path(low):
            continue
        ex_attr.Set(Sdf.AssetPath(normalized_path))
    # FillLight 在部分合成/渲染路径下 intensity 会落到极低，环境贴图几乎不可见。
    if primary_path_str == "/World/FillLight":
        int_attr = prim.GetAttribute("inputs:intensity")
        if int_attr and int_attr.IsValid():
            try:
                cur_i = float(int_attr.Get())
            except (TypeError, ValueError):
                cur_i = 0.0
            if cur_i < 100.0:
                int_attr.Set(650.0)
        # 始终强制使用 ptz_config.yaml 中配置的标称曝光，避免在 RTXRealTime 下几何体欠曝发黑
        try:
            fill_exp_nominal = float(cfg.get("hdri_fill_exposure_nominal", 1.0))
        except (TypeError, ValueError):
            fill_exp_nominal = 1.0
        for ex_name in ("inputs:exposure", "exposure"):
            ea = prim.GetAttribute(ex_name)
            if ea and ea.IsValid():
                try:
                    ea.Set(fill_exp_nominal)
                except Exception:
                    pass
                break
        for in_name in ("inputs:intensity", "intensity"):
            ia = prim.GetAttribute(in_name)
            if ia and ia.IsValid():
                try:
                    ia.Set(650.0)
                except Exception:
                    pass
                break
    amb = stage.GetPrimAtPath("/World/AmbientBoost")
    if amb and amb.IsValid() and amb.GetTypeName() == "DomeLight":
        for ex_name in ("inputs:exposure", "exposure"):
            ea = amb.GetAttribute(ex_name)
            if ea and ea.IsValid():
                try:
                    ea.Set(fill_exp_nominal)
                except Exception:
                    pass
                break
    current_after_set = _extract_asset_path(attr.Get()) if attr and attr.IsValid() else ""
    candidate_report = _hdri_candidate_file_report()
    print(
        "[HDRI] apply "
        f"prim={prim.GetPath().pathString} attr={_HDRI_TEXTURE_ATTR_NAME} "
        f"old={previous_hdri or '<empty>'} new={normalized_path} after_set={current_after_set or '<empty>'}",
        flush=True,
    )
    print(f"[HDRI] whitelist_exists={json.dumps(candidate_report, ensure_ascii=False)}", flush=True)
    _replicator_flush_after_lighting_usd_write()
    _sync_hdri_environment_dome_repairs(stage)
    try:
        _rule11_snapshot_hdri_dome_energy_baseline(stage)
    except Exception as _bl_exc:
        print(f"[rule11-baseline] refresh after HDRI apply failed: {_bl_exc}", flush=True)
    _pending_hdri_audits.append(
        {
            "due_at": time.monotonic() + 3.0,
            "prim_path": prim.GetPath().pathString,
            "expected_hdri": normalized_path,
            "action": "apply_hdri_texture",
        }
    )
    return {
        "hdri_dome_light_path": prim.GetPath().pathString,
        "hdri_texture_attr": _HDRI_TEXTURE_ATTR_NAME,
        "previous_hdri": previous_hdri or None,
        "previous_hdri_basename": os.path.basename(previous_hdri) if previous_hdri else None,
        "current_hdri": normalized_path,
        "current_hdri_basename": os.path.basename(normalized_path),
        "after_set_hdri": current_after_set or None,
        "hdri_candidate_file_report": candidate_report,
    }


def _ensure_hdri_render_consistency(stage) -> None:
    if stage is None or not _environment_allows_hdri():
        return
    prim, attr = _resolve_hdri_dome_light(stage)
    if prim is None or not prim.IsValid():
        return
    current_hdri = _normalize_hdri_path(_extract_asset_path(attr.Get()) if attr and attr.IsValid() else "")
    if not current_hdri:
        return

    reason = None
    prim_path = prim.GetPath().pathString
    snap = _read_dome_energy_state(prim)
    try:
        cur_intensity = float(snap.get("intensity")) if snap.get("intensity") is not None else 0.0
    except (TypeError, ValueError):
        cur_intensity = 0.0
    if prim_path == "/World/FillLight" and cur_intensity < 100.0:
        reason = f"filllight_underpowered={cur_intensity}"

    if reason is None:
        for extra in stage.Traverse():
            if not extra.IsValid() or extra.GetTypeName() != "DomeLight":
                continue
            extra_path = extra.GetPath().pathString
            if extra_path == prim_path:
                continue
            ex_attr = extra.GetAttribute(_HDRI_TEXTURE_ATTR_NAME)
            if not (ex_attr and ex_attr.IsValid()):
                continue
            extra_hdri = _extract_asset_path(ex_attr.Get())
            if not _looks_like_hdri_asset_path(extra_hdri):
                continue
            if _normalize_hdri_path(extra_hdri) != current_hdri:
                reason = f"mismatched_dome={extra_path}"
                break

    if reason is None:
        return

    result = _apply_hdri_texture(stage, current_hdri)
    print(
        "[hdri-consistency] reapplied_current_hdri "
        f"reason={reason} current={result.get('current_hdri') or current_hdri}",
        flush=True,
    )


def _stage_meters_per_unit(stage) -> float:
    try:
        meters_per_unit = float(_UsdGeom.GetStageMetersPerUnit(stage))
        if meters_per_unit > 0.0:
            return meters_per_unit
    except Exception:
        pass
    return 1.0


def _stage_units_to_cm(stage, stage_units: float) -> float:
    return float(stage_units) * _stage_meters_per_unit(stage) * 100.0


def _cm_to_stage_units(stage, height_cm: float) -> float:
    return (float(height_cm) / 100.0) / _stage_meters_per_unit(stage)


def _diaolan_candidate_meta(path_value: str, target_prim_path=None, workers_max=0) -> dict:
    path_value = str(path_value or "").strip()
    parts = [x for x in path_value.strip("/").split("/") if x]
    cluster_name = parts[1] if len(parts) >= 2 and parts[0] == "World" else ""
    instance_name = parts[2] if len(parts) >= 3 and parts[0] == "World" else (os.path.basename(path_value.rstrip("/")) or path_value)
    cluster_path = f"/World/{cluster_name}" if cluster_name else ""
    cluster_label = cluster_name or "Diaolan"
    label = f"{cluster_label} / {instance_name}" if cluster_label and instance_name else (instance_name or path_value)
    try:
        workers_max_i = int(workers_max or 0)
    except Exception:
        workers_max_i = 0
    return {
        "path": path_value,
        "target_prim_path": str(target_prim_path or "").strip() or None,
        "label": label,
        "workers_max": workers_max_i,
        "cluster_path": cluster_path or None,
        "cluster_label": cluster_label,
        "instance_name": instance_name,
    }


def _scene_state_snapshot(stage=None, *, runtime_lock_timeout: float | None = None) -> dict:
    stage = stage or omni.usd.get_context().get_stage()
    runtime = _scene_state_runtime_snapshot(lock_timeout=runtime_lock_timeout)
    scanned = scan_diaolan_prims(stage)
    diaolan_candidates = []
    for item in scanned:
        path_value = str(item.get("path") or "").strip()
        if not path_value:
            continue
        diaolan_candidates.append(_diaolan_candidate_meta(path_value, item.get("group1"), len(item.get("persons") or [])))
    scanned_paths = [str(d.get("path") or "").strip() for d in scanned if str(d.get("path") or "").strip()]
    all_paths_snap = list(runtime.get("all_diaolan_paths") or [])
    if not all_paths_snap:
        all_paths_snap = list(scanned_paths)
    else:
        seen = set(all_paths_snap)
        for p in scanned_paths:
            if p not in seen:
                all_paths_snap.append(p)
                seen.add(p)
    cand_paths_set = {c["path"] for c in diaolan_candidates}
    for p in all_paths_snap:
        pv = str(p).strip()
        if not pv or pv in cand_paths_set:
            continue
        g1 = None
        workers_max = 0
        if stage is not None:
            try:
                ht, asm = resolve_diaolan_height_and_assembly(stage, pv.rstrip("/"))
                if ht:
                    g1 = str(ht).strip() or None
                wlist = _collect_worker_prims(stage, asm) if asm else []
                if not wlist and ht:
                    wlist = _collect_worker_prims(stage, ht)
                workers_max = len(wlist)
            except Exception:
                pass
        diaolan_candidates.append(_diaolan_candidate_meta(pv, g1, workers_max))
        cand_paths_set.add(pv)
    sel_snap = str(runtime.get("selected_diaolan_path") or runtime.get("active_diaolan_path") or "").strip() or None
    if sel_snap and sel_snap not in cand_paths_set:
        pv = sel_snap
        g1 = None
        workers_max = 0
        if stage is not None:
            try:
                ht, asm = resolve_diaolan_height_and_assembly(stage, pv.rstrip("/"))
                if ht:
                    g1 = str(ht).strip() or None
                wlist = _collect_worker_prims(stage, asm) if asm else []
                if not wlist and ht:
                    wlist = _collect_worker_prims(stage, ht)
                workers_max = len(wlist)
            except Exception:
                pass
        diaolan_candidates.append(_diaolan_candidate_meta(pv, g1, workers_max))
        cand_paths_set.add(pv)
    workers_max_for_selected = 0
    resolved_height_for_selected = None
    if sel_snap:
        for c in diaolan_candidates:
            if c.get("path") == sel_snap:
                workers_max_for_selected = int(c.get("workers_max") or 0)
                resolved_height_for_selected = str(c.get("target_prim_path") or "").strip() or None
                break
    if resolved_height_for_selected is None and sel_snap and stage is not None:
        try:
            ht, _asm = resolve_diaolan_height_and_assembly(stage, sel_snap.rstrip("/"))
            resolved_height_for_selected = str(ht or "").strip() or None
        except Exception:
            resolved_height_for_selected = None
    current_target_path = resolved_height_for_selected or (_GONDOLA_PRIM or None)
    eff_look = _effective_lookat_target_prim_path(stage) if stage is not None else None
    building_context = _context_lookat_selection(stage) if stage is not None else {"prim_path": None, "reason": "stage_unavailable"}
    gondola_renderable_paths = list(runtime.get("gondola_renderable_paths") or [])
    gondola_visible_renderable_paths = list(runtime.get("gondola_visible_renderable_paths") or [])
    gondola_hidden_paths = list(runtime.get("gondola_hidden_paths") or [])
    gondola_renderable_debug = list(runtime.get("gondola_renderable_debug") or [])
    out = {
        "gondola_height_cm": _stage_units_to_cm(stage, runtime["gondola_y"]),
        "workers": runtime["workers"],
        "selected_diaolan_path": sel_snap,
        "active_diaolan_path": runtime["active_diaolan_path"] or None,
        "active_diaolan_semantics": (
            "active_diaolan_path 与 selected_diaolan_path 均为当前 Web/PTZ 与吊篮高度滑条绑定的吊篮根路径；"
            "所有吊篮实例默认同时可见（apply_diaolan_visibility）。"
        ),
        "lookat_target_prim_path": eff_look,
        "active_lookat_target_prim_path": eff_look,
        "lookat_building_context_prim_path": building_context.get("prim_path"),
        "lookat_building_context_selection": building_context,
        "all_diaolan_paths": all_paths_snap,
        "workers_visible_count_by_diaolan_path": dict(
            runtime.get("workers_visible_count_by_diaolan_path") or {}
        ),
        "workers_max_for_selected": workers_max_for_selected,
        "target_prim_path": current_target_path,
        "selected_target_path": current_target_path,
        "all_worker_paths": runtime["all_worker_paths"],
        "visible_worker_paths": runtime["visible_worker_paths"],
        "workers_visible_logical_count": count_logical_workers_from_paths(
            list(runtime.get("visible_worker_paths") or [])
        ),
        "gondola_renderable_paths": gondola_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_visible_renderable_paths": gondola_visible_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_hidden_paths": gondola_hidden_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_debug": gondola_renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_counts": {
            "total": len(gondola_renderable_paths),
            "visible": len(gondola_visible_renderable_paths),
            "hidden": len(gondola_hidden_paths),
            "sample_limit": _GONDOLA_RENDERABLE_DETAIL_LIMIT,
        },
        "height_debug": runtime["height_debug"],
        "gondola_heights": dict(runtime.get("gondola_heights") or {}),
        "diaolan_candidates": diaolan_candidates,
        "random_config": runtime["random_config"],
        "randomize_fast_response": bool(_RANDOMIZE_FAST_RESPONSE),
        "randomize_render_settle_max_frames": int(_RANDOMIZE_RENDER_SETTLE_MAX_FRAMES),
        "randomize_render_stabilize_window_s": float(_RANDOMIZE_RENDER_STABILIZE_WINDOW_S),
        "randomize_context_orientation_max_candidates": int(_RANDOMIZE_CONTEXT_ORIENTATION_MAX_CANDIDATES),
        "randomize_context_down_tilt_policy": {
            "window": int(_RANDOMIZE_CONTEXT_DOWN_TILT_WINDOW),
            "max_down_in_window": int(_RANDOMIZE_CONTEXT_DOWN_TILT_MAX_IN_WINDOW),
            "down_probability": float(_RANDOMIZE_CONTEXT_DOWN_TILT_PROBABILITY),
            "down_threshold_deg": float(_TILT_DIRECTION_THRESHOLD_DEG),
            "down_condition": "tilt > down_threshold_deg",
            "tilt_semantics": "positive_down_negative_up",
            "recent_tilts": [float(v) for v in _randomize_context_tilt_history],
            "recent_tilt_direction_labels": [
                _tilt_direction_label(v) for v in _randomize_context_tilt_history
            ],
            "recent_down_count": sum(1 for v in _randomize_context_tilt_history if _is_down_tilt(v)),
        },
        "hdri_control": runtime["hdri_control"],
        "last_random_result": runtime["last_random_result"],
        "pending_active_diaolan_path": runtime["pending_active_diaolan_path"] or None,
        # 运行态挂墙采样配置（与 yaml 启动加载一致；供 /scene/state 快速核对，不改采样语义）
        "wall_sampling_config": {
            "wall_collection_mode": _WALL_COLLECTION_MODE,
            "wall_collection_root_path": (_WALL_COLLECTION_ROOT_PATH or None),
            "wall_constraint_prim_path": _CAMERA_WALL_CONSTRAINT_PRIM,
            "wall_mount_inset_m": float(_CAMERA_WALL_MOUNT_INSET_M),
            "wall_mount_inset_mode": (_CAMERA_WALL_MOUNT_INSET_MODE or None),
            "wall_candidate_region": copy.deepcopy(_WALL_CANDIDATE_REGION),
            "camera_lookat_target_xyz": [float(v) for v in _CAMERA_LOOKAT_TARGET_XYZ],
        },
    }
    out.update(_describe_hdri_state(stage, runtime["random_config"]))
    out["hdri_groups"] = _describe_hdri_control_state(stage)
    out["environment"] = _environment_public_status(stage)
    return out


def _scene_state_lightweight_snapshot(stage=None, *, result: dict | None = None) -> dict:
    runtime = _scene_state_runtime_snapshot(lock_timeout=0.05)
    last = result if isinstance(result, dict) else runtime.get("last_random_result")
    wall_status = last.get("wall_constraint_status") if isinstance(last, dict) else None
    return {
        "ok": True,
        "state_deferred": True,
        "full_state_endpoint": "/scene/state",
        "selected_diaolan_path": runtime.get("selected_diaolan_path") or None,
        "active_diaolan_path": runtime.get("active_diaolan_path") or None,
        "gondola_y": runtime.get("gondola_y"),
        "gondola_heights": dict(runtime.get("gondola_heights") or {}),
        "workers": runtime.get("workers"),
        "workers_visible_count_by_diaolan_path": dict(
            runtime.get("workers_visible_count_by_diaolan_path") or {}
        ),
        "random_config": runtime.get("random_config"),
        "camera_xyz": last.get("camera_xyz") if isinstance(last, dict) else None,
        "startup_view_visible": last.get("startup_view_visible") if isinstance(last, dict) else None,
        "wall_constraint_status": wall_status if isinstance(wall_status, dict) else None,
        "last_random_request_id": last.get("request_id") if isinstance(last, dict) else None,
        "last_random_timestamp": last.get("timestamp") if isinstance(last, dict) else None,
    }


def _describe_hdri_control_state(stage=None) -> dict:
    runtime = _scene_state_runtime_snapshot()
    control = runtime.get("hdri_control") or _default_hdri_control_state()
    backend_status = _sanitize_hdri_backend_status(runtime.get("hdri_backend_status"))
    groups = _build_hdri_groups()
    current_group_id = control.get("current_group_id") if control.get("current_group_id") in groups else _HDRI_FIXED_GROUP_ID
    selected_by_group = dict(control.get("selected_by_group") or {})
    selected_entry = _get_hdri_entry(current_group_id, selected_by_group.get(current_group_id))
    selected_entry_id = selected_entry.get("id") if isinstance(selected_entry, dict) else None
    actual_group_id, actual_entry = _match_hdri_entry_by_path(backend_status.get("current_hdri_path"))
    actual_group = groups.get(actual_group_id) if actual_group_id else None
    current_index = int(selected_entry.get("slot") or 0) if isinstance(selected_entry, dict) else None
    actual_binding_prim = backend_status.get("hdri_dome_light_path")
    actual_binding_attr = backend_status.get("hdri_texture_attr") or _HDRI_TEXTURE_ATTR_NAME
    actual_binding_target = backend_status.get("actual_binding_target") or (
        f"{actual_binding_prim}.{actual_binding_attr}" if actual_binding_prim and actual_binding_attr else None
    )
    return {
        "current_group_id": current_group_id,
        "current_group_name": groups[current_group_id]["name"],
        "current_group_index": current_index,
        "current_hdri_id": selected_entry_id,
        "current_hdri_name": selected_entry.get("name") if isinstance(selected_entry, dict) else None,
        "current_hdri_file_path": selected_entry.get("path") if isinstance(selected_entry, dict) else None,
        "current_hdri_path": backend_status.get("current_hdri_path"),
        "current_group": backend_status.get("current_group"),
        "updated_at": backend_status.get("updated_at"),
        "confirmed_by_backend": backend_status.get("confirmed_by_backend"),
        "actual_group_id": actual_group_id,
        "actual_group_name": actual_group.get("name") if isinstance(actual_group, dict) else backend_status.get("current_group"),
        "actual_hdri_id": actual_entry.get("id") if isinstance(actual_entry, dict) else None,
        "actual_hdri_name": actual_entry.get("name") if isinstance(actual_entry, dict) else backend_status.get("current_hdri_name"),
        "actual_hdri_file_path": backend_status.get("current_hdri_path"),
        "recent_switch_time": control.get("last_switch_time"),
        "actual_binding_target": actual_binding_target,
        "actual_binding_prim": actual_binding_prim,
        "actual_binding_attr": actual_binding_attr,
        "apply_ok": control.get("last_apply_ok"),
        "apply_message": control.get("last_apply_message") or "",
        "apply_verified": bool(
            backend_status.get("confirmed_by_backend")
            and backend_status.get("current_hdri_path")
            and isinstance(selected_entry, dict)
            and backend_status.get("current_hdri_path") == selected_entry.get("path")
        ),
        "last_action": control.get("last_action"),
        "backend_status": backend_status,
        "other_light_evidence": list(backend_status.get("other_light_evidence") or []),
        "dominant_environment_light": backend_status.get("dominant_environment_light"),
        "dominant_direct_light": backend_status.get("dominant_direct_light"),
        "groups": groups,
        "group_a_list": list(groups[_HDRI_FIXED_GROUP_ID]["entries"]),
        "group_b_list": [],
    }


def _queue_hdri_apply(req: dict | None = None, route: str = "/scene/hdri") -> dict:
    start_ts = time.time()
    payload = req if isinstance(req, dict) else {}
    event = threading.Event()
    holder = {"request": payload, "event": event, "response": None, "route": route}
    deadline = time.monotonic() + 20.0
    while True:
        with _scene_hdri_lock:
            global _scene_hdri_request
            pending = _scene_hdri_request
            if not isinstance(pending, dict) or pending.get("response") is not None:
                _scene_hdri_request = holder
                break
        if time.monotonic() >= deadline:
            exc = TimeoutError("HDRI apply queue busy")
            _log_hdri_timing(route, start_ts, end_ts=time.time(), timeout=True, exception=exc, cache_hit=False, phase="apply_wait")
            raise exc
        time.sleep(0.05)
    _scene_hdri_dirty.set()
    if not event.wait(timeout=20.0):
        exc = TimeoutError("HDRI apply timed out")
        _log_hdri_timing(route, start_ts, end_ts=time.time(), timeout=True, exception=exc, cache_hit=False, phase="apply_wait")
        raise exc
    response = holder.get("response")
    if not isinstance(response, dict):
        exc = RuntimeError("HDRI apply response missing")
        _log_hdri_timing(route, start_ts, end_ts=time.time(), exception=exc, cache_hit=False, phase="apply_wait")
        raise exc
    _log_hdri_timing(route, start_ts, end_ts=time.time(), cache_hit=False, phase="apply_wait")
    return response


def _queue_environment_request(req: dict | None = None) -> dict:
    payload = req if isinstance(req, dict) else {}
    event = threading.Event()
    holder = {"request": payload, "event": event, "response": None}
    deadline = time.monotonic() + 30.0
    while True:
        with _scene_environment_lock:
            global _scene_environment_request
            pending = _scene_environment_request
            if not isinstance(pending, dict) or pending.get("response") is not None:
                _scene_environment_request = holder
                break
        if time.monotonic() >= deadline:
            raise TimeoutError("environment apply queue busy")
        time.sleep(0.05)
    _scene_environment_dirty.set()
    if not event.wait(timeout=30.0):
        raise TimeoutError("environment apply timed out")
    response = holder.get("response")
    if not isinstance(response, dict):
        raise RuntimeError("environment apply response missing")
    return response


def _runtime_random_config_from_request(base_cfg: dict, request: dict | None = None) -> dict:
    merged = _sanitize_random_config(base_cfg)
    if not isinstance(request, dict):
        return merged
    runtime_cfg = request.get("runtime_random_config")
    if not isinstance(runtime_cfg, dict):
        return merged
    effective = dict(merged)
    for key in (
        "auto_random_on_start",
        "random_gondola",
        "random_workers",
        "random_camera",
        "random_hdri",
        "keep_target_visible",
        "keep_wall_install_constraint",
        "auto_look_at_target",
    ):
        if key in runtime_cfg:
            effective[key] = bool(runtime_cfg.get(key))
    if "hdri_candidates" in runtime_cfg:
        effective["hdri_candidates"] = runtime_cfg.get("hdri_candidates")
    return _sanitize_random_config(effective)


def _apply_hdri_control_request(stage, req: dict | None = None, route: str = "/scene/hdri") -> dict:
    start_ts = time.time()
    request = req if isinstance(req, dict) else {}
    try:
        action = str(request.get("action") or request.get("trigger") or "refresh").strip() or "refresh"
        groups = _build_hdri_groups()
        with _scene_lock:
            control = _sanitize_hdri_control_state(_scene_state.get("hdri_control"))
            current_group_id = control.get("current_group_id") or _HDRI_FIXED_GROUP_ID
            selected_by_group = dict(control.get("selected_by_group") or {})
        requested_group_id = str(request.get("group_id") or current_group_id).strip().upper() or current_group_id
        if requested_group_id not in groups:
            requested_group_id = current_group_id
        if action == "switch_group":
            current_entry = _get_hdri_entry(current_group_id, selected_by_group.get(current_group_id))
            preferred_slot = int(current_entry.get("slot") or 1) if isinstance(current_entry, dict) else 1
            target_entry = _get_hdri_entry(requested_group_id, _entry_id_for_slot(requested_group_id, preferred_slot))
            if not (isinstance(target_entry, dict) and target_entry.get("exists") and target_entry.get("path")):
                available_entries = _available_hdri_entries(requested_group_id)
                target_entry = available_entries[0] if available_entries else _get_hdri_entry(requested_group_id)
            selected_by_group[requested_group_id] = target_entry.get("id") if isinstance(target_entry, dict) else selected_by_group.get(requested_group_id)
            current_group_id = requested_group_id
        elif action == "select_hdri":
            current_group_id = requested_group_id
            target_entry = _get_hdri_entry(current_group_id, str(request.get("hdri_id") or "").strip())
            if not isinstance(target_entry, dict):
                raise ValueError("hdri_id is invalid")
            selected_by_group[current_group_id] = target_entry.get("id")
        elif action == "random_hdri":
            current_group_id = requested_group_id or current_group_id
            available_entries = _available_hdri_entries(current_group_id)
            if not available_entries:
                raise RuntimeError(f"组 {current_group_id} 没有可用 HDRI")
            current_entry = _get_hdri_entry(current_group_id, selected_by_group.get(current_group_id))
            target_entry = current_entry if isinstance(current_entry, dict) else available_entries[0]
            if len(available_entries) > 1:
                for _ in range(6):
                    candidate = random.choice(available_entries)
                    if not isinstance(current_entry, dict) or candidate.get("id") != current_entry.get("id"):
                        target_entry = candidate
                        break
            selected_by_group[current_group_id] = target_entry.get("id")
        else:
            target_entry = _get_hdri_entry(current_group_id, selected_by_group.get(current_group_id))
            if not (isinstance(target_entry, dict) and target_entry.get("exists") and target_entry.get("path")):
                available_entries = _available_hdri_entries(current_group_id)
                target_entry = available_entries[0] if available_entries else _get_hdri_entry(current_group_id)
                if isinstance(target_entry, dict):
                    selected_by_group[current_group_id] = target_entry.get("id")
        target_entry = _get_hdri_entry(current_group_id, selected_by_group.get(current_group_id))
        apply_result = None
        apply_ok = False
        apply_message = ""
        hdri_blocked = not _environment_allows_hdri()
        if hdri_blocked:
            apply_message = "HDRI 已禁用：当前为 dynamic_sky 环境模式（与 Dynamic Sky 互斥）"
        elif isinstance(target_entry, dict) and target_entry.get("exists") and target_entry.get("path"):
            apply_result = _apply_hdri_texture(stage, target_entry.get("path"))
            direct_light_coordination = _coordinate_dominant_direct_light(stage)
            apply_ok = bool(_normalize_hdri_path(apply_result.get("current_hdri")) == _normalize_hdri_path(target_entry.get("path")))
            apply_message = "HDRI 应用成功并已切换到目标文件" if apply_ok else "HDRI 已执行切换，但结果与目标文件不一致"
            if isinstance(direct_light_coordination, dict):
                apply_result["direct_light_coordination"] = direct_light_coordination
        else:
            apply_message = f"HDRI 目标不可用：{target_entry.get('name') if isinstance(target_entry, dict) else current_group_id}"
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        hdri_state = _describe_hdri_state(stage)
        actual_group_id, actual_entry = _match_hdri_entry_by_path(hdri_state.get("current_hdri"))
        if actual_group_id and isinstance(actual_entry, dict):
            current_group_id = actual_group_id
            selected_by_group[actual_group_id] = actual_entry.get("id")
        with _scene_lock:
            control = _sanitize_hdri_control_state(_scene_state.get("hdri_control"))
            control["current_group_id"] = current_group_id
            control["selected_by_group"] = selected_by_group
            control["last_switch_time"] = timestamp
            control["last_apply_ok"] = apply_ok
            control["last_apply_message"] = apply_message
            control["last_action"] = action
            _scene_state["hdri_control"] = control
            random_cfg = _sanitize_random_config(_scene_state.get("random_config"))
            random_cfg["hdri_candidates"] = _current_group_hdri_candidates(control)
            _scene_state["random_config"] = random_cfg
        if apply_ok or hdri_blocked:
            _refresh_hdri_backend_status_from_stage(stage, control_state=control, updated_at=timestamp)
        _invalidate_hdri_state_http_cache()
        _invalidate_scene_state_http_cache()
        _invalidate_status_http_cache()
        state = _scene_state_snapshot(stage)
        hdri_groups = state.get("hdri_groups") if isinstance(state.get("hdri_groups"), dict) else _describe_hdri_control_state()
        response = {
            "ok": apply_ok,
            "action": action,
            "request_group_id": requested_group_id,
            "selected_group_id": current_group_id,
            "selected_hdri_id": target_entry.get("id") if isinstance(target_entry, dict) else None,
            "selected_hdri_name": target_entry.get("name") if isinstance(target_entry, dict) else None,
            "selected_hdri_path": target_entry.get("path") if isinstance(target_entry, dict) else None,
            "timestamp": timestamp,
            "apply_result": apply_result,
            "hdri_groups": hdri_groups,
            "state": state,
            "actual_hdri_path": hdri_groups.get("actual_hdri_file_path"),
            "actual_binding_target": hdri_groups.get("actual_binding_target"),
            "apply_message": apply_message,
            "apply_verified": hdri_groups.get("apply_verified"),
        }
        _log_hdri_timing(route, start_ts, end_ts=time.time(), cache_hit=False, phase="apply_main")
        return response
    except Exception as exc:
        _log_hdri_timing(route, start_ts, end_ts=time.time(), exception=exc, cache_hit=False, phase="apply_main")
        raise


def _queue_randomize_scene(req: dict | None = None) -> dict:
    payload = req if isinstance(req, dict) else {}
    event = threading.Event()
    holder = {
        "request": payload,
        "event": event,
        "response": None,
    }
    with _scene_randomize_lock:
        global _scene_randomize_request
        if _scene_randomize_request is not None:
            raise RuntimeError("randomize request already pending")
        _scene_randomize_request = holder
        _scene_randomize_dirty.set()
    if not event.wait(timeout=_SCENE_RANDOMIZE_WAIT_TIMEOUT_S):
        raise TimeoutError("scene randomize timed out waiting for main-thread execution")
    response = holder.get("response") if isinstance(holder, dict) else None
    if not isinstance(response, dict):
        raise RuntimeError("scene randomize returned invalid response")
    return response


def _select_diaolan_response_state(stage) -> dict:
    """select_diaolan 专用：不调用 scan_diaolan_prims / 全量 snapshot，避免响应路径变慢。"""
    runtime = _scene_state_runtime_snapshot()
    st = stage
    paths = list(runtime.get("all_diaolan_paths") or [])
    sel = str(runtime.get("selected_diaolan_path") or "").strip() or None
    diaolan_candidates = []
    for p in paths:
        pv = str(p).strip()
        if not pv:
            continue
        g1 = None
        workers_max = 3
        if st is not None:
            try:
                ht, asm = resolve_diaolan_height_and_assembly(st, pv)
                if ht:
                    g1 = ht
                wlist = _collect_worker_prims(st, asm) if asm else []
                if not wlist and ht:
                    wlist = _collect_worker_prims(st, ht)
                workers_max = len(wlist) if wlist else workers_max
            except Exception:
                pass
        diaolan_candidates.append(_diaolan_candidate_meta(pv, g1, workers_max))
    workers_max_for_selected = 0
    if sel:
        for c in diaolan_candidates:
            if c.get("path") == sel:
                workers_max_for_selected = int(c.get("workers_max") or 0)
                break
    gcm = _stage_units_to_cm(st, runtime["gondola_y"]) if st is not None else 0.0
    current_target = _GONDOLA_PRIM or None
    if sel:
        picked_tp = None
        for c in diaolan_candidates:
            if c.get("path") == sel:
                picked_tp = str(c.get("target_prim_path") or "").strip() or None
                break
        if picked_tp:
            current_target = picked_tp
        elif st is not None:
            try:
                ht, _a = resolve_diaolan_height_and_assembly(st, str(sel).rstrip("/"))
                if ht:
                    current_target = ht
            except Exception:
                pass
    eff_look = _effective_lookat_target_prim_path(st) if st is not None else None
    building_context = _context_lookat_selection(st) if st is not None else {"prim_path": None, "reason": "stage_unavailable"}
    return {
        "gondola_height_cm": gcm,
        "workers": runtime["workers"],
        "selected_diaolan_path": sel,
        "active_diaolan_path": runtime["active_diaolan_path"] or None,
        "active_diaolan_semantics": (
            "active_diaolan_path 与 selected_diaolan_path 均为当前 Web/PTZ 与吊篮高度滑条绑定的吊篮根路径；"
            "所有吊篮实例默认同时可见（apply_diaolan_visibility）。"
        ),
        "lookat_target_prim_path": eff_look,
        "active_lookat_target_prim_path": eff_look,
        "lookat_building_context_prim_path": building_context.get("prim_path"),
        "lookat_building_context_selection": building_context,
        "all_diaolan_paths": paths,
        "workers_visible_count_by_diaolan_path": dict(
            runtime.get("workers_visible_count_by_diaolan_path") or {}
        ),
        "workers_max_for_selected": workers_max_for_selected,
        "target_prim_path": current_target,
        "selected_target_path": current_target,
        "all_worker_paths": list(runtime.get("all_worker_paths") or []),
        "visible_worker_paths": list(runtime.get("visible_worker_paths") or []),
        "gondola_renderable_paths": list(runtime.get("gondola_renderable_paths") or []),
        "gondola_visible_renderable_paths": list(runtime.get("gondola_visible_renderable_paths") or []),
        "gondola_hidden_paths": list(runtime.get("gondola_hidden_paths") or []),
        "gondola_renderable_debug": list(runtime.get("gondola_renderable_debug") or []),
        "height_debug": dict(runtime.get("height_debug") or {}),
        "gondola_heights": dict(runtime.get("gondola_heights") or {}),
        "diaolan_candidates": diaolan_candidates,
        "random_config": runtime["random_config"],
        "hdri_control": runtime["hdri_control"],
        "last_random_result": runtime["last_random_result"],
        "pending_active_diaolan_path": runtime["pending_active_diaolan_path"] or None,
        "wall_sampling_config": {
            "wall_collection_mode": _WALL_COLLECTION_MODE,
            "wall_collection_root_path": (_WALL_COLLECTION_ROOT_PATH or None),
            "wall_constraint_prim_path": _CAMERA_WALL_CONSTRAINT_PRIM,
            "wall_mount_inset_m": float(_CAMERA_WALL_MOUNT_INSET_M),
            "wall_mount_inset_mode": (_CAMERA_WALL_MOUNT_INSET_MODE or None),
            "wall_candidate_region": copy.deepcopy(_WALL_CANDIDATE_REGION),
            "camera_lookat_target_xyz": [float(v) for v in _CAMERA_LOOKAT_TARGET_XYZ],
        },
    }


def _apply_select_diaolan_http(path: str) -> dict:
    """HTTP 线程：仅改运行时状态 + 标脏；不遍历 stage、不阻塞等待主循环。"""
    global _GONDOLA_PRIM, _WORKER1_PRIM, _WORKER2_PRIM, _last_gondola_init_group1_source
    path = str(path or "").strip()
    try:
        st0 = omni.usd.get_context().get_stage()
        if st0 is None:
            raise RuntimeError("USD stage not ready")
        with _scene_lock:
            allowed = [str(p).strip() for p in (_scene_state.get("all_diaolan_paths") or []) if str(p).strip()]
        if not allowed:
            scanned0 = scan_diaolan_prims(st0)
            allowed = [str(d.get("path") or "").strip() for d in scanned0 if str(d.get("path") or "").strip()]
            with _scene_lock:
                _scene_state["all_diaolan_paths"] = list(allowed)
        if not allowed:
            raise ValueError("no diaolan roots found on stage")
        if path not in allowed:
            raise ValueError(f"selected_diaolan_path not in all_diaolan_paths: {path!r}")
        root = path.rstrip("/")
        height_path, asm_path = resolve_diaolan_height_and_assembly(st0, root)
        if not height_path or not asm_path:
            raise ValueError(f"cannot resolve gondola height/assembly for diaolan root {root!r}")
        persons = _collect_worker_prims(st0, asm_path)
        if not persons:
            persons = _collect_worker_prims(st0, height_path)
        minimal_d = {"path": path, "group1": height_path, "persons": persons}
        _GONDOLA_PRIM = height_path
        _WORKER1_PRIM = persons[0] if persons else ""
        _WORKER2_PRIM = persons[1] if len(persons) > 1 else ""
        _last_gondola_init_group1_source = "web_select_diaolan"
        with _scene_lock:
            _scene_state["active_diaolan_path"] = path
            _scene_state["selected_diaolan_path"] = path
        _sync_worker_scalar_fields_for_control_diaolan([minimal_d], path)
        _scene_dirty.set()
        st = None
        try:
            st = omni.usd.get_context().get_stage()
        except Exception:
            pass
        _invalidate_scene_state_http_cache()
        _invalidate_status_http_cache()
        return {"ok": True, "state": _select_diaolan_response_state(st)}
    except Exception as exc:
        st = None
        try:
            st = omni.usd.get_context().get_stage()
        except Exception:
            pass
        _invalidate_scene_state_http_cache()
        _invalidate_status_http_cache()
        return {"ok": False, "error": str(exc), "state": _select_diaolan_response_state(st)}


def _try_schedule_scene_randomize_nonblocking(req: dict | None = None) -> bool:
    """
    非阻塞入队随机请求（与 HTTP 共用 _scene_randomize_dirty / 主循环执行体）。
    event 为 None 时表示无需唤醒等待线程（主循环发起的定时随机）。
    """
    payload = req if isinstance(req, dict) else {}
    holder = {
        "request": payload,
        "event": None,
        "response": None,
    }
    with _scene_randomize_lock:
        global _scene_randomize_request
        if _scene_randomize_request is not None:
            return False
        _scene_randomize_request = holder
        _scene_randomize_dirty.set()
    return True


def _assign_randomize_request_trace(req: dict, *, route_source: str) -> None:
    """为 HTTP 随机请求写入 request_id 与 source（web|api|auto），供追踪与 GET /api/scene/randomize/last。"""
    if not isinstance(req, dict):
        return
    if not str(req.get("request_id") or "").strip():
        req["request_id"] = str(uuid.uuid4())
    if route_source == "api":
        req["source"] = "api"
        return
    trig = str(req.get("trigger") or "")
    if req.get("is_auto") or trig.startswith("auto_random"):
        req["source"] = "auto"
    else:
        req["source"] = "web"


def _annotate_randomize_result_for_api(result: dict, request_meta: dict | None) -> None:
    """在写入 last_random_result 前补充算法组约定字段与追踪日志。"""
    if not isinstance(result, dict):
        return
    explicit_r11 = bool(result.get("rule11_explicit_request"))
    jpg_bytes_for_rule11_archive = None
    base_meta = request_meta if isinstance(request_meta, dict) else {}
    meta, event_meta = _ensure_randomize_rule_metadata(result, base_meta)
    if isinstance(event_meta, dict):
        result["randomize_event_meta"] = event_meta
    rid = str(meta.get("request_id") or "").strip()
    if rid:
        result["request_id"] = rid
    src = str(meta.get("source") or "").strip()
    if src in ("web", "api", "auto"):
        result["source"] = src
    trig = str(meta.get("trigger") or "").strip()
    result["random_event"] = trig if trig else None
    cam_xyz = result.get("camera_xyz")
    result["camera_pose"] = {
        "translate_xyz": [float(x) for x in cam_xyz]
        if isinstance(cam_xyz, (list, tuple)) and len(cam_xyz) >= 3
        else cam_xyz,
        "orientation": dict(result.get("orientation") or {})
        if isinstance(result.get("orientation"), dict)
        else result.get("orientation"),
        "camera_meta": dict(result.get("camera_meta") or {})
        if isinstance(result.get("camera_meta"), dict)
        else result.get("camera_meta"),
    }
    try:
        result["hazard_eval"] = _safe_build_hazard_eval(result, request_meta=meta)
        he = result.get("hazard_eval")
        if isinstance(he, dict) and isinstance(event_meta, dict):
            he["event_id"] = event_meta.get("event_id")
            he["event_attribution"] = event_meta.get("source")
        if explicit_r11:
            jpg_bytes_for_rule11_archive = _get_cached_snapshot_jpeg_bytes()
        # --- 新链路（感知骨架）：在 hazard_eval 之后追加命名空间，不修改 hazard_eval 语义 ---
        with _stream_diag_lock:
            _diag_for_perception = dict(_STREAM_DIAG)
        attach_perception_to_randomize_result(
            result,
            meta,
            perception_cfg_raw=cfg.get("perception_migration"),
            stream_diag=_diag_for_perception,
            resolution_wh=(W, H),
            camera_prim_path=str(camera_prim),
            focal_length_ref=FOCAL_LENGTH_1X,
        )
        print(
            "[scene-randomize-trace] "
            + json.dumps(
                {
                    "source": result.get("source"),
                    "request_id": result.get("request_id"),
                    "timestamp": result.get("timestamp"),
                    "random_event": result.get("random_event"),
                    "event_id": result.get("event_id"),
                    "final_hazard_source": (result.get("final_hazard_result") or {}).get("source"),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    finally:
        if explicit_r11:
            result["rule11_exposure_restore"] = _rule11_camera_chain_restore_baseline()
    if explicit_r11:
        he = result.get("hazard_eval")
        arch = _rule11_archive_eval_jpeg_and_evidence(
            jpeg_bytes=jpg_bytes_for_rule11_archive,
            result=result,
            request_meta=meta,
            hazard_eval=he if isinstance(he, dict) else None,
            exposure_restore=result.get("rule11_exposure_restore"),
        )
        result["rule11_eval_archive"] = arch
        if isinstance(he, dict):
            ev = he.get("evidence")
            if isinstance(ev, dict):
                ev["rule11_archive_jpeg_path"] = arch.get("jpeg_path")
                ev["rule11_archive_evidence_json_path"] = arch.get("evidence_json_path")
                ev["rule11_camera_exposure_adjustment"] = result.get("rule11_camera_exposure_adjustment")
                ev["rule11_exposure_restore"] = result.get("rule11_exposure_restore")


def _wrap_api_randomize_response(resp: dict) -> dict:
    """POST /api/scene/randomize 稳定顶层字段，与 GET /api/scene/randomize/last 摘要一致。"""
    if not isinstance(resp, dict):
        return {"ok": False, "error": "invalid response"}
    out = dict(resp)
    if resp.get("ok") and isinstance(resp.get("result"), dict):
        r = resp["result"]
        out["timestamp"] = r.get("timestamp")
        out["timing"] = r.get("timing")
        out["state_deferred"] = bool(r.get("state_deferred", resp.get("state_deferred", _RANDOMIZE_FAST_RESPONSE)))
        out["full_state_endpoint"] = r.get("full_state_endpoint") or resp.get("full_state_endpoint") or "/scene/state"
        out["source"] = r.get("source")
        out["request_id"] = r.get("request_id")
        out["random_event"] = r.get("random_event")
        out["active_diaolan_path"] = r.get("active_diaolan_path")
        out["camera_pose"] = r.get("camera_pose")
        out["random_config"] = r.get("random_config")
        out["randomize_event_meta"] = r.get("randomize_event_meta")
        out["hazard_eval"] = r.get("hazard_eval")
        # 感知迁移追加字段（与 GET /api/scene/randomize/last 摘要对齐）
        out["event_id"] = r.get("event_id")
        out["event_type"] = r.get("event_type")
        out["hazard_category"] = r.get("hazard_category")
        out["legacy_rule_result"] = r.get("legacy_rule_result")
        out["perception_result"] = r.get("perception_result")
        out["final_hazard_result"] = r.get("final_hazard_result")
        out["random_event_hazard"] = r.get("random_event_hazard")
        out["event_registry"] = r.get("event_registry")
        out["scene_observation"] = r.get("scene_observation")
        out["perception_inputs"] = r.get("perception_inputs")
        out["hazard_evaluation"] = r.get("hazard_evaluation")
        out["scene_state_evaluation"] = r.get("scene_state_evaluation")
        out["camera_observability"] = r.get("camera_observability")
        for _rk in (
            "render_commit_status",
            "randomize_stream_freeze_used",
            "randomize_stream_commit_source",
            "randomize_stream_black_frames_blocked",
            "randomize_stream_stable_wait_s",
            "randomize_stream_recovery_attempted",
        ):
            out[_rk] = r.get(_rk)
    elif not resp.get("ok"):
        for _rk in (
            "render_commit_status",
            "randomize_stream_freeze_used",
            "randomize_stream_commit_source",
            "randomize_stream_black_frames_blocked",
            "randomize_stream_stable_wait_s",
            "randomize_stream_recovery_attempted",
        ):
            if _rk in resp:
                out[_rk] = resp.get(_rk)
    return out


_RANDOMIZE_LAST_API_CACHE_LOCK = threading.Lock()
_randomize_last_api_cache_dict: dict | None = None


def _build_api_scene_randomize_last_payload_from_lr(lr: dict | None) -> dict:
    """组装 GET /api/scene/randomize/last 的完整 JSON（在 randomize 完成线程写入缓存；GET 仅深拷贝此结构）。"""
    if not isinstance(lr, dict):
        out: dict = {
            "ok": True,
            "last": None,
            "stale": False,
            "degraded": False,
            "updated_at": time.time(),
            "error": "no_last_randomize",
        }
        out.update(_build_randomize_last_answer_top(None, raw_snapshot=None))
        return out
    last_block = {
        "timestamp": lr.get("timestamp"),
        "source": lr.get("source"),
        "request_id": lr.get("request_id"),
        "random_event": lr.get("random_event"),
        "active_diaolan_path": lr.get("active_diaolan_path"),
        "camera_pose": lr.get("camera_pose"),
        "random_config": lr.get("random_config"),
        "randomize_event_meta": lr.get("randomize_event_meta"),
        "hazard_eval": lr.get("hazard_eval"),
        "event_id": lr.get("event_id"),
        "event_type": lr.get("event_type"),
        "hazard_category": lr.get("hazard_category"),
        "legacy_rule_result": lr.get("legacy_rule_result"),
        "perception_result": lr.get("perception_result"),
        "final_hazard_result": lr.get("final_hazard_result"),
        "random_event_hazard": lr.get("random_event_hazard"),
        "event_registry": lr.get("event_registry"),
        "scene_observation": lr.get("scene_observation"),
        "perception_inputs": lr.get("perception_inputs"),
        "hazard_evaluation": lr.get("hazard_evaluation"),
        "scene_state_evaluation": lr.get("scene_state_evaluation"),
        "camera_observability": lr.get("camera_observability"),
        "result": lr,
    }
    out2 = {
        "ok": True,
        "last": last_block,
        "stale": False,
        "degraded": False,
        "updated_at": time.time(),
    }
    out2.update(_build_randomize_last_answer_top(lr, raw_snapshot=last_block))
    return out2


def _set_randomize_last_api_cache_from_lr(lr: dict) -> None:
    global _randomize_last_api_cache_dict
    payload = _build_api_scene_randomize_last_payload_from_lr(lr)
    with _RANDOMIZE_LAST_API_CACHE_LOCK:
        _randomize_last_api_cache_dict = payload


def _api_scene_randomize_last_dict() -> dict:
    with _RANDOMIZE_LAST_API_CACHE_LOCK:
        cached = _randomize_last_api_cache_dict
    if isinstance(cached, dict) and cached:
        return copy.deepcopy(cached)
    return _build_api_scene_randomize_last_payload_from_lr(None)


def _api_scene_randomize_last_post_payload(body: bytes) -> tuple[dict, int]:
    req: dict = {}
    if body:
        try:
            parsed = json.loads(body.decode("utf-8", errors="ignore"))
            if isinstance(parsed, dict):
                req = parsed
        except Exception as exc:
            return (
                {
                    "ok": False,
                    "api_version": "v1",
                    "method": "POST",
                    "project_id": "diaolan",
                    "scene_type": "gondola",
                    "request_id": None,
                    "data": None,
                    "error": f"invalid_json: {exc}",
                },
                400,
            )
    project_id = str(req.get("project_id") or "diaolan").strip() or "diaolan"
    scene_type = str(req.get("scene_type") or "gondola").strip() or "gondola"
    request_id = req.get("request_id")
    if request_id is not None:
        request_id = str(request_id)
    include_image = bool(req.get("include_image", False))
    out = {
        "ok": True,
        "api_version": "v1",
        "method": "POST",
        "project_id": project_id,
        "scene_type": scene_type,
        "request_id": request_id,
        "data": _api_scene_randomize_last_dict(),
        "error": None,
    }
    if include_image:
        out["image"] = None
    return out, 200


def _http_json_scene_randomize(body: bytes, *, route_source: str, api_envelope: bool) -> str:
    trace_id = None
    try:
        req = json.loads(body) if body else {}
        if not isinstance(req, dict):
            req = {}
        _assign_randomize_request_trace(req, route_source=route_source)
        trace_id = req.get("request_id")
        if isinstance(req, dict) and "active_diaolan_path" in req:
            ap = str(req.get("active_diaolan_path") or "").strip()
            with _scene_lock:
                _scene_state["pending_active_diaolan_path"] = ap
        if req:
            with _scene_lock:
                current_cfg = _sanitize_random_config(_scene_state.get("random_config"))
                merged_cfg = dict(current_cfg)
                if "random_config" in req and isinstance(req.get("random_config"), dict):
                    patch = req.get("random_config") or {}
                    if patch:
                        sanitized = _sanitize_random_config(patch)
                        for k in patch.keys():
                            merged_cfg[k] = sanitized[k]
                else:
                    legacy_patch = {
                        k: v
                        for k, v in req.items()
                        if k not in _RANDOMIZE_REQ_META_KEYS
                    }
                    if legacy_patch:
                        sanitized = _sanitize_random_config(legacy_patch)
                        for k in legacy_patch.keys():
                            merged_cfg[k] = sanitized[k]
                _scene_state["random_config"] = _sanitize_random_config(merged_cfg)
        resp_data = _queue_randomize_scene(req)
        if resp_data.get("ok") and isinstance(resp_data.get("result"), dict):
            br = resp_data["result"]
            resp_data["timing"] = br.get("timing")
            resp_data["state_deferred"] = bool(br.get("state_deferred", _RANDOMIZE_FAST_RESPONSE))
            resp_data["full_state_endpoint"] = br.get("full_state_endpoint") or "/scene/state"
            resp_data["hazard_eval"] = br.get("hazard_eval")
            for _pk in (
                "randomize_event_meta",
                "render_commit_status",
                "randomize_stream_freeze_used",
                "randomize_stream_commit_source",
                "randomize_stream_black_frames_blocked",
                "randomize_stream_stable_wait_s",
                "randomize_stream_recovery_attempted",
                "event_id",
                "event_type",
                "hazard_category",
                "legacy_rule_result",
                "perception_result",
                "final_hazard_result",
                "random_event_hazard",
                "event_registry",
                "scene_observation",
                "perception_inputs",
                "hazard_evaluation",
                "scene_state_evaluation",
                "camera_observability",
            ):
                resp_data[_pk] = br.get(_pk)
        elif not resp_data.get("ok"):
            for _pk in (
                "render_commit_status",
                "randomize_stream_freeze_used",
                "randomize_stream_commit_source",
                "randomize_stream_black_frames_blocked",
                "randomize_stream_stable_wait_s",
                "randomize_stream_recovery_attempted",
            ):
                if _pk in resp_data:
                    resp_data[_pk] = resp_data.get(_pk)
        if api_envelope:
            resp_data = _wrap_api_randomize_response(resp_data)
        _invalidate_scene_state_http_cache()
        _invalidate_status_http_cache()
        return json.dumps(resp_data, ensure_ascii=False)
    except Exception as exc:
        with _scene_lock:
            _scene_state["pending_active_diaolan_path"] = ""
        err_obj: dict = {
            "ok": False,
            "error": str(exc),
            "state": _scene_state_lightweight_snapshot()
                if _RANDOMIZE_FAST_RESPONSE
                else _scene_state_runtime_snapshot(lock_timeout=0.05),
            "state_deferred": bool(_RANDOMIZE_FAST_RESPONSE),
            "full_state_endpoint": "/scene/state",
        }
        if trace_id:
            err_obj["request_id"] = trace_id
        if api_envelope:
            err_obj = _wrap_api_randomize_response(err_obj)
        return json.dumps(err_obj, ensure_ascii=False)


def _poll_auto_random_timer_main(frame_idx: int) -> None:
    """SimulationApp 主线程每帧调用：到期则非阻塞入队，走与手动随机相同的主循环随机分支。"""
    global _auto_random_deadline_frame_idx
    with _scene_lock:
        cfg = _sanitize_random_config(_scene_state.get("random_config"))
        enabled = bool(cfg.get("auto_random_timer_enabled"))
        interval_s = int(cfg.get("auto_random_interval_seconds", 600))
    interval_s = max(10, min(86400, interval_s))
    stride_frames = max(1, int(round(float(interval_s) * float(sim_hz))))
    if not enabled:
        _auto_random_deadline_frame_idx = None
        return
    if _auto_random_deadline_frame_idx is None:
        _auto_random_deadline_frame_idx = int(frame_idx) + stride_frames
        print(
            "[scene-auto-random-timer] armed "
            f"(stride_frames={stride_frames}, approx_interval_s={interval_s}, sim_hz={sim_hz})",
            flush=True,
        )
        return
    if int(frame_idx) < int(_auto_random_deadline_frame_idx):
        return
    if not _try_schedule_scene_randomize_nonblocking(
        {
            "trigger": "auto_random_timer",
            "is_auto": True,
            "source": "auto",
            "request_id": str(uuid.uuid4()),
        }
    ):
        print(
            "[scene-auto-random-timer] skip tick: randomize already pending "
            f"(interval_s={interval_s})",
            flush=True,
        )
        _auto_random_deadline_frame_idx = int(frame_idx) + max(1, int(round(float(sim_hz))))
        return
    _auto_random_deadline_frame_idx = int(frame_idx) + stride_frames
    print(
        "[scene-auto-random-timer] tick: queue scene randomize "
        f"(trigger=auto_random_timer, interval_s={interval_s}, frame_idx={frame_idx})",
        flush=True,
    )


# ── MJPEG 帧缓冲（主循环写，HTTP handler 读）────────────────
_mjpeg = {"jpeg": None, "frame_id": 0}
_mjpeg_lock = threading.Lock()
_jpeg_encode_fn = None   # 在 main() 里初始化
_snapshot_cache = {
    "jpeg": None,
    "frame_id": 0,
    "ts": 0.0,
    "capture_seq": None,
    "last_good_jpeg": None,
    "last_good_frame_id": 0,
    "last_good_ts": 0.0,
    "last_good_capture_seq": None,
}
_render_recover_state_lock = threading.Lock()
_render_recover_in_progress = False
_last_render_recover_attempt_mono = 0.0
_last_render_recover_finish_mono = 0.0
_last_randomize_render_apply_mono = 0.0
_CTRL_HTTP_MAX_INFLIGHT = max(8, int(cfg.get("ctrl_http_max_inflight", 32)))
_CTRL_HTTP_ACQUIRE_TIMEOUT_S = max(
    0.1, float(cfg.get("ctrl_http_acquire_timeout_s", 0.6))
)
# 控制面：当前正在执行 do_GET/do_POST 的请求数（已通过 BoundedSemaphore 槽位）
_CTRL_HTTP_ACTIVE_REQUESTS = 0
_CTRL_HTTP_ACTIVE_REQUESTS_LOCK = threading.Lock()
_PTZ_STATE_HTTP_LOCK_TIMEOUT_S = 0.12
_STATUS_HTTP_SINGLEFLIGHT_WAIT_S = 0.08
_SNAPSHOT_JPG_LIVE_VP_QUEUE_WAIT_S = 0.35
_last_health_fast_path_ts: float = 0.0
_last_snapshot_http_dt_ms: float = 0.0
_last_rep_orchestrator_step_ms: float = 0.0
_stream_diag_stale_cache: dict = {}
_scene_runtime_stale_cache: dict = {}

# These endpoints are intentionally served even while heavy control-plane work is
# saturated. They either read short-lived in-memory caches or return liveness
# data needed by the launcher/UI to recover instead of filling the accept queue.
_CTRL_HTTP_LIGHT_GET_PATHS = {
    "/api/health",
    "/api/stream_ready",
    "/ptz_state",
    "/status",
    "/scene/random-config",
    "/api/scene/random-config",
    "/api/scene/randomize/last",
    "/scene/hdri",
    "/scene/dynamic-sky-presets",
}

# 重型 GET 短 TTL 缓存：减少与 PTZ/场景 POST 争用同一组控制面槽位的时间（不改变各接口 JSON 字段含义，仅允许极短延迟内读到略旧快照）
_STATUS_HTTP_CACHE_LOCK = threading.Lock()
_status_http_cache_body: bytes | None = None
_status_http_cache_ts: float = 0.0
_STATUS_HTTP_CACHE_TTL_S = max(0.05, float(cfg.get("status_http_cache_ttl_s", 0.75)))
# /status JSON 重建：缓存失效时只允许一个线程执行 _compose_status_dict，其余在锁上排队后读到新缓存（防并发 stampede）
_STATUS_HTTP_SINGLEFLIGHT_LOCK = threading.Lock()

# /status 内嵌的 renderer_runtime_observation：与全量 status TTL 解耦，单独限频（减轻 omni.kit.viewport / carb 轮询压力）
_RENDERER_OBS_STATUS_CACHE_LOCK = threading.Lock()
_renderer_obs_status_cache: dict | None = None
_renderer_obs_status_cache_ts: float = 0.0
_RENDERER_OBS_STATUS_REFRESH_LOCK = threading.Lock()
_RENDERER_OBS_STATUS_CACHE_TTL_S = max(0.5, float(cfg.get("status_renderer_obs_poll_ttl_s", 2.0)))

_SCENE_STATE_HTTP_CACHE_LOCK = threading.Lock()
_scene_state_http_cache_body: bytes | None = None
_scene_state_http_cache_ts: float = 0.0
_SCENE_STATE_HTTP_CACHE_TTL_S = max(0.05, float(cfg.get("scene_state_http_cache_ttl_s", 0.35)))

_HDRI_STATE_HTTP_CACHE_LOCK = threading.Lock()
_hdri_state_http_cache_body: bytes | None = None
_hdri_state_http_cache_ts: float = 0.0
_HDRI_STATE_HTTP_CACHE_TTL_S = max(0.05, float(cfg.get("hdri_state_http_cache_ttl_s", 0.35)))

# Launcher 就绪探测：主线程完成 open_stage + 吊篮/相机首帧初始化后才 set（避免仅 8081 监听误判「已启动」）
_STREAM_INIT_READY = threading.Event()


def _mark_randomize_render_apply() -> None:
    global _last_randomize_render_apply_mono
    _last_randomize_render_apply_mono = time.monotonic()


def _randomize_render_stabilizing(now: float | None = None) -> bool:
    if _RANDOMIZE_RENDER_STABILIZE_WINDOW_S <= 0.0:
        return False
    now_mono = time.monotonic() if now is None else float(now)
    last = float(_last_randomize_render_apply_mono)
    return last > 0.0 and (now_mono - last) < _RANDOMIZE_RENDER_STABILIZE_WINDOW_S


def _render_recover_state(now: float | None = None) -> dict:
    now_mono = time.monotonic() if now is None else float(now)
    with _render_recover_state_lock:
        in_progress = bool(_render_recover_in_progress)
        last_attempt = float(_last_render_recover_attempt_mono)
    cooldown_remaining = 0.0
    if last_attempt > 0.0:
        cooldown_remaining = max(
            0.0, _NEAR_BLACK_RECOVER_COOLDOWN_S - (now_mono - last_attempt)
        )
    return {
        "in_progress": in_progress,
        "last_attempt_mono": last_attempt,
        "cooldown_remaining_s": cooldown_remaining,
    }


def _snapshot_should_prefer_last_good(now: float | None = None) -> tuple[bool, str | None]:
    state = _render_recover_state(now)
    if state["in_progress"]:
        return True, "recover_in_progress"
    now_mono = time.monotonic() if now is None else float(now)
    if (
        _POST_RECOVER_SNAPSHOT_GATE_S > 0.0
        and _last_render_recover_finish_mono > 0.0
        and (now_mono - _last_render_recover_finish_mono) < _POST_RECOVER_SNAPSHOT_GATE_S
    ):
        return True, "post_render_recover_snapshot_gate"
    if _randomize_render_stabilizing(now):
        return True, "recent_randomize_stabilizing"
    return False, None


def _cache_good_snapshot_jpeg(
    jpg: bytes | bytearray,
    capture_seq: int | None,
    *,
    mirror_mjpeg: bool,
) -> tuple[int, int | None]:
    if not isinstance(jpg, (bytes, bytearray)) or len(jpg) <= 0:
        return 0, None
    now = time.monotonic()
    blob = bytes(jpg)
    cap_seq_value = int(capture_seq) if capture_seq is not None else None
    with _mjpeg_lock:
        _snapshot_cache["jpeg"] = blob
        _snapshot_cache["frame_id"] += 1
        frame_id = int(_snapshot_cache["frame_id"])
        _snapshot_cache["ts"] = now
        _snapshot_cache["capture_seq"] = cap_seq_value
        _snapshot_cache["last_good_jpeg"] = blob
        _snapshot_cache["last_good_frame_id"] = frame_id
        _snapshot_cache["last_good_ts"] = now
        _snapshot_cache["last_good_capture_seq"] = cap_seq_value
        if mirror_mjpeg:
            _mjpeg["jpeg"] = blob
            _mjpeg["frame_id"] += 1
    _STREAM_INIT_READY.set()
    return frame_id, cap_seq_value


def _get_last_good_snapshot_jpeg() -> tuple[bytes | None, int, int | None]:
    with _mjpeg_lock:
        jpg = _snapshot_cache.get("last_good_jpeg")
        frame_id = int(_snapshot_cache.get("last_good_frame_id") or 0)
        capture_seq = _snapshot_cache.get("last_good_capture_seq")
    if not isinstance(jpg, (bytes, bytearray)) or len(jpg) <= 0:
        return None, 0, None
    return bytes(jpg), frame_id, capture_seq


_RTSP_RANDOMIZE_KEEPALIVE_LOG_MONO: float = 0.0
_RTSP_RANDOMIZE_KEEPALIVE_LOG_INTERVAL_S = 2.0
_RANDOMIZE_STREAM_GUARD_LOCK = threading.Lock()
_RANDOMIZE_STREAM_GUARD: dict = {
    "active": False,
    "freeze_active": False,
    "mode": "idle",
    "frozen": None,
    "black_frames_blocked_total": 0,
    "black_frames_blocked_start": 0,
    "last_commit_source": None,
    "candidate_health": None,
}


def _rtsp_latest_frame_snapshot_for_randomize_freeze() -> dict | None:
    with _rtsp_latest_cond:
        frame = _rtsp_latest_frame
        if not isinstance(frame, (bytes, bytearray)) or len(frame) != int(W) * int(H) * 4:
            return None
        try:
            return {
                "raw": bytes(frame),
                "seq": int(_rtsp_latest_seq),
                "source": str(_rtsp_latest_source or "rtsp_latest"),
                "mono_ts": float(_rtsp_latest_mono or 0.0),
                "capture_epoch_ms": _rtsp_latest_capture_epoch_ms,
                "capture_iso": _rtsp_latest_capture_iso,
                "osd_text": _rtsp_latest_osd_text,
                "osd_draw_ms": _rtsp_latest_osd_draw_ms,
            }
        except Exception:
            return None


def _randomize_stream_diag_update() -> None:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        guard = dict(_RANDOMIZE_STREAM_GUARD)
        frozen = guard.get("frozen")
    age_ms = None
    if isinstance(frozen, dict):
        try:
            mono_ts = float(frozen.get("mono_ts") or 0.0)
            if mono_ts > 0:
                age_ms = round((time.monotonic() - mono_ts) * 1000.0, 1)
        except Exception:
            age_ms = None
    if age_ms is None and str(guard.get("mode") or "") == "frozen_last_good":
        try:
            with _rtsp_latest_cond:
                if _rtsp_latest_randomize_mode == "frozen_last_good" and float(_rtsp_latest_mono or 0.0) > 0.0:
                    age_ms = round((time.monotonic() - float(_rtsp_latest_mono)) * 1000.0, 1)
        except Exception:
            age_ms = None
    _stream_diag_update(
        randomize_active=bool(guard.get("active")),
        randomize_stream_mode=str(guard.get("mode") or "idle"),
        randomize_frozen_frame_age_ms=age_ms,
        randomize_candidate_health=copy.deepcopy(guard.get("candidate_health")),
        randomize_last_commit_source=guard.get("last_commit_source"),
        randomize_black_frames_blocked_total=int(guard.get("black_frames_blocked_total") or 0),
    )


def _randomize_stream_guard_diag_snapshot() -> dict:
    _randomize_stream_diag_update()
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        freeze_active = bool(_RANDOMIZE_STREAM_GUARD.get("freeze_active"))
    with _stream_diag_lock:
        return {
            "randomize_stream_mode": _STREAM_DIAG.get("randomize_stream_mode"),
            "randomize_last_commit_source": _STREAM_DIAG.get("randomize_last_commit_source"),
            "randomize_black_frames_blocked_total": _STREAM_DIAG.get("randomize_black_frames_blocked_total"),
            "randomize_active": _STREAM_DIAG.get("randomize_active"),
            "randomize_freeze_active": freeze_active,
        }


def _randomize_stream_guard_begin() -> dict:
    frozen = _rtsp_latest_frame_snapshot_for_randomize_freeze()
    freeze_active = bool(_RANDOMIZE_FREEZE_STREAM_DURING_APPLY and frozen is not None)
    mode = "frozen_last_good" if freeze_active else "active_no_frozen_frame"
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        blocked_start = int(_RANDOMIZE_STREAM_GUARD.get("black_frames_blocked_total") or 0)
        _RANDOMIZE_STREAM_GUARD.update(
            active=True,
            freeze_active=freeze_active,
            mode=mode,
            frozen=frozen,
            black_frames_blocked_start=blocked_start,
            candidate_health=None,
        )
    _randomize_stream_diag_update()
    if freeze_active and isinstance(frozen, dict):
        meta = {
            "capture_epoch_ms": frozen.get("capture_epoch_ms"),
            "capture_iso": frozen.get("capture_iso"),
            "osd_text": frozen.get("osd_text"),
            "osd_draw_ms": frozen.get("osd_draw_ms"),
            "osd_draw_method": "frozen_last_good",
            "osd_applied": True,
            "randomize_mode": "frozen_last_good",
        }
        _rtsp_put_frame_bytes(
            frozen["raw"],
            "randomize_frozen_last_good",
            meta,
        )
    print(
        "[scene-randomize][stream-guard] begin "
        f"freeze_active={freeze_active} frozen_seq={(frozen or {}).get('seq') if isinstance(frozen, dict) else None}",
        flush=True,
    )
    return {
        "active": True,
        "freeze_active": freeze_active,
        "frozen": frozen,
        "mode": mode,
        "black_frames_blocked_start": blocked_start,
    }


def _randomize_stream_guard_finish(*, commit_source: str | None = None, mode: str = "idle") -> None:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        _RANDOMIZE_STREAM_GUARD["active"] = False
        _RANDOMIZE_STREAM_GUARD["freeze_active"] = False
        _RANDOMIZE_STREAM_GUARD["mode"] = mode
        if mode != "frozen_last_good":
            _RANDOMIZE_STREAM_GUARD["frozen"] = None
        if commit_source:
            _RANDOMIZE_STREAM_GUARD["last_commit_source"] = commit_source
    _randomize_stream_diag_update()


def _randomize_stream_guard_note_candidate(health: dict | None) -> None:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        _RANDOMIZE_STREAM_GUARD["candidate_health"] = copy.deepcopy(health) if isinstance(health, dict) else health
    _randomize_stream_diag_update()


def _randomize_stream_guard_block_black(stage_label: str, source: str, health: dict | None, *, reason: str | None = None) -> None:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        _RANDOMIZE_STREAM_GUARD["black_frames_blocked_total"] = int(
            _RANDOMIZE_STREAM_GUARD.get("black_frames_blocked_total") or 0
        ) + 1
        if _RANDOMIZE_STREAM_GUARD.get("active"):
            if isinstance(_RANDOMIZE_STREAM_GUARD.get("frozen"), dict):
                _RANDOMIZE_STREAM_GUARD["mode"] = "frozen_last_good"
            else:
                _RANDOMIZE_STREAM_GUARD["mode"] = "black_blocked_no_frozen_frame"
        _RANDOMIZE_STREAM_GUARD["candidate_health"] = copy.deepcopy(health) if isinstance(health, dict) else health
        total = int(_RANDOMIZE_STREAM_GUARD.get("black_frames_blocked_total") or 0)
    _randomize_stream_diag_update()
    print(
        "[scene-randomize][stream-guard] black_frame_blocked "
        f"stage={stage_label} source={source} reason={reason or (health or {}).get('black_reason')} total={total}",
        flush=True,
    )


def _randomize_stream_guard_should_block_publish(health: dict | None = None) -> bool:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        mode = str(_RANDOMIZE_STREAM_GUARD.get("mode") or "")
        active = bool(_RANDOMIZE_STREAM_GUARD.get("active"))
        frozen = _RANDOMIZE_STREAM_GUARD.get("frozen")
    if active:
        return True
    if mode != "frozen_last_good":
        return False
    if isinstance(health, dict) and bool(health.get("healthy")):
        _randomize_stream_guard_finish(mode="idle")
        return False
    if not isinstance(frozen, dict):
        _randomize_stream_guard_finish(mode="idle")
        return False
    return True


def _randomize_stream_guard_publish_block_active() -> bool:
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        mode = str(_RANDOMIZE_STREAM_GUARD.get("mode") or "")
        active = bool(_RANDOMIZE_STREAM_GUARD.get("active"))
        frozen = _RANDOMIZE_STREAM_GUARD.get("frozen")
    if active:
        return True
    if mode == "frozen_last_good" and isinstance(frozen, dict):
        return True
    if mode == "frozen_last_good":
        _randomize_stream_guard_finish(mode="idle")
    return False


def _randomize_stream_guard_commit(
    rgba,
    source: str,
    *,
    capture_epoch_s: float | None = None,
) -> tuple[bool, dict]:
    commit_source = f"randomize_commit:{source}"
    raw, meta = _prepare_rtsp_rgba_frame(rgba, capture_epoch_s=capture_epoch_s)
    meta["randomize_mode"] = "committed"
    dropped = _rtsp_put_frame_bytes(raw, commit_source, meta)
    with _RANDOMIZE_STREAM_GUARD_LOCK:
        _RANDOMIZE_STREAM_GUARD["last_commit_source"] = commit_source
        _RANDOMIZE_STREAM_GUARD["mode"] = "committed"
    _randomize_stream_diag_update()
    return dropped, meta


def _decode_last_good_jpeg_to_rgba_hw4() -> np.ndarray | None:
    """将 snapshot `last_good_jpeg` 解码为 (H,W,4) uint8；供 randomize 期间 RTSP 补帧兜底。"""
    jpg, _fid, _cseq = _get_last_good_snapshot_jpeg()
    if not jpg:
        return None
    try:
        from PIL import Image as _PILImg  # noqa
        im = _PILImg.open(_io.BytesIO(jpg))
        im = im.convert("RGBA")
        arr = np.ascontiguousarray(np.asarray(im, dtype=np.uint8))
        if arr.ndim != 3 or arr.shape[2] != 4:
            return None
        if arr.shape[:2] != (H, W):
            arr = np.ascontiguousarray(np.resize(arr, (H, W, 4)))
        return arr
    except Exception:
        pass
    try:
        import cv2 as _cv2  # noqa
        buf = np.frombuffer(bytes(jpg), dtype=np.uint8)
        bgr = _cv2.imdecode(buf, _cv2.IMREAD_COLOR)
        if bgr is None or getattr(bgr, "size", 0) <= 0:
            return None
        rgb = bgr[:, :, ::-1].copy()
        al = np.full((rgb.shape[0], rgb.shape[1], 1), 255, dtype=np.uint8)
        rgba = np.ascontiguousarray(np.concatenate([rgb, al], axis=-1))
        if rgba.shape[:2] != (H, W):
            rgba = np.ascontiguousarray(np.resize(rgba, (H, W, 4)))
        return rgba
    except Exception:
        return None


def _log_randomize_rtsp_keepalive(
    *,
    phase: str,
    source: str,
    empty_frame: bool,
    queued_ok: bool,
) -> None:
    global _RTSP_RANDOMIZE_KEEPALIVE_LOG_MONO
    now = time.monotonic()
    if now - _RTSP_RANDOMIZE_KEEPALIVE_LOG_MONO < _RTSP_RANDOMIZE_KEEPALIVE_LOG_INTERVAL_S:
        return
    _RTSP_RANDOMIZE_KEEPALIVE_LOG_MONO = now
    try:
        qsz = _frame_queue.qsize()
    except Exception:
        qsz = -1
    try:
        age_ms = (
            (now - float(_rtsp_main_last_enqueue_mono)) * 1000.0
            if _rtsp_main_last_enqueue_mono > 0.0
            else -1.0
        )
    except Exception:
        age_ms = -1.0
    print(
        f"[scene-randomize][rtsp-keepalive] phase={phase} source={source} "
        f"empty_frame={empty_frame} queued_ok={queued_ok} "
        f"last_frame_age_ms={age_ms:.0f} queue_size={qsz}",
        flush=True,
    )


def _compose_renderer_hydra_observation_for_status_cached() -> dict:
    """供 _compose_status_dict 使用：限频 + 单飞刷新，避免多线程同时读 viewport/carb。"""
    global _renderer_obs_status_cache, _renderer_obs_status_cache_ts
    now = time.monotonic()
    with _RENDERER_OBS_STATUS_CACHE_LOCK:
        if (
            _renderer_obs_status_cache is not None
            and (now - _renderer_obs_status_cache_ts) < _RENDERER_OBS_STATUS_CACHE_TTL_S
        ):
            return dict(_renderer_obs_status_cache) if isinstance(_renderer_obs_status_cache, dict) else _renderer_obs_status_cache
    if not _RENDERER_OBS_STATUS_REFRESH_LOCK.acquire(timeout=0.12):
        with _RENDERER_OBS_STATUS_CACHE_LOCK:
            if _renderer_obs_status_cache is not None:
                return dict(_renderer_obs_status_cache) if isinstance(_renderer_obs_status_cache, dict) else _renderer_obs_status_cache
        return {"error": "busy", "note": "renderer_obs_refresh_contended"}
    try:
        now2 = time.monotonic()
        with _RENDERER_OBS_STATUS_CACHE_LOCK:
            if (
                _renderer_obs_status_cache is not None
                and (now2 - _renderer_obs_status_cache_ts) < _RENDERER_OBS_STATUS_CACHE_TTL_S
            ):
                return dict(_renderer_obs_status_cache) if isinstance(_renderer_obs_status_cache, dict) else _renderer_obs_status_cache
        try:
            fresh = _compose_renderer_hydra_observation()
        except Exception as exc:
            fresh = {"error": f"{type(exc).__name__}:{exc}"}
        with _RENDERER_OBS_STATUS_CACHE_LOCK:
            _renderer_obs_status_cache = fresh
            _renderer_obs_status_cache_ts = time.monotonic()
        return dict(fresh) if isinstance(fresh, dict) else fresh
    finally:
        _RENDERER_OBS_STATUS_REFRESH_LOCK.release()


def _http_get_status_body_cached() -> bytes:
    global _status_http_cache_body, _status_http_cache_ts
    now = time.monotonic()
    with _STATUS_HTTP_CACHE_LOCK:
        if (
            _status_http_cache_body is not None
            and (now - _status_http_cache_ts) < _STATUS_HTTP_CACHE_TTL_S
        ):
            return _status_http_cache_body
    stale_fallback: bytes | None = None
    with _STATUS_HTTP_CACHE_LOCK:
        stale_fallback = _status_http_cache_body
    if not _STATUS_HTTP_SINGLEFLIGHT_LOCK.acquire(timeout=_STATUS_HTTP_SINGLEFLIGHT_WAIT_S):
        if stale_fallback is not None:
            return stale_fallback
        with _CTRL_HTTP_ACTIVE_REQUESTS_LOCK:
            inf = int(_CTRL_HTTP_ACTIVE_REQUESTS)
        with _CTRL_PLANE_MAIN_DEGRADED_LOCK:
            main_busy = bool(_ctrl_plane_main_thread_degraded)
            main_reason = _ctrl_plane_main_thread_degraded_reason
        hp = {
            "from_main_thread_cache": True,
            "scene_randomize_lock_busy": main_busy,
        }
        fb = {
            "pan": float(_ptz_state_http_stale_cache.get("pan", 0.0)),
            "tilt": float(_ptz_state_http_stale_cache.get("tilt", 0.0)),
            "zoom": float(_ptz_state_http_stale_cache.get("zoom", 1.0)),
            "stream": {"degraded": True, "note": "status_singleflight_contended_cold_cache"},
            "scene": _read_status_http_scene_cache_dict(),
            "orientation": dict(_orientation_state),
            "startup_view": dict(_startup_view_state),
            "stale": True,
            "degraded": True,
            "degraded_reason": "status_compose_contended",
            "ctrl_plane": {
                "ctrl_http_inflight": inf,
                "ctrl_http_max_inflight": int(_CTRL_HTTP_MAX_INFLIGHT),
                "heavy_task_active": main_busy,
                "heavy_task_probe": hp,
                "last_health_fast_path_ts": float(_last_health_fast_path_ts),
                "last_snapshot_dt_ms": float(_last_snapshot_http_dt_ms),
                "last_rep_orchestrator_step_ms": float(_last_rep_orchestrator_step_ms),
                "busy_reason": main_reason or "status_compose_contended",
            },
        }
        return json.dumps(fb, ensure_ascii=False).encode("utf-8")
    try:
        now2 = time.monotonic()
        with _STATUS_HTTP_CACHE_LOCK:
            if (
                _status_http_cache_body is not None
                and (now2 - _status_http_cache_ts) < _STATUS_HTTP_CACHE_TTL_S
            ):
                return _status_http_cache_body
        body = json.dumps(
            _compose_status_dict(use_main_thread_scene_cache=True),
            ensure_ascii=False,
        ).encode("utf-8")
        with _STATUS_HTTP_CACHE_LOCK:
            _status_http_cache_body = body
            _status_http_cache_ts = time.monotonic()
        return body
    finally:
        _STATUS_HTTP_SINGLEFLIGHT_LOCK.release()


_SCENE_STATE_HTTP_REFRESH_LOCK = threading.Lock()

def _http_get_scene_state_body_cached() -> bytes:
    global _scene_state_http_cache_body, _scene_state_http_cache_ts
    now = time.monotonic()
    with _SCENE_STATE_HTTP_CACHE_LOCK:
        cached_body = _scene_state_http_cache_body
        cached_ts = _scene_state_http_cache_ts

    if cached_body is not None and (now - cached_ts) < _SCENE_STATE_HTTP_CACHE_TTL_S:
        return cached_body

    def _do_refresh():
        global _scene_state_http_cache_body, _scene_state_http_cache_ts
        try:
            body = json.dumps(_scene_state_snapshot(), ensure_ascii=False).encode("utf-8")
            with _SCENE_STATE_HTTP_CACHE_LOCK:
                _scene_state_http_cache_body = body
                _scene_state_http_cache_ts = time.monotonic()
        except Exception:
            pass
        finally:
            _SCENE_STATE_HTTP_REFRESH_LOCK.release()

    if _SCENE_STATE_HTTP_REFRESH_LOCK.acquire(blocking=False):
        threading.Thread(target=_do_refresh, daemon=True).start()

    if cached_body is not None:
        return cached_body
    return b'{"ok": false, "error": "unavailable_refreshing_cache"}'


def _invalidate_status_http_cache() -> None:
    global _status_http_cache_body, _status_http_cache_ts
    global _renderer_obs_status_cache, _renderer_obs_status_cache_ts
    with _STATUS_HTTP_CACHE_LOCK:
        _status_http_cache_body = None
        _status_http_cache_ts = 0.0
    with _RENDERER_OBS_STATUS_CACHE_LOCK:
        _renderer_obs_status_cache = None
        _renderer_obs_status_cache_ts = 0.0


def _invalidate_scene_state_http_cache() -> None:
    global _scene_state_http_cache_body, _scene_state_http_cache_ts
    with _SCENE_STATE_HTTP_CACHE_LOCK:
        _scene_state_http_cache_body = None
        _scene_state_http_cache_ts = 0.0


def _invalidate_hdri_state_http_cache() -> None:
    global _hdri_state_http_cache_body, _hdri_state_http_cache_ts
    with _HDRI_STATE_HTTP_CACHE_LOCK:
        _hdri_state_http_cache_body = None
        _hdri_state_http_cache_ts = 0.0


def _http_get_hdri_state_body_cached() -> bytes:
    global _hdri_state_http_cache_body, _hdri_state_http_cache_ts
    start_ts = time.time()
    now = time.monotonic()
    with _HDRI_STATE_HTTP_CACHE_LOCK:
        if (
            _hdri_state_http_cache_body is not None
            and (now - _hdri_state_http_cache_ts) < _HDRI_STATE_HTTP_CACHE_TTL_S
        ):
            _log_hdri_timing("/scene/hdri", start_ts, end_ts=time.time(), cache_hit=True, phase="status_get")
            return _hdri_state_http_cache_body
    body = json.dumps(_describe_hdri_control_state(), ensure_ascii=False).encode("utf-8")
    with _HDRI_STATE_HTTP_CACHE_LOCK:
        _hdri_state_http_cache_body = body
        _hdri_state_http_cache_ts = time.monotonic()
    _log_hdri_timing("/scene/hdri", start_ts, end_ts=time.time(), cache_hit=False, phase="status_get")
    return body


class _BoundedThreadingHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    # 略增 backlog，配合轻量 /api/health 与超时策略；避免单独依赖大队列掩盖根因
    request_queue_size = 96

    def __init__(self, server_address, request_handler_class):
        super().__init__(server_address, request_handler_class)
        self._request_slots = threading.BoundedSemaphore(_CTRL_HTTP_MAX_INFLIGHT)
        self._request_slot_flags: dict[int, bool] = {}
        self._request_slot_flags_lock = threading.Lock()

    @staticmethod
    def _request_path_is_light(request) -> bool:
        try:
            request.settimeout(0.03)
            peek = request.recv(512, socket.MSG_PEEK)
        except Exception:
            return False
        finally:
            try:
                request.settimeout(None)
            except Exception:
                pass
        try:
            line = peek.split(b"\r\n", 1)[0].decode("ascii", errors="ignore")
            parts = line.split()
            if len(parts) < 2 or parts[0] != "GET":
                return False
            path = parts[1].split("?", 1)[0]
            return path in _CTRL_HTTP_LIGHT_GET_PATHS
        except Exception:
            return False

    def process_request(self, request, client_address):
        is_light = self._request_path_is_light(request)
        acquired_slot = False
        if not is_light:
            acquired_slot = self._request_slots.acquire(timeout=_CTRL_HTTP_ACQUIRE_TIMEOUT_S)
        if not is_light and not acquired_slot:
            try:
                busy_body = json.dumps(
                    {"ok": False, "error": "control_plane_busy", "degraded": True},
                    ensure_ascii=False,
                ).encode("utf-8")
                cl = str(len(busy_body)).encode("ascii", "replace")
                request.sendall(
                    b"HTTP/1.1 503 Service Unavailable\r\n"
                    b"Connection: close\r\n"
                    b"Content-Type: application/json; charset=utf-8\r\n"
                    b"Content-Length: " + cl + b"\r\n\r\n" + busy_body
                )
            except Exception:
                pass
            self.shutdown_request(request)
            return
        with self._request_slot_flags_lock:
            self._request_slot_flags[id(request)] = acquired_slot
        try:
            super().process_request(request, client_address)
        except Exception:
            with self._request_slot_flags_lock:
                self._request_slot_flags.pop(id(request), None)
            if acquired_slot:
                self._request_slots.release()
            raise

    def process_request_thread(self, request, client_address):
        global _CTRL_HTTP_ACTIVE_REQUESTS
        with _CTRL_HTTP_ACTIVE_REQUESTS_LOCK:
            _CTRL_HTTP_ACTIVE_REQUESTS += 1
        try:
            super().process_request_thread(request, client_address)
        finally:
            with _CTRL_HTTP_ACTIVE_REQUESTS_LOCK:
                _CTRL_HTTP_ACTIVE_REQUESTS -= 1
            with self._request_slot_flags_lock:
                acquired_slot = bool(self._request_slot_flags.pop(id(request), False))
            if acquired_slot:
                self._request_slots.release()


def _init_jpeg_encoder(quality: int):
    """按优先级检测 JPEG 编码器：Pillow → OpenCV → 不可用。"""
    try:
        from PIL import Image as _PILImg  # noqa
        def _enc(rgba):
            img = _PILImg.fromarray(rgba[:, :, :3])
            buf = _io.BytesIO()
            img.save(buf, "JPEG", quality=quality, optimize=False)
            return buf.getvalue()
        print("[PTZ-RTSP] MJPEG 编码器：Pillow ✓")
        return _enc
    except ImportError:
        pass
    try:
        import cv2 as _cv2  # noqa
        _params = [_cv2.IMWRITE_JPEG_QUALITY, quality]
        def _enc(rgba):
            bgr = rgba[:, :, :3][:, :, ::-1].copy()
            ok, buf = _cv2.imencode(".jpg", bgr, _params)
            return buf.tobytes() if ok else None
        print("[PTZ-RTSP] MJPEG 编码器：OpenCV ✓")
        return _enc
    except ImportError:
        pass
    print("[PTZ-RTSP] ⚠ 未找到 PIL/OpenCV，MJPEG 不可用（仅 RTSP）")
    return None


def _list_cam_tilt_xform_op_names(tilt_prim) -> list[str]:
    """CamTilt prim 上 GetOrderedXformOps 的 op 名列表（只读）。"""
    from pxr import UsdGeom

    if not tilt_prim.IsValid():
        return []
    xf = UsdGeom.Xformable(tilt_prim)
    return [op.GetOpName() for op in xf.GetOrderedXformOps()]


def _resolve_or_create_tilt_rotate_attr(tilt_prim, preferred_name: str) -> str | None:
    """在 CamTilt 上解析/创建用于俯仰的 rotate 属性。

    关键原则：一旦 preferred 不存在，就强制创建 preferred，而不是“复用已有 rotateX/Y/Z”。
    否则很容易把 tilt 写到 yaw 轴上（例如当前场景 CamTilt 只有 rotateY，导致 tilt=-78 实际变成水平转动）。
    """
    from pxr import UsdGeom

    if not tilt_prim.IsValid():
        return None
    pa = tilt_prim.GetAttribute(preferred_name)
    if pa and pa.IsValid():
        return preferred_name
    xf = UsdGeom.Xformable(tilt_prim)
    if preferred_name == "xformOp:rotateX":
        xf.AddRotateXOp()
    elif preferred_name == "xformOp:rotateY":
        xf.AddRotateYOp()
    else:
        xf.AddRotateZOp()
    return preferred_name


def _read_rotate_attr_float(prim, attr_name: str) -> float | None:
    if not prim or not prim.IsValid():
        return None
    a = prim.GetAttribute(attr_name)
    if not (a and a.IsValid()):
        return None
    v = a.Get()
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None




def _get_world_translation(stage, prim_path: str) -> tuple[float, float, float] | None:
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return (float(world[0]), float(world[1]), float(world[2]))


def _compute_dynamic_lookat_pan_tilt(
    camera_xyz, target_xyz, *, tilt_max_deg: float = 30.0
):
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
    tmax = float(tilt_max_deg)
    pan_deg = max(-170.0, min(170.0, pan_deg))
    tilt_deg = max(-90.0, min(tmax, tilt_deg))
    return pan_deg, tilt_deg


def _record_orientation_state(*, camera_xyz, target_xyz, base_pan, base_tilt, applied_pan, applied_tilt, source, preset_name, fallback, fallback_reason):
    _orientation_state.update({
        "mode": str(_CAMERA_ORIENTATION_MODE),
        "camera_xyz": None if camera_xyz is None else tuple(float(v) for v in camera_xyz),
        "target_xyz": None if target_xyz is None else tuple(float(v) for v in target_xyz),
        "base_pan": None if base_pan is None else float(base_pan),
        "base_tilt": None if base_tilt is None else float(base_tilt),
        "base_tilt_direction_label": _tilt_direction_label(base_tilt),
        "applied_pan": None if applied_pan is None else float(applied_pan),
        "applied_tilt": None if applied_tilt is None else float(applied_tilt),
        "tilt_direction_label": _tilt_direction_label(applied_tilt),
        "applied_roll": 0.0,
        "last_source": str(source),
        "last_preset_name": str(preset_name),
        "fallback": bool(fallback),
        "fallback_reason": None if fallback_reason is None else str(fallback_reason),
    })


def _resolve_orientation_profile(base_pan: float, base_tilt: float, preset_name: str | None) -> tuple[float, float]:
    name = str(preset_name or "default_initial").strip().lower()
    if name in ("default_initial", "default"):
        return (
            float(base_pan + _DYNAMIC_STARTUP_PAN_OFFSET_DEG),
            float(base_tilt + _DYNAMIC_STARTUP_TILT_OFFSET_DEG),
        )
    if "??" in name or "??" in name or name in ("2", "forward", "front"):
        return float(base_pan), float(base_tilt)
    if "left" in name:
        return float(base_pan + _PRESET_RIGHT_PAN_OFFSET_DEG), float(base_tilt)
    if "right" in name:
        return float(base_pan + _PRESET_LEFT_PAN_OFFSET_DEG), float(base_tilt)
    if "??" in name or "overlook" in name or "top" in name:
        return float(base_pan), float(_PRESET_OVERLOOK_TILT_DEG)
    return float(base_pan), float(base_tilt)


def _apply_dynamic_lookat_after_random_camera(
    stage,
    camera_world_xyz,
    *,
    source: str = "random_camera",
    preset_name: str = "default_initial",
    apply_to_stage: bool = True,
    target_xyz=None,
    tilt_max_deg: float = 30.0,
) -> tuple[float, float] | None:
    mode = str(_CAMERA_ORIENTATION_MODE or "").strip().lower()
    resolved_target_xyz = tuple(float(v) for v in (target_xyz or _CAMERA_LOOKAT_TARGET_XYZ))
    if mode != "dynamic_lookat":
        _record_orientation_state(
            camera_xyz=camera_world_xyz,
            target_xyz=resolved_target_xyz,
            base_pan=None,
            base_tilt=None,
            applied_pan=None,
            applied_tilt=None,
            source=source,
            preset_name=preset_name,
            fallback=True,
            fallback_reason="mode_not_dynamic_lookat",
        )
        print(
            f"[camera-orientation] mode={_CAMERA_ORIENTATION_MODE} preset={preset_name} fallback_to_legacy_orientation=PASS reason=mode_not_dynamic_lookat",
            flush=True,
        )
        return None

    try:
        target_xyz = resolved_target_xyz
        base_pan, base_tilt = _compute_dynamic_lookat_pan_tilt(
            camera_world_xyz, target_xyz, tilt_max_deg=float(tilt_max_deg)
        )
        pan_deg, tilt_deg = _resolve_orientation_profile(base_pan, base_tilt, preset_name)
    except Exception as exc:
        _record_orientation_state(
            camera_xyz=camera_world_xyz,
            target_xyz=resolved_target_xyz,
            base_pan=None,
            base_tilt=None,
            applied_pan=None,
            applied_tilt=None,
            source=source,
            preset_name=preset_name,
            fallback=True,
            fallback_reason=str(exc),
        )
        print(
            f"[camera-orientation] mode={_CAMERA_ORIENTATION_MODE} preset={preset_name} fallback_to_legacy_orientation=PASS reason={exc}",
            flush=True,
        )
        return None

    startup_fallback_applied = False
    startup_fallback_reason = None
    startup_preferred_pan = None
    startup_preferred_tilt = None
    src_l = str(source or "").strip().lower()
    if (
        (
            src_l == "random_camera"
            or src_l == "randomize_perception_refine"
            or src_l == "random_scene_api"
            or src_l == "random_scene_api_keep_camera"
        )
        and str(preset_name or "").strip().lower() in ("default_initial", "default")
        and _CAMERA_RIG_PRIM
        and _GONDOLA_PRIM
    ):
        startup_metrics = resolve_dynamic_startup_view_metrics(
            stage,
            _CAMERA_RIG_PRIM,
            _GONDOLA_PRIM,
            lookat_target_xyz=target_xyz,
            resolution_wh=(W, H),
            prefer_target_prim_center=False,
            dynamic_startup_pan_offset_deg=_DYNAMIC_STARTUP_PAN_OFFSET_DEG,
            dynamic_startup_tilt_offset_deg=_DYNAMIC_STARTUP_TILT_OFFSET_DEG,
        )
        startup_preferred_pan = startup_metrics.get("startup_preferred_pan")
        startup_preferred_tilt = startup_metrics.get("startup_preferred_tilt")
        if startup_metrics.get("visible", False):
            chosen_pan = startup_metrics.get("applied_pan")
            chosen_tilt = startup_metrics.get("applied_tilt")
            if chosen_pan is not None and chosen_tilt is not None:
                pan_deg = float(chosen_pan)
                tilt_deg = float(chosen_tilt)
                startup_fallback_applied = bool(startup_metrics.get("startup_fallback_applied", False))
                if startup_fallback_applied:
                    startup_fallback_reason = str(
                        startup_metrics.get("startup_preferred_rejection_reason")
                        or "preferred_startup_orientation_not_visible"
                    )

    pan_deg = max(-170.0, min(170.0, float(pan_deg)))
    tilt_deg = max(-90.0, min(float(tilt_max_deg), float(tilt_deg)))

    with _ptz_lock:
        _ptz_state["pan"] = float(pan_deg)
        _ptz_state["tilt"] = float(tilt_deg)
        zoom_now = float(_ptz_state["zoom"])

    if apply_to_stage:
        _apply_ptz_state(stage)
    applied_camera_xyz = _get_world_translation(stage, _CAMERA_RIG_PRIM) or tuple(float(v) for v in camera_world_xyz)
    _record_orientation_state(
        camera_xyz=applied_camera_xyz,
        target_xyz=target_xyz,
        base_pan=base_pan,
        base_tilt=base_tilt,
        applied_pan=pan_deg,
        applied_tilt=tilt_deg,
        source=source,
        preset_name=preset_name,
        fallback=False,
        fallback_reason=None,
    )
    if (
        src_l in (
            "random_camera",
            "randomize_perception_refine",
            "random_scene_api",
            "random_scene_api_keep_camera",
        )
        and str(preset_name or "").strip().lower() in ("default_initial", "default")
        and _CAMERA_RIG_PRIM
        and _GONDOLA_PRIM
    ):
        _refine_committed_orientation_for_context_visibility(
            stage,
            camera_world_xyz=applied_camera_xyz,
            target_xyz=target_xyz,
            visibility_detail={
                "visible": True,
                "base_pan": base_pan,
                "base_tilt": base_tilt,
                "startup_preferred_pan": startup_preferred_pan,
                "startup_preferred_tilt": startup_preferred_tilt,
            },
            base_pan=base_pan,
            base_tilt=base_tilt,
            source=source,
            preset_name=preset_name,
            tilt_max_deg=tilt_max_deg,
        )
        applied_camera_xyz = _get_world_translation(stage, _CAMERA_RIG_PRIM) or applied_camera_xyz
    print(
        "[camera-orientation] "
        f"mode={_CAMERA_ORIENTATION_MODE} source={source} preset={preset_name} "
        f"camera_xyz={tuple(round(float(v), 4) for v in applied_camera_xyz)} "
        f"lookat_target_xyz={tuple(round(float(v), 4) for v in target_xyz)} "
        f"base_pan={base_pan:.4f} base_tilt={base_tilt:.4f} "
        f"applied_pan={pan_deg:.4f} applied_tilt={tilt_deg:.4f} roll=0.0000 zoom={zoom_now:.4f} "
        f"startup_fallback_applied={startup_fallback_applied} "
        f"startup_preferred_pan={startup_preferred_pan} startup_preferred_tilt={startup_preferred_tilt} "
        f"startup_fallback_reason={startup_fallback_reason} "
        f"fallback=False applied=PASS",
        flush=True,
    )
    return (pan_deg, tilt_deg)


def _commit_visibility_checked_orientation(
    stage,
    camera_world_xyz,
    target_xyz,
    visibility_detail: dict | None,
    *,
    source: str,
    preset_name: str = "default_initial",
    tilt_max_deg: float = 30.0,
) -> tuple[float, float] | None:
    """Apply the visible pan/tilt found by the startup visibility search."""
    if not isinstance(visibility_detail, dict) or not visibility_detail.get("visible", False):
        return None
    try:
        pan_deg = float(visibility_detail["applied_pan"])
        tilt_deg = float(visibility_detail["applied_tilt"])
    except (KeyError, TypeError, ValueError):
        return None

    target_tuple = tuple(float(v) for v in (target_xyz or _CAMERA_LOOKAT_TARGET_XYZ))
    try:
        base_pan = float(visibility_detail.get("base_pan"))
        base_tilt = float(visibility_detail.get("base_tilt"))
    except (TypeError, ValueError):
        try:
            base_pan, base_tilt = _compute_dynamic_lookat_pan_tilt(
                camera_world_xyz,
                target_tuple,
                tilt_max_deg=float(tilt_max_deg),
            )
        except Exception:
            base_pan = base_tilt = None

    pan_deg = max(-170.0, min(170.0, float(pan_deg)))
    tilt_deg = max(-90.0, min(float(tilt_max_deg), float(tilt_deg)))
    with _ptz_lock:
        _ptz_state["pan"] = float(pan_deg)
        _ptz_state["tilt"] = float(tilt_deg)
        zoom_now = float(_ptz_state["zoom"])

    _apply_ptz_state(stage)
    applied_camera_xyz = (
        _get_world_translation(stage, _CAMERA_RIG_PRIM)
        or tuple(float(v) for v in camera_world_xyz)
    )
    _record_orientation_state(
        camera_xyz=applied_camera_xyz,
        target_xyz=target_tuple,
        base_pan=base_pan,
        base_tilt=base_tilt,
        applied_pan=pan_deg,
        applied_tilt=tilt_deg,
        source=source,
        preset_name=preset_name,
        fallback=False,
        fallback_reason=None,
    )
    print(
        "[camera-orientation] "
        f"source={source} committed_visibility_checked_orientation=PASS "
        f"camera_xyz={tuple(round(float(v), 4) for v in applied_camera_xyz)} "
        f"lookat_target_xyz={tuple(round(float(v), 4) for v in target_tuple)} "
        f"applied_pan={pan_deg:.4f} applied_tilt={tilt_deg:.4f} zoom={zoom_now:.4f}",
        flush=True,
    )
    _refine_committed_orientation_for_context_visibility(
        stage,
        camera_world_xyz=applied_camera_xyz,
        target_xyz=target_tuple,
        visibility_detail=visibility_detail,
        base_pan=base_pan,
        base_tilt=base_tilt,
        source=source,
        preset_name=preset_name,
        tilt_max_deg=tilt_max_deg,
    )
    return (pan_deg, tilt_deg)


def _projection_frame_overlap_ratio(metrics: dict | None) -> float:
    if not isinstance(metrics, dict) or metrics.get("error"):
        return 0.0
    bbox = metrics.get("屏幕包围框px")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return 0.0
    try:
        min_x, min_y, max_x, max_y = [float(v) for v in bbox]
    except (TypeError, ValueError):
        return 0.0
    inter_w = max(0.0, min(max_x, float(W)) - max(min_x, 0.0))
    inter_h = max(0.0, min(max_y, float(H)) - max(min_y, 0.0))
    return max(0.0, min(1.0, (inter_w * inter_h) / max(1.0, float(W * H))))


def _projection_metric_float(metrics: dict | None, key: str, default: float = 0.0) -> float:
    if not isinstance(metrics, dict):
        return float(default)
    try:
        val = float(metrics.get(key))
        if math.isfinite(val):
            return val
    except (TypeError, ValueError):
        pass
    return float(default)


def _projection_target_visible_enough(metrics: dict | None) -> bool:
    if not isinstance(metrics, dict) or metrics.get("error"):
        return False
    if metrics.get("frustum内可见") is False:
        return False
    if not bool(metrics.get("中心在画面内")):
        return False
    if not bool(metrics.get("相机前方", True)):
        return False
    overlap = _projection_frame_overlap_ratio(metrics)
    if overlap < 0.001:
        return False
    overflow = _projection_metric_float(metrics, "越界面积占比", 1.0)
    if overflow > 0.005:
        return False
    bbox = metrics.get("屏幕包围框px")
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        return False
    try:
        min_x, min_y, max_x, max_y = [float(v) for v in bbox]
    except (TypeError, ValueError):
        return False
    if not all(math.isfinite(v) for v in (min_x, min_y, max_x, max_y)):
        return False
    margin = max(4.0, min(float(W), float(H)) * 0.01)
    return (
        min_x >= margin
        and min_y >= margin
        and max_x <= float(W) - margin
        and max_y <= float(H) - margin
    )


def _projection_context_visible_enough(metrics: dict | None) -> bool:
    if not isinstance(metrics, dict) or metrics.get("error"):
        return False
    if metrics.get("frustum内可见") is False:
        return False
    if not bool(metrics.get("中心在画面内")):
        return False
    if _projection_metric_float(metrics, "越界面积占比", 1.0) > 0.5:
        return False
    return _projection_frame_overlap_ratio(metrics) >= 0.01


def _bbox_info_for_prim(stage, prim_path: str | None, bbox_cache=None) -> dict | None:
    try:
        from pxr import Usd, UsdGeom

        ps = str(prim_path or "").strip()
        if not ps:
            return None
        prim = stage.GetPrimAtPath(ps)
        if not prim.IsValid():
            return None
        cache = bbox_cache or UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
        rng = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        mn = rng.GetMin()
        mx = rng.GetMax()
        vals = [float(mn[0]), float(mn[1]), float(mn[2]), float(mx[0]), float(mx[1]), float(mx[2])]
        if not all(math.isfinite(v) for v in vals):
            return None
        return {
            "path": ps,
            "min": (vals[0], vals[1], vals[2]),
            "max": (vals[3], vals[4], vals[5]),
            "mid": ((vals[0] + vals[3]) * 0.5, (vals[1] + vals[4]) * 0.5, (vals[2] + vals[5]) * 0.5),
            "spans": (max(0.0, vals[3] - vals[0]), max(0.0, vals[4] - vals[1]), max(0.0, vals[5] - vals[2])),
        }
    except Exception:
        return None


def _interval_gap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
    if a_max < b_min:
        return float(b_min - a_max)
    if b_max < a_min:
        return float(a_min - b_max)
    return 0.0


def _interval_overlap(a_min: float, a_max: float, b_min: float, b_max: float) -> float:
    return max(0.0, min(float(a_max), float(b_max)) - max(float(a_min), float(b_min)))


def _active_diaolan_runtime_paths(stage) -> dict:
    with _scene_lock:
        active_root = str(
            _scene_state.get("selected_diaolan_path")
            or _scene_state.get("active_diaolan_path")
            or ""
        ).strip()
    if not active_root and _GONDOLA_PRIM:
        active_root = infer_world_diaolan_instance_root(_GONDOLA_PRIM) or ""
    height_path = str(_GONDOLA_PRIM or "").strip()
    assembly_path = ""
    if active_root:
        try:
            ht, asm = resolve_diaolan_height_and_assembly(stage, active_root.rstrip("/"))
            if ht:
                height_path = str(ht).strip()
            if asm:
                assembly_path = str(asm).strip()
        except Exception:
            pass
    return {
        "active_root": active_root or None,
        "gondola_prim": height_path or None,
        "assembly_prim": assembly_path or None,
    }


def _is_excluded_building_context_path(path: str, active_paths: dict) -> bool:
    p = str(path or "").strip().rstrip("/")
    if not p:
        return True
    low = p.lower()
    for token in (
        "camera",
        "light",
        "domelight",
        "sky",
        "hdri",
        "background",
        "backdrop",
        "billboard",
        "sticker",
        "decal",
        "poster",
        "dynamic_sky",
        "dynamicsky",
        "gondola",
        "diaolan",
        "worker",
        "people",
        "person",
    ):
        if token in low:
            return True
    for raw in (
        active_paths.get("active_root"),
        active_paths.get("gondola_prim"),
        active_paths.get("assembly_prim"),
        _CAMERA_RIG_PRIM,
    ):
        base = str(raw or "").strip().rstrip("/")
        if base and (p == base or p.startswith(base + "/")):
            return True
    return False


def _nearby_building_context_selection(stage) -> dict:
    """
    选择“当前活动吊篮所在楼体/立面”的局部 USD 几何。

    关键约束：只以活动吊篮 bbox 为锚点做近邻筛选，不再把配置里的
    Architecture_High 或整场景根包围盒当作已对准的楼体上下文。
    """
    try:
        from pxr import Usd, UsdGeom

        active_paths = _active_diaolan_runtime_paths(stage)
        gondola_path = active_paths.get("gondola_prim")
        gbox = _bbox_info_for_prim(stage, gondola_path)
        if not gbox:
            return {
                "prim_path": None,
                "reason": "active_gondola_bbox_unavailable",
                "active_paths": active_paths,
            }

        root_path = str(_CHANGJING_PRIM_PATH or "").strip() or "/World"
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            root_path = "/World"
            root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            return {
                "prim_path": None,
                "reason": "scene_root_unavailable",
                "active_paths": active_paths,
            }

        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
        scene_box = _bbox_info_for_prim(stage, root_path, cache)
        h_axis = _height_axis_index()
        h_axes = [idx for idx in (0, 1, 2) if idx != h_axis]
        g_min, g_max, g_spans = gbox["min"], gbox["max"], gbox["spans"]
        g_h_span = max(0.1, float(g_spans[h_axis]))
        max_horizontal_gap = max(8.0, min(28.0, max(float(g_spans[h_axes[0]]), float(g_spans[h_axes[1]])) * 4.0 + 8.0))
        max_vertical_gap = max(6.0, min(18.0, g_h_span * 4.0 + 4.0))
        scene_spans = scene_box.get("spans") if isinstance(scene_box, dict) else None
        candidates: list[dict] = []
        scanned_meshes = 0

        for prim in Usd.PrimRange(root):
            if prim == root or not prim.IsA(UsdGeom.Mesh):
                continue
            scanned_meshes += 1
            path = prim.GetPath().pathString
            if _is_excluded_building_context_path(path, active_paths):
                continue
            info = _bbox_info_for_prim(stage, path, cache)
            if not info:
                continue
            mn, mx, spans = info["min"], info["max"], info["spans"]
            if any(float(v) <= 1e-5 for v in spans):
                continue

            vertical_span = float(spans[h_axis])
            horiz_spans = [float(spans[h_axes[0]]), float(spans[h_axes[1]])]
            if vertical_span < 1.2:
                continue
            if max(horiz_spans) < 0.8:
                continue
            if isinstance(scene_spans, tuple):
                if max(horiz_spans) > max(45.0, max(float(scene_spans[h_axes[0]]), float(scene_spans[h_axes[1]])) * 0.55):
                    continue
                if vertical_span > max(45.0, float(scene_spans[h_axis]) * 0.9):
                    continue

            gaps = [
                _interval_gap(g_min[axis], g_max[axis], mn[axis], mx[axis])
                for axis in h_axes
            ]
            horizontal_gap = math.hypot(float(gaps[0]), float(gaps[1]))
            vertical_gap = _interval_gap(g_min[h_axis], g_max[h_axis], mn[h_axis], mx[h_axis])
            if horizontal_gap > max_horizontal_gap or vertical_gap > max_vertical_gap:
                continue

            overlap_h = _interval_overlap(g_min[h_axis], g_max[h_axis], mn[h_axis], mx[h_axis])
            overlap_ratio = overlap_h / max(g_h_span, 1e-6)
            facade_like = 1.0 if min(horiz_spans) <= max(1.8, max(horiz_spans) * 0.32) else 0.0
            low = path.lower()
            semantic_bonus = 1.0 if any(t in low for t in ("architecture", "building", "wall", "facade", "qiang", "lou", "jianzhu")) else 0.0
            size_penalty = max(0.0, (max(horiz_spans) - 18.0) * 0.02) + max(0.0, (vertical_span - 30.0) * 0.03)
            score = (
                -horizontal_gap * 3.0
                -vertical_gap * 0.6
                +overlap_ratio * 4.0
                +facade_like * 1.5
                +semantic_bonus
                -size_penalty
            )
            candidates.append(
                {
                    "prim_path": path,
                    "score": round(float(score), 4),
                    "horizontal_gap": round(float(horizontal_gap), 4),
                    "vertical_gap": round(float(vertical_gap), 4),
                    "height_overlap_ratio": round(float(overlap_ratio), 4),
                    "spans": [round(float(v), 4) for v in spans],
                    "mid": [round(float(v), 4) for v in info["mid"]],
                }
            )

        candidates.sort(key=lambda c: (-float(c["score"]), float(c["horizontal_gap"]), str(c["prim_path"])))
        best = candidates[0] if candidates else None
        return {
            "prim_path": best.get("prim_path") if best else None,
            "reason": "nearby_active_gondola_building_mesh" if best else "no_nearby_building_mesh_candidate",
            "active_paths": active_paths,
            "active_gondola_bbox": {
                "min": [round(float(v), 4) for v in gbox["min"]],
                "max": [round(float(v), 4) for v in gbox["max"]],
                "mid": [round(float(v), 4) for v in gbox["mid"]],
                "spans": [round(float(v), 4) for v in gbox["spans"]],
            },
            "search_root": root_path,
            "scanned_meshes": int(scanned_meshes),
            "candidate_count": int(len(candidates)),
            "candidate_sample": candidates[:8],
        }
    except Exception as exc:
        return {"prim_path": None, "reason": f"error:{type(exc).__name__}:{exc}"}


def _context_lookat_selection(stage) -> dict:
    selection = _nearby_building_context_selection(stage)
    prim_path = str(selection.get("prim_path") or "").strip()
    if prim_path and prim_path != str(_GONDOLA_PRIM or "").strip():
        return selection
    selection["prim_path"] = None
    return selection


_BUILDING_CONTEXT_SELECTION_CACHE = {}


def _building_context_cache_key(stage, active_paths: dict | None = None, gbox: dict | None = None):
    try:
        root = stage.GetRootLayer() if stage else None
        stage_key = str(getattr(root, "identifier", "") or id(stage))
    except Exception:
        stage_key = str(id(stage))
    active_paths = active_paths if isinstance(active_paths, dict) else _active_diaolan_runtime_paths(stage)
    if not isinstance(gbox, dict):
        gbox = _bbox_info_for_prim(stage, active_paths.get("gondola_prim")) if stage is not None else None
    bbox_key = None
    if isinstance(gbox, dict):
        bbox_key = (
            tuple(round(float(v), 1) for v in gbox.get("min", ())),
            tuple(round(float(v), 1) for v in gbox.get("max", ())),
        )
    return (stage_key, str(active_paths.get("active_root") or ""), bbox_key)


def _clear_building_context_selection_cache() -> None:
    _BUILDING_CONTEXT_SELECTION_CACHE.clear()


def _cached_context_lookat_selection(stage) -> dict:
    try:
        active_paths = _active_diaolan_runtime_paths(stage)
        gbox = _bbox_info_for_prim(stage, active_paths.get("gondola_prim"))
        key = _building_context_cache_key(stage, active_paths, gbox)
        cached = _BUILDING_CONTEXT_SELECTION_CACHE.get(key)
        if isinstance(cached, dict):
            out = copy.deepcopy(cached)
            out["cache_hit"] = True
            return out
        selection = _nearby_building_context_selection(stage)
        selection["cache_hit"] = False
        _BUILDING_CONTEXT_SELECTION_CACHE[key] = copy.deepcopy(selection)
        return selection
    except Exception as exc:
        return {"prim_path": None, "reason": f"cache_error:{type(exc).__name__}:{exc}", "cache_hit": False}


def _building_context_from_config_target(stage) -> dict | None:
    cfg_path = str(_LOOKAT_TARGET_PRIM_PATH or "").strip()
    if not cfg_path:
        return None
    active_paths = _active_diaolan_runtime_paths(stage)
    gbox = _bbox_info_for_prim(stage, active_paths.get("gondola_prim"))
    bbox = _bbox_info_for_prim(stage, cfg_path)
    if not gbox or not bbox:
        return None
    h_axis = _height_axis_index()
    h_axes = [idx for idx in (0, 1, 2) if idx != h_axis]
    gaps = [
        _interval_gap(gbox["min"][axis], gbox["max"][axis], bbox["min"][axis], bbox["max"][axis])
        for axis in h_axes
    ]
    horizontal_gap = math.hypot(float(gaps[0]), float(gaps[1]))
    vertical_gap = _interval_gap(gbox["min"][h_axis], gbox["max"][h_axis], bbox["min"][h_axis], bbox["max"][h_axis])
    if horizontal_gap > 45.0 or vertical_gap > 30.0:
        return None
    return {
        "prim_path": cfg_path,
        "reason": "configured_building_context_near_active_gondola",
        "active_paths": active_paths,
        "horizontal_gap": round(float(horizontal_gap), 4),
        "vertical_gap": round(float(vertical_gap), 4),
        "active_gondola_bbox": {
            "min": [round(float(v), 4) for v in gbox["min"]],
            "max": [round(float(v), 4) for v in gbox["max"]],
            "mid": [round(float(v), 4) for v in gbox["mid"]],
            "spans": [round(float(v), 4) for v in gbox["spans"]],
        },
    }


def _wall_sampling_target_context(stage, active_diaolan: dict | None = None) -> dict:
    selection = _cached_context_lookat_selection(stage)
    prim_path = str(selection.get("prim_path") or "").strip()
    box = _bbox_info_for_prim(stage, prim_path) if prim_path else None
    if not box and isinstance(active_diaolan, dict):
        fallback_path = str(active_diaolan.get("group1") or active_diaolan.get("path") or "").strip()
        box = _bbox_info_for_prim(stage, fallback_path)
        if box:
            prim_path = fallback_path
            selection = {
                "prim_path": fallback_path,
                "reason": "fallback_active_diaolan_bbox",
                "context_selection_before_fallback": selection,
            }
    if not box:
        return {
            "prim_path": None,
            "bbox": None,
            "selection": selection,
            "reason": selection.get("reason") or "target_context_bbox_unavailable",
        }
    return {
        "prim_path": prim_path,
        "bbox": {
            "prim_path": prim_path,
            "min": [float(v) for v in box["min"]],
            "max": [float(v) for v in box["max"]],
            "mid": [float(v) for v in box["mid"]],
            "spans": [float(v) for v in box["spans"]],
        },
        "selection": selection,
        "reason": selection.get("reason") or "ok",
    }


def _context_lookat_prim_path(stage) -> str | None:
    selection = _building_context_from_config_target(stage) or _context_lookat_selection(stage)
    return str(selection.get("prim_path") or "").strip() or None


def _refine_committed_orientation_for_context_visibility(
    stage,
    *,
    camera_world_xyz,
    target_xyz,
    visibility_detail: dict,
    base_pan,
    base_tilt,
    source: str,
    preset_name: str,
    tilt_max_deg: float,
) -> None:
    context_selection = _building_context_from_config_target(stage) or _context_lookat_selection(stage)
    context_prim = str(context_selection.get("prim_path") or "").strip()
    if not context_prim or not _GONDOLA_PRIM:
        visibility_detail["context_selection"] = context_selection
        return

    with _ptz_lock:
        initial_pan = float(_ptz_state["pan"])
        initial_tilt = float(_ptz_state["tilt"])
        initial_zoom = float(_ptz_state["zoom"])

    def _apply_candidate(pan: float, tilt: float, zoom: float) -> tuple[dict | None, dict | None]:
        with _ptz_lock:
            _ptz_state["pan"] = max(-170.0, min(170.0, float(pan)))
            _ptz_state["tilt"] = max(-90.0, min(float(tilt_max_deg), float(tilt)))
            _ptz_state["zoom"] = max(1.0, min(32.0, float(zoom)))
        _apply_ptz_state(stage)
        target_m = _prim_projection_metrics(stage, camera_prim, _GONDOLA_PRIM)
        context_m = _prim_projection_metrics(stage, camera_prim, context_prim)
        return target_m, context_m

    pan_anchors: list[float] = [initial_pan, 0.0]
    for raw in (base_pan, visibility_detail.get("startup_preferred_pan")):
        try:
            val = float(raw)
        except (TypeError, ValueError):
            continue
        if all(abs(val - existing) > 1e-4 for existing in pan_anchors):
            pan_anchors.append(val)
    recent_down_count = sum(1 for v in _randomize_context_tilt_history if _is_down_tilt(v))
    down_allowed = recent_down_count < int(_RANDOMIZE_CONTEXT_DOWN_TILT_MAX_IN_WINDOW)
    prefer_down_this_time = (
        down_allowed and random.random() < float(_RANDOMIZE_CONTEXT_DOWN_TILT_PROBABILITY)
    )
    neutral_tilt_candidates = (-8.0, -4.0, 0.0, 4.0)
    down_tilt_candidates = (
        8.0,
        16.0,
        24.0,
    )
    dynamic_tilt_candidates = (
        initial_tilt,
        base_tilt,
        visibility_detail.get("startup_preferred_tilt"),
    )
    if prefer_down_this_time:
        raw_tilt_candidates = down_tilt_candidates + neutral_tilt_candidates + dynamic_tilt_candidates
    else:
        raw_tilt_candidates = neutral_tilt_candidates + dynamic_tilt_candidates + down_tilt_candidates

    tilt_values: list[float] = []
    for raw in raw_tilt_candidates:
        try:
            val = max(-90.0, min(float(tilt_max_deg), float(raw)))
        except (TypeError, ValueError):
            continue
        if not down_allowed and _is_down_tilt(val):
            continue
        if all(abs(val - existing) > 1e-4 for existing in tilt_values):
            tilt_values.append(val)
    zoom_values: list[float] = []
    for raw in (1.0, 1.15, 1.3, initial_zoom):
        val = max(1.0, min(32.0, float(raw)))
        if all(abs(val - existing) > 1e-4 for existing in zoom_values):
            zoom_values.append(val)

    pan_offsets = [0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -90.0, 90.0]
    candidates: list[tuple[float, float, float]] = []
    if down_allowed or not _is_down_tilt(initial_tilt):
        candidates.append((initial_pan, initial_tilt, initial_zoom))
    for anchor in pan_anchors:
        for offset in pan_offsets:
            pan = max(-170.0, min(170.0, float(anchor) + float(offset)))
            for tilt in tilt_values:
                for zoom in zoom_values:
                    cand = (round(pan, 4), round(float(tilt), 4), round(float(zoom), 4))
                    if cand not in candidates:
                        candidates.append(cand)
    if len(candidates) > _RANDOMIZE_CONTEXT_ORIENTATION_MAX_CANDIDATES:
        candidates = candidates[:_RANDOMIZE_CONTEXT_ORIENTATION_MAX_CANDIDATES]

    incoming_target_visible = bool(visibility_detail.get("visible", False))
    best = None
    best_score = None
    evaluated_count = 0
    for pan, tilt, zoom in candidates:
        evaluated_count += 1
        target_m, context_m = _apply_candidate(pan, tilt, zoom)
        target_ok = _projection_target_visible_enough(target_m)
        target_overlap = _projection_frame_overlap_ratio(target_m)
        target_overflow = _projection_metric_float(target_m, "越界面积占比", 1.0)
        context_overlap = _projection_frame_overlap_ratio(context_m)
        context_ok = _projection_context_visible_enough(context_m)
        center = target_m.get("中心点px") if isinstance(target_m, dict) else None
        if isinstance(center, (list, tuple)) and len(center) >= 2:
            center_penalty = (
                abs(float(center[0]) - float(W) * 0.5) / max(1.0, float(W))
                + abs(float(center[1]) - float(H) * 0.5) / max(1.0, float(H))
            )
        else:
            center_penalty = 9.0
        level_tilt_score = 0.0
        if not prefer_down_this_time or not down_allowed:
            level_tilt_score = -abs(float(tilt) - -2.0)
        score = (
            10000.0 if target_ok else 0.0,
            1000.0 if target_ok and context_ok else 0.0,
            float(target_overlap) * 100.0,
            -float(target_overflow) * 20.0,
            float(context_overlap),
            -float(center_penalty),
            float(level_tilt_score),
            -float(zoom),
        )
        if best is None or score > best_score:
            best = (
                pan,
                tilt,
                zoom,
                target_m,
                context_m,
                target_ok,
                context_ok,
                context_overlap,
                target_overlap,
                target_overflow,
            )
            best_score = score
        if target_ok and context_ok and (prefer_down_this_time or not _is_down_tilt(tilt)):
            break
        if (
            incoming_target_visible
            and context_ok
            and context_overlap >= 0.08
            and target_overflow <= 0.25
            and (prefer_down_this_time or not _is_down_tilt(tilt))
        ):
            break

    if best is None:
        _apply_candidate(initial_pan, initial_tilt, initial_zoom)
        return

    (
        pan,
        tilt,
        zoom,
        target_m,
        context_m,
        target_ok,
        context_ok,
        context_overlap,
        target_overlap,
        target_overflow,
    ) = best
    if not target_ok and not incoming_target_visible:
        target_m, context_m = _apply_candidate(initial_pan, initial_tilt, initial_zoom)
        pan, tilt, zoom = initial_pan, initial_tilt, initial_zoom
        target_ok = _projection_target_visible_enough(target_m)
        target_overlap = _projection_frame_overlap_ratio(target_m)
        target_overflow = _projection_metric_float(target_m, "越界面积占比", 1.0)
        context_ok = _projection_context_visible_enough(context_m)
        context_overlap = _projection_frame_overlap_ratio(context_m)
    else:
        _apply_candidate(pan, tilt, zoom)

    applied_camera_xyz = (
        _get_world_translation(stage, _CAMERA_RIG_PRIM)
        or tuple(float(v) for v in camera_world_xyz)
    )
    _record_orientation_state(
        camera_xyz=applied_camera_xyz,
        target_xyz=tuple(float(v) for v in target_xyz),
        base_pan=base_pan,
        base_tilt=base_tilt,
        applied_pan=pan,
        applied_tilt=tilt,
        source=f"{source}_context",
        preset_name=preset_name,
        fallback=False,
        fallback_reason=None,
    )
    visibility_detail["context_prim_path"] = context_prim
    visibility_detail["context_selection"] = context_selection
    visibility_detail["context_projection_metrics"] = context_m
    visibility_detail["context_visible"] = bool(context_ok)
    visibility_detail["context_frame_overlap_ratio"] = round(float(context_overlap), 6)
    visibility_detail["target_frame_overlap_ratio_after_context"] = round(float(target_overlap), 6)
    visibility_detail["target_overflow_ratio_after_context"] = round(float(target_overflow), 6)
    visibility_detail["context_commit_pan"] = round(float(pan), 4)
    visibility_detail["context_commit_tilt"] = round(float(tilt), 4)
    visibility_detail["context_commit_tilt_direction_label"] = _tilt_direction_label(tilt)
    visibility_detail["context_commit_zoom"] = round(float(zoom), 4)
    visibility_detail["context_candidates_evaluated"] = int(evaluated_count)
    visibility_detail["context_candidate_limit"] = int(_RANDOMIZE_CONTEXT_ORIENTATION_MAX_CANDIDATES)
    _randomize_context_tilt_history.append(float(tilt))
    visibility_detail["context_down_tilt_policy"] = {
        "window": int(_RANDOMIZE_CONTEXT_DOWN_TILT_WINDOW),
        "max_down_in_window": int(_RANDOMIZE_CONTEXT_DOWN_TILT_MAX_IN_WINDOW),
        "down_probability": float(_RANDOMIZE_CONTEXT_DOWN_TILT_PROBABILITY),
        "down_threshold_deg": float(_TILT_DIRECTION_THRESHOLD_DEG),
        "down_condition": "tilt > down_threshold_deg",
        "tilt_semantics": "positive_down_negative_up",
        "recent_down_count_before": int(recent_down_count),
        "down_allowed": bool(down_allowed),
        "prefer_down_this_time": bool(prefer_down_this_time),
        "recent_tilts_after": [round(float(v), 4) for v in _randomize_context_tilt_history],
        "recent_tilt_direction_labels_after": [
            _tilt_direction_label(v) for v in _randomize_context_tilt_history
        ],
    }
    _stream_diag_update(lookat_context_projection_metrics=context_m)
    print(
        "[camera-orientation] "
        f"source={source}_context target_ok={target_ok} context_ok={context_ok} "
        f"target_overlap={target_overlap:.6f} target_overflow={target_overflow:.6f} "
        f"context_prim={context_prim!r} context_overlap={context_overlap:.6f} "
        f"applied_pan={pan:.4f} applied_tilt={tilt:.4f} zoom={zoom:.4f}",
        flush=True,
    )

def _apply_ptz_state(stage) -> None:
    """将 _ptz_state 写入 USD Stage：Pan 旋转、Tilt 旋转、相机焦距。

    自动适配坐标轴：
      Z-up 场景（本项目）：Pan → rotateZ，Tilt → rotateX
      Y-up 场景（PTZ_SecurityDome）：Pan → rotateY，Tilt → rotateZ
    """
    global _last_tilt_attr_used, _tilt_axis_diag_printed
    global _ptz_base_initialized, _ptz_base_pan_deg, _ptz_base_tilt_deg
    with _ptz_lock:
        pan_deg  = _ptz_state["pan"]
        tilt_deg = _ptz_state["tilt"]
        zoom     = _ptz_state["zoom"]

    # 从 camera_prim 路径推断 pan/tilt prim 路径
    parts     = camera_prim.split("/")
    pan_path  = "/".join(parts[:-2])   # e.g. /World/CameraRig
    tilt_path = "/".join(parts[:-1])   # e.g. /World/CameraRig/CamTilt

    pan_p  = stage.GetPrimAtPath(pan_path)
    tilt_p = stage.GetPrimAtPath(tilt_path)
    cam_p  = stage.GetPrimAtPath(camera_prim)

    # Z-up：水平旋转绕 Z，俯仰绕 X（避免将俯仰误写到横摆轴）
    # Y-up：水平旋转绕 Y，俯仰绕 Z
    if _scene_up_axis == "Z":
        pan_attr_name  = "xformOp:rotateZ"
        tilt_attr_name = "xformOp:rotateX"
        legacy_tilt_attr_name = "xformOp:rotateY"
    else:
        pan_attr_name  = "xformOp:rotateY"
        tilt_attr_name = "xformOp:rotateZ"
        legacy_tilt_attr_name = None

    if not _ptz_base_initialized:
        _ptz_base_pan_deg = _read_rotate_attr_float(pan_p, pan_attr_name) or 0.0
        if tilt_p.IsValid():
            preferred_val = _read_rotate_attr_float(tilt_p, tilt_attr_name)
            if preferred_val is not None:
                _ptz_base_tilt_deg = preferred_val
            elif legacy_tilt_attr_name:
                # 兼容历史资产：若此前 tilt 用 rotateY，迁移其基准值到 rotateX。
                _ptz_base_tilt_deg = _read_rotate_attr_float(tilt_p, legacy_tilt_attr_name) or 0.0
            else:
                _ptz_base_tilt_deg = 0.0
        else:
            _ptz_base_tilt_deg = 0.0
        _ptz_base_initialized = True
        pan_base_for_apply = _ptz_base_pan_deg + (180.0 if _scene_up_axis == "Z" else 0.0)
        print(
            "[PTZ-RTSP][ptz-base] "
            f"pan_base_raw={_ptz_base_pan_deg} pan_base_apply={pan_base_for_apply} ({pan_attr_name}) "
            f"tilt_base={_ptz_base_tilt_deg} ({tilt_attr_name})",
            flush=True,
        )

    if pan_p.IsValid():
        attr = pan_p.GetAttribute(pan_attr_name)
        if attr and attr.IsValid():
            # Z-up 下修正 yaw 语义：+pan 应向右；并补偿 180° 基准偏置避免“水平正视背对目标”。
            if _scene_up_axis == "Z":
                pan_final = (_ptz_base_pan_deg + 180.0) - pan_deg
            else:
                pan_final = _ptz_base_pan_deg + pan_deg
            attr.Set(float(pan_final))
    if tilt_p.IsValid():
        tname = _resolve_or_create_tilt_rotate_attr(tilt_p, tilt_attr_name)
        _last_tilt_attr_used = tname
        before = after = None
        if tname:
            attr = tilt_p.GetAttribute(tname)
            if attr and attr.IsValid():
                try:
                    v = attr.Get()
                    before = float(v) if v is not None else None
                except Exception:
                    pass
                attr.Set(float(_ptz_base_tilt_deg + (-tilt_deg)))
                try:
                    after = float(attr.Get())
                except Exception:
                    pass
        if legacy_tilt_attr_name:
            # 迁移后清零旧轴，避免 rotateY 与 rotateX 叠加导致姿态扭曲/横倒。
            lg = tilt_p.GetAttribute(legacy_tilt_attr_name)
            if lg and lg.IsValid():
                lg.Set(0.0)
        if not _tilt_axis_diag_printed:
            ops = _list_cam_tilt_xform_op_names(tilt_p)
            tp = tilt_p.GetPath().pathString
            err = None
            if not tname:
                err = "resolve_or_create returned None"
            elif not (tilt_p.GetAttribute(tname) and tilt_p.GetAttribute(tname).IsValid()):
                err = f"attr missing after resolve tname={tname!r}"
            print(
                "[PTZ-RTSP][tilt-axis] "
                f"cam_tilt={tp!r} xform_ops={ops} "
                f"preferred={tilt_attr_name!r} hit_attr={tname!r} "
                f"before={before} after={after}"
                + (f" ERROR={err!r}" if err else ""),
                flush=True,
            )
            _tilt_axis_diag_printed = True
    elif not _tilt_axis_diag_printed:
        print(
            f"[PTZ-RTSP][tilt-axis] ERROR: invalid tilt_path={tilt_path!r} (CamTilt not on stage)",
            flush=True,
        )
        _tilt_axis_diag_printed = True
    if cam_p.IsValid():
        attr = cam_p.GetAttribute("focalLength")
        if attr and attr.IsValid():
            attr.Set(float(FOCAL_LENGTH_1X * zoom))


def _log_usd_ptz_debug_snapshot(stage) -> None:
    """只读：打印当前 Stage 上 rig 平移、Pan/Tilt 对应旋转、focalLength（与 _apply_ptz_state 轴映射一致）。"""
    try:
        parts = [x for x in camera_prim.split("/") if x]
        pan_path = "/" + "/".join(parts[:-2]) if len(parts) >= 3 else ""
        tilt_path = "/" + "/".join(parts[:-1]) if len(parts) >= 2 else ""
        rig_path = _CAMERA_RIG_PRIM or pan_path
        if _scene_up_axis == "Z":
            pan_attr_name, tilt_attr_name = "xformOp:rotateZ", "xformOp:rotateX"
        else:
            pan_attr_name, tilt_attr_name = "xformOp:rotateY", "xformOp:rotateZ"

        def _getf(prim_path: str, attr_name: str):
            p = stage.GetPrimAtPath(prim_path)
            if not p.IsValid():
                return None
            a = p.GetAttribute(attr_name)
            if not (a and a.IsValid()):
                return None
            try:
                return float(a.Get())
            except Exception:
                return None

        tr = None
        rp = stage.GetPrimAtPath(rig_path) if rig_path else None
        if rp and rp.IsValid():
            ta = rp.GetAttribute("xformOp:translate")
            if ta and ta.IsValid() and ta.Get() is not None:
                v = ta.Get()
                tr = (float(v[0]), float(v[1]), float(v[2]))

        pan_r = _getf(pan_path, pan_attr_name)
        tilt_resolved = _last_tilt_attr_used or tilt_attr_name
        tilt_r = _getf(tilt_path, tilt_resolved)
        tp = stage.GetPrimAtPath(tilt_path) if tilt_path else None
        tilt_ops = _list_cam_tilt_xform_op_names(tp) if (tp and tp.IsValid()) else []
        fl = _getf(camera_prim, "focalLength")

        print(
            "[PTZ-RTSP][usd-debug] "
            f"rig={rig_path!r} xformOp:translate={tr} "
            f"pan_axis={pan_attr_name} value={pan_r} "
            f"tilt_preferred={tilt_attr_name} tilt_resolved={tilt_resolved} "
            f"cam_tilt_ops={tilt_ops} value={tilt_r} "
            f"camera={camera_prim!r} focalLength={fl}",
            flush=True,
        )
    except Exception as exc:
        print(f"[PTZ-RTSP][usd-debug] ERROR: {exc!r}", flush=True)


def _prim_projection_metrics(stage, cam_prim_path: str, tgt_prim_path: str) -> dict | None:
    """
    将 prim 世界包围盒经当前相机视锥投影到与 RTSP/MJPEG 一致的渲染分辨率平面。
    返回与历史 `target_projection_metrics` 相同键集，便于 scene_perception 消费。
    """
    try:
        from pxr import Gf, Usd, UsdGeom

        cam_prim = stage.GetPrimAtPath(cam_prim_path)
        tgt_prim = stage.GetPrimAtPath(tgt_prim_path)
        if not cam_prim.IsValid() or not tgt_prim.IsValid():
            return None

        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
        target_bound = bbox_cache.ComputeWorldBound(tgt_prim)
        target_range = target_bound.ComputeAlignedRange()
        if target_range.IsEmpty():
            return {"error": "empty_world_bbox", "prim_path": tgt_prim_path}

        target_mid = target_range.GetMidpoint()

        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        cam_world = xcache.GetLocalToWorldTransform(cam_prim)
        cam_pos = cam_world.ExtractTranslation()
        dist = float((target_mid - cam_pos).GetLength())

        gf_cam = UsdGeom.Camera(cam_prim).GetCamera(Usd.TimeCode.Default())
        frustum = gf_cam.frustum
        frustum.Transform(cam_world)
        in_frustum = bool(frustum.Intersects(target_bound))

        vm = frustum.ComputeViewMatrix()
        pm = frustum.ComputeProjectionMatrix()
        pixels: list[tuple[float, float]] = []
        mn = target_range.GetMin()
        mx = target_range.GetMax()
        for px in (mn[0], mx[0]):
            for py in (mn[1], mx[1]):
                for pz in (mn[2], mx[2]):
                    clip = Gf.Vec4d(px, py, pz, 1.0) * vm * pm
                    wclip = float(clip[3])
                    if abs(wclip) < 1e-9:
                        continue
                    ndc_x = float(clip[0]) / wclip
                    ndc_y = float(clip[1]) / wclip
                    sx = (ndc_x * 0.5 + 0.5) * W
                    sy = (1.0 - (ndc_y * 0.5 + 0.5)) * H
                    pixels.append((sx, sy))

        if not pixels:
            return None

        min_x = min(v[0] for v in pixels)
        min_y = min(v[1] for v in pixels)
        max_x = max(v[0] for v in pixels)
        max_y = max(v[1] for v in pixels)
        bbox_w = max_x - min_x
        bbox_h = max_y - min_y
        cx = (min_x + max_x) * 0.5
        cy = (min_y + max_y) * 0.5
        inter_w = max(0.0, min(max_x, W) - max(min_x, 0.0))
        inter_h = max(0.0, min(max_y, H) - max(min_y, 0.0))
        bbox_area = max(1e-6, bbox_w * bbox_h)
        overflow_ratio = max(0.0, 1.0 - ((inter_w * inter_h) / bbox_area))
        on_screen_ratio = max(0.0, min(1.0, (inter_w * inter_h) / float(W * H)))

        return {
            "prim_path": tgt_prim_path,
            "distance_to_camera": round(dist, 2),
            "frustum内可见": in_frustum,
            "屏幕包围框px": [round(min_x, 2), round(min_y, 2), round(max_x, 2), round(max_y, 2)],
            "中心点px": [round(cx, 2), round(cy, 2)],
            "宽度占比%": round(bbox_w / W * 100.0, 2),
            "高度占比%": round(bbox_h / H * 100.0, 2),
            "越界面积占比": round(overflow_ratio, 4),
            "中心在画面内": bool(0 <= cx <= W and 0 <= cy <= H),
            "目标像素占比": round(max(0.0, min(1.0, bbox_area / float(W * H))), 4),
            "画面内交集像素占比": round(on_screen_ratio, 6),
        }
    except Exception as exc:
        return {"error": repr(exc), "prim_path": tgt_prim_path}


def _compute_target_projection_metrics(stage) -> dict | None:
    """计算当前 target（篮体控制 prim）在运行 Stage 上的投影指标。"""
    if not _GONDOLA_PRIM:
        return None
    return _prim_projection_metrics(stage, camera_prim, _GONDOLA_PRIM)


def _compute_camera_view_perception(stage) -> dict | None:
    """
    摄像头视角观测包：对「当前激活吊篮下、USD 标为可见」的作业人员 prim 逐个体做视锥+成像矩形裁剪，
    统计几何上落入画面且占比足够的数量；篮体高度取当前控制 prim 的世界高度轴（仅作可观测时的几何读数）。
    """
    try:
        runtime = _scene_state_runtime_snapshot()
        active = str(runtime.get("active_diaolan_path") or "").strip()
        visible_workers = [str(p) for p in (runtime.get("visible_worker_paths") or []) if str(p).strip()]
        prefix = active.rstrip("/") + "/" if active else ""
        under_active = [
            p
            for p in visible_workers
            if active and (p == active or p.startswith(prefix))
        ]

        worker_rows: list[dict] = []
        n_cam = 0
        pmig = cfg.get("perception_migration")
        min_w = float((pmig or {}).get("min_worker_area_ratio", 0.00012))
        if min_w <= 0:
            min_w = 0.00012

        for wp in under_active:
            m = _prim_projection_metrics(stage, camera_prim, wp)
            if not isinstance(m, dict):
                worker_rows.append({"path": wp, "error": "no_metrics"})
                continue
            if "error" in m:
                worker_rows.append({"path": wp, "projection": m, "camera_sees_worker": False})
                continue
            in_f = bool(m.get("frustum内可见"))
            try:
                onr = float(m.get("画面内交集像素占比") or 0.0)
            except (TypeError, ValueError):
                onr = 0.0
            sees = bool(in_f and onr >= min_w)
            if sees:
                n_cam += 1
            worker_rows.append(
                {
                    "path": wp,
                    "projection": m,
                    "min_worker_area_ratio": min_w,
                    "camera_sees_worker": sees,
                }
            )

        n_ren = len(under_active)
        all_seen = n_ren == 0 or all(
            (isinstance(r, dict) and r.get("camera_sees_worker") is True) for r in worker_rows
        )

        ghz = None
        if _GONDOLA_PRIM:
            ghz = _prim_world_height_axis(stage, _GONDOLA_PRIM)

        gondola_m = _prim_projection_metrics(stage, camera_prim, _GONDOLA_PRIM) if _GONDOLA_PRIM else None

        return {
            "image_wh": [int(W), int(H)],
            "camera_prim_path": str(camera_prim),
            "active_diaolan_path": active or None,
            "gondola_prim_path": str(_GONDOLA_PRIM) if _GONDOLA_PRIM else None,
            "gondola_world_height_axis": ghz,
            "gondola_projection": gondola_m,
            "workers": worker_rows,
            "camera_view_worker_count": n_cam,
            "rendered_worker_paths_under_active_count": n_ren,
            "workers_all_projected_in_view": bool(all_seen),
            "min_worker_area_ratio": min_w,
        }
    except Exception as exc:
        return {"error": repr(exc)}


def _refresh_projection_metrics(stage) -> None:
    tgt = _compute_target_projection_metrics(stage)
    cv = _compute_camera_view_perception(stage)
    _stream_diag_update(target_projection_metrics=tgt, camera_view_perception=cv)


def _log_ptz_applied_debug(stage, cmd: dict | None) -> None:
    """当 /control 设置 PTZ 后，打印每层 Xform/Camera 的实际旋转与 forward 向量。"""
    try:
        from pxr import UsdGeom as _UsdGeom
        from pxr import Gf as _Gf

        parts = [x for x in camera_prim.split("/") if x]
        pan_path = "/" + "/".join(parts[:-2]) if len(parts) >= 3 else ""
        tilt_path = "/" + "/".join(parts[:-1]) if len(parts) >= 2 else ""
        rig_path = _CAMERA_RIG_PRIM or pan_path

        pan_attr_name = "xformOp:rotateZ" if _scene_up_axis == "Z" else "xformOp:rotateY"
        default_tilt_attr_name = "xformOp:rotateX" if _scene_up_axis == "Z" else "xformOp:rotateZ"
        tilt_attr_name_used = _last_tilt_attr_used or default_tilt_attr_name

        with _ptz_lock:
            pan_ap = float(_ptz_state["pan"])
            tilt_ap = float(_ptz_state["tilt"])
            zoom_ap = float(_ptz_state["zoom"])

        in_dict = cmd.get("input", {}) if isinstance(cmd, dict) else {}
        in_pan = in_dict.get("pan", pan_ap)
        in_tilt = in_dict.get("tilt", tilt_ap)
        in_zoom = in_dict.get("zoom", zoom_ap)

        def _snapshot_rotate_ops(prim_path: str) -> list[str]:
            p = stage.GetPrimAtPath(prim_path) if prim_path else None
            if not (p and p.IsValid()):
                return []
            xf = _UsdGeom.Xformable(p)
            out: list[str] = []
            for op in xf.GetOrderedXformOps():
                name = op.GetOpName()
                if name.startswith("xformOp:rotate"):
                    a = p.GetAttribute(name)
                    val = None
                    try:
                        if a and a.IsValid():
                            val = a.Get()
                    except Exception:
                        val = None
                    out.append(f"{name}={val}")
            return out

        # camera forward：USD Camera 本地 forward = -Z
        cam_p = stage.GetPrimAtPath(camera_prim)
        cache = _UsdGeom.XformCache()
        mat = cache.GetLocalToWorldTransform(cam_p)
        f_local = _Gf.Vec3d(0, 0, -1)
        f_world = mat.TransformDir(f_local)
        if f_world.GetLength() > 1e-9:
            f_world = f_world.GetNormalized()

        pan_ops = _snapshot_rotate_ops(pan_path)
        tilt_ops = _snapshot_rotate_ops(tilt_path)
        cam_ops = _snapshot_rotate_ops(camera_prim)

        print(
            "[PTZ-RTSP][ptz-debug] "
            f"in pan={in_pan}° tilt={in_tilt}° zoom={in_zoom}× | "
            f"write pan={pan_path!r}:{pan_attr_name} tilt={tilt_path!r}:{tilt_attr_name_used} "
            f"camera={camera_prim!r}:focalLength",
            flush=True,
        )
        print(
            f"[PTZ-RTSP][ptz-debug] rot(CameraRig)={pan_ops} "
            f"rot(CamTilt)={tilt_ops} rot(Camera)={cam_ops}",
            flush=True,
        )
        print(
            "[PTZ-RTSP][ptz-debug] forward_world="
            f"({float(f_world[0]):.6f},{float(f_world[1]):.6f},{float(f_world[2]):.6f})",
            flush=True,
        )
    except Exception as exc:
        print(f"[PTZ-RTSP][ptz-debug] ERROR: {exc!r}", flush=True)


def _height_axis_index() -> int:
    """世界高度轴在 Vec3d translate 中的下标：Z-up → 2，Y-up → 1。"""
    return 2 if _scene_up_axis == "Z" else 1


def _path_is_strict_descendant(ancestor_path: str, prim_path: str) -> bool:
    """prim_path 是否为 ancestor_path 的子孙（非自身）。"""
    a = ancestor_path.rstrip("/")
    p = prim_path.rstrip("/")
    if not a or not p:
        return False
    return p.startswith(a + "/")


def _get_translate_height(stage, prim_path: str) -> float | None:
    from pxr import Gf as _Gf
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    attr = prim.GetAttribute("xformOp:translate")
    if not (attr and attr.IsValid()):
        return None
    cur = attr.Get()
    if cur is None:
        return None
    v = _Gf.Vec3d(cur[0], cur[1], cur[2])
    return float(v[_height_axis_index()])


def _set_prim_translate_height(stage, prim_path: str, height_val: float) -> dict | None:
    """将 height_val 写入世界「高度轴」分量（随 upAxis：Z-up→Z，Y-up→Y），其余世界水平分量保持不变。"""
    from pxr import Gf as _Gf
    from pxr import Usd as _Usd
    from pxr import UsdGeom as _UsdGeom
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    xform = _UsdGeom.Xformable(prim)
    translate_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == _UsdGeom.XformOp.TypeTranslate:
            translate_op = op
            break
    if translate_op is None:
        translate_op = xform.AddTranslateOp()

    xform_cache = _UsdGeom.XformCache(_Usd.TimeCode.Default())
    world_before = xform_cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    hi = _height_axis_index()
    wx, wy, wz = float(world_before[0]), float(world_before[1]), float(world_before[2])
    if hi == 2:
        target_world = _Gf.Vec3d(wx, wy, float(height_val))
    else:
        target_world = _Gf.Vec3d(wx, float(height_val), wz)

    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_world = xform_cache.GetLocalToWorldTransform(parent)
        target_local = parent_world.GetInverse().TransformAffine(target_world)
    else:
        target_local = target_world

    if translate_op.GetPrecision() == _UsdGeom.XformOp.PrecisionFloat:
        translate_value = _Gf.Vec3f(
            float(target_local[0]),
            float(target_local[1]),
            float(target_local[2]),
        )
    else:
        translate_value = _Gf.Vec3d(
            float(target_local[0]),
            float(target_local[1]),
            float(target_local[2]),
        )
    translate_op.Set(translate_value)

    print(
        "[GONDOLA_HEIGHT_AXIS] "
        f"prim_path={prim_path} "
        f"up_axis={_scene_up_axis!r} height_axis_idx={hi} "
        f"target_world_height={float(height_val):.6f} "
        f"world_before=({float(world_before[0]):.6f}, {float(world_before[1]):.6f}, {float(world_before[2]):.6f}) "
        f"target_local_translate=({float(target_local[0]):.6f}, {float(target_local[1]):.6f}, {float(target_local[2]):.6f}) "
        f"world_after_expected=({float(target_world[0]):.6f}, {float(target_world[1]):.6f}, {float(target_world[2]):.6f})"
    )
    return {
        "prim_path": prim_path,
        "final_world_z": float(target_world[2]),
        "final_local_translate": [
            float(target_local[0]),
            float(target_local[1]),
            float(target_local[2]),
        ],
    }


def _set_prim_y(stage, prim_path: str, y_val: float) -> None:
    """兼容旧名：按当前场景 upAxis 写入高度轴分量。"""
    _set_prim_translate_height(stage, prim_path, y_val)


def _set_prim_visibility(stage, prim_path: str, visible: bool) -> None:
    """设置 prim 的 visibility 属性（invisible / inherited）。"""
    from pxr import UsdGeom as _UG
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    attr = prim.GetAttribute("visibility")
    if not (attr and attr.IsValid()):
        attr = _UG.Imageable(prim).CreateVisibilityAttr()
    attr.Set(_UG.Tokens.inherited if visible else _UG.Tokens.invisible)


def _set_prim_purpose_default(stage, prim_path: str) -> None:
    from pxr import UsdGeom as _UG

    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    imageable = _UG.Imageable(prim)
    if not imageable:
        return
    purpose_attr = imageable.GetPurposeAttr()
    current = purpose_attr.Get() if purpose_attr and purpose_attr.IsValid() else None
    if current in (_UG.Tokens.guide, _UG.Tokens.proxy):
        imageable.CreatePurposeAttr().Set(_UG.Tokens.default_)


def _choose_visible_worker_paths(person_paths: list[str], visible_count: int, rng=None) -> list[str]:
    normalized = dedupe_worker_prim_paths_ordered([str(p) for p in person_paths if str(p).strip()])
    if not normalized:
        return []
    count = max(0, min(len(normalized), int(visible_count)))
    if count <= 0:
        return []
    if rng is not None:
        picked = list(rng.sample(normalized, count))
        order = {path: idx for idx, path in enumerate(normalized)}
        return sorted(picked, key=lambda path: order.get(path, 0))
    if count == 1:
        idx = max(0, min(len(normalized) - 1, _chosen_worker - 1))
        return [normalized[idx]]
    return normalized[:count]


def _set_active_worker_paths(active_diaolan_path: str, person_paths: list[str], visible_paths: list[str]) -> None:
    allowed = dedupe_worker_prim_paths_ordered([str(p) for p in person_paths if str(p).strip()])
    allowed_set = set(allowed)
    allowed_key_to_path = {logical_worker_root_path(p): p for p in allowed}
    visible_raw = [str(p) for p in visible_paths if str(p).strip()]
    visible: list[str] = []
    seen_keys: set[str] = set()
    for vp in visible_raw:
        if vp in allowed_set:
            k = logical_worker_root_path(vp)
            if k in seen_keys:
                continue
            seen_keys.add(k)
            visible.append(vp)
            continue
        k = logical_worker_root_path(vp)
        canon = allowed_key_to_path.get(k)
        if canon and k not in seen_keys:
            seen_keys.add(k)
            visible.append(canon)
    ap = str(active_diaolan_path or "")
    with _scene_lock:
        _scene_state["active_diaolan_path"] = ap
        _scene_state["selected_diaolan_path"] = ap
        _scene_state["all_worker_paths"] = list(allowed)
        _scene_state["visible_worker_paths"] = list(visible)
        wb = dict(_scene_state.get("workers_visible_count_by_diaolan_path") or {})
        if ap:
            wb[ap] = max(0, min(len(allowed), count_logical_workers_from_paths(visible)))
            _scene_state["workers_visible_count_by_diaolan_path"] = wb


def _normalize_workers_by_diaolan_raw(raw) -> dict[str, int]:
    out: dict[str, int] = {}
    if isinstance(raw, dict):
        for k, v in raw.items():
            ks = str(k or "").strip()
            if not ks:
                continue
            try:
                out[ks] = int(v)
            except (TypeError, ValueError):
                continue
    return out


def _diaolan_path_set(all_diaolans: list) -> set[str]:
    return {str(d.get("path") or "").strip() for d in all_diaolans if str(d.get("path") or "").strip()}


def _apply_all_diaolans_worker_visibility(stage, all_diaolans: list | None = None) -> None:
    if all_diaolans is None:
        all_diaolans = scan_diaolan_prims(stage)
    with _scene_lock:
        workers_by = _normalize_workers_by_diaolan_raw(_scene_state.get("workers_visible_count_by_diaolan_path"))
    for d in all_diaolans:
        rp = str(d.get("path") or "").strip()
        persons = dedupe_worker_prim_paths_ordered([str(p) for p in (d.get("persons") or []) if str(p).strip()])
        n = len(persons)
        c = max(0, min(n, int(workers_by.get(rp, 0))))
        visible_set = set(_choose_visible_worker_paths(persons, c, rng=None))
        for p in persons:
            apply_worker_logical_branch_visibility(stage, p, p in visible_set)


def _sync_worker_scalar_fields_for_control_diaolan(all_diaolans: list, control_root_path: str) -> None:
    rp = str(control_root_path or "").strip()
    with _scene_lock:
        workers_by = _normalize_workers_by_diaolan_raw(_scene_state.get("workers_visible_count_by_diaolan_path"))
    d = next((x for x in all_diaolans if str(x.get("path") or "").strip() == rp), None)
    if not d:
        return
    persons = dedupe_worker_prim_paths_ordered([str(p) for p in (d.get("persons") or []) if str(p).strip()])
    n = len(persons)
    c = max(0, min(n, int(workers_by.get(rp, 0))))
    vis = _choose_visible_worker_paths(persons, c, rng=None)
    with _scene_lock:
        _scene_state["all_worker_paths"] = list(persons)
        _scene_state["visible_worker_paths"] = list(vis)
        _scene_state["workers"] = c


def _apply_active_worker_visibility(stage) -> int:
    runtime = _scene_state_runtime_snapshot()
    all_paths = list(runtime["all_worker_paths"])
    visible_paths = set(runtime["visible_worker_paths"])
    for worker_path in all_paths:
        apply_worker_logical_branch_visibility(stage, worker_path, False)
    visible_count = 0
    for worker_path in all_paths:
        should_show = worker_path in visible_paths
        apply_worker_logical_branch_visibility(stage, worker_path, should_show)
        if should_show:
            visible_count += 1
    return visible_count


def _prim_path_under_any_prefix(prim_path: str, prefixes: list[str]) -> bool:
    ps = str(prim_path or "").strip().rstrip("/")
    if not ps:
        return False
    for pre in prefixes or []:
        pfx = str(pre or "").strip().rstrip("/")
        if not pfx:
            continue
        if ps == pfx or ps.startswith(pfx + "/"):
            return True
    return False


def _iter_active_gondola_renderables(stage, active_diaolan_path: str, worker_exclusion_prefixes: list[str]):
    from pxr import Usd as _Usd
    from pxr import UsdGeom as _UsdGeom

    if not active_diaolan_path:
        return []
    root_prim = stage.GetPrimAtPath(active_diaolan_path)
    if not root_prim.IsValid():
        return []

    renderables = []
    for prim in _Usd.PrimRange(root_prim):
        prim_path = prim.GetPath().pathString
        if _prim_path_under_any_prefix(prim_path, worker_exclusion_prefixes):
            continue
        if prim.IsA(_UsdGeom.Gprim):
            renderables.append(prim)
    return renderables


def _prim_path_is_protected_experiment_hidden(prim_path: str) -> bool:
    """scene/experiment apply_visibility 隐藏的 prim 及其祖先：修复渲染链时不得改回 inherited。"""
    pp = str(prim_path or "").rstrip("/")
    if not pp:
        return False
    with _scene_lock:
        hiddens = list(_scene_experiment_state.get("active_hidden_paths") or [])
    for h_raw in hiddens:
        h = str(h_raw or "").rstrip("/")
        if not h:
            continue
        if pp == h or h.startswith(pp + "/"):
            return True
    return False


def _repair_active_gondola_renderables(stage, all_diaolans: list | None = None) -> dict:
    from pxr import Usd as _Usd
    from pxr import UsdGeom as _UsdGeom

    runtime = _scene_state_runtime_snapshot()
    active_diaolan_path = runtime["active_diaolan_path"]
    worker_paths = list(runtime["all_worker_paths"])
    if all_diaolans is None:
        all_diaolans = scan_diaolan_prims(stage)
    active_d = next(
        (
            x
            for x in (all_diaolans or [])
            if str(x.get("path") or "").strip() == str(active_diaolan_path or "").strip()
        ),
        None,
    )
    exclusion: set[str] = set()
    if active_d is not None:
        exclusion.update(worker_render_exclusion_prefixes(stage, active_d))
    for wp in worker_paths:
        w = str(wp or "").strip().rstrip("/")
        if w:
            exclusion.add(w)
            lr = logical_worker_root_path(w).rstrip("/")
            if lr:
                exclusion.add(lr)
    worker_exclusion_prefixes = sorted(exclusion, key=lambda s: (-len(s), s.lower()))
    renderables = _iter_active_gondola_renderables(stage, active_diaolan_path, worker_exclusion_prefixes)

    repaired_ancestors = set()
    renderable_paths = []
    visible_renderable_paths = []
    hidden_renderable_paths = []
    renderable_debug = []

    for prim in renderables:
        prim_path = prim.GetPath().pathString
        renderable_paths.append(prim_path)

        parent = prim
        while parent and parent.IsValid():
            parent_path = parent.GetPath().pathString
            if not parent_path.startswith(str(active_diaolan_path).rstrip("/") + "/") and parent_path != active_diaolan_path:
                break
            if parent_path not in repaired_ancestors:
                if not _prim_path_is_protected_experiment_hidden(parent_path):
                    _set_prim_visibility(stage, parent_path, True)
                    _set_prim_purpose_default(stage, parent_path)
                repaired_ancestors.add(parent_path)
            parent = parent.GetParent()

        imageable = _UsdGeom.Imageable(prim)
        visibility = str(imageable.ComputeVisibility(_Usd.TimeCode.Default()))
        purpose = str(imageable.GetPurposeAttr().Get() or _UsdGeom.Tokens.default_)
        detail = {
            "path": prim_path,
            "visibility": visibility,
            "purpose": purpose,
            "typeName": prim.GetTypeName(),
        }
        renderable_debug.append(detail)
        if visibility != str(_UsdGeom.Tokens.invisible) and purpose not in (
            str(_UsdGeom.Tokens.guide),
            str(_UsdGeom.Tokens.proxy),
        ):
            visible_renderable_paths.append(prim_path)
        else:
            hidden_renderable_paths.append(prim_path)

    with _scene_lock:
        _scene_state["gondola_renderable_paths"] = list(renderable_paths)
        _scene_state["gondola_visible_renderable_paths"] = list(visible_renderable_paths)
        _scene_state["gondola_hidden_paths"] = list(hidden_renderable_paths)
        _scene_state["gondola_renderable_debug"] = list(renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT])

    print(
        "[scene-gondola-debug] "
        f"active_diaolan_path={active_diaolan_path!r} "
        f"renderables={len(renderable_paths)} "
        f"visible={len(visible_renderable_paths)} "
        f"hidden={len(hidden_renderable_paths)} "
        f"sample={renderable_paths[:3]} "
        f"hidden_sample={hidden_renderable_paths[:3]}",
        flush=True,
    )
    if _GONDOLA_RENDERABLE_VERBOSE_LOG:
        for detail in renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT]:
            print(
                "[scene-gondola-renderable] "
                f"path={detail['path']!r} visibility={detail['visibility']} "
                f"purpose={detail['purpose']} typeName={detail['typeName']}",
                flush=True,
            )
    return {
        "active_diaolan_path": active_diaolan_path,
        "gondola_renderable_paths": renderable_paths,
        "gondola_visible_renderable_paths": visible_renderable_paths,
        "gondola_hidden_paths": hidden_renderable_paths,
        "gondola_renderable_debug": renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_counts": {
            "total": len(renderable_paths),
            "visible": len(visible_renderable_paths),
            "hidden": len(hidden_renderable_paths),
            "debug_sample_limit": _GONDOLA_RENDERABLE_DETAIL_LIMIT,
        },
    }


def _set_workers_visible_count(diaolan_info: dict, visible_count: int, *, rng=None) -> int:
    """按完整 persons 列表更新 worker 可见集合。"""
    person_paths = dedupe_worker_prim_paths_ordered(list(diaolan_info.get("persons") or []))
    visible_paths = _choose_visible_worker_paths(person_paths, visible_count, rng=rng)
    root_path = str(diaolan_info.get("path", "") or "")
    with _scene_lock:
        wb = dict(_scene_state.get("workers_visible_count_by_diaolan_path") or {})
        wb[root_path] = max(0, min(len(person_paths), len(visible_paths)))
        _scene_state["workers_visible_count_by_diaolan_path"] = wb
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return len(visible_paths)
    all_d = scan_diaolan_prims(stage)
    with _scene_lock:
        sel = str(_scene_state.get("selected_diaolan_path") or _scene_state.get("active_diaolan_path") or root_path)
    _sync_worker_scalar_fields_for_control_diaolan(all_d, sel)
    _apply_all_diaolans_worker_visibility(stage, all_d)
    return len(visible_paths)


def _reconcile_gondola_prim_to_resolved_height(stage) -> None:
    """主线程：按当前选中的吊篮根路径解析高度写入目标，纠正漂移的 _GONDOLA_PRIM（与 resolve 结果对齐）。"""
    global _GONDOLA_PRIM, _WORKER1_PRIM, _WORKER2_PRIM
    if stage is None:
        return
    with _scene_lock:
        sel = str(
            _scene_state.get("selected_diaolan_path")
            or _scene_state.get("active_diaolan_path")
            or ""
        ).strip()
    if not sel:
        return
    try:
        height_path, asm_path = resolve_diaolan_height_and_assembly(stage, sel.rstrip("/"))
    except Exception:
        return
    if not height_path:
        return
    if not stage.GetPrimAtPath(height_path).IsValid():
        return
    if height_path == (_GONDOLA_PRIM or ""):
        return
    persons = _collect_worker_prims(stage, asm_path) if asm_path else []
    if not persons:
        persons = _collect_worker_prims(stage, height_path)
    _GONDOLA_PRIM = height_path
    _WORKER1_PRIM = persons[0] if persons else ""
    _WORKER2_PRIM = persons[1] if len(persons) > 1 else ""
    print(
        "[diaolan-reconcile] "
        f"selected_root={sel!r} gondola_control_prim={height_path!r} "
        f"(was drifted from previous global)",
        flush=True,
    )


def _apply_scene_state(stage) -> None:
    """运行时同步吊篮高度、工人显隐与吊篮可渲染节点可见性。"""
    _reconcile_gondola_prim_to_resolved_height(stage)
    _apply_gondola_height_only(stage)
    all_di = scan_diaolan_prims(stage)
    _apply_all_diaolans_worker_visibility(stage, all_di)
    _repair_active_gondola_renderables(stage, all_di)


def _visibility_attr_snapshot(prim):
    attr = prim.GetAttribute("visibility")
    if not attr or not attr.IsValid():
        return {"authored": False, "value": None}
    return {
        "authored": bool(attr.HasAuthoredValueOpinion()),
        "value": str(attr.Get()) if attr.Get() is not None else None,
    }


def _restore_scene_experiment_visibility(stage) -> None:
    from pxr import UsdGeom as _UG

    with _scene_lock:
        original_visibility = dict(_scene_experiment_state["original_visibility"])
    for prim_path, snap in original_visibility.items():
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("visibility")
        if not attr or not attr.IsValid():
            img = _UG.Imageable(prim)
            attr = img.CreateVisibilityAttr()
        if snap.get("authored"):
            value = snap.get("value") or str(_UG.Tokens.inherited)
            attr.Set(_UG.Tokens.invisible if value == str(_UG.Tokens.invisible) else _UG.Tokens.inherited)
        else:
            attr.Clear()
    with _scene_lock:
        _scene_experiment_state["active_hidden_paths"] = []
        _scene_experiment_state["original_visibility"] = {}


def _normalize_root_paths(paths: list[str]) -> list[str]:
    cleaned = []
    for path in sorted({str(p).rstrip("/") for p in paths if str(p).strip()}):
        if any(path == root or path.startswith(root + "/") for root in cleaned):
            continue
        cleaned.append(path)
    return cleaned


def _set_scene_experiment_hidden_paths(stage, hidden_paths: list[str]) -> dict:
    valid_hidden = []
    invalid_hidden = []
    original_visibility = {}
    for prim_path in _normalize_root_paths(hidden_paths):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            invalid_hidden.append(prim_path)
            continue
        original_visibility[prim_path] = _visibility_attr_snapshot(prim)
        _set_prim_visibility(stage, prim_path, False)
        valid_hidden.append(prim_path)
    with _scene_lock:
        _scene_experiment_state["active_hidden_paths"] = list(valid_hidden)
        _scene_experiment_state["original_visibility"] = dict(original_visibility)
    return {
        "active_hidden_paths": valid_hidden,
        "invalid_hidden_paths": invalid_hidden,
    }


def _match_suspicious_name(path: str) -> bool:
    path_l = path.lower()
    tokens = ("card", "plane", "quad", "billboard", "decal", "screen")
    return any(tok in path_l for tok in tokens)


def _describe_flat_candidates(stage, root_prim, limit: int = 24) -> list[dict]:
    from pxr import Usd, UsdGeom

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
    out = []
    for prim in Usd.PrimRange(root_prim):
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Xform)):
            continue
        rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        size = rng.GetSize()
        dims = sorted(float(v) for v in size)
        if dims[2] < 0.25 or dims[1] < 0.15:
            continue
        thin_ratio = dims[0] / max(dims[1], 1e-6)
        if thin_ratio > 0.045:
            continue
        out.append(
            {
                "path": prim.GetPath().pathString,
                "size_xyz": [round(float(size[0]), 4), round(float(size[1]), 4), round(float(size[2]), 4)],
                "thin_ratio": round(thin_ratio, 5),
                "type": prim.GetTypeName(),
            }
        )
        if len(out) >= limit:
            break
    return out


def _describe_active_scene_branch(stage) -> dict:
    from pxr import Usd

    runtime = _scene_state_runtime_snapshot()
    active_path = str(runtime["active_diaolan_path"] or "")
    if not active_path and _GONDOLA_PRIM:
        active_path = infer_world_diaolan_instance_root(_GONDOLA_PRIM) or ""
    if not active_path and _GONDOLA_PRIM:
        parts = [p for p in _GONDOLA_PRIM.split("/") if p]
        if len(parts) >= 2:
            active_path = "/" + "/".join(parts[:2])
    model_path = f"{active_path}/Model" if active_path else ""
    model_prim = stage.GetPrimAtPath(model_path) if model_path else None
    model_children = []
    auxiliary_model_children = []
    suspicious_named_paths = []
    flat_candidates = []
    if model_prim and model_prim.IsValid():
        for child in model_prim.GetChildren():
            c_path = child.GetPath().pathString
            model_children.append(c_path)
            is_target = bool(_GONDOLA_PRIM) and (c_path == _GONDOLA_PRIM or _GONDOLA_PRIM.startswith(c_path + "/"))
            is_worker = c_path in set(runtime["all_worker_paths"])
            if not is_target and not is_worker:
                auxiliary_model_children.append(c_path)
        suspicious_named_paths = [
            prim.GetPath().pathString
            for prim in Usd.PrimRange(model_prim)
            if _match_suspicious_name(prim.GetPath().pathString)
        ]
        flat_candidates = _describe_flat_candidates(stage, model_prim)
    with _scene_lock:
        scene_experiment = {
            "active_hidden_paths": list(_scene_experiment_state["active_hidden_paths"]),
        }
    return {
        "active_path": active_path or None,
        "active_diaolan_path": active_path or None,
        "target_prim_path": _GONDOLA_PRIM or None,
        "selected_target_path": _GONDOLA_PRIM or None,
        "model_path": model_path or None,
        "target_branch_path": _GONDOLA_PRIM or None,
        "camera_rig_path": _CAMERA_RIG_PRIM or None,
        "camera_rig_translate_xyz": [round(float(v), 2) for v in _CAMERA_RIG_TRANSLATE_XYZ],
        "all_worker_paths": list(runtime["all_worker_paths"]),
        "visible_worker_paths": list(runtime["visible_worker_paths"]),
        "worker_paths": list(runtime["all_worker_paths"]),
        "gondola_renderable_paths": gondola_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_visible_renderable_paths": gondola_visible_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_hidden_paths": gondola_hidden_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_debug": gondola_renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_counts": {
            "total": len(gondola_renderable_paths),
            "visible": len(gondola_visible_renderable_paths),
            "hidden": len(gondola_hidden_paths),
            "sample_limit": _GONDOLA_RENDERABLE_DETAIL_LIMIT,
        },
        "height_debug": dict(runtime["height_debug"]),
        "model_children": model_children,
        "auxiliary_model_children": auxiliary_model_children,
        "suspicious_named_paths": suspicious_named_paths,
        "flat_candidate_paths": flat_candidates,
        "scene_experiment": scene_experiment,
    }


def _set_camera_rig_translate_runtime(stage, xyz: list[float]) -> list[float]:
    from pxr import Gf as _Gf
    global _CAMERA_RIG_TRANSLATE_XYZ

    if not _CAMERA_RIG_PRIM:
        raise RuntimeError("camera rig prim unresolved")
    prim = stage.GetPrimAtPath(_CAMERA_RIG_PRIM)
    if not prim.IsValid():
        raise RuntimeError(f"invalid camera rig prim: {_CAMERA_RIG_PRIM}")
    attr = prim.GetAttribute("xformOp:translate")
    if not attr or not attr.IsValid():
        raise RuntimeError(f"camera rig translate attr missing: {_CAMERA_RIG_PRIM}")
    x, y, z = [round(float(v), 2) for v in xyz]
    attr.Set(_Gf.Vec3d(x, y, z))
    _CAMERA_RIG_TRANSLATE_XYZ = (x, y, z)
    print(f"[camera-rig-runtime] translate=({x:.2f},{y:.2f},{z:.2f}) prim={_CAMERA_RIG_PRIM}", flush=True)
    return [x, y, z]


def _discover_diaolan_group1_path(stage) -> str:
    """吊篮高度写入目标（最高优先级固定为 /World/DiaoLan/Model/Group1）。

    不再通过 substring 遍历来“碰到”其他同名/相似层级的吊篮（例如 Group8761 旧层级），避免高度写错对象。
    """
    preferred = "/World/DiaoLan/Model/Group1"
    p = stage.GetPrimAtPath(preferred)
    if p.IsValid():
        return preferred
    return ""


def _resolve_sibling_worker_nodes(stage, group1_path: str) -> tuple[str, str]:
    """在 Group1 的父 prim（一般为 Model）下按 prim 名找 node______1/node_1、node______2/node_2。"""
    g = stage.GetPrimAtPath(group1_path)
    if not g.IsValid():
        return "", ""
    parent = g.GetParent()
    if not parent.IsValid():
        return "", ""
    w1, w2 = "", ""
    for ch in parent.GetChildren():
        nm = ch.GetName()
        ps = ch.GetPath().pathString
        if nm in _NODE1_SIBLING_NAMES:
            w1 = ps
        elif nm in _NODE2_SIBLING_NAMES:
            w2 = ps
    return w1, w2


def _apply_gondola_height_only(stage) -> None:
    """仅写当前选中吊篮 group1 的高度轴；兄弟 node 同步规则与旧版一致。"""
    global _node1_vs_group1_height_offset, _node2_vs_group1_height_offset, _GONDOLA_HEIGHT_DIAG_PRINTED
    if not _GONDOLA_PRIM:
        return
    with _scene_lock:
        gondola_y = _scene_state["gondola_y"]
        input_height_cm = _stage_units_to_cm(stage, gondola_y)

    n1 = stage.GetPrimAtPath(_WORKER1_PRIM)
    node1_under_g1 = n1.IsValid() and _path_is_strict_descendant(_GONDOLA_PRIM, _WORKER1_PRIM)
    n2 = stage.GetPrimAtPath(_WORKER2_PRIM)
    node2_under_g1 = n2.IsValid() and _path_is_strict_descendant(_GONDOLA_PRIM, _WORKER2_PRIM)

    if n1.IsValid() and not node1_under_g1 and _node1_vs_group1_height_offset is None:
        h_g = _get_translate_height(stage, _GONDOLA_PRIM)
        h_n = _get_translate_height(stage, _WORKER1_PRIM)
        if h_g is not None and h_n is not None:
            _node1_vs_group1_height_offset = h_n - h_g
        else:
            _node1_vs_group1_height_offset = 0.0

    if n2.IsValid() and not node2_under_g1 and _node2_vs_group1_height_offset is None:
        h_g = _get_translate_height(stage, _GONDOLA_PRIM)
        h_n = _get_translate_height(stage, _WORKER2_PRIM)
        if h_g is not None and h_n is not None:
            _node2_vs_group1_height_offset = h_n - h_g
        else:
            _node2_vs_group1_height_offset = 0.0

    before_g = _get_translate_tuple(stage, _GONDOLA_PRIM)
    before_n = _get_translate_tuple(stage, _WORKER1_PRIM) if _WORKER1_PRIM else None
    before_n2 = _get_translate_tuple(stage, _WORKER2_PRIM) if _WORKER2_PRIM else None

    gondola_debug = _set_prim_translate_height(stage, _GONDOLA_PRIM, gondola_y) or {}
    node1_target_y = None
    if n1.IsValid() and not node1_under_g1:
        node1_target_y = gondola_y + float(_node1_vs_group1_height_offset or 0.0)
        _set_prim_translate_height(stage, _WORKER1_PRIM, node1_target_y)

    node2_target_y = None
    if n2.IsValid() and not node2_under_g1:
        node2_target_y = gondola_y + float(_node2_vs_group1_height_offset or 0.0)
        _set_prim_translate_height(stage, _WORKER2_PRIM, node2_target_y)

    if not _GONDOLA_HEIGHT_DIAG_PRINTED:
        _print_gondola_height_line(
            stage, "gondola_root", _GONDOLA_PRIM, float(gondola_y), before_g
        )
        if n1.IsValid() and not node1_under_g1 and node1_target_y is not None:
            _print_gondola_height_line(
                stage, "node1_sync", _WORKER1_PRIM, float(node1_target_y), before_n
            )
        if n2.IsValid() and not node2_under_g1 and node2_target_y is not None:
            _print_gondola_height_line(
                stage, "node2_sync", _WORKER2_PRIM, float(node2_target_y), before_n2
            )
        _GONDOLA_HEIGHT_DIAG_PRINTED = True

    height_debug = {
        "input_height_cm": round(float(input_height_cm), 4),
        "converted_stage_units": round(float(gondola_y), 6),
        "final_world_z": gondola_debug.get("final_world_z"),
        "final_local_translate": gondola_debug.get("final_local_translate"),
    }
    with _scene_lock:
        _scene_state["height_debug"] = dict(height_debug)
    print(
        "[scene-height-debug] "
        f"input_height_cm={height_debug['input_height_cm']} "
        f"converted_stage_units={height_debug['converted_stage_units']} "
        f"final_world_z={height_debug['final_world_z']} "
        f"final_local_translate={height_debug['final_local_translate']}",
        flush=True,
    )


def _first_valid_prim_path(stage, candidates: tuple[str, ...]) -> str:
    for p in candidates:
        if p and stage.GetPrimAtPath(p).IsValid():
            return p
    return ""


def _resolve_camera_rig_path(stage) -> str:
    parts = [x for x in camera_prim.split("/") if x]
    if len(parts) >= 3:
        pan_path = "/" + "/".join(parts[:-2])
        if stage.GetPrimAtPath(pan_path).IsValid():
            return pan_path
    return _first_valid_prim_path(stage, _CAMERA_RIG_PATH_CANDIDATES_EXTRA)


def _set_camera_rig_fixed_translate(stage, rig_path: str) -> None:
    from pxr import Gf as _Gf
    if not rig_path:
        return
    prim = stage.GetPrimAtPath(rig_path)
    if not prim.IsValid():
        return
    attr = prim.GetAttribute("xformOp:translate")
    if attr and attr.IsValid():
        x, y, z = _CAMERA_RIG_TRANSLATE_XYZ
        attr.Set(_Gf.Vec3d(float(x), float(y), float(z)))
        print(f"[camera-rig] fixed translate=({x}, {y}, {z}) prim={rig_path}")


def _summarize_startup_attempts(attempt_rows: list[dict], *, width: int, height: int) -> None:
    if not attempt_rows:
        return
    reason_counter = Counter()
    near_miss_count = 0
    for row in attempt_rows:
        reason_counter[str(row.get("rejection_reason") or "unknown")] += 1
        if row.get("near_miss"):
            near_miss_count += 1
    top_reason, top_count = reason_counter.most_common(1)[0]
    print(
        "[camera-startup-diagnostics] "
        f"attempts={len(attempt_rows)} width={width} height={height} "
        f"top_rejection_reason={top_reason} top_rejection_count={top_count} "
        f"near_miss_count={near_miss_count} rejection_histogram={dict(reason_counter)}"
    )


def _preset_visibility_log_line(presets_cfg: dict, preset_vis: dict) -> str:
    """按配置里的 name 打印各预置位可见性。"""
    parts = []
    for k in sorted(preset_vis.keys(), key=lambda x: str(x)):
        v = preset_vis[k]
        name = str(k)
        if isinstance(presets_cfg, dict) and isinstance(presets_cfg.get(k), dict):
            name = presets_cfg[k].get("name") or k
        parts.append(f"{name}={v}")
    return " ".join(parts)


def _log_disabled_black_model_workarounds() -> None:
    raw = cfg.get("diaolan_ambient_boost_intensity")
    if raw not in (None, "", 0, 0.0, "0", "0.0"):
        print(
            "[renderer-fix] ignoring diaolan_ambient_boost_intensity; "
            "lighting/exposure black-model workaround disabled"
        )


import random
import yaml
import math
from diaolan_randomizer import (
    scan_diaolan_prims,
    pick_active_diaolan,
    apply_diaolan_visibility,
    sync_workers_to_group1,
    check_safety_hazard,
    compute_changjing_aabb,
    sample_camera_in_changjing,
    check_preset_visibility,
    log_target_branch_render_state,
    force_debug_emissive_on_target_branch,
    apply_hydrastorm_formal_materials_on_target_branch,
    prepare_geometry_only_target_branch,
    _compute_target_world_midpoint,
    resolve_dynamic_startup_view_metrics,
    resolve_diaolan_height_and_assembly,
    _collect_worker_prims,
    infer_world_diaolan_instance_root,
    count_logical_workers_from_paths,
    dedupe_worker_prim_paths_ordered,
    logical_worker_root_path,
    apply_worker_logical_branch_visibility,
    worker_render_exclusion_prefixes,
    evaluate_scene_rule_guardrails,
    evaluate_scene_rule_safety_ropes,
    evaluate_scene_rule_limitstops,
    evaluate_scene_rule_fallarrestors,
    apply_diaolan_safety_component,
    summarize_diaolan_safety_components,
    randomize_active_diaolan_safety_components,
    clear_wall_mount_candidate_cache,
)


def _get_cached_snapshot_jpeg_bytes() -> bytes | None:
    with _mjpeg_lock:
        jpg = (
            _snapshot_cache.get("last_good_jpeg")
            or _snapshot_cache.get("jpeg")
            or _mjpeg.get("jpeg")
        )
    return jpg if isinstance(jpg, (bytes, bytearray)) and len(jpg) > 0 else None


_RULE11_EVAL_ARCHIVE_DIR = os.path.join(script_dir, "archive", "rule11_eval")


def _rule11_camera_chain_restore_baseline() -> dict:
    """恢复 rule11 显式请求期间改动的 carb / USD 曝光相关状态（幂等）。"""
    out = {"carb_restored": 0, "usd_restored": 0, "had_pending": False}
    with _scene_lock:
        bundle = _scene_state.get("rule11_exposure_restore_bundle")
        _scene_state["rule11_exposure_restore_bundle"] = None
    if not isinstance(bundle, dict):
        return out
    out["had_pending"] = True
    try:
        import carb

        settings = carb.settings.get_settings()
    except Exception as exc:
        out["carb_error"] = str(exc)
        settings = None
    if settings is not None:
        for row in reversed(list(bundle.get("carb") or [])):
            if not isinstance(row, dict):
                continue
            p = str(row.get("path") or "").strip()
            if not p:
                continue
            prev = row.get("prev")
            try:
                settings.set(p, prev)
                out["carb_restored"] += 1
            except Exception as exc:
                out.setdefault("carb_restore_errors", []).append({"path": p, "error": str(exc)})
    for row in reversed(list(bundle.get("usd") or [])):
        if not isinstance(row, dict):
            continue
        prim_path = str(row.get("prim_path") or "").strip()
        attr_name = str(row.get("attr_name") or "").strip()
        prev = row.get("prev")
        if not prim_path or not attr_name:
            continue
        try:
            st = omni.usd.get_context().get_stage()
            if st is None:
                continue
            prim = st.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                continue
            attr = prim.GetAttribute(attr_name)
            if not attr or not attr.IsValid():
                continue
            attr.Set(prev)
            out["usd_restored"] += 1
        except Exception as exc:
            out.setdefault("usd_restore_errors", []).append(
                {"prim_path": prim_path, "attr_name": attr_name, "error": str(exc)}
            )
    try:
        sim_app.update()
    except Exception:
        pass
    return out


def _rule11_carb_probe_exposure_related_keys(settings) -> list[str]:
    """枚举当前 Kit 中与 autoexposure/tonemap 相关的 settings 键，便于判定「卡在哪一层」。"""
    keys: list[str] = []
    try:
        d = settings.get_settings_dictionary()
    except Exception:
        return keys
    try:
        for k in d:
            ks = str(k).lower()
            if "autoexposure" in ks or "auto_exposure" in ks:
                keys.append(str(k))
            elif "tonemap" in ks and "/rtx/" in str(k):
                keys.append(str(k))
            elif "/rtx/post/" in str(k) and ("exposure" in ks or "tonemap" in ks):
                keys.append(str(k))
    except Exception:
        return keys
    return sorted(set(keys))[:80]


def _rule11_carb_try_read(settings, path: str):
    try:
        return True, settings.get(path)
    except Exception as exc:
        return False, str(exc)


def _rule11_carb_try_write(settings, path: str, value) -> tuple[bool, str | None]:
    try:
        settings.set(path, value)
        return True, None
    except Exception as exc:
        return False, str(exc)


def _rule11_usd_try_boost_exposure_like_attrs(stage, cam_path: str) -> tuple[list[dict], list[dict]]:
    """在相机 prim 上扫描名称含 exposure/iso/fstop 的 float 属性并临时推高（若有）。"""
    touched: list[dict] = []
    errors: list[dict] = []
    prim = stage.GetPrimAtPath(cam_path)
    if not prim or not prim.IsValid():
        errors.append({"error": "camera_prim_invalid", "path": cam_path})
        return touched, errors
    for attr in prim.GetAttributes():
        try:
            an = attr.GetName()
        except Exception:
            continue
        al = an.lower()
        if "exposure" not in al and "iso" not in al and "fstop" not in al and "f_stop" not in al:
            continue
        tn = str(attr.GetTypeName() or "")
        if tn not in ("float", "double"):
            continue
        try:
            old = float(attr.Get())
        except Exception:
            continue
        if "fstop" in al or "f_stop" in al:
            new_v = max(0.05, old / 8.0)
        elif "iso" in al:
            new_v = old * 16.0
        else:
            new_v = old + 10.0
        try:
            attr.Set(float(new_v))
            touched.append(
                {
                    "prim_path": cam_path,
                    "attr_name": an,
                    "type_name": tn,
                    "old": float(old),
                    "new": float(new_v),
                }
            )
        except Exception as exc:
            errors.append({"attr": an, "error": str(exc)})
    return touched, errors


def _rule11_camera_chain_enter_manual_overexposure(stage, cam_path: str) -> dict:
    """
    显式 rule_id=11：在现有穹顶/直射光之外，临时切到「手动过曝」渲染/相机链（carb RTX post + 相机 float 属性），
    抓图与 hazard 判定结束后由 _rule11_camera_chain_restore_baseline 恢复。
    """
    _rule11_camera_chain_restore_baseline()
    diag: dict = {
        "mode": "explicit_rule11_manual_overexposure",
        "camera_prim": str(cam_path),
        "carb": {"attempts": [], "applied": []},
        "usd": {"applied": [], "errors": []},
        "matching_setting_keys_sample": [],
    }
    bundle: dict = {"carb": [], "usd": []}
    try:
        import carb

        settings = carb.settings.get_settings()
    except Exception as exc:
        diag["carb"]["fatal"] = str(exc)
        with _scene_lock:
            _scene_state["rule11_exposure_restore_bundle"] = bundle
        return diag

    diag["matching_setting_keys_sample"] = _rule11_carb_probe_exposure_related_keys(settings)

    # 1) 关闭 auto exposure（多路径兼容不同 Kit/插件前缀）
    for path, new_v, tag in (
        ("/rtx/post/autoExposure/enabled", False, "post_auto_exposure"),
        ("/rtx/autoExposure/enabled", False, "rtx_auto_exposure"),
    ):
        ok_r, prev = _rule11_carb_try_read(settings, path)
        row = {"path": path, "tag": tag, "read_ok": ok_r, "prev": _json_safe_value(prev)}
        if ok_r:
            ok_w, err = _rule11_carb_try_write(settings, path, new_v)
            row["write_ok"] = ok_w
            row["write_error"] = err
            if ok_w:
                bundle["carb"].append({"path": path, "prev": prev})
                diag["carb"]["applied"].append({**row, "new": new_v})
        diag["carb"]["attempts"].append(row)

    # 2) 尝试 Clamp/线性类 tonemap：减少对高亮的 filmic 压缩（版本差异大，失败则跳过）
    for path, new_v, tag in (
        ("/rtx/post/tonemap/op", 0, "tonemap_op_int"),
        ("/rtx/post/tonemap/operator", 0, "tonemap_operator_int"),
    ):
        ok_r, prev = _rule11_carb_try_read(settings, path)
        row = {"path": path, "tag": tag, "read_ok": ok_r, "prev": _json_safe_value(prev)}
        if ok_r:
            ok_w, err = _rule11_carb_try_write(settings, path, new_v)
            row["write_ok"] = ok_w
            row["write_error"] = err
            if ok_w:
                bundle["carb"].append({"path": path, "prev": prev})
                diag["carb"]["applied"].append({**row, "new": new_v})
        diag["carb"]["attempts"].append(row)

    # 3) 在 tonemap exposure 上叠加固定 EV（不改变 scene_perception 阈值，只抬高输出 JPEG 亮度）
    boost = 8.0
    for path, tag in (
        ("/rtx/post/tonemap/exposure", "tonemap_exposure"),
        ("/rtx/post/tonemap/exposureValue", "tonemap_exposure_value"),
    ):
        ok_r, prev = _rule11_carb_try_read(settings, path)
        row = {"path": path, "tag": tag, "read_ok": ok_r, "prev": _json_safe_value(prev)}
        if not ok_r:
            diag["carb"]["attempts"].append(row)
            continue
        try:
            base = float(prev)
        except (TypeError, ValueError):
            diag["carb"]["attempts"].append({**row, "skip": "prev_not_float"})
            continue
        new_v = base + boost
        ok_w, err = _rule11_carb_try_write(settings, path, new_v)
        row["write_ok"] = ok_w
        row["write_error"] = err
        if ok_w:
            bundle["carb"].append({"path": path, "prev": prev})
            diag["carb"]["applied"].append({**row, "new": float(new_v), "boost": boost})
        diag["carb"]["attempts"].append(row)

    usd_touched, usd_err = _rule11_usd_try_boost_exposure_like_attrs(stage, cam_path)
    diag["usd"]["errors"] = usd_err
    for u in usd_touched:
        prim_path = str(u.get("prim_path") or "")
        an = str(u.get("attr_name") or "")
        bundle["usd"].append({"prim_path": prim_path, "attr_name": an, "prev": u.get("old")})
        diag["usd"]["applied"].append(u)

    diag["carb"]["applied_count"] = len(bundle["carb"])
    diag["usd"]["applied_count"] = len(bundle["usd"])
    diag["render_chain_controllable"] = bool(bundle["carb"] or bundle["usd"])
    if not diag["render_chain_controllable"]:
        diag["bottleneck_note"] = (
            "未发现可写入的 carb tonemap/autoExposure 键，且相机 prim 上无 exposure/iso/fstop 类 float 属性；"
            "高亮可能被全局 PathTracing + RTX post（auto exposure / filmic tonemap）在 Replicator rgb 输出前压回。"
        )

    with _scene_lock:
        _scene_state["rule11_exposure_restore_bundle"] = bundle

    try:
        sim_app.update()
    except Exception as exc:
        diag["sim_app_update_error"] = str(exc)

    print("[rule11-exposure] " + json.dumps(diag, ensure_ascii=False, default=str), flush=True)
    return diag


def _rule11_archive_eval_jpeg_and_evidence(
    *,
    jpeg_bytes: bytes | None,
    result: dict,
    request_meta: dict | None,
    hazard_eval: dict | None,
    exposure_restore: dict | None = None,
) -> dict:
    """将用于判定的 JPEG 与同次证据 JSON 落盘到 archive/rule11_eval。"""
    out: dict = {"archive_dir": _RULE11_EVAL_ARCHIVE_DIR, "jpeg_path": None, "evidence_json_path": None}
    try:
        os.makedirs(_RULE11_EVAL_ARCHIVE_DIR, exist_ok=True)
    except Exception as exc:
        out["mkdir_error"] = str(exc)
        return out
    rid = None
    if isinstance(request_meta, dict):
        rid = str(request_meta.get("request_id") or "").strip() or None
    stem = time.strftime("%Y%m%d_%H%M%S") + "_" + (rid or uuid.uuid4().hex[:10])
    jpg_path = os.path.join(_RULE11_EVAL_ARCHIVE_DIR, f"{stem}_eval.jpg")
    json_path = os.path.join(_RULE11_EVAL_ARCHIVE_DIR, f"{stem}_evidence.json")
    if isinstance(jpeg_bytes, (bytes, bytearray)) and len(jpeg_bytes) > 0:
        try:
            with open(jpg_path, "wb") as f:
                f.write(jpeg_bytes)
            out["jpeg_path"] = jpg_path
        except Exception as exc:
            out["jpeg_write_error"] = str(exc)
    else:
        out["jpeg_write_error"] = "no_jpeg_bytes"
    import copy

    hazard_for_file = copy.deepcopy(hazard_eval) if isinstance(hazard_eval, dict) else hazard_eval
    if isinstance(hazard_for_file, dict):
        evo = hazard_for_file.get("evidence")
        if isinstance(evo, dict):
            evo["rule11_archive_jpeg_path"] = out.get("jpeg_path")
            evo["rule11_archive_evidence_json_path"] = json_path

    ev_doc = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "request_id": rid,
        "rule11_explicit_request": bool(result.get("rule11_explicit_request")),
        "camera_prim": str(camera_prim),
        "rule11_camera_exposure_adjustment": result.get("rule11_camera_exposure_adjustment"),
        "rule11_lighting_adjustment": result.get("rule11_lighting_adjustment"),
        "rule11_exposure_restore": exposure_restore,
        "hazard_eval": hazard_for_file,
        "jpeg_metrics": analyze_jpeg_overexposure_metrics(
            jpeg_bytes if isinstance(jpeg_bytes, (bytes, bytearray)) else None
        ),
    }
    try:
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(ev_doc, f, ensure_ascii=False, indent=2, default=str)
        out["evidence_json_path"] = json_path
    except Exception as exc:
        out["json_write_error"] = str(exc)
    return out


def _rule11_collect_dome_paths_for_rule11(stage) -> list[str]:
    dsky = str(cfg.get("dynamic_sky_root_prim") or "/World/DynamicSkyRoot")
    paths = list(_collect_hdri_environment_dome_paths(stage, dsky))
    for aux in ("/World/AmbientBoost",):
        if aux not in paths:
            ap = stage.GetPrimAtPath(aux)
            if ap and ap.IsValid() and ap.GetTypeName() == "DomeLight":
                paths.append(aux)
    return paths


def _rule11_snapshot_hdri_dome_energy_baseline(stage) -> None:
    """记录 HDRI 穹顶标称 exposure/intensity，供 rule11 每次先还原再加亮。"""
    paths = _rule11_collect_dome_paths_for_rule11(stage)
    snap: dict[str, dict] = {}
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            snap[p] = _read_dome_energy_state(prim)
    with _scene_lock:
        _scene_state["rule11_hdri_dome_energy_baseline"] = snap


def _rule11_restore_dome_baseline_then_paths(stage) -> tuple[list[str], dict]:
    """供 rule11 光照：还原标称后再返回待写穹顶路径列表。"""
    paths = _rule11_collect_dome_paths_for_rule11(stage)
    with _scene_lock:
        baseline = _scene_state.get("rule11_hdri_dome_energy_baseline")
        have_baseline = isinstance(baseline, dict) and len(baseline) > 0
    if not have_baseline:
        _rule11_snapshot_hdri_dome_energy_baseline(stage)
        with _scene_lock:
            baseline = dict(_scene_state.get("rule11_hdri_dome_energy_baseline") or {})
    else:
        baseline = dict(baseline)
    _restore_hdri_dome_env_from_snapshot(stage, baseline)
    return paths, baseline


def _clear_rule11_overexposure_residue_if_disabled(stage, random_cfg: dict | None) -> None:
    """当关闭 random_overexposure_event 或 probability<=0 时，尝试用 rule11 baseline 还原 DomeLight exposure/intensity。"""
    cfg_in = random_cfg if isinstance(random_cfg, dict) else {}
    enabled = bool(cfg_in.get("random_overexposure_event"))
    try:
        prob = float(cfg_in.get("random_overexposure_event_probability") or 0.2)
    except (TypeError, ValueError):
        prob = 0.2
    prob = max(0.0, min(1.0, prob))
    disabled_or_zero = (not enabled) or prob <= 0.0
    if not disabled_or_zero:
        return

    with _scene_lock:
        baseline0 = _scene_state.get("rule11_hdri_dome_energy_baseline")
        baseline = dict(baseline0) if isinstance(baseline0, dict) else None
    have_baseline = isinstance(baseline, dict) and len(baseline) > 0
    if not have_baseline:
        print(
            "[rule11-overexposure-restore] disabled_or_zero_prob skip "
            f"enabled={enabled} probability={prob} baseline_exists={bool(have_baseline)} reason=baseline_missing",
            flush=True,
        )
        return

    try:
        paths = _rule11_collect_dome_paths_for_rule11(stage) if stage is not None else []
    except Exception as exc:
        print(
            "[rule11-overexposure-restore] disabled_or_zero_prob skip "
            f"enabled={enabled} probability={prob} baseline_exists={bool(have_baseline)} "
            f"reason=collect_paths_failed error={exc}",
            flush=True,
        )
        return

    snap_for_restore: dict[str, dict] = {}
    missing_in_baseline = 0
    for p in paths:
        saved = baseline.get(p) if isinstance(baseline, dict) else None
        if isinstance(saved, dict) and saved:
            snap_for_restore[str(p)] = saved
        else:
            missing_in_baseline += 1

    if not snap_for_restore:
        print(
            "[rule11-overexposure-restore] disabled_or_zero_prob skip "
            f"enabled={enabled} probability={prob} baseline_exists={bool(have_baseline)} "
            f"paths={len(paths)} restore_paths=0 missing_in_baseline={missing_in_baseline}",
            flush=True,
        )
        return

    try:
        _restore_hdri_dome_env_from_snapshot(stage, snap_for_restore)
        print(
            "[rule11-overexposure-restore] disabled_or_zero_prob restore attempted "
            f"enabled={enabled} probability={prob} baseline_exists={bool(have_baseline)} "
            f"paths={len(paths)} restore_paths={len(snap_for_restore)} missing_in_baseline={missing_in_baseline} "
            f"targets_sample={(list(snap_for_restore.keys())[:4])}",
            flush=True,
        )
    except Exception as exc:
        print(
            "[rule11-overexposure-restore] disabled_or_zero_prob restore failed "
            f"enabled={enabled} probability={prob} baseline_exists={bool(have_baseline)} "
            f"restore_paths={len(snap_for_restore)} error={exc}",
            flush=True,
        )


# POST {"rule_id":11}：固定强过曝（可复现），与 random_config 无关；优先写 DomeLight exposure/intensity
_RULE11_REQUEST_FIXED_EXPOSURE_MAIN = 48.0
_RULE11_REQUEST_FIXED_INTENSITY_MAIN = 2500000.0
_RULE11_REQUEST_FIXED_EXPOSURE_AMBIENT = 46.0
_RULE11_REQUEST_FIXED_INTENSITY_AMBIENT = 2800000.0
# 根场景 KeyLight（DistantLight）：穹顶拉满仍可能 tone-map/构图偏暗，补一条固定直射能量保证 JPEG 统计过阈
_RULE11_REQUEST_FIXED_KEYLIGHT_INTENSITY = 8.0e7


def _rule11_apply_request_fixed_overexposure_dome_lights(stage) -> dict:
    """显式 rule_id=11 请求：先还原标称穹顶，再一次性写入固定绝对 exposure/intensity（确定性）。"""
    paths, baseline = _rule11_restore_dome_baseline_then_paths(stage)
    kl0 = stage.GetPrimAtPath("/World/KeyLight")
    if kl0 and kl0.IsValid() and kl0.GetTypeName() == "DistantLight":
        for in_name in ("inputs:intensity", "intensity"):
            ia0 = kl0.GetAttribute(in_name)
            if not (ia0 and ia0.IsValid()):
                continue
            with _scene_lock:
                kb0 = _scene_state.get("rule11_keylight_intensity_baseline")
            if not isinstance(kb0, dict):
                try:
                    v0 = float(ia0.Get())
                except (TypeError, ValueError):
                    v0 = 3000.0
                with _scene_lock:
                    _scene_state["rule11_keylight_intensity_baseline"] = {"attr": in_name, "value": v0}
            else:
                try:
                    ia0.Set(float(kb0.get("value") or 3000.0))
                except Exception:
                    pass
            break
    details: list[dict] = []
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if prim is None or not prim.IsValid():
            continue
        ps = str(p or "")
        is_ambient = ps.endswith("AmbientBoost") or "/AmbientBoost" in ps
        tgt_e = float(_RULE11_REQUEST_FIXED_EXPOSURE_AMBIENT if is_ambient else _RULE11_REQUEST_FIXED_EXPOSURE_MAIN)
        tgt_i = float(_RULE11_REQUEST_FIXED_INTENSITY_AMBIENT if is_ambient else _RULE11_REQUEST_FIXED_INTENSITY_MAIN)
        row: dict = {
            "prim_path": ps,
            "mode": "request_rule11_fixed_abs",
            "target_exposure": tgt_e,
            "target_intensity": tgt_i,
        }
        changed = False
        for ex_name in ("inputs:exposure", "exposure"):
            ea = prim.GetAttribute(ex_name)
            if ea and ea.IsValid():
                try:
                    row["old_exposure"] = float(ea.Get())
                except (TypeError, ValueError):
                    row["old_exposure"] = None
                try:
                    ea.Set(float(tgt_e))
                    row["exposure_attr"] = ex_name
                    changed = True
                except Exception:
                    pass
                break
        for in_name in ("inputs:intensity", "intensity"):
            ia = prim.GetAttribute(in_name)
            if ia and ia.IsValid():
                try:
                    row["old_intensity"] = float(ia.Get())
                except (TypeError, ValueError):
                    row["old_intensity"] = None
                try:
                    ia.Set(float(tgt_i))
                    row["intensity_attr"] = in_name
                    changed = True
                except Exception:
                    pass
                break
        if changed:
            details.append(row)
    key_row = None
    kl = stage.GetPrimAtPath("/World/KeyLight")
    if kl and kl.IsValid() and kl.GetTypeName() == "DistantLight":
        for in_name in ("inputs:intensity", "intensity"):
            ia = kl.GetAttribute(in_name)
            if ia and ia.IsValid():
                try:
                    old_k = float(ia.Get())
                except (TypeError, ValueError):
                    old_k = None
                try:
                    ia.Set(float(_RULE11_REQUEST_FIXED_KEYLIGHT_INTENSITY))
                    key_row = {
                        "prim_path": "/World/KeyLight",
                        "mode": "request_rule11_fixed_abs",
                        "intensity_attr": in_name,
                        "old_intensity": old_k,
                        "new_intensity": float(_RULE11_REQUEST_FIXED_KEYLIGHT_INTENSITY),
                    }
                except Exception:
                    key_row = None
                break
    return {
        "applied": bool(details) or bool(key_row),
        "mode": "request_rule11_fixed_abs",
        "dome_paths_considered": list(paths),
        "domes_adjusted": details,
        "key_light_adjustment": key_row,
        "baseline_paths": list(baseline.keys()) if isinstance(baseline, dict) else [],
    }


def _rule11_boost_environment_exposure(stage, delta_exposure: float) -> dict:
    """在 HDRI 模式下抬高环境穹顶 exposure/intensity（随机 rule11 分支用 delta，非显式请求固定值）。"""
    paths, _baseline = _rule11_restore_dome_baseline_then_paths(stage)
    details: list[dict] = []
    de = float(delta_exposure)
    # exposure 与 intensity 同步抬高：intensity 按档位倍增，避免仅靠 exposure 仍达不到 JPEG 阈值
    # 但是在 RTXRealTime + AutoExposure 时，只要场景不是过暗就不会发黑，这里先不要动 intensity
    intensity_scale = 1.0 # 1.0 + min(30.0, max(0.0, de) / 2.25)
    for p in paths:
        prim = stage.GetPrimAtPath(p)
        if prim is None or not prim.IsValid():
            continue
        snap = _read_dome_energy_state(prim)
        eattr = snap.get("exposure_attr")
        old = snap.get("exposure")
        try:
            old_f = float(old) if old is not None else 0.0
        except (TypeError, ValueError):
            old_f = 0.0
        new_f = old_f + de
        exp_attr = None
        if isinstance(eattr, str) and eattr:
            exp_attr = prim.GetAttribute(eattr)
        if exp_attr is None or not exp_attr.IsValid():
            for name in ("inputs:exposure", "exposure"):
                t = prim.GetAttribute(name)
                if t and t.IsValid():
                    exp_attr = t
                    eattr = name
                    break
        row: dict = {"prim_path": p}
        changed = False
        if exp_attr is not None and exp_attr.IsValid():
            try:
                exp_attr.Set(float(new_f))
                row["exposure_attr"] = eattr
                row["old_exposure"] = old_f
                row["new_exposure"] = new_f
                changed = True
            except Exception:
                pass
        iattr = snap.get("intensity_attr")
        iold = snap.get("intensity")
        iattr_use = iattr if isinstance(iattr, str) and iattr else None
        ia = prim.GetAttribute(iattr_use) if iattr_use else None
        if ia is None or not ia.IsValid():
            for name in ("inputs:intensity", "intensity"):
                t = prim.GetAttribute(name)
                if t and t.IsValid():
                    ia = t
                    iattr_use = name
                    break
        if ia is not None and ia.IsValid():
            try:
                i_base = float(iold) if iold is not None else float(ia.Get())
            except (TypeError, ValueError):
                i_base = 0.0
            if i_base > 1e-9:
                i_new = i_base * intensity_scale
            else:
                i_new = max(15000.0, 55000.0 * (intensity_scale / 10.0))
            try:
                ia.Set(float(i_new))
                row["intensity_attr"] = iattr_use
                row["old_intensity"] = float(i_base)
                row["new_intensity"] = float(i_new)
                row["intensity_scale"] = float(intensity_scale)
                changed = True
            except Exception:
                pass
        if changed:
            details.append(row)
    return {
        "applied": bool(details),
        "delta_exposure": float(delta_exposure),
        "intensity_scale": float(intensity_scale),
        "dome_paths_considered": list(paths),
        "domes_adjusted": details,
    }


def _evaluate_rule_11_overexposure_camera(context: dict) -> dict:
    _ = context
    jpg = _get_cached_snapshot_jpeg_bytes()
    metrics = analyze_jpeg_overexposure_metrics(jpg)
    ev = {
        "mean_brightness": metrics.get("mean_brightness"),
        "bright_pixel_ratio": metrics.get("bright_pixel_ratio"),
        "saturated_pixel_ratio": metrics.get("saturated_pixel_ratio"),
        "capture_effective": metrics.get("capture_effective"),
        "jpeg_bytes_len": len(jpg) if jpg else 0,
        "thresholds": rule11_overexposure_thresholds_meta(),
    }
    if metrics.get("decode_error"):
        ev["decode_error"] = metrics.get("decode_error")
        return _build_hazard_eval_payload(
            11,
            rule_name=_HAZARD_RULE_DEFS.get(11),
            supported=False,
            has_hazard=None,
            reason="snapshot_jpeg_unavailable_or_decode_failed",
            evidence=ev,
        )
    cap = metrics.get("capture_effective")
    if not isinstance(cap, bool):
        return _build_hazard_eval_payload(
            11,
            rule_name=_HAZARD_RULE_DEFS.get(11),
            supported=False,
            has_hazard=None,
            reason="exposure_metrics_incomplete",
            evidence=ev,
        )
    has_hazard = not cap
    reason = (
        "camera_overexposed_cannot_capture_effectively"
        if has_hazard
        else "camera_exposure_allows_effective_capture"
    )
    return _build_hazard_eval_payload(
        11,
        rule_name=_HAZARD_RULE_DEFS.get(11),
        supported=True,
        has_hazard=bool(has_hazard),
        reason=reason,
        evidence=ev,
    )


_HAZARD_RULE_DEFS = {
    1: "防护栏/挡脚板缺失或不可用",
    2: "吊篮内作业人员超过 2 人",
    3: "吊篮单人作业",
    4: "作业结束后未将吊篮降至地面",
    5: "吊篮作业人员安全绳未单人使用（数量口径）",
    12: "限位装置不符",
    13: "防坠安全锁不符",
    11: "光照过大导致模拟相机过曝，无法有效捕捉目标画面",
}
_CHECK_SAFETY_HAZARD_DEFAULTS = getattr(check_safety_hazard, "__defaults__", ()) or ()
_HAZARD_GROUND_Z_BASELINE = (
    float(_CHECK_SAFETY_HAZARD_DEFAULTS[0])
    if len(_CHECK_SAFETY_HAZARD_DEFAULTS) >= 1
    else 0.12
)
_HAZARD_GROUND_Z_EPS = (
    float(_CHECK_SAFETY_HAZARD_DEFAULTS[1])
    if len(_CHECK_SAFETY_HAZARD_DEFAULTS) >= 2
    else 0.5
)

# GET /api/scene/randomize/last 对外标准答案：rule_id、hazard_type、random_factor、hazard_name 的唯一权威表
# （内部 hazard_eval 仍可能为历史 rule_id=11 过曝；对外统一映射为 rule_id=10）
HAZARD_RULE_STANDARD_SPEC: dict[int, dict[str, str]] = {
    1: {
        "random_factor": "防护栏杆挡脚板",
        "hazard_name": "挡脚板缺失、不可用",
        "hazard_type": "toe_board_missing",
    },
    2: {
        "random_factor": "吊篮内人数",
        "hazard_name": "吊篮内作业人员超过2人",
        "hazard_type": "worker_count_over_2",
    },
    3: {
        "random_factor": "吊篮内人数",
        "hazard_name": "吊篮里单人作业",
        "hazard_type": "single_worker_in_gondola",
    },
    4: {
        "random_factor": "吊篮高度",
        "hazard_name": "作业结束后未将吊篮降至地面",
        "hazard_type": "gondola_not_lowered_to_ground_after_work",
    },
    5: {
        "random_factor": "安全绳",
        "hazard_name": "吊篮作业人员安全绳未单人使用",
        "hazard_type": "safety_rope_not_individual",
    },
    6: {
        "random_factor": "工人进入方式",
        "hazard_name": "人员未从地面进入",
        "hazard_type": "worker_not_enter_from_ground",
    },
    7: {
        "random_factor": "吊篮稳定块",
        "hazard_name": "未采取防摆动措施",
        "hazard_type": "anti_sway_measure_missing",
    },
    8: {
        "random_factor": "天气（雾）",
        "hazard_name": "雾气过大导致摄像头捕捉画面模糊",
        "hazard_type": "fog_blur",
    },
    9: {
        "random_factor": "天气（雨）",
        "hazard_name": "雨水过大导致摄像头捕捉画面模糊",
        "hazard_type": "rain_blur",
    },
    10: {
        "random_factor": "光照",
        "hazard_name": "光照过大导致摄像头曝光无法正常捕捉画面",
        "hazard_type": "overexposure",
    },
    12: {
        "random_factor": "限位装置",
        "hazard_name": "限位装置不符",
        "hazard_type": "limit_device_noncompliant",
    },
    13: {
        "random_factor": "防坠安全锁",
        "hazard_name": "防坠安全锁不符",
        "hazard_type": "fall_arrestor_noncompliant",
    },
}


def _normalize_public_rule_id(raw_rule_id) -> int | None:
    """将内部 rule_id 规范为对外 SaaS rule_id（当前仅 11→10）。"""
    if raw_rule_id in (None, ""):
        return None
    try:
        rid = int(raw_rule_id)
    except (TypeError, ValueError):
        return None
    if rid == 11:
        return 10
    return rid


def _hazard_rule_standard_meta(public_rule_id: int | None) -> dict[str, str] | None:
    if public_rule_id is None:
        return None
    row = HAZARD_RULE_STANDARD_SPEC.get(int(public_rule_id))
    return dict(row) if isinstance(row, dict) else None


def _standard_answer_evidence_from_hazard_eval(hazard_eval: dict | None) -> dict:
    """从 hazard_eval.evidence 复制并补充兼容字段（如 limit 别名），不修改原始 hazard_eval。"""
    if not isinstance(hazard_eval, dict):
        return {}
    ev0 = hazard_eval.get("evidence")
    if not isinstance(ev0, dict):
        return {}
    ev = copy.deepcopy(ev0)
    if "limit" not in ev and "threshold" in ev:
        try:
            ev["limit"] = int(ev["threshold"])
        except (TypeError, ValueError):
            try:
                ev["limit"] = int(float(ev["threshold"]))
            except (TypeError, ValueError):
                pass
    return ev


def _standard_answer_shell_null() -> dict:
    return {
        "hazard": None,
        "rule_id": None,
        "hazard_id": None,
        "hazard_name": None,
        "random_factor": None,
    }


def _build_randomize_last_answer_top(
    last: dict | None,
    *,
    raw_snapshot: dict | None = None,
) -> dict:
    """
    组装 GET /api/scene/randomize/last 顶层标准答案字段（与 last 块并存，兼容旧客户端）。
    """
    if not isinstance(last, dict):
        return {
            "has_last_result": False,
            "hazard": None,
            "rule_id": None,
            "hazard_id": None,
            "hazard_type": None,
            "random_factor": None,
            "hazard_name": None,
            "description": "尚无有效随机结果或当前结果无法判定",
            "evidence": {},
            "standard_answer": _standard_answer_shell_null(),
            "raw": None,
        }
    he = last.get("hazard_eval")
    he = he if isinstance(he, dict) else {}
    hh = he.get("has_hazard")
    if hh is None and "hazard" in he:
        hh = he.get("hazard")
    rid_raw = he.get("rule_id")
    pub = _normalize_public_rule_id(rid_raw)
    meta = _hazard_rule_standard_meta(pub)
    ev = _standard_answer_evidence_from_hazard_eval(he)
    sa0 = _standard_answer_shell_null()
    he_snap = raw_snapshot.get("hazard_eval") if isinstance(raw_snapshot, dict) else None
    compat: dict = {}
    if isinstance(raw_snapshot, dict):
        compat["hazard_eval"] = he_snap
        compat["timestamp"] = raw_snapshot.get("timestamp")
        compat["event_id"] = last.get("event_id") or (
            he_snap.get("event_id") if isinstance(he_snap, dict) else None
        )
        compat["metadata"] = last.get("randomize_event_meta")
        compat["reason"] = he.get("reason")

    if hh is True:
        hn = (meta or {}).get("hazard_name") or str(he.get("rule_name") or "").strip() or None
        hf = (meta or {}).get("random_factor")
        ht = (meta or {}).get("hazard_type") if meta else None
        desc = f"本次随机结果命中隐患：{hn}" if hn else "本次随机结果命中隐患"
        out = {
            "has_last_result": True,
            "hazard": True,
            "rule_id": pub,
            "hazard_id": pub,
            "hazard_type": ht,
            "random_factor": hf,
            "hazard_name": hn,
            "description": desc,
            "evidence": ev,
            "standard_answer": {
                "hazard": True,
                "rule_id": pub,
                "hazard_id": pub,
                "hazard_name": hn,
                "random_factor": hf,
            },
            "raw": raw_snapshot,
        }
        out.update(compat)
        return out
    if hh is False:
        out = {
            "has_last_result": True,
            "hazard": False,
            "rule_id": None,
            "hazard_id": None,
            "hazard_type": None,
            "random_factor": None,
            "hazard_name": None,
            "description": "本次随机结果未命中隐患",
            "evidence": ev,
            "standard_answer": {
                "hazard": False,
                "rule_id": None,
                "hazard_id": None,
                "hazard_name": None,
                "random_factor": None,
            },
            "raw": raw_snapshot,
        }
        out.update(compat)
        return out
    out = {
        "has_last_result": True,
        "hazard": None,
        "rule_id": None,
        "hazard_id": None,
        "hazard_type": None,
        "random_factor": None,
        "hazard_name": None,
        "description": "当前随机结果无法判定",
        "evidence": ev,
        "standard_answer": sa0,
        "raw": raw_snapshot,
    }
    out.update(compat)
    return out


def _coerce_rule_id(raw_value) -> int | None:
    if raw_value in (None, ""):
        return None
    try:
        return int(raw_value)
    except (TypeError, ValueError):
        return None


def _extract_requested_hazard_rule(request_meta: dict | None) -> tuple[int | None, str | None, object]:
    if not isinstance(request_meta, dict):
        return None, None, None
    for key in ("rule_id", "event_id"):
        if key in request_meta:
            return _coerce_rule_id(request_meta.get(key)), key, request_meta.get(key)
    return None, None, None


def _ensure_randomize_rule_metadata(result: dict, request_meta: dict) -> tuple[dict, dict | None]:
    """
    为本次 randomize 补齐「规则实例」元数据，供 hazard_eval / scene_state_evaluation 消费。

    - 若请求体已含 rule_id / event_id（或 debug_force_rule_id 且未显式 rule_id）：视为**事件生成侧显式指定**，不写 scene 归因。
    - 否则按活动吊篮**场景人数 workers**做稳定归因（非几何推断）：
        0 人 → rule 4；1 人 → rule 3；≥2 人 → rule 2。
    """
    meta = dict(request_meta)
    if _coerce_rule_id(meta.get("rule_id")) is None and _coerce_rule_id(meta.get("event_id")) is None:
        df = _coerce_rule_id(meta.get("debug_force_rule_id"))
        if df is not None:
            meta["rule_id"] = int(df)

    rid_req, key_req, raw_req = _extract_requested_hazard_rule(meta)
    if rid_req is not None:
        eid = str(meta.get("event_instance_id") or "").strip()
        if not eid:
            eid = f"evt_r{int(rid_req)}_{uuid.uuid4().hex[:12]}"
        event_meta = {
            "rule_id": int(rid_req),
            "rule_name": _HAZARD_RULE_DEFS.get(int(rid_req)),
            "event_id": eid,
            "source": "request",
            "request_rule_key": key_req,
            "request_rule_raw": raw_req,
        }
        return meta, event_meta

    wc, wsrc = _resolve_active_worker_count_for_hazard(result)
    if wc is None:
        return meta, None
    wc_i = int(wc)
    if wc_i == 0:
        rid = 4
    elif wc_i == 1:
        rid = 3
    else:
        rid = 2
    meta["rule_id"] = int(rid)
    eid = str(meta.get("event_instance_id") or "").strip()
    if not eid:
        eid = f"evt_r{rid}_{uuid.uuid4().hex[:12]}"
    event_meta = {
        "rule_id": int(rid),
        "rule_name": _HAZARD_RULE_DEFS.get(int(rid)),
        "event_id": eid,
        "source": "scene_state_attribution",
        "worker_count_at_attribution": wc_i,
        "worker_count_source": wsrc,
    }
    return meta, event_meta


def _build_hazard_eval_payload(
    rule_id,
    *,
    rule_name=None,
    supported: bool,
    has_hazard,
    reason: str,
    evidence: dict | None = None,
) -> dict:
    return {
        "rule_id": rule_id,
        "rule_name": rule_name,
        "supported": bool(supported),
        "has_hazard": has_hazard,
        "hazard": has_hazard,
        "severity": "warning",
        "reason": str(reason),
        "evidence": _safe_plain_dict(evidence),
    }


def _resolve_active_worker_count_for_hazard(result: dict) -> tuple[int | None, str | None]:
    """人数优先取 randomize 写入的 `workers`（场景侧启用数量），与 scene_state 隐患评估一致。"""
    wk = result.get("workers")
    if wk is not None:
        try:
            return int(wk), "workers"
        except (TypeError, ValueError):
            pass
    active_path = str(result.get("active_diaolan_path") or "").strip()
    workers_by = result.get("workers_visible_count_by_diaolan_path")
    if isinstance(workers_by, dict) and active_path and active_path in workers_by:
        try:
            return int(workers_by.get(active_path)), "workers_visible_count_by_diaolan_path"
        except (TypeError, ValueError):
            pass
    for key in ("workers_count", "visible_workers", "workers"):
        if key in result and result.get(key) is not None:
            try:
                return int(result.get(key)), key
            except (TypeError, ValueError):
                continue
    visible_paths = result.get("visible_worker_paths")
    if isinstance(visible_paths, list):
        return count_logical_workers_from_paths(visible_paths), "visible_worker_paths_logical"
    return None, None


def _resolve_active_gondola_world_height_for_hazard(result: dict) -> tuple[float | None, str | None]:
    active_path = str(result.get("active_diaolan_path") or "").strip()
    gondola_heights = result.get("gondola_heights")
    if isinstance(gondola_heights, dict) and active_path and active_path in gondola_heights:
        try:
            return float(gondola_heights.get(active_path)), "gondola_heights"
        except (TypeError, ValueError):
            pass
    height_debug = result.get("height_debug")
    if isinstance(height_debug, dict) and height_debug.get("final_world_z") is not None:
        try:
            return float(height_debug.get("final_world_z")), "height_debug.final_world_z"
        except (TypeError, ValueError):
            pass
    if result.get("gondola_y") is not None:
        try:
            return float(result.get("gondola_y")), "gondola_y"
        except (TypeError, ValueError):
            pass
    return None, None


def _build_random_event_hazard_context(result: dict, request_meta: dict | None) -> dict:
    rule_id, rule_key, raw_rule_value = _extract_requested_hazard_rule(request_meta)
    worker_count, worker_source = _resolve_active_worker_count_for_hazard(result)
    gondola_world_height, gondola_height_source = _resolve_active_gondola_world_height_for_hazard(result)
    active_path = str(result.get("active_diaolan_path") or "").strip() or None
    gondola_height_cm = result.get("gondola_height_cm")
    try:
        gondola_height_cm = None if gondola_height_cm is None else float(gondola_height_cm)
    except (TypeError, ValueError):
        gondola_height_cm = None
    return {
        "rule_id": rule_id,
        "rule_key": rule_key,
        "raw_rule_value": raw_rule_value,
        "rule_name": _HAZARD_RULE_DEFS.get(rule_id),
        "random_event": result.get("random_event"),
        "active_diaolan_path": active_path,
        "worker_count": worker_count,
        "worker_count_source": worker_source,
        "gondola_world_height": gondola_world_height,
        "gondola_world_height_source": gondola_height_source,
        "gondola_height_cm": gondola_height_cm,
        "ground_z_baseline": _HAZARD_GROUND_Z_BASELINE,
        "ground_eps": _HAZARD_GROUND_Z_EPS,
    }


def _evaluate_rule_worker_count_gt_two(context: dict) -> dict:
    worker_count = context.get("worker_count")
    evidence = {
        "worker_count": worker_count,
        "threshold": 2,
        "worker_count_source": context.get("worker_count_source"),
        "active_diaolan_path": context.get("active_diaolan_path"),
    }
    if worker_count is None:
        return _build_hazard_eval_payload(
            2,
            rule_name=_HAZARD_RULE_DEFS[2],
            supported=True,
            has_hazard=None,
            reason="当前缺少吊篮人数状态，无法判断是否超过2人",
            evidence=evidence,
        )
    has_hazard = int(worker_count) > 2
    reason = (
        f"当前吊篮人数为{worker_count}，超过2人"
        if has_hazard
        else f"当前吊篮人数为{worker_count}，未超过2人"
    )
    return _build_hazard_eval_payload(
        2,
        rule_name=_HAZARD_RULE_DEFS[2],
        supported=True,
        has_hazard=has_hazard,
        reason=reason,
        evidence=evidence,
    )


def _evaluate_rule_single_worker(context: dict) -> dict:
    worker_count = context.get("worker_count")
    evidence = {
        "worker_count": worker_count,
        "expected_worker_count": 1,
        "worker_count_source": context.get("worker_count_source"),
        "active_diaolan_path": context.get("active_diaolan_path"),
    }
    if worker_count is None:
        return _build_hazard_eval_payload(
            3,
            rule_name=_HAZARD_RULE_DEFS[3],
            supported=True,
            has_hazard=None,
            reason="当前缺少吊篮人数状态，无法判断是否为单人作业",
            evidence=evidence,
        )
    has_hazard = int(worker_count) == 1
    reason = (
        f"当前吊篮人数为{worker_count}，构成单人作业"
        if has_hazard
        else f"当前吊篮人数为{worker_count}，非单人作业"
    )
    return _build_hazard_eval_payload(
        3,
        rule_name=_HAZARD_RULE_DEFS[3],
        supported=True,
        has_hazard=has_hazard,
        reason=reason,
        evidence=evidence,
    )


def _try_get_usd_stage_for_hazard():
    """隐患场景态读 USD：在 Kit 外返回 None（降级为 unsupported / 证据不足）。"""
    try:
        import omni.usd

        return omni.usd.get_context().get_stage()
    except Exception:
        return None


def _evaluate_rule_guardrails_scene(context: dict) -> dict:
    """新模型适配：护栏/挡脚板（Fanghulangan + Front_01/02）场景态判定，非相机视觉。"""
    stage = _try_get_usd_stage_for_hazard()
    ap = context.get("active_diaolan_path")
    if stage is None:
        return _build_hazard_eval_payload(
            1,
            rule_name=_HAZARD_RULE_DEFS.get(1),
            supported=False,
            has_hazard=None,
            reason="usd_stage_unavailable",
            evidence={"active_diaolan_path": ap, "evaluation": "scene_state"},
        )
    r = evaluate_scene_rule_guardrails(stage, str(ap or ""))
    ev = _safe_plain_dict(r.get("evidence"))
    ev["scene_evaluation"] = "new_model_scene_state_not_camera"
    return _build_hazard_eval_payload(
        1,
        rule_name=_HAZARD_RULE_DEFS.get(1),
        supported=bool(r.get("supported")),
        has_hazard=r.get("has_hazard"),
        reason=str(r.get("reason") or ""),
        evidence=ev,
    )


def _evaluate_rule_safety_rope_per_worker_scene(context: dict) -> dict:
    """
    新模型适配：安全绳数量 vs 作业人数（场景态）。
    本版按「数量满足单人单绳」做场景态判定，不做人物-绳索一一绑定跟踪。
    """
    stage = _try_get_usd_stage_for_hazard()
    ap = context.get("active_diaolan_path")
    wc = context.get("worker_count")
    if stage is None:
        return _build_hazard_eval_payload(
            5,
            rule_name=_HAZARD_RULE_DEFS.get(5),
            supported=False,
            has_hazard=None,
            reason="usd_stage_unavailable",
            evidence={
                "active_diaolan_path": ap,
                "workers_count": wc,
                "evaluation": "scene_state",
            },
        )
    r = evaluate_scene_rule_safety_ropes(stage, str(ap or ""), wc)
    ev = _safe_plain_dict(r.get("evidence"))
    ev["worker_count_source"] = context.get("worker_count_source")
    ev["scene_evaluation"] = "new_model_scene_state_not_camera"
    return _build_hazard_eval_payload(
        5,
        rule_name=_HAZARD_RULE_DEFS.get(5),
        supported=bool(r.get("supported")),
        has_hazard=r.get("has_hazard"),
        reason=str(r.get("reason") or ""),
        evidence=ev,
    )


def _evaluate_rule_limitstop_scene(context: dict) -> dict:
    """新模型适配：钢丝绳分支下 Limitstop 场景态判定，非相机视觉。"""
    stage = _try_get_usd_stage_for_hazard()
    ap = context.get("active_diaolan_path")
    if stage is None:
        return _build_hazard_eval_payload(
            12,
            rule_name=_HAZARD_RULE_DEFS.get(12),
            supported=False,
            has_hazard=None,
            reason="usd_stage_unavailable",
            evidence={"active_diaolan_path": ap, "evaluation": "scene_state"},
        )
    r = evaluate_scene_rule_limitstops(stage, str(ap or ""))
    ev = _safe_plain_dict(r.get("evidence"))
    ev["scene_evaluation"] = "new_model_scene_state_not_camera"
    return _build_hazard_eval_payload(
        12,
        rule_name=_HAZARD_RULE_DEFS.get(12),
        supported=bool(r.get("supported")),
        has_hazard=r.get("has_hazard"),
        reason=str(r.get("reason") or ""),
        evidence=ev,
    )


def _evaluate_rule_not_lowered_to_ground(context: dict) -> dict:
    worker_count = context.get("worker_count")
    gondola_world_height = context.get("gondola_world_height")
    ground_threshold = float(context.get("ground_z_baseline", 0.0)) + float(
        context.get("ground_eps", 0.0)
    )
    evidence = {
        "worker_count": worker_count,
        "worker_count_source": context.get("worker_count_source"),
        "gondola_world_height": gondola_world_height,
        "gondola_world_height_source": context.get("gondola_world_height_source"),
        "gondola_height_cm": context.get("gondola_height_cm"),
        "ground_z_baseline": context.get("ground_z_baseline"),
        "ground_eps": context.get("ground_eps"),
        "ground_threshold": ground_threshold,
        "active_diaolan_path": context.get("active_diaolan_path"),
    }
    if worker_count is None or gondola_world_height is None:
        return _build_hazard_eval_payload(
            4,
            rule_name=_HAZARD_RULE_DEFS[4],
            supported=True,
            has_hazard=None,
            reason="当前缺少吊篮人数或吊篮高度状态，无法判断是否已降至地面",
            evidence=evidence,
        )
    if int(worker_count) != 0:
        return _build_hazard_eval_payload(
            4,
            rule_name=_HAZARD_RULE_DEFS[4],
            supported=True,
            has_hazard=False,
            reason=f"当前吊篮人数为{worker_count}，作业未结束，本规则不构成隐患",
            evidence=evidence,
        )
    has_hazard, _ = check_safety_hazard(
        float(gondola_world_height),
        int(worker_count),
        ground_z_baseline=float(context.get("ground_z_baseline", _HAZARD_GROUND_Z_BASELINE)),
        ground_eps=float(context.get("ground_eps", _HAZARD_GROUND_Z_EPS)),
    )
    reason = (
        f"当前吊篮人数为0，吊篮高度为{float(gondola_world_height):.4f}，高于地面阈值{ground_threshold:.4f}"
        if has_hazard
        else f"当前吊篮人数为0，吊篮高度为{float(gondola_world_height):.4f}，未高于地面阈值{ground_threshold:.4f}"
    )
    return _build_hazard_eval_payload(
        4,
        rule_name=_HAZARD_RULE_DEFS[4],
        supported=True,
        has_hazard=bool(has_hazard),
        reason=reason,
        evidence=evidence,
    )


def _evaluate_rule_fallarrestor_scene(context: dict) -> dict:
    ap = context.get("active_diaolan_path")
    if not ap:
        return _build_hazard_eval_payload(
            13,
            rule_name=_HAZARD_RULE_DEFS.get(13),
            supported=False,
            has_hazard=None,
            reason="active_diaolan_path_missing",
            evidence={"active_diaolan_path": ap, "evaluation": "scene_state"},
        )
    r = evaluate_scene_rule_fallarrestors(stage, str(ap or ""))
    return _build_hazard_eval_payload(
        13,
        rule_name=_HAZARD_RULE_DEFS.get(13),
        supported=bool(r.get("supported")),
        has_hazard=r.get("has_hazard"),
        reason=str(r.get("reason") or ""),
        evidence=r.get("evidence"),
    )


HAZARD_RULE_EVALUATORS = {
    1: _evaluate_rule_guardrails_scene,
    2: _evaluate_rule_worker_count_gt_two,
    3: _evaluate_rule_single_worker,
    4: _evaluate_rule_not_lowered_to_ground,
    5: _evaluate_rule_safety_rope_per_worker_scene,
    11: _evaluate_rule_11_overexposure_camera,
    12: _evaluate_rule_limitstop_scene,
    13: _evaluate_rule_fallarrestor_scene,
}


def _build_random_event_hazard_eval(result: dict, request_meta: dict | None) -> dict:
    context = _build_random_event_hazard_context(result, request_meta)
    rule_id = context.get("rule_id")
    base_evidence = {
        "rule_key": context.get("rule_key"),
        "raw_rule_value": context.get("raw_rule_value"),
        "random_event": context.get("random_event"),
        "active_diaolan_path": context.get("active_diaolan_path"),
    }
    if context.get("rule_key") and rule_id is None:
        hazard_eval = _build_hazard_eval_payload(
            None,
            rule_name=None,
            supported=False,
            has_hazard=None,
            reason="当前规则暂未接入判定",
            evidence={},
        )
    elif rule_id is None:
        hazard_eval = _build_hazard_eval_payload(
            None,
            rule_name=None,
            supported=False,
            has_hazard=None,
            reason="当前规则暂未接入判定",
            evidence={},
        )
    else:
        evaluator = HAZARD_RULE_EVALUATORS.get(rule_id)
        if evaluator is None:
            hazard_eval = _build_hazard_eval_payload(
                rule_id,
                rule_name=_HAZARD_RULE_DEFS.get(rule_id),
                supported=False,
                has_hazard=None,
                reason="当前规则暂未接入判定",
                evidence=base_evidence,
            )
        else:
            hazard_eval = evaluator(context)
    print(
        "[hazard-eval] "
        + json.dumps(
            {
                "rule_id": hazard_eval.get("rule_id"),
                "supported": hazard_eval.get("supported"),
                "has_hazard": hazard_eval.get("has_hazard"),
                "evidence": hazard_eval.get("evidence"),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return hazard_eval


def _safe_build_hazard_eval(result: dict, request_meta: dict | None) -> dict:
    """隐患判定为附加输出：失败时降级返回稳定结构，不得影响随机主接口。"""
    try:
        return _build_random_event_hazard_eval(result, request_meta)
    except Exception as exc:
        print(
            "[hazard-eval][degraded] "
            f"{type(exc).__name__}: {exc}\n{traceback.format_exc()}",
            flush=True,
        )
        return _build_hazard_eval_payload(
            None,
            rule_name=None,
            supported=False,
            has_hazard=None,
            reason="hazard_eval 生成失败，已降级（详见服务日志）",
            evidence={"error_type": type(exc).__name__},
        )


def _prim_world_height_axis(stage, prim_path: str) -> float | None:
    """指定 prim 世界平移在场景高度轴上的分量（与 _set_prim_translate_height / 探针同口径）。"""
    from pxr import Usd, UsdGeom

    if stage is None or not str(prim_path or "").strip():
        return None
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    hi = _height_axis_index()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return round(float(t[hi]), 6)


def _snapshot_diaolan_group1_world_heights_on_stage(stage) -> dict[str, float | None]:
    """运行态诊断：live stage 上各吊篮 group1 的世界「高度轴」分量（与 _height_axis_index 一致）。"""
    if stage is None:
        return {}
    out: dict[str, float | None] = {}
    for d in scan_diaolan_prims(stage):
        root = str(d.get("path") or "").strip()
        g1 = str(d.get("group1") or "").strip()
        if not root or not g1:
            continue
        out[root] = _prim_world_height_axis(stage, g1)
    return out


def _snapshot_gondola_suspend_world_evidence(stage) -> dict:
    """
    运行态验收：各吊篮篮体控制 Prim、装配体根（Diaolan_01）、Rope 根及其直接子节点
    在世界「高度轴」上的分量（与 _height_axis_index 一致）。

    说明：
    - 篮体写入目标是 group1（如 .../Diaolan），其世界高度应随滑条/HTTP 变化。
    - 装配体根（.../Diaolan_01）与兄弟 Rope 根的局部未由高度逻辑改写时，世界高度应保持不变。
    - 若绳体为骨骼/多段子 Prim，Rope 下直接子节点的 world 轴分量可能局部变化；Rope 根 xform 仍可作为「悬挂机构附着于装配体」的对照。
    """
    if stage is None:
        return {"ok": False, "error": "stage is None"}
    per: list[dict] = []
    for d in scan_diaolan_prims(stage):
        root = str(d.get("path") or "").strip()
        basket = str(d.get("group1") or "").strip()
        assembly = str(d.get("assembly") or "").strip()
        if not root or not basket or not assembly:
            continue
        rope_path = f"{assembly.rstrip('/')}/Rope"
        rope_prim = stage.GetPrimAtPath(rope_path)
        rope_children: list[dict] = []
        if rope_prim.IsValid():
            for ch in rope_prim.GetChildren():
                cp = ch.GetPath().pathString
                rope_children.append(
                    {
                        "path": cp,
                        "world_axis": _prim_world_height_axis(stage, cp),
                    }
                )
                if len(rope_children) >= 12:
                    break
        per.append(
            {
                "diaolan_root": root,
                "basket_body_prim": basket,
                "basket_body_world_axis": _prim_world_height_axis(stage, basket),
                "assembly_prim": assembly,
                "assembly_world_axis": _prim_world_height_axis(stage, assembly),
                "rope_root_prim": rope_path if rope_prim.IsValid() else None,
                "rope_root_world_axis": _prim_world_height_axis(stage, rope_path)
                if rope_prim.IsValid()
                else None,
                "rope_direct_children_world_axis": rope_children,
            }
        )
    with _scene_lock:
        sel = str(
            _scene_state.get("selected_diaolan_path")
            or _scene_state.get("active_diaolan_path")
            or ""
        ).strip()
    return {
        "ok": True,
        "height_axis": str(_scene_up_axis),
        "world_height_axis_index": _height_axis_index(),
        "selected_diaolan_path": sel or None,
        "gondola_control_prim": _GONDOLA_PRIM or None,
        "per_instance": per,
    }


def _prim_world_bbox_midpoint(stage, prim_path: str | None) -> tuple[float, float, float] | None:
    """当前 prim 世界对齐包围盒中心（用于对准真实吊篮/人员，而非固定配置点）。"""
    try:
        from pxr import Usd, UsdGeom

        ps = str(prim_path or "").strip()
        if not ps:
            return None
        prim = stage.GetPrimAtPath(ps)
        if not prim.IsValid():
            return None
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
        bound = cache.ComputeWorldBound(prim)
        rng = bound.ComputeAlignedRange()
        if rng.IsEmpty():
            return None
        mid = rng.GetMidpoint()
        return (float(mid[0]), float(mid[1]), float(mid[2]))
    except Exception:
        return None


def _effective_lookat_target_prim_path(stage) -> str | None:
    """随机相机/动态 look-at：硬目标优先当前 active 吊篮，禁止默认全局 Architecture_High 抢目标。"""
    global _GONDOLA_PRIM
    ordered: list[str] = []
    active_paths = _active_diaolan_runtime_paths(stage) if stage is not None else {}
    gp = str(active_paths.get("gondola_prim") or _GONDOLA_PRIM or "").strip()
    if gp:
        ordered.append(gp)
    lt = str(_LOOKAT_TARGET_PRIM_PATH or "").strip()
    default_arch = str(_DEFAULT_LOOKAT_TARGET_BUILDING_PRIM or "").strip()
    if lt and lt != default_arch and lt not in ordered:
        ordered.append(lt)
    cj = str(_CHANGJING_PRIM_PATH or "").strip()
    if not gp and cj and cj not in ordered:
        ordered.append(cj)
    for cfg_p in ordered:
        if cfg_p and _prim_world_bbox_midpoint(stage, cfg_p) is not None:
            return cfg_p
    return None


def _resolve_startup_lookat_target_xyz(stage, target_prim_path: str | None) -> tuple[float, float, float]:
    cfg_xyz = tuple(float(v) for v in _CAMERA_LOOKAT_TARGET_XYZ)
    try:
        active_paths = _active_diaolan_runtime_paths(stage) if stage is not None else {}
        gondola_mid = _prim_world_bbox_midpoint(stage, active_paths.get("gondola_prim"))
        building_context = _building_context_from_config_target(stage) if stage is not None else None
        building_path = str((building_context or {}).get("prim_path") or "").strip()
        building_mid = _prim_world_bbox_midpoint(stage, building_path)
        if gondola_mid is not None and building_mid is not None:
            target = (
                float(gondola_mid[0]) * 0.72 + float(building_mid[0]) * 0.28,
                float(gondola_mid[1]) * 0.72 + float(building_mid[1]) * 0.28,
                float(gondola_mid[2]) * 0.82 + float(building_mid[2]) * 0.18,
            )
            print(
                "[camera-startup-target] "
                f"source=active_gondola_building_blend gondola={active_paths.get('gondola_prim')!r} "
                f"building={building_path!r} target_xyz={tuple(round(float(v), 4) for v in target)}",
                flush=True,
            )
            return target
        if gondola_mid is not None:
            print(
                "[camera-startup-target] "
                f"source=active_gondola_bbox_mid prim={active_paths.get('gondola_prim')!r} "
                f"target_xyz={tuple(round(float(v), 4) for v in gondola_mid)}",
                flush=True,
            )
            return gondola_mid
    except Exception as exc:
        print(f"[camera-startup-target] WARN: active target resolve failed: {exc!r}", flush=True)
    mid = _prim_world_bbox_midpoint(stage, target_prim_path)
    if mid is not None:
        print(
            "[camera-startup-target] "
            f"source=prim_bbox_mid prim={target_prim_path!r} "
            f"target_xyz={tuple(round(float(v), 4) for v in mid)}",
            flush=True,
        )
        return mid
    print(
        "[camera-startup-target] "
        f"source=config_camera_lookat_target_xyz (prim_bbox_unresolved) "
        f"target_xyz={tuple(round(float(v), 4) for v in cfg_xyz)}",
        flush=True,
    )
    return cfg_xyz


def _sample_camera_from_config_box(stage, rng, seed: int):
    sample_box = cfg.get("diaolan_camera_sample_box")
    aabb = compute_changjing_aabb(stage)
    fallback = tuple(float(v) for v in _CAMERA_RIG_TRANSLATE_XYZ)

    if isinstance(sample_box, dict):
        x_min = float(sample_box.get("x_min", aabb["xmin"]))
        x_max = float(sample_box.get("x_max", aabb["xmax"]))
        y_min = float(sample_box.get("y_min", aabb["ymin"]))
        y_max = float(sample_box.get("y_max", aabb["ymax"]))
        z_min = float(sample_box.get("z_min", max(0.0, aabb["zmin"])))
        z_max = float(sample_box.get("z_max", max(3.0, aabb["zmax"])))
        source = "config_box"
    else:
        x_min, x_max = float(aabb["xmin"]), float(aabb["xmax"])
        y_min, y_max = float(aabb["ymin"]), float(aabb["ymax"])
        z_min = max(0.0, float(aabb["zmin"]))
        z_max = max(z_min + 1.0, float(aabb["zmax"]))
        source = "scene_aabb"

    if not (x_min < x_max and y_min < y_max and z_min < z_max):
        x, y, z = fallback
        meta = {
            "mode": "fallback_fixed_camera",
            "source": source,
            "seed": int(seed),
            "effective_box": {
                "x_min": x,
                "x_max": x,
                "y_min": y,
                "y_max": y,
                "z_min": z,
                "z_max": z,
            },
            "aabb_clip_notes": ["invalid_sample_box_fallback_to_current_camera"],
        }
        _set_camera_rig_translate_runtime(stage, [x, y, z])
        return x, y, z, meta

    x = round(rng.uniform(x_min, x_max), 2)
    y = round(rng.uniform(y_min, y_max), 2)
    z = round(rng.uniform(z_min, z_max), 2)
    _set_camera_rig_translate_runtime(stage, [x, y, z])
    meta = {
        "mode": "config_box_random",
        "source": source,
        "seed": int(seed),
        "effective_box": {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "z_min": z_min,
            "z_max": z_max,
        },
        "aabb_clip_notes": [],
    }
    return x, y, z, meta


def _apply_active_diaolan_switch(
    stage,
    active_diaolan: dict,
    all_diaolans: list,
    *,
    group1_source: str = "diaolan_randomizer_runtime",
) -> dict:
    """切换当前「选中」吊篮（控制目标）：所有吊篮保持可见，不再隐藏非选中实例。"""
    global _GONDOLA_PRIM, _WORKER1_PRIM, _WORKER2_PRIM, _last_gondola_init_group1_source

    apply_diaolan_visibility(stage, active_diaolan, all_diaolans)
    new_path = str(active_diaolan.get("path") or "").strip()
    with _scene_lock:
        prev_sel = str(
            _scene_state.get("selected_diaolan_path") or _scene_state.get("active_diaolan_path") or ""
        ).strip()
        exp_paths = [
            str(p).strip()
            for p in (_scene_experiment_state.get("active_hidden_paths") or [])
            if str(p).strip()
        ]
    # apply_diaolan_visibility 会对吊篮根 MakeVisible，从而抹掉 /scene/experiment 的隐藏；
    # 若本次未切换吊篮实例且仍有实验隐藏，则重放 hide_paths 以保持与 evaluator 一致。
    preserve_exp = bool(exp_paths) and bool(prev_sel) and prev_sel == new_path
    if preserve_exp:
        hide_result = _set_scene_experiment_hidden_paths(stage, exp_paths)
    else:
        _restore_scene_experiment_visibility(stage)
        hide_result = _set_scene_experiment_hidden_paths(stage, [])

    _GONDOLA_PRIM = active_diaolan["group1"]
    _WORKER1_PRIM = active_diaolan["persons"][0] if len(active_diaolan["persons"]) > 0 else ""
    _WORKER2_PRIM = active_diaolan["persons"][1] if len(active_diaolan["persons"]) > 1 else ""
    _last_gondola_init_group1_source = str(group1_source or "diaolan_randomizer_runtime")

    path = str(active_diaolan.get("path") or "")
    with _scene_lock:
        paths = [str(d.get("path") or "").strip() for d in all_diaolans if str(d.get("path") or "").strip()]
        _scene_state["all_diaolan_paths"] = paths
        _scene_state["active_diaolan_path"] = path
        _scene_state["selected_diaolan_path"] = path
    return hide_result


def _apply_per_diaolan_gondola_heights_for_randomize(
    stage,
    all_diaolans: list,
    *,
    random_gondola: bool,
    rng: random.Random,
    height_min: float,
    height_max: float,
    state_before: dict,
) -> tuple[dict[str, float], list[str], dict[str, str]]:
    """按吊篮根 path 对应 group1 写入或读取高度轴（stage 单位）；不切换选中吊篮。

    random_gondola 为 True 时对每台吊篮各自随机并 _set_prim_translate_height；
    为 False 时仅从 stage 读取当前高度写入映射，不改动 prim。
    """
    prev = state_before.get("gondola_heights")
    if not isinstance(prev, dict):
        prev = {}
    prev_paths = {str(k): v for k, v in prev.items()}
    active_hint = str(
        state_before.get("active_diaolan_path") or state_before.get("selected_diaolan_path") or ""
    ).strip()
    gondola_heights: dict[str, float] = {}
    group1_by_path: dict[str, str] = {}
    randomized_paths: list[str] = []

    for d in all_diaolans:
        root_path = str(d.get("path") or "").strip()
        g1 = str(d.get("group1") or "").strip()
        if not root_path or not g1:
            continue
        group1_by_path[root_path] = g1
        if random_gondola:
            h = round(rng.uniform(float(height_min), float(height_max)), 2)
            _set_prim_translate_height(stage, g1, h)
            gondola_heights[root_path] = h
            randomized_paths.append(root_path)
        else:
            cur = _prim_world_height_axis(stage, g1)
            if cur is None:
                fb = prev_paths.get(root_path)
                if fb is None and root_path == active_hint:
                    fb = state_before.get("gondola_y")
                h = float(fb if fb is not None else 0.0)
            else:
                h = float(cur)
            gondola_heights[root_path] = h

    return gondola_heights, randomized_paths, group1_by_path


def _service_stream_frame_during_randomize(
    annotator,
    *,
    stage_label: str,
    force_snapshot: bool = False,
    publish: bool = False,
    cache_snapshot: bool = False,
) -> dict:
    if annotator is None:
        return {"published": False, "healthy": False, "reason": "no_annotator"}
    rgba = None
    empty_replicator = True
    rtsp_source = "randomize_replicator"
    frame_health = None
    capture_epoch_s = None
    try:
        # 光照类 USD 写入后偶有一帧滞后；force_snapshot 时多 step 一次再抓图，避免 hazard 读到增亮前缓存
        if force_snapshot:
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
        rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
        capture_epoch_s = time.time()
        rgba = annotator.get_data()
        rgba = _normalize_replicator_rgba_for_output(rgba)
    except Exception as exc:
        print(
            f"[scene-randomize][frame-service] stage={stage_label} step/get_data failed: {exc}",
            flush=True,
        )
        rgba = None

    if rgba is not None and getattr(rgba, "size", 0) > 0:
        empty_replicator = False
        try:
            frame_health = _render_capture_rgb_frame_health(rgba)
        except Exception:
            frame_health = None
        if isinstance(frame_health, dict) and frame_health.get("should_try_viewport_fallback"):
            rep_frame_health = dict(frame_health)
            if _RANDOMIZE_FORCE_VIEWPORT_PRIMARY_ON_BLACK:
                _rtsp_set_viewport_primary(True, "randomize_replicator_black")
            try:
                vp, vp_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
            except Exception:
                vp = None
                vp_meta = {}
            if vp is not None and getattr(vp, "size", 0) > 0:
                vp_health = _render_capture_rgb_frame_health(vp)
                if bool(vp_health.get("healthy")):
                    rgba = vp
                    frame_health = vp_health
                    rtsp_source = "randomize_viewport_delegate"
                    try:
                        capture_epoch_s = float((vp_meta or {}).get("capture_epoch_s"))
                    except Exception:
                        capture_epoch_s = time.time()
                    print(
                        "[scene-randomize][frame-service] viewport fallback used "
                        f"stage={stage_label} rep_health={rep_frame_health}",
                        flush=True,
                    )
    else:
        empty_replicator = True
        rgba = None
        try:
            vp, vp_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
        except Exception:
            vp = None
            vp_meta = {}
        if vp is not None and getattr(vp, "size", 0) > 0:
            rgba = vp
            rtsp_source = "randomize_viewport_delegate"
            try:
                capture_epoch_s = float((vp_meta or {}).get("capture_epoch_s"))
            except Exception:
                capture_epoch_s = time.time()
        else:
            dec = _decode_last_good_jpeg_to_rgba_hw4()
            if dec is not None and getattr(dec, "size", 0) > 0:
                rgba = dec
                rtsp_source = "randomize_last_good"
            else:
                print(
                    f"[scene-randomize][frame-service] stage={stage_label} empty_frame=True",
                    flush=True,
                )
                _log_randomize_rtsp_keepalive(
                    phase=stage_label,
                    source=rtsp_source,
                    empty_frame=True,
                    queued_ok=False,
                )
                return {
                    "published": False,
                    "healthy": False,
                    "empty_frame": True,
                    "source": rtsp_source,
                    "reason": "empty_frame",
                    "rgba": None,
                }

    try:
        if rgba.shape[:2] != (H, W):
            rgba = np.ascontiguousarray(
                np.resize(rgba, (H, W, rgba.shape[2] if rgba.ndim == 3 else 4))
            )
        if not isinstance(frame_health, dict):
            frame_health = _render_capture_rgb_frame_health(rgba)
        if rtsp_source == "randomize_last_good":
            frame_health["source_is_last_good_fallback"] = True
        frame_healthy = bool(frame_health.get("healthy")) and rtsp_source != "randomize_last_good"
        _randomize_stream_guard_note_candidate(
            {
                "stage": stage_label,
                "source": rtsp_source,
                "healthy": bool(frame_healthy),
                "health": frame_health,
            }
        )

        if not frame_healthy:
            _randomize_stream_guard_block_black(
                stage_label,
                rtsp_source,
                frame_health,
                reason="candidate_unhealthy",
            )

        if frame_healthy and _jpeg_encode_fn is not None:
            need_snapshot = bool(cache_snapshot) or preview_enabled
            if not need_snapshot:
                now_snapshot = time.monotonic()
                with _mjpeg_lock:
                    last_snapshot_ts = float(_snapshot_cache["ts"])
                    snapshot_empty = _snapshot_cache["jpeg"] is None
                need_snapshot = snapshot_empty or (
                    now_snapshot - last_snapshot_ts >= snapshot_interval_s
                )
            if need_snapshot:
                is_black = False
                try:
                    rmax = int(np.max(rgba[:, :, :3]))
                    if rmax <= 2:
                        is_black = True
                except Exception:
                    pass
                if is_black:
                    print(f"[scene-randomize] skip caching black frame", flush=True)
                elif rtsp_source == "randomize_last_good":
                    print(
                        "[scene-randomize] skip caching last_good fallback frame "
                        f"stage={stage_label}",
                        flush=True,
                    )
                elif force_snapshot and not frame_healthy:
                    print(
                        "[scene-randomize] skip caching unstable frame "
                        f"stage={stage_label} health={frame_health}",
                        flush=True,
                    )
                else:
                    jpg = _jpeg_encode_fn(rgba)
                    if jpg:
                        _cache_good_snapshot_jpeg(
                            jpg,
                            capture_seq=None,
                            mirror_mjpeg=preview_enabled,
                        )

        ffmpeg_alive = bool(rtsp_enabled and _ffmpeg_proc is not None and _ffmpeg_proc.poll() is None)
        queue_status = "skipped"
        queued_ok_rtsp = False
        if publish and frame_healthy and ffmpeg_alive:
            dropped, _rtsp_meta = _randomize_stream_guard_commit(
                rgba,
                rtsp_source.replace("randomize_", ""),
                capture_epoch_s=capture_epoch_s,
            )
            queue_status = "queued_replaced" if dropped else "queued"
            queued_ok_rtsp = True
        else:
            queued_ok_rtsp = False

        if empty_replicator:
            _log_randomize_rtsp_keepalive(
                phase=stage_label,
                source=rtsp_source,
                empty_frame=True,
                queued_ok=queued_ok_rtsp,
            )

        _empty_note = (
            f" empty_frame=True keepalive_source={rtsp_source}" if empty_replicator else ""
        )
        print(
            f"[scene-randomize][frame-service] stage={stage_label}{_empty_note} "
            f"ffmpeg_alive={ffmpeg_alive} queue={queue_status} healthy={frame_healthy}",
            flush=True,
        )
        return {
            "published": bool(publish and frame_healthy and queued_ok_rtsp),
            "healthy": frame_healthy,
            "empty_frame": bool(empty_replicator),
            "source": rtsp_source,
            "ffmpeg_alive": bool(ffmpeg_alive),
            "queue": queue_status,
            "health": frame_health,
            "rgba": rgba if frame_healthy else None,
            "capture_epoch_s": capture_epoch_s,
        }
    except Exception as exc:
        print(
            f"[scene-randomize][frame-service] stage={stage_label} publish failed: {exc}",
            flush=True,
        )
        return {"published": False, "healthy": False, "reason": f"publish_failed:{exc}"}


def _randomize_force_recover_render_capture(world, rp, annotator, *, reason: str) -> tuple[bool, object, object, str | None]:
    global _render_recover_in_progress, _last_render_recover_attempt_mono
    try:
        _rtsp_set_viewport_primary(True, f"randomize_force_recover:{reason}")
    except Exception:
        pass
    try:
        _enforce_renderer_mode(f"randomize_force_recover:{reason}")
    except Exception as exc:
        print(f"[scene-randomize][recover] renderer_enforce_failed: {exc}", flush=True)
    with _render_recover_state_lock:
        prev_attempt = float(_last_render_recover_attempt_mono or 0.0)
        _last_render_recover_attempt_mono = 0.0
    try:
        ok, rp2, annotator2, err = _recover_render_capture(
            world,
            rp,
            annotator,
            reason=f"randomize:{reason}",
            camera_prim_path=camera_prim,
            width=W,
            height=H,
        )
        return bool(ok), rp2, annotator2, err
    finally:
        if not bool(_render_recover_in_progress):
            with _render_recover_state_lock:
                if _last_render_recover_attempt_mono <= 0.0 and prev_attempt > 0.0:
                    _last_render_recover_attempt_mono = prev_attempt


def _settle_render_after_randomize(world, rp, annotator, *, stage_label: str = "final") -> tuple[dict, object, object]:
    if annotator is None:
        return {"enabled": False, "reason": "no_annotator", "render_commit_status": "failed"}, rp, annotator
    if world is None:
        return {"enabled": False, "reason": "no_world", "render_commit_status": "failed"}, rp, annotator
    min_good = int(_RANDOMIZE_STABLE_MIN_GOOD_FRAMES)
    max_wait_s = float(_RANDOMIZE_STABLE_MAX_WAIT_S)
    interval_s = float(_RANDOMIZE_CANDIDATE_INTERVAL_MS) / 1000.0
    started = time.monotonic()
    good_frames = 0
    attempts: list[dict] = []
    black_streak = 0
    recovery_attempted = False
    recovery_errors: list[str] = []
    last_good_info: dict | None = None
    while (time.monotonic() - started) < max_wait_s:
        info = _service_stream_frame_during_randomize(
            annotator,
            stage_label=f"{stage_label}_settle_{len(attempts) + 1}",
            force_snapshot=True,
            publish=False,
            cache_snapshot=False,
        )
        if bool(info.get("healthy")):
            good_frames += 1
            black_streak = 0
            last_good_info = info
        else:
            good_frames = 0
            black_streak += 1
        attempts.append(
            {
                "index": len(attempts) + 1,
                "healthy": bool(info.get("healthy")),
                "published": bool(info.get("published")),
                "source": info.get("source"),
                "reason": info.get("reason"),
            }
        )
        if good_frames >= min_good and isinstance(last_good_info, dict):
            rgba = last_good_info.get("rgba")
            if rgba is not None and getattr(rgba, "size", 0) > 0:
                dropped, _meta = _randomize_stream_guard_commit(
                    rgba,
                    str(last_good_info.get("source") or "candidate").replace("randomize_", ""),
                    capture_epoch_s=last_good_info.get("capture_epoch_s"),
                )
                commit_source = str(_meta.get("randomize_mode") or "committed")
                _cache_randomize_commit_snapshot(rgba)
                return (
                    {
                        "enabled": True,
                        "attempts": len(attempts),
                        "good_frames": int(good_frames),
                        "min_good_frames": int(min_good),
                        "elapsed_s": round(float(time.monotonic() - started), 3),
                        "stable": True,
                        "render_commit_status": "committed",
                        "commit_source": f"randomize_commit:{str(last_good_info.get('source') or 'candidate').replace('randomize_', '')}",
                        "commit_dropped_old": bool(dropped),
                        "recovery_attempted": bool(recovery_attempted),
                        "recovery_errors": recovery_errors[-4:],
                        "attempt_sample": attempts[-8:],
                    },
                    rp,
                    annotator,
                )
        if black_streak >= 2 and not recovery_attempted:
            recovery_attempted = True
            ok, rp, annotator, err = _randomize_force_recover_render_capture(
                world,
                rp,
                annotator,
                reason=f"{stage_label}_black_streak_{black_streak}",
            )
            if not ok and err:
                recovery_errors.append(str(err))
        if interval_s > 0:
            time.sleep(interval_s)
    return (
        {
            "enabled": True,
            "attempts": len(attempts),
            "good_frames": int(good_frames),
            "min_good_frames": int(min_good),
            "elapsed_s": round(float(time.monotonic() - started), 3),
            "stable": False,
            "render_commit_status": "failed",
            "reason": "render_not_stable_after_randomize",
            "recovery_attempted": bool(recovery_attempted),
            "recovery_errors": recovery_errors[-4:],
            "attempt_sample": attempts[-8:],
        },
        rp,
        annotator,
    )


def _cache_randomize_commit_snapshot(rgba) -> None:
    if _jpeg_encode_fn is None or rgba is None or getattr(rgba, "size", 0) <= 0:
        return
    try:
        jpg = _jpeg_encode_fn(rgba)
        if jpg:
            _cache_good_snapshot_jpeg(
                jpg,
                capture_seq=None,
                mirror_mjpeg=preview_enabled,
            )
    except Exception as exc:
        print(f"[scene-randomize] commit snapshot cache failed: {exc}", flush=True)


def _ptz_gondola_projection_coverage_ok(
    projection_metrics: dict | None, min_area_ratio: float
) -> tuple[bool | None, str, float]:
    """与 scene_perception 篮体可观测条件对齐，供随机后自动对准判断是否达标。"""
    if not isinstance(projection_metrics, dict):
        return None, "缺少篮体 target_projection_metrics", 0.0
    if "error" in projection_metrics:
        return None, f"篮体 USD 投影失败：{projection_metrics.get('error')}", 0.0
    if not projection_metrics.get("frustum内可见"):
        return False, "篮体世界包围盒与当前相机视锥不相交", 0.0
    if not projection_metrics.get("中心在画面内"):
        return False, "篮体投影中心不在成像矩形内", 0.0
    try:
        ar = float(projection_metrics.get("目标像素占比") or 0.0)
    except (TypeError, ValueError):
        ar = 0.0
    try:
        onr = float(projection_metrics.get("画面内交集像素占比") or 0.0)
    except (TypeError, ValueError):
        onr = 0.0
    eff = max(ar, onr)
    if eff < float(min_area_ratio):
        return (
            False,
            f"篮体在画面内有效占比 {eff:.6f} < 阈值 {min_area_ratio}，覆盖不足",
            eff,
        )
    return True, "篮体视锥命中且画面占比满足阈值", eff


def _blend_lookat_xyz_for_perception_rule(
    stage,
    *,
    rule_id: int,
    gondola_prim: str,
    visible_worker_paths: list[str],
    active_diaolan_path: str,
) -> tuple[float, float, float]:
    """规则 2/3 在篮体与可见作业人员包围盒中心之间混合 look-at，利于整篮+人员同屏。"""
    g = _prim_world_bbox_midpoint(stage, gondola_prim)
    if g is None:
        return tuple(float(v) for v in _CAMERA_LOOKAT_TARGET_XYZ)
    if rule_id not in (2, 3):
        return g
    ap = str(active_diaolan_path or "").strip()
    prefix = ap.rstrip("/") + "/" if ap else ""
    mids: list[tuple[float, float, float]] = []
    for wp in visible_worker_paths or []:
        ws = str(wp).strip()
        if not ws or not ap:
            continue
        if ws == ap or ws.startswith(prefix):
            m = _prim_world_bbox_midpoint(stage, ws)
            if m:
                mids.append(m)
    if not mids:
        return g
    ax = sum(t[0] for t in mids) / len(mids)
    ay = sum(t[1] for t in mids) / len(mids)
    az = sum(t[2] for t in mids) / len(mids)
    return (0.5 * g[0] + 0.5 * ax, 0.5 * g[1] + 0.5 * ay, 0.5 * g[2] + 0.5 * az)


def _randomize_run_perception_camera_refine(
    stage,
    request_meta: dict | None,
    annotator,
    *,
    visible_worker_paths: list[str],
    active_diaolan_path: str,
) -> dict:
    """
    随机场景落位后：对 camera_perception 已支持规则做有限次 zoom + look-at 补救，
    再刷新投影；与 legacy 规则真值无关。
    """
    rid, _, _ = _extract_requested_hazard_rule(request_meta)
    if rid is None or int(rid) not in CAMERA_PERCEPTION_SUPPORTED_RULE_IDS:
        return {"skipped": True, "reason": "rule_not_camera_perception_supported"}

    if not _GONDOLA_PRIM:
        return {"skipped": True, "reason": "no_gondola_prim"}

    pmig = cfg.get("perception_migration")
    min_g = 0.001
    if isinstance(pmig, dict):
        try:
            v = float(pmig.get("min_target_area_ratio", min_g))
            if v > 0:
                min_g = v
        except (TypeError, ValueError):
            pass

    with _ptz_lock:
        zoom_orig = float(_ptz_state["zoom"])

    z_candidates = [
        1.0,
        min(zoom_orig, 1.08),
        zoom_orig,
        1.18,
        1.32,
        1.5,
    ]
    zoom_schedule: list[float] = []
    seen_z: set[float] = set()
    for z in z_candidates:
        z = float(max(0.88, min(2.85, z)))
        z = round(z, 4)
        if z not in seen_z:
            seen_z.add(z)
            zoom_schedule.append(z)

    attempts: list[dict] = []
    best_attempt = 0
    covered = False

    for i, z_try in enumerate(zoom_schedule):
        with _ptz_lock:
            _ptz_state["zoom"] = float(z_try)
        _apply_ptz_state(stage)

        tgt = _blend_lookat_xyz_for_perception_rule(
            stage,
            rule_id=int(rid),
            gondola_prim=str(_GONDOLA_PRIM),
            visible_worker_paths=list(visible_worker_paths or []),
            active_diaolan_path=str(active_diaolan_path or ""),
        )
        cam_xyz = _get_world_translation(stage, camera_prim)
        if cam_xyz is None:
            cam_xyz = _get_world_translation(stage, _CAMERA_RIG_PRIM)
        if cam_xyz is None:
            cam_xyz = tuple(float(v) for v in _CAMERA_RIG_TRANSLATE_XYZ)
        _apply_dynamic_lookat_after_random_camera(
            stage,
            cam_xyz,
            source="randomize_perception_refine",
            preset_name="default_initial",
            target_xyz=tgt,
            tilt_max_deg=55.0,
        )
        _service_stream_frame_during_randomize(
            annotator, stage_label=f"perception_align_attempt_{i + 1}"
        )

        pm = _prim_projection_metrics(stage, camera_prim, _GONDOLA_PRIM)
        cv = _compute_camera_view_perception(stage)
        g_obs, g_note, eff = _ptz_gondola_projection_coverage_ok(pm, min_g)
        n_ren = int((cv or {}).get("rendered_worker_paths_under_active_count") or 0)
        all_seen = bool((cv or {}).get("workers_all_projected_in_view"))

        attempts.append(
            {
                "attempt_index": i + 1,
                "zoom": z_try,
                "look_at_target_xyz": [round(t, 4) for t in tgt],
                "gondola_observable": g_obs,
                "gondola_observable_reason": g_note,
                "effective_area_ratio": round(eff, 6),
                "min_target_area_ratio": min_g,
                "rendered_workers_under_active": n_ren,
                "workers_all_projected_in_view": all_seen,
            }
        )

        rid_i = int(rid)
        done = False
        if g_obs is True:
            if rid_i == 4:
                if n_ren == 0 or all_seen:
                    done = True
            elif rid_i in (2, 3):
                if n_ren == 0 or all_seen:
                    done = True
        if done:
            best_attempt = i + 1
            covered = True
            break

    return {
        "skipped": False,
        "rule_id": int(rid),
        "gondola_prim_path": str(_GONDOLA_PRIM),
        "min_target_area_ratio": min_g,
        "attempts_total": len(attempts),
        "stopped_after_attempt": best_attempt if covered else len(attempts),
        "coverage_goal_reached": covered,
        "attempts": attempts,
        "note": "随机完成后对当前吊篮/人员的自动对准与 zoom 补救；仍 inconclusive 时表示补救后仍不足最小可见条件。",
    }


def _randomize_scene_runtime(request_meta: dict | None = None, annotator=None, world_obj=None, rp_obj=None) -> tuple[dict, object, object]:
    global _GONDOLA_PRIM, _WORKER1_PRIM, _WORKER2_PRIM, _CAMERA_RIG_TRANSLATE_XYZ
    global _last_gondola_init_group1_source

    randomize_started = time.monotonic()
    guard_info = _randomize_stream_guard_begin()
    timing = {}
    last_timing_mark = randomize_started

    def _mark_timing(name: str) -> None:
        nonlocal last_timing_mark
        now = time.monotonic()
        timing[name] = round(float(now - last_timing_mark), 4)
        last_timing_mark = now

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("USD stage not ready")

    rng = random.Random()
    state_before = _scene_state_runtime_snapshot()
    random_cfg = _runtime_random_config_from_request(state_before["random_config"], request_meta)
    all_diaolans = scan_diaolan_prims(stage)
    if not all_diaolans:
        raise RuntimeError("no diaolan prims found")
    _mark_timing("init_scan")

    path_set = _diaolan_path_set(all_diaolans)
    forced_from_req = str(
        (request_meta.get("active_diaolan_path") if isinstance(request_meta, dict) else "")
        or state_before.get("pending_active_diaolan_path")
        or ""
    ).strip()
    control_path = forced_from_req if forced_from_req in path_set else ""
    if not control_path:
        control_path = str(state_before.get("selected_diaolan_path") or state_before.get("active_diaolan_path") or "").strip()
    if control_path not in path_set:
        control_path = ""
    if not control_path:
        control_path = str(all_diaolans[0].get("path") or "").strip()
    active_diaolan = pick_active_diaolan(all_diaolans, rng, forced_path=control_path)
    hide_result = _apply_active_diaolan_switch(
        stage, active_diaolan, all_diaolans, group1_source="diaolan_randomizer_runtime"
    )
    with _scene_lock:
        _scene_state["pending_active_diaolan_path"] = ""
    _service_stream_frame_during_randomize(annotator, stage_label="after_diaolan_switch")
    _mark_timing("diaolan_switch")

    aabb = compute_changjing_aabb(stage)
    dynamic_height_max = min(_GONDOLA_HEIGHT_MAX, aabb["zmax"]) if aabb["zmax"] > 5.0 else 36.0
    gondola_heights, gondola_height_randomized_paths, gondola_group1_by_diaolan = (
        _apply_per_diaolan_gondola_heights_for_randomize(
            stage,
            all_diaolans,
            random_gondola=bool(random_cfg["random_gondola"]),
            rng=rng,
            height_min=_GONDOLA_HEIGHT_MIN,
            height_max=float(dynamic_height_max),
            state_before=state_before,
        )
    )
    ap_sel = str(active_diaolan.get("path") or "").strip()
    gondola_y = float(gondola_heights.get(ap_sel, state_before["gondola_y"]))

    prev_by = _normalize_workers_by_diaolan_raw(state_before.get("workers_visible_count_by_diaolan_path"))
    workers_by: dict[str, int] = {}
    for d in all_diaolans:
        rp = str(d.get("path") or "").strip()
        persons = dedupe_worker_prim_paths_ordered([str(p) for p in (d.get("persons") or []) if str(p).strip()])
        n = len(persons)
        if random_cfg["random_workers"]:
            workers_by[rp] = rng.randint(0, n) if n else 0
        else:
            c = prev_by.get(rp)
            if c is None:
                ap0 = str(state_before.get("active_diaolan_path") or "").strip()
                if rp == ap0:
                    c = int(state_before.get("workers", 0))
                else:
                    c = 0
            workers_by[rp] = max(0, min(n, int(c)))

    with _scene_lock:
        _scene_state["workers_visible_count_by_diaolan_path"] = dict(workers_by)
        _scene_state["gondola_y"] = gondola_y
        _scene_state["gondola_heights"] = dict(gondola_heights)

    _sync_worker_scalar_fields_for_control_diaolan(all_diaolans, ap_sel)
    _apply_scene_state(stage)
    _mark_timing("scene_apply")

    for d in all_diaolans:
        rp = str(d.get("path") or "").strip()
        g1p = str(d.get("group1") or "").strip()
        if not rp or not g1p:
            continue
        w_h = _prim_world_height_axis(stage, g1p)
        if w_h is not None:
            gondola_heights[rp] = float(w_h)
    ap_sync = str(active_diaolan.get("path") or "").strip()
    with _scene_lock:
        if ap_sync and ap_sync in gondola_heights:
            _scene_state["gondola_y"] = float(gondola_heights[ap_sync])
        _scene_state["gondola_heights"] = dict(gondola_heights)

    for d in all_diaolans:
        sync_workers_to_group1(stage, d["group1"], d.get("persons") or [], 0)
    _service_stream_frame_during_randomize(annotator, stage_label="after_scene_apply")

    safety_rand_pack = None
    if (
        random_cfg.get("random_guardrail")
        or random_cfg.get("random_safety_rope")
        or random_cfg.get("random_limitstop")
        or random_cfg.get("random_fallarrestor")
    ):
        wc_for_safety = int(workers_by.get(ap_sel, 0))
        
        # 对于 fallarrestor 的 weighted_random，在这里根据概率动态决定 compliant 还是 non_compliant
        fm = str(random_cfg.get("fallarrestor_mode") or "random")
        if fm == "weighted_random":
            fp = float(random_cfg.get("fallarrestor_noncompliant_probability", 0.5))
            if rng.random() < fp:
                fm_effective = "non_compliant"
            else:
                fm_effective = "compliant"
        else:
            fm_effective = fm

        safety_rand_pack = randomize_active_diaolan_safety_components(
            stage,
            ap_sel,
            wc_for_safety,
            rng,
            random_guardrail=bool(random_cfg.get("random_guardrail")),
            random_safety_rope=bool(random_cfg.get("random_safety_rope")),
            random_limitstop=bool(random_cfg.get("random_limitstop")),
            random_fallarrestor=bool(random_cfg.get("random_fallarrestor")),
            guardrail_mode=str(random_cfg.get("guardrail_mode") or "random"),
            safety_rope_mode=str(random_cfg.get("safety_rope_mode") or "random"),
            limitstop_mode=str(random_cfg.get("limitstop_mode") or "random"),
            fallarrestor_mode=fm_effective,
        )

    hdri_apply_result = None
    if random_cfg.get("random_hdri") and _environment_allows_hdri():
        hdri_candidates = _sanitize_hdri_candidates(
            random_cfg.get("hdri_candidates"),
            fallback=_default_hdri_candidates(),
        )
        existing_hdri_candidates = [path for path in hdri_candidates if os.path.isfile(path)]
        missing_hdri_candidates = [path for path in hdri_candidates if path not in existing_hdri_candidates]
        if not existing_hdri_candidates:
            raise RuntimeError(
                "no valid HDRI candidates found; candidates=" + json.dumps(hdri_candidates, ensure_ascii=False)
            )
        hdri_choice = rng.choice(existing_hdri_candidates)
        hdri_apply_result = _apply_hdri_texture(stage, hdri_choice)
        hdri_apply_result["hdri_candidates"] = hdri_candidates
        hdri_apply_result["hdri_candidates_existing"] = existing_hdri_candidates
        hdri_apply_result["hdri_candidates_missing"] = missing_hdri_candidates
        actual_group_id, actual_entry = _match_hdri_entry_by_path(hdri_apply_result.get("current_hdri"))
        with _scene_lock:
            control = _sanitize_hdri_control_state(_scene_state.get("hdri_control"))
            if actual_group_id and isinstance(actual_entry, dict):
                control["current_group_id"] = actual_group_id
                selected_by_group = dict(control.get("selected_by_group") or {})
                selected_by_group[actual_group_id] = actual_entry.get("id")
                control["selected_by_group"] = selected_by_group
            control["last_switch_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
            control["last_apply_ok"] = True
            control["last_apply_message"] = f"随机事件已切换 HDRI: {hdri_apply_result.get('current_hdri_basename') or hdri_apply_result.get('current_hdri') or '-'}"
            control["last_action"] = "random_hdri"
            _scene_state["hdri_control"] = control
        _refresh_hdri_backend_status_from_stage(stage, control_state=control, updated_at=control.get("last_switch_time"))
        _service_stream_frame_during_randomize(annotator, stage_label="after_hdri_apply")
    _mark_timing("safety_hdri")

    scene_after = _scene_state_runtime_snapshot(lock_timeout=0.05)
    scene_after.setdefault("gondola_y", float(gondola_y))
    scene_after["gondola_heights"] = dict(gondola_heights)
    scene_after["workers_visible_count_by_diaolan_path"] = dict(workers_by)
    scene_after["all_diaolan_paths"] = [str(d.get("path") or "").strip() for d in all_diaolans if str(d.get("path") or "").strip()]
    scene_after["gondola_height_cm"] = _stage_units_to_cm(stage, float(gondola_y))
    scene_after.setdefault("height_debug", {})
    scene_after.setdefault("all_worker_paths", [])
    scene_after.setdefault("visible_worker_paths", [])
    scene_after.setdefault("gondola_renderable_paths", [])
    scene_after.setdefault("gondola_visible_renderable_paths", [])
    scene_after.setdefault("gondola_hidden_paths", [])
    scene_after.setdefault("gondola_renderable_debug", [])
    hdri_state = _describe_hdri_state(stage, random_cfg)
    _mark_timing("runtime_state")

    target_prim_for_camera = _effective_lookat_target_prim_path(stage)
    target_xyz = _resolve_startup_lookat_target_xyz(stage, target_prim_for_camera)
    wall_target_context = _wall_sampling_target_context(stage, active_diaolan)
    _mark_timing("wall_context")
    camera_meta = {
        "mode": "unchanged",
        "source": "keep_current_camera",
    }
    preset_vis = {}
    preset_vis_detail = {}
    startup_visible = None

    previous_camera_xyz = _get_translate_tuple(stage, _CAMERA_RIG_PRIM) or tuple(float(v) for v in _CAMERA_RIG_TRANSLATE_XYZ)
    if random_cfg["random_camera"]:
        max_cam_retries = 20 if random_cfg["keep_target_visible"] else 1
        cam_x = cam_y = cam_z = 0.0
        last_seed = None
        last_attempt = 0
        for attempt in range(max_cam_retries):
            seed = rng.randint(0, 1000000)
            if random_cfg["keep_wall_install_constraint"]:
                cam_x, cam_y, cam_z, _, camera_meta = sample_camera_in_changjing(
                    stage,
                    _CHANGJING_PRIM_PATH,
                    _CAMERA_RIG_PRIM,
                    rng,
                    seed,
                    sample_box=cfg.get("diaolan_camera_sample_box") if isinstance(cfg.get("diaolan_camera_sample_box"), dict) else None,
                    wall_prim_path=_CAMERA_WALL_CONSTRAINT_PRIM,
                    wall_constraint_xy_margin=_CAMERA_WALL_CONSTRAINT_XY_MARGIN,
                    wall_constraint_z_margin=_CAMERA_WALL_CONSTRAINT_Z_MARGIN,
                    wall_mount_inset_m=_CAMERA_WALL_MOUNT_INSET_M,
                    wall_mount_inset_mode=_CAMERA_WALL_MOUNT_INSET_MODE,
                    target_world_xyz=target_xyz,
                    wall_collection_mode=_WALL_COLLECTION_MODE,
                    wall_collection_root_path=_WALL_COLLECTION_ROOT_PATH,
                    target_context_bbox=wall_target_context.get("bbox"),
                    wall_candidate_region=_WALL_CANDIDATE_REGION,
                )
            else:
                cam_x, cam_y, cam_z, camera_meta = _sample_camera_from_config_box(stage, rng, seed)

            last_seed = seed
            last_attempt = attempt + 1
            _CAMERA_RIG_TRANSLATE_XYZ = (cam_x, cam_y, cam_z)

            if random_cfg["auto_look_at_target"]:
                orient = _apply_dynamic_lookat_after_random_camera(
                    stage,
                    (cam_x, cam_y, cam_z),
                    source="random_scene_api",
                    preset_name="default_initial",
                    target_xyz=target_xyz,
                )
                if orient is not None and not random_cfg["keep_target_visible"]:
                    _refine_committed_orientation_for_context_visibility(
                        stage,
                        camera_world_xyz=(cam_x, cam_y, cam_z),
                        target_xyz=target_xyz,
                        visibility_detail={},
                        base_pan=orient[0],
                        base_tilt=orient[1],
                        source="random_scene_api",
                        preset_name="default_initial",
                        tilt_max_deg=30.0,
                    )
            _service_stream_frame_during_randomize(
                annotator,
                stage_label=f"camera_attempt_{attempt + 1}",
            )

            if not random_cfg["keep_target_visible"]:
                startup_visible = None
                break

            preset_vis, preset_vis_detail = check_preset_visibility(
                stage,
                _CAMERA_RIG_PRIM,
                cfg.get("presets", {}),
                _effective_lookat_target_prim_path(stage) or _GONDOLA_PRIM,
                orientation_mode=_CAMERA_ORIENTATION_MODE,
                lookat_target_xyz=target_xyz,
                resolution_wh=(W, H),
                return_details=True,
                prefer_target_prim_center=False,
                dynamic_startup_pan_offset_deg=_DYNAMIC_STARTUP_PAN_OFFSET_DEG,
                dynamic_startup_tilt_offset_deg=_DYNAMIC_STARTUP_TILT_OFFSET_DEG,
            )
            startup_visible = bool(preset_vis.get("__startup_default__", False))
            if startup_visible:
                _commit_visibility_checked_orientation(
                    stage,
                    (cam_x, cam_y, cam_z),
                    target_xyz,
                    preset_vis_detail.get("__startup_default__", {}),
                    source="random_scene_api_visibility_commit",
                    preset_name="default_initial",
                )
                break

        camera_meta = dict(camera_meta or {})
        camera_meta["seed"] = last_seed
        camera_meta["attempts"] = last_attempt
        _mark_timing("camera_sampling")
    else:
        current_xyz = _get_translate_tuple(stage, _CAMERA_RIG_PRIM) or tuple(float(v) for v in _CAMERA_RIG_TRANSLATE_XYZ)
        if random_cfg["auto_look_at_target"]:
            orient = _apply_dynamic_lookat_after_random_camera(
                stage,
                current_xyz,
                source="random_scene_api_keep_camera",
                preset_name="default_initial",
                target_xyz=target_xyz,
            )
            if orient is not None:
                _refine_committed_orientation_for_context_visibility(
                    stage,
                    camera_world_xyz=current_xyz,
                    target_xyz=target_xyz,
                    visibility_detail={},
                    base_pan=orient[0],
                    base_tilt=orient[1],
                    source="random_scene_api_keep_camera",
                    preset_name="default_initial",
                    tilt_max_deg=30.0,
                )
            _service_stream_frame_during_randomize(
                annotator,
                stage_label="after_keep_camera_reorient",
            )
        _mark_timing("camera_sampling")

    perception_alignment_diag = _randomize_run_perception_camera_refine(
        stage,
        request_meta,
        annotator,
        visible_worker_paths=list(scene_after.get("visible_worker_paths") or []),
        active_diaolan_path=str(active_diaolan.get("path") or ""),
    )
    _refresh_projection_metrics(stage)
    _mark_timing("perception_refine")
    effective_request_meta: dict = dict(request_meta) if isinstance(request_meta, dict) else {}
    rid_pre = _coerce_rule_id(effective_request_meta.get("rule_id"))
    if rid_pre is None:
        rid_pre = _coerce_rule_id(effective_request_meta.get("event_id"))
    # 随机 rule11 分支会把 effective_request_meta["rule_id"] 置 11；此标志仅表示「请求体显式 rule/event 11」
    rule11_explicit_request = rid_pre == 11
    rule11_camera_exposure_adjustment = None
    rule11_lighting_adjustment = None
    rule11_random_draw = False
    rule11_lighting_applied = False
    if rid_pre != 11 and bool(random_cfg.get("random_overexposure_event")):
        try:
            rp = float(random_cfg.get("random_overexposure_event_probability") or 0.2)
        except (TypeError, ValueError):
            rp = 0.2
        rp = max(0.0, min(1.0, rp))
        if rng.random() < rp:
            rule11_random_draw = True
    if rid_pre == 11 or rule11_random_draw:
        if _environment_allows_hdri():
            if rid_pre == 11:
                rule11_lighting_adjustment = _rule11_apply_request_fixed_overexposure_dome_lights(stage)
            else:
                try:
                    base_delta = float(random_cfg.get("rule11_overexposure_exposure_delta") or 30.0)
                except (TypeError, ValueError):
                    base_delta = 30.0
                base_delta = max(0.5, min(48.0, base_delta))
                delta = base_delta + rng.uniform(0.5, 3.0)
                rule11_lighting_adjustment = _rule11_boost_environment_exposure(stage, delta)
        else:
            rule11_lighting_adjustment = {
                "applied": False,
                "skipped": True,
                "reason": "environment_not_hdri_or_blocked",
                "mode": "request_rule11_fixed_abs" if rid_pre == 11 else "random_rule11_delta",
            }
        effective_request_meta["rule_id"] = 11
        rule11_lighting_applied = True
        if rule11_explicit_request:
            rule11_camera_exposure_adjustment = _rule11_camera_chain_enter_manual_overexposure(
                stage, str(camera_prim)
            )
        _service_stream_frame_during_randomize(
            annotator, stage_label="after_rule11_lighting", force_snapshot=True
        )
    render_stabilization, rp_obj, annotator = _settle_render_after_randomize(
        world_obj,
        rp_obj,
        annotator,
        stage_label="before_result_snapshot",
    )
    _mark_timing("render_settle")
    if not bool(render_stabilization.get("stable")):
        _fail_diag = _randomize_stream_guard_diag_snapshot()
        _randomize_stream_guard_finish(mode="frozen_last_good")
        stable_wait = render_stabilization.get("elapsed_s")
        blocked_total = int(_fail_diag.get("randomize_black_frames_blocked_total") or 0)
        err = RuntimeError("render_not_stable_after_randomize")
        setattr(
            err,
            "randomize_response_fields",
            {
                "render_commit_status": render_stabilization.get("render_commit_status") or "failed",
                "render_stabilization": render_stabilization,
                "randomize_stream_freeze_used": bool(_fail_diag.get("randomize_freeze_active")),
                "randomize_stream_commit_source": _fail_diag.get("randomize_last_commit_source"),
                "randomize_stream_black_frames_blocked": max(
                    0,
                    blocked_total - int(guard_info.get("black_frames_blocked_start") or 0),
                ),
                "randomize_stream_stable_wait_s": stable_wait,
                "randomize_stream_recovery_attempted": bool(render_stabilization.get("recovery_attempted")),
            },
        )
        raise err
    final_camera_xyz = list(_CAMERA_RIG_TRANSLATE_XYZ)
    selected_target_path = active_diaolan["group1"]
    camera_randomization_status = "randomized" if random_cfg["random_camera"] else "kept_current"
    if (not random_cfg["random_camera"]) and random_cfg["auto_look_at_target"]:
        camera_randomization_status = "kept_current_reoriented"
    visibility_check = {
        "enabled": bool(random_cfg["keep_target_visible"]),
        "startup_view_visible": startup_visible,
        "detail": preset_vis_detail.get("__startup_default__", {}) if preset_vis_detail else {},
        "preset_visibility": {k: v for k, v in preset_vis.items() if k != "__startup_default__"},
    }
    wall_height_detail = camera_meta.get("wall_height_constraint") if isinstance(camera_meta, dict) and isinstance(camera_meta.get("wall_height_constraint"), dict) else {}
    wall_constraint_status = {
        "enabled": bool(random_cfg["keep_wall_install_constraint"]),
        "mode": (camera_meta.get("constraint_mode") if isinstance(camera_meta, dict) else None) or (camera_meta.get("mode") if isinstance(camera_meta, dict) else None) or wall_height_detail.get("constraint_mode"),
        "source": (camera_meta.get("constraint_source") if isinstance(camera_meta, dict) else None) or wall_height_detail.get("constraint_source"),
        "mounted_on_wall": (camera_meta.get("mounted_on_wall") if isinstance(camera_meta, dict) else None) if isinstance(camera_meta, dict) and camera_meta.get("mounted_on_wall") is not None else wall_height_detail.get("mounted_on_wall"),
        "within_wall_constraint_box": (camera_meta.get("within_wall_constraint_box") if isinstance(camera_meta, dict) else None) if isinstance(camera_meta, dict) and camera_meta.get("within_wall_constraint_box") is not None else wall_height_detail.get("within_wall_constraint_box"),
        "fallback_used": (camera_meta.get("fallback_used") if isinstance(camera_meta, dict) else None) if isinstance(camera_meta, dict) and camera_meta.get("fallback_used") is not None else wall_height_detail.get("fallback_used"),
        "fallback_reason": (camera_meta.get("fallback_reason") if isinstance(camera_meta, dict) else None) or wall_height_detail.get("fallback_reason"),
    }
    all_paths_result = [str(d.get("path") or "").strip() for d in all_diaolans if str(d.get("path") or "").strip()]
    diaolan_candidates_result = []
    for item in all_diaolans:
        path_value = str(item.get("path") or "").strip()
        if not path_value:
            continue
        diaolan_candidates_result.append(
            _diaolan_candidate_meta(
                path_value,
                str(item.get("group1") or "").strip() or None,
                len(dedupe_worker_prim_paths_ordered([str(p) for p in (item.get("persons") or []) if str(p).strip()])),
            )
        )
    with _scene_lock:
        gondola_y_result = float(_scene_state.get("gondola_y", 0.0))
    eff_lookat_prim = _effective_lookat_target_prim_path(stage)
    building_context = _cached_context_lookat_selection(stage)
    _vis_paths_list = list(scene_after.get("visible_worker_paths") or [])
    _logical_visible_workers = int(count_logical_workers_from_paths(_vis_paths_list))
    _active_persons_logical = dedupe_worker_prim_paths_ordered(active_diaolan.get("persons") or [])
    _logical_capacity = len(_active_persons_logical)
    gondola_renderable_paths = list(scene_after.get("gondola_renderable_paths") or [])
    gondola_visible_renderable_paths = list(scene_after.get("gondola_visible_renderable_paths") or [])
    gondola_hidden_paths = list(scene_after.get("gondola_hidden_paths") or [])
    gondola_renderable_debug = list(scene_after.get("gondola_renderable_debug") or [])
    gondola_renderable_counts = dict(scene_after.get("gondola_renderable_counts") or {})
    result = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "diaolan_candidates": diaolan_candidates_result,
        "requested_active_diaolan_path": forced_from_req or None,
        "forced_active_diaolan_path": forced_from_req or None,
        "selected_diaolan_path": active_diaolan["path"],
        "active_diaolan_path": active_diaolan["path"],
        "all_diaolan_paths": all_paths_result,
        "target_prim_path": active_diaolan["group1"],
        "selected_target_path": selected_target_path,
        "lookat_target_prim_path": eff_lookat_prim,
        "active_lookat_target_prim_path": eff_lookat_prim,
        "lookat_building_context_prim_path": building_context.get("prim_path"),
        "lookat_building_context_selection": building_context,
        "all_worker_paths": list(scene_after["all_worker_paths"]),
        "visible_worker_paths": list(scene_after["visible_worker_paths"]),
        "worker_paths": list(scene_after["all_worker_paths"]),
        "hidden_paths": list(hide_result.get("active_hidden_paths") or []),
        "invalid_hidden_paths": list(hide_result.get("invalid_hidden_paths") or []),
        "gondola_height_cm": float(scene_after["gondola_height_cm"]),
        "gondola_y": gondola_y_result,
        "gondola_heights": dict(gondola_heights),
        "gondola_group1_paths": dict(gondola_group1_by_diaolan),
        "gondola_height_randomized_paths": list(gondola_height_randomized_paths),
        "workers": _logical_visible_workers,
        "visible_workers": _logical_visible_workers,
        "workers_count": _logical_visible_workers,
        "workers_total_available": _logical_capacity,
        "workers_visible_count_by_diaolan_path": dict(
            scene_after.get("workers_visible_count_by_diaolan_path") or {}
        ),
        "gondola_renderable_paths": gondola_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_visible_renderable_paths": gondola_visible_renderable_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_hidden_paths": gondola_hidden_paths[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_debug": gondola_renderable_debug[:_GONDOLA_RENDERABLE_DETAIL_LIMIT],
        "gondola_renderable_counts": gondola_renderable_counts,
        "height_debug": dict(scene_after["height_debug"]),
        "camera_xyz": final_camera_xyz,
        "camera_previous_xyz": [float(v) for v in previous_camera_xyz],
        "camera_randomization_status": camera_randomization_status,
        "random_config": random_cfg,
        "applied_random_flags": {
            "random_gondola": bool(random_cfg["random_gondola"]),
            "random_workers": bool(random_cfg["random_workers"]),
            "random_camera": bool(random_cfg["random_camera"]),
            "random_hdri": bool(random_cfg.get("random_hdri")),
            "random_guardrail": bool(random_cfg.get("random_guardrail")),
            "random_safety_rope": bool(random_cfg.get("random_safety_rope")),
            "random_limitstop": bool(random_cfg.get("random_limitstop")),
            "keep_target_visible": bool(random_cfg["keep_target_visible"]),
            "keep_wall_install_constraint": bool(random_cfg["keep_wall_install_constraint"]),
            "auto_look_at_target": bool(random_cfg["auto_look_at_target"]),
        },
        "safety_components_randomize": safety_rand_pack,
        "hdri_randomization_status": "randomized" if random_cfg.get("random_hdri") else "kept_current",
        "current_hdri": hdri_state.get("current_hdri"),
        "current_hdri_basename": hdri_state.get("current_hdri_basename"),
        "hdri_dome_light_path": hdri_state.get("hdri_dome_light_path"),
        "hdri_texture_attr": hdri_state.get("hdri_texture_attr"),
        "hdri_candidates": list(hdri_state.get("hdri_candidates") or []),
        "hdri_candidates_existing": list(hdri_state.get("hdri_candidates_existing") or []),
        "hdri_candidates_missing": list(hdri_state.get("hdri_candidates_missing") or []),
        "hdri": {
            **hdri_state,
            "status": "randomized" if random_cfg.get("random_hdri") else "kept_current",
            "previous_hdri": hdri_apply_result.get("previous_hdri") if isinstance(hdri_apply_result, dict) else hdri_state.get("current_hdri"),
            "previous_hdri_basename": hdri_apply_result.get("previous_hdri_basename") if isinstance(hdri_apply_result, dict) else hdri_state.get("current_hdri_basename"),
            "selected_hdri": hdri_apply_result.get("current_hdri") if isinstance(hdri_apply_result, dict) else hdri_state.get("current_hdri"),
            "selected_hdri_basename": hdri_apply_result.get("current_hdri_basename") if isinstance(hdri_apply_result, dict) else hdri_state.get("current_hdri_basename"),
        },
        "camera_meta": camera_meta,
        "wall_sampling_target_context": wall_target_context,
        "render_stabilization": render_stabilization,
        "render_commit_status": render_stabilization.get("render_commit_status"),
        "randomize_stream_freeze_used": bool(guard_info.get("freeze_active")),
        "randomize_stream_commit_source": render_stabilization.get("commit_source"),
        "randomize_stream_black_frames_blocked": max(
            0,
            int((_STREAM_DIAG.get("randomize_black_frames_blocked_total") if isinstance(_STREAM_DIAG, dict) else 0) or 0)
            - int(guard_info.get("black_frames_blocked_start") or 0),
        ),
        "randomize_stream_stable_wait_s": render_stabilization.get("elapsed_s"),
        "randomize_stream_recovery_attempted": bool(render_stabilization.get("recovery_attempted")),
        "startup_view_visible": startup_visible,
        "visibility_check": visibility_check,
        "preset_visibility": visibility_check["preset_visibility"],
        "startup_view_detail": visibility_check["detail"],
        "wall_constraint_status": wall_constraint_status,
        "orientation": dict(_orientation_state),
        "perception_camera_alignment": perception_alignment_diag,
        "rule11_lighting_adjustment": rule11_lighting_adjustment,
        "rule11_random_draw": rule11_random_draw,
        "rule11_explicit_request": bool(rule11_explicit_request),
        "rule11_camera_exposure_adjustment": rule11_camera_exposure_adjustment,
    }
    _mark_timing("result_build")
    _annotate_randomize_result_for_api(result, effective_request_meta)
    _mark_timing("hazard_annotate")
    timing["total_s"] = round(float(time.monotonic() - randomize_started), 4)
    result["timing"] = timing
    result["state_deferred"] = bool(_RANDOMIZE_FAST_RESPONSE)
    result["full_state_endpoint"] = "/scene/state"
    with _scene_lock:
        _scene_state["last_random_result"] = result
    _set_randomize_last_api_cache_from_lr(result)
    _invalidate_hdri_state_http_cache()
    _invalidate_scene_state_http_cache()
    _invalidate_status_http_cache()
    result_summary = {
        "timestamp": result.get("timestamp"),
        "request_id": result.get("request_id"),
        "selected_diaolan_path": result.get("selected_diaolan_path"),
        "camera_xyz": result.get("camera_xyz"),
        "camera_randomization_status": result.get("camera_randomization_status"),
        "startup_view_visible": result.get("startup_view_visible"),
        "wall_constraint_status": result.get("wall_constraint_status"),
        "gondola_renderable_counts": result.get("gondola_renderable_counts"),
        "hazard": result.get("hazard"),
        "rule_id": result.get("rule_id"),
    }
    print("[scene-randomize] " + json.dumps(result_summary, ensure_ascii=False))
    _randomize_stream_guard_finish(
        commit_source=str(render_stabilization.get("commit_source") or ""),
        mode="idle",
    )
    return result, rp_obj, annotator


def _init_gondola_paths_camera_and_random_height_once(stage) -> None:
    """stage 已加载且 upAxis 已读后调用：固定相机 rig、动态解析吊篮 prim、随机一次高度；
    仅写 Group1/node1/node2 高度轴，不调 _apply_scene_state。"""
    global _GONDOLA_PRIM, _WORKER1_PRIM, _WORKER2_PRIM, _CAMERA_RIG_PRIM
    global _node1_vs_group1_height_offset, _node2_vs_group1_height_offset
    global _last_gondola_init_sampled_height, _last_gondola_init_synced_node1, _last_gondola_init_relation
    global _last_gondola_init_synced_node2
    global _last_gondola_init_group1_source

    _CAMERA_RIG_PRIM = _resolve_camera_rig_path(stage)

    _log_disabled_black_model_workarounds()

    # 1. Scan diaolan prims
    all_diaolans = scan_diaolan_prims(stage)
    print(f"[diaolan-scan] found {len(all_diaolans)} diaolans: {[d['path'] for d in all_diaolans]}")

    if not all_diaolans:
        print("[diaolan-scan] ERROR: No diaolans found!")
        return

    # Random generator
    rng = random.Random()

    # 2. 选中吊篮（启动时仍可随机或按 force_active_diaolan_path）；所有吊篮保持可见
    forced_active_path = str(cfg.get("force_active_diaolan_path", "") or "").strip()
    active_diaolan = pick_active_diaolan(
        all_diaolans,
        rng,
        forced_path=forced_active_path if forced_active_path else None,
    )
    hide_result = _apply_active_diaolan_switch(
        stage, active_diaolan, all_diaolans, group1_source="diaolan_randomizer"
    )
    print(
        f"[diaolan-active] selected={active_diaolan['path']} "
        f"experiment_hidden={hide_result['active_hidden_paths']}"
    )
    log_target_branch_render_state(stage, _GONDOLA_PRIM, renderer_mode)
    hydra_material_mode = str(cfg.get("hydra_target_material_mode", "none")).strip().lower()
    if renderer_mode == "HydraStorm" and hydra_material_mode == "debug":
        force_debug_emissive_on_target_branch(stage, _GONDOLA_PRIM)
        log_target_branch_render_state(stage, _GONDOLA_PRIM, renderer_mode)
    elif renderer_mode == "HydraStorm" and hydra_material_mode == "formal":
        apply_hydrastorm_formal_materials_on_target_branch(stage, _GONDOLA_PRIM)
        log_target_branch_render_state(stage, _GONDOLA_PRIM, renderer_mode)

    aabb = compute_changjing_aabb(stage)
    max_retries = 10
    gondola_y = 0.0
    visible_count = 0
    visible_paths = []
    hazard = True
    reason = None
    dynamic_height_max = min(_GONDOLA_HEIGHT_MAX, aabb["zmax"]) if aabb["zmax"] > 5.0 else 36.0

    forced_height = None
    forced_height_raw = cfg.get("force_gondola_height")
    if forced_height_raw not in (None, ""):
        try:
            forced_height = max(
                _GONDOLA_HEIGHT_MIN,
                min(dynamic_height_max, float(forced_height_raw)),
            )
        except (TypeError, ValueError):
            forced_height = None

    forced_workers = None
    forced_workers_raw = cfg.get("force_workers_count")
    if forced_workers_raw not in (None, ""):
        try:
            forced_workers = max(0, min(len(active_diaolan["persons"]), int(forced_workers_raw)))
        except (TypeError, ValueError):
            forced_workers = None

    if forced_height is not None or forced_workers is not None:
        gondola_y = forced_height if forced_height is not None else rng.uniform(_GONDOLA_HEIGHT_MIN, dynamic_height_max)
        requested_worker_count = forced_workers if forced_workers is not None else rng.randint(0, len(active_diaolan["persons"]))
        visible_paths = _choose_visible_worker_paths(active_diaolan.get("persons") or [], requested_worker_count, rng=rng)
        visible_count = len(visible_paths)
        hazard, reason = check_safety_hazard(gondola_y, visible_count)
        print(
            "[diaolan-workers] "
            f"forced_height={forced_height} forced_workers={forced_workers} "
            f"group1_z={gondola_y:.2f} persons_visible={visible_count} hazard={hazard}"
        )
        if hazard:
            print(f"[diaolan-workers] WARNING: forced state still has hazard. reason={reason}")
    else:
        for attempt in range(max_retries):
            gondola_y = rng.uniform(_GONDOLA_HEIGHT_MIN, dynamic_height_max)
            requested_worker_count = rng.randint(0, len(active_diaolan["persons"]))
            visible_paths = _choose_visible_worker_paths(active_diaolan.get("persons") or [], requested_worker_count, rng=rng)
            visible_count = len(visible_paths)
            hazard, reason = check_safety_hazard(gondola_y, visible_count)
            if not hazard:
                break
        if hazard:
            print(f"[diaolan-workers] WARNING: Failed to find safe state after {max_retries} retries. Last reason: {reason}")

    print(
        f"[diaolan-workers] persons_visible={visible_count} group1_z={gondola_y:.2f} "
        f"is_on_ground={gondola_y < 0.12 + 0.5} hazard={hazard}"
    )

    workers_by_init: dict[str, int] = {}
    ap_init = str(active_diaolan.get("path") or "").strip()
    for d in all_diaolans:
        rp = str(d.get("path") or "").strip()
        workers_by_init[rp] = int(visible_count) if rp == ap_init else 0
    with _scene_lock:
        _scene_state["workers_visible_count_by_diaolan_path"] = dict(workers_by_init)
    _sync_worker_scalar_fields_for_control_diaolan(all_diaolans, ap_init)
    with _scene_lock:
        _scene_state["gondola_y"] = gondola_y
        _scene_state["workers"] = int(visible_count)
    _last_gondola_init_sampled_height = gondola_y

    _apply_scene_state(stage)
    for d in all_diaolans:
        sync_workers_to_group1(stage, d["group1"], d.get("persons") or [], 0)

    print(f"[changjing-aabb] xmin={aabb['xmin']:.2f} xmax={aabb['xmax']:.2f} ymin={aabb['ymin']:.2f} ymax={aabb['ymax']:.2f} zmin={aabb['zmin']:.2f} zmax={aabb['zmax']:.2f} → yaml written")


    # 5. 相机：可选关闭随机采样，仅用 camera_rig_translate_xyz（缺省回退与 scene_single CameraRig 一致）
    presets_cfg = cfg.get("presets", {})
    max_cam_retries = 20
    cam_x, cam_y, cam_z = 0.0, 0.0, 0.0
    preset_vis = {}
    sampling_on = bool(cfg.get("diaolan_camera_sampling_enabled", True))
    sample_box = cfg.get("diaolan_camera_sample_box")
    if isinstance(sample_box, dict) and sampling_on:
        _box = sample_box
    else:
        _box = None

    global _CAMERA_RIG_TRANSLATE_XYZ

    startup_lookat_target_xyz = _resolve_startup_lookat_target_xyz(
        stage, _effective_lookat_target_prim_path(stage)
    )
    startup_wall_target_context = _wall_sampling_target_context(stage, active_diaolan)

    if not sampling_on:
        _set_camera_rig_fixed_translate(stage, _CAMERA_RIG_PRIM)
        cam_x, cam_y, cam_z = _CAMERA_RIG_TRANSLATE_XYZ
        preset_vis, preset_vis_detail = check_preset_visibility(
            stage,
            _CAMERA_RIG_PRIM,
            presets_cfg,
            _effective_lookat_target_prim_path(stage) or _GONDOLA_PRIM,
            orientation_mode=_CAMERA_ORIENTATION_MODE,
            lookat_target_xyz=startup_lookat_target_xyz,
            resolution_wh=(W, H),
            return_details=True,
            prefer_target_prim_center=False,
            dynamic_startup_pan_offset_deg=_DYNAMIC_STARTUP_PAN_OFFSET_DEG,
            dynamic_startup_tilt_offset_deg=_DYNAMIC_STARTUP_TILT_OFFSET_DEG,
        )
        pv_line = _preset_visibility_log_line(
            presets_cfg, {k: v for k, v in preset_vis.items() if k != "__startup_default__"}
        )
        ok = bool(preset_vis.get("__startup_default__", False))
        startup_detail = preset_vis_detail.get("__startup_default__", {})
        print(
            "[camera-startup-view] "
            f"camera_xyz=({cam_x:.4f},{cam_y:.4f},{cam_z:.4f}) "
            f"mode={_CAMERA_ORIENTATION_MODE} "
            f"startup_pan={startup_detail.get('applied_pan')} "
            f"startup_tilt={startup_detail.get('applied_tilt')} "
            f"visible={startup_detail.get('visible')} "
            f"frustum_visible={startup_detail.get('frustum_visible')} "
            f"center_in_frame={startup_detail.get('center_in_frame')} "
            f"intersection_ratio={startup_detail.get('intersection_ratio')} "
            f"accept={ok}"
        )
        print(
            f"[camera-rig] sampling=OFF final_translate=({cam_x:.2f},{cam_y:.2f},{cam_z:.2f}) "
            f"(camera_rig_translate_xyz / 内置回退)"
        )
        print(f"[preset-visibility] {pv_line} → {'OK' if ok else 'FAILED'}")
    else:
        last_seed = None
        last_box_meta = None
        last_attempt_idx = 0
        startup_attempt_diagnostics = []
        for attempt in range(max_cam_retries):
            seed = rng.randint(0, 1000000)
            cam_x, cam_y, cam_z, _, box_meta = sample_camera_in_changjing(
                stage,
                _CHANGJING_PRIM_PATH,
                _CAMERA_RIG_PRIM,
                rng,
                seed,
                sample_box=_box,
                wall_prim_path=_CAMERA_WALL_CONSTRAINT_PRIM,
                wall_constraint_xy_margin=_CAMERA_WALL_CONSTRAINT_XY_MARGIN,
                wall_constraint_z_margin=_CAMERA_WALL_CONSTRAINT_Z_MARGIN,
                wall_mount_inset_m=_CAMERA_WALL_MOUNT_INSET_M,
                wall_mount_inset_mode=_CAMERA_WALL_MOUNT_INSET_MODE,
                target_world_xyz=startup_lookat_target_xyz,
                wall_collection_mode=_WALL_COLLECTION_MODE,
                wall_collection_root_path=_WALL_COLLECTION_ROOT_PATH,
                target_context_bbox=startup_wall_target_context.get("bbox"),
                wall_candidate_region=_WALL_CANDIDATE_REGION,
            )
            last_seed = seed
            last_box_meta = box_meta
            last_attempt_idx = attempt + 1

            preset_vis, preset_vis_detail = check_preset_visibility(
                stage,
                _CAMERA_RIG_PRIM,
                presets_cfg,
                _effective_lookat_target_prim_path(stage) or _GONDOLA_PRIM,
                orientation_mode=_CAMERA_ORIENTATION_MODE,
                lookat_target_xyz=startup_lookat_target_xyz,
                resolution_wh=(W, H),
                return_details=True,
                prefer_target_prim_center=False,
                dynamic_startup_pan_offset_deg=_DYNAMIC_STARTUP_PAN_OFFSET_DEG,
                dynamic_startup_tilt_offset_deg=_DYNAMIC_STARTUP_TILT_OFFSET_DEG,
            )
            startup_detail = preset_vis_detail.get("__startup_default__", {})
            startup_visible = bool(preset_vis.get("__startup_default__", False))
            center_px = startup_detail.get("center_px")
            startup_row = {
                "candidate_idx": attempt + 1,
                "camera_xyz": (round(float(cam_x), 4), round(float(cam_y), 4), round(float(cam_z), 4)),
                "mode": _CAMERA_ORIENTATION_MODE,
                "startup_pan": startup_detail.get("applied_pan"),
                "startup_tilt": startup_detail.get("applied_tilt"),
                "center_px": center_px,
                "frustum_visible": startup_detail.get("frustum_visible"),
                "center_in_frame": startup_detail.get("center_in_frame"),
                "intersection_ratio": startup_detail.get("intersection_ratio"),
                "intersection_ratio_threshold": startup_detail.get("intersection_ratio_threshold"),
                "rejection_reason": startup_detail.get("rejection_reason", "accepted" if startup_visible else "unknown"),
                "near_miss": bool(startup_detail.get("near_miss", False)),
                "visible": startup_detail.get("visible"),
            }
            startup_attempt_diagnostics.append(startup_row)
            print(
                "[camera-startup-view] "
                f"candidate_idx={attempt + 1} "
                f"camera_xyz=({cam_x:.4f},{cam_y:.4f},{cam_z:.4f}) "
                f"mode={_CAMERA_ORIENTATION_MODE} "
                f"startup_pan={startup_detail.get('applied_pan')} "
                f"startup_tilt={startup_detail.get('applied_tilt')} "
                f"center_px={center_px} "
                f"frustum_visible={startup_detail.get('frustum_visible')} "
                f"center_in_frame={startup_detail.get('center_in_frame')} "
                f"intersection_ratio={startup_detail.get('intersection_ratio')} "
                f"intersection_ratio_threshold={startup_detail.get('intersection_ratio_threshold')} "
                f"rejection_reason={startup_row['rejection_reason']} "
                f"near_miss={startup_row['near_miss']} "
                f"accept={startup_visible}"
            )
            if startup_visible:
                break
            print(
                "[camera-startup-view] reject_candidate "
                f"reason={startup_row['rejection_reason']} candidate_idx={attempt + 1}"
            )

        _CAMERA_RIG_TRANSLATE_XYZ = (cam_x, cam_y, cam_z)
        _apply_dynamic_lookat_after_random_camera(stage, (cam_x, cam_y, cam_z), source="random_camera", preset_name="default_initial", target_xyz=startup_lookat_target_xyz)

        pv_line = _preset_visibility_log_line(
            presets_cfg, {k: v for k, v in preset_vis.items() if k != "__startup_default__"}
        )
        ok = bool(preset_vis.get("__startup_default__", False))
        eb = last_box_meta["effective_box"]
        bm = last_box_meta["mode"]
        clip = "; ".join(last_box_meta["aabb_clip_notes"]) if last_box_meta["aabb_clip_notes"] else "无"
        cfg_b = last_box_meta.get("config_box")
        cfg_hint = ""
        if cfg_b:
            cfg_hint = (
                f" config_box_x=[{cfg_b['x_min']:.2f},{cfg_b['x_max']:.2f}] "
                f"y=[{cfg_b['y_min']:.2f},{cfg_b['y_max']:.2f}] "
                f"z=[{cfg_b['z_min']:.2f},{cfg_b['z_max']:.2f}]"
            )
        print(
            f"[camera-rig] sampling=ON final_translate=({cam_x:.2f},{cam_y:.2f},{cam_z:.2f}) "
            f"seed={last_seed} attempts={last_attempt_idx}/{max_cam_retries} box_mode={bm}{cfg_hint} "
            f"eff_x=[{eb['x_min']:.2f},{eb['x_max']:.2f}] "
            f"eff_y=[{eb['y_min']:.2f},{eb['y_max']:.2f}] "
            f"eff_z=[{eb['z_min']:.2f},{eb['z_max']:.2f}] "
            f"aabb_clip={clip}"
        )
        if last_box_meta.get("fallback_invalid"):
            print(
                "[camera-rig] WARN: 配置采样盒与 Changjing AABB 无有效交集，已回退 legacy 大盒"
            )
        print(f"[preset-visibility] {pv_line} → {'OK' if ok else 'FAILED'}")

    if not ok:
        raise RuntimeError(
            f"default startup view not visible after camera sampling: mode={_CAMERA_ORIENTATION_MODE} "
            f"camera_xyz=({cam_x:.4f},{cam_y:.4f},{cam_z:.4f})"
        )

    # Set relations for diagnostics
    _last_gondola_init_relation = "diaolan_randomizer_managed"
    _last_gondola_init_synced_node1 = True
    _last_gondola_init_synced_node2 = True
    
    _stream_diag_update(
        gondola_resolved_prim=_GONDOLA_PRIM or None,
        node1_resolved_prim=_WORKER1_PRIM or None,
        node2_resolved_prim=_WORKER2_PRIM or None,
        camera_rig_resolved_prim=_CAMERA_RIG_PRIM or None,
        gondola_init_height=gondola_y,
        gondola_init_workers_visible=visible_count,
        gondola_init_relation=_last_gondola_init_relation,
        gondola_group1_source=_last_gondola_init_group1_source,
    )

    try:
        with _scene_lock:
            ctrl = _sanitize_hdri_control_state(_scene_state.get("hdri_control"))
        _refresh_hdri_backend_status_from_stage(stage, control_state=ctrl)
    except Exception as exc:
        print(f"[hdri] init backend status refresh skipped: {exc}", flush=True)


def _log_ctrl_http(phase: str, path: str, t0: float, extra: str = "") -> None:
    """控制面请求耗时打点（8081）：用于区分 handler 卡死 vs 槽位/内核积压。"""
    dt_ms = (time.monotonic() - t0) * 1000.0
    tag = ""
    if dt_ms >= 1000.0:
        tag = " WARN>=1000ms"
    elif dt_ms >= 500.0:
        tag = " WARN>=500ms"
    elif dt_ms >= 200.0:
        tag = " WARN>=200ms"
    tail = f" {extra}" if extra else ""
    print(
        f"[ctrl-http] {phase} path={path} tid={threading.get_ident()} dt_ms={dt_ms:.1f}{tag}{tail}",
        flush=True,
    )


class _PTZHandler(BaseHTTPRequestHandler):
    """轻量 HTTP handler：提供控制面板 HTML 和 REST 接口。"""
    timeout = 5.0  # 控制面 HTTP 强制超时 5 秒

    def log_message(self, fmt, *args):  # 静默访问日志
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Connection", "close")
        self.close_connection = True

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
        global _snapshot_http_viewport_holder, _diag_live_once_holder, _snapshot_jpg_live_vp_holder
        global _snapshot_live_viewport_http_frame_id
        global _last_snapshot_http_dt_ms
        global _last_health_fast_path_ts
        if self.path in ("/", "/index.html"):
            html_path = os.path.join(script_dir, "ptz_web_control.html")
            if os.path.isfile(html_path):
                data = open(html_path, "rb").read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"ptz_web_control.html not found")
            return

        if self.path == "/ptz_state":
            if _ptz_lock.acquire(timeout=_PTZ_STATE_HTTP_LOCK_TIMEOUT_S):
                try:
                    snap = {
                        "pan": float(_ptz_state["pan"]),
                        "tilt": float(_ptz_state["tilt"]),
                        "zoom": float(_ptz_state["zoom"]),
                    }
                    _ptz_state_http_stale_cache["pan"] = snap["pan"]
                    _ptz_state_http_stale_cache["tilt"] = snap["tilt"]
                    _ptz_state_http_stale_cache["zoom"] = snap["zoom"]
                finally:
                    _ptz_lock.release()
            else:
                snap = {
                    "pan": float(_ptz_state_http_stale_cache["pan"]),
                    "tilt": float(_ptz_state_http_stale_cache["tilt"]),
                    "zoom": float(_ptz_state_http_stale_cache["zoom"]),
                    "stale": True,
                    "ptz_lock_busy": True,
                }
            body = json.dumps(snap, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/status":
            _t_status = time.monotonic()
            _log_ctrl_http("enter", "/status", _t_status)
            body = _http_get_status_body_cached()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            _log_ctrl_http("exit", "/status", _t_status, extra=f"bytes={len(body)}")
            return

        if self.path == "/diagnostics":
            d = _compose_status_dict()
            d["diagnostics_version"] = 1
            body = json.dumps(d, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/diagnostics/render-capture":
            body = json.dumps(
                _compose_render_capture_diagnostics_dict(),
                ensure_ascii=False,
                indent=2,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/diagnostics/render-capture-live":
            with _same_tick_diag_lock:
                live = dict(_same_tick_latest_diag) if isinstance(_same_tick_latest_diag, dict) else None
            if live is None:
                payload = {"ok": False, "error": "no_same_tick_capture_yet"}
            else:
                payload = {"ok": True, "same_tick_pipeline": live}
            body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/diagnostics/snapshot-live-once":
            _t_lo = time.monotonic()
            _log_ctrl_http("enter", "/diagnostics/snapshot-live-once", _t_lo)
            with _ptz_lock:
                hdr_pan = float(_ptz_state["pan"])
                hdr_tilt = float(_ptz_state["tilt"])
                hdr_zoom = float(_ptz_state["zoom"])

            def _hdr_float(v: float) -> str:
                s = f"{v:.6f}"
                return s.encode("ascii", "replace").decode("ascii")[:64]

            def _send_live_once(
                status: int,
                body: bytes,
                src: str,
                err: str | None = None,
                ctype: str = "image/jpeg",
            ):
                self.send_response(status)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("X-PTZ-Diag-Live-Once", "1")
                self.send_header("X-PTZ-Diag-Source", src[:80])
                self.send_header("X-PTZ-Diag-Bypass-Last-Good", "1")
                self.send_header("X-PTZ-Diag-Pan", _hdr_float(hdr_pan))
                self.send_header("X-PTZ-Diag-Tilt", _hdr_float(hdr_tilt))
                self.send_header("X-PTZ-Diag-Zoom", _hdr_float(hdr_zoom))
                if err:
                    self.send_header("X-PTZ-Diag-Error", err[:200].encode("ascii", "replace").decode("ascii"))
                if body:
                    self.send_header("Content-Length", str(len(body)))
                self._cors()
                self.end_headers()
                if body:
                    self.wfile.write(body)

            ev = threading.Event()
            holder: dict = {"event": ev, "response": None}
            deadline = time.monotonic() + 15.0
            queued = False
            while time.monotonic() < deadline:
                with _diag_live_once_lock:
                    if _diag_live_once_holder is None:
                        _diag_live_once_holder = holder
                        queued = True
                        break
                time.sleep(0.02)
            if not queued:
                msg = b"diag_live_once_queue_busy"
                _send_live_once(503, msg, "error", "queue_busy", ctype="text/plain; charset=utf-8")
                _log_ctrl_http("exit", "/diagnostics/snapshot-live-once", _t_lo, extra="status=503 queue_busy")
                return
            _diag_live_once_dirty.set()
            if not ev.wait(15.0):
                with _diag_live_once_lock:
                    if _diag_live_once_holder is holder:
                        _diag_live_once_holder = None
                msg = b"diag_live_once_timeout"
                _send_live_once(504, msg, "error", "main_thread_timeout", ctype="text/plain; charset=utf-8")
                _log_ctrl_http("exit", "/diagnostics/snapshot-live-once", _t_lo, extra="status=504 timeout")
                return
            resp = holder.get("response")
            if not isinstance(resp, dict):
                msg = b"diag_live_once_bad_response"
                _send_live_once(500, msg, "error", "bad_response", ctype="text/plain; charset=utf-8")
                _log_ctrl_http("exit", "/diagnostics/snapshot-live-once", _t_lo, extra="status=500 bad_response")
                return
            if resp.get("ok") and isinstance(resp.get("jpg"), (bytes, bytearray)) and len(resp["jpg"]) > 0:
                jpg = bytes(resp["jpg"])
                src = str(resp.get("source") or "viewport_delegate")
                _send_live_once(200, jpg, src)
                _log_ctrl_http(
                    "exit",
                    "/diagnostics/snapshot-live-once",
                    _t_lo,
                    extra=f"status=200 bytes={len(jpg)} src={src}",
                )
                return
            err_s = str(resp.get("error") or "capture_failed")
            msg = err_s.encode("utf-8", errors="replace")[:2048]
            _send_live_once(503, msg, "error", err_s[:120], ctype="text/plain; charset=utf-8")
            _log_ctrl_http("exit", "/diagnostics/snapshot-live-once", _t_lo, extra=f"status=503 err={err_s[:80]}")
            return

        if self.path == "/render/volumetric":
            try:
                payload = _vol_snapshot()
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(body)
            except Exception as exc:
                body = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False).encode("utf-8")
                self.send_response(500)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(body)
            return

        if self.path == "/scene/state":
            body = _http_get_scene_state_body_cached()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/gondola_stage_heights":
            st = omni.usd.get_context().get_stage()
            snap = _snapshot_diaolan_group1_world_heights_on_stage(st)
            body = json.dumps(
                {
                    "ok": True,
                    "height_axis": str(_scene_up_axis),
                    "world_height_axis_index": _height_axis_index(),
                    "world_heights_by_diaolan": snap,
                    "gondola_prim": _GONDOLA_PRIM or None,
                },
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/gondola_suspend_evidence":
            st = omni.usd.get_context().get_stage()
            body = json.dumps(_snapshot_gondola_suspend_world_evidence(st), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/random-config":
            body = json.dumps(
                _random_config_http_payload_best_effort(api_wrap=False),
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/api/scene/randomize/last":
            body = json.dumps(_api_scene_randomize_last_dict(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/api/scene/random-config":
            body = json.dumps(
                _random_config_http_payload_best_effort(api_wrap=True),
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/api/health":
            _t_h = time.monotonic()
            _log_ctrl_http("enter", "/api/health", _t_h)
            body = json.dumps(_api_health_fast_path_payload(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            _log_ctrl_http("exit", "/api/health", _t_h, extra=f"bytes={len(body)}")
            return

        if self.path == "/api/stream_ready":
            ready = _STREAM_INIT_READY.is_set()
            body = json.dumps(
                {
                    "ok": bool(ready),
                    "phase": "post_camera_init" if ready else "awaiting_main_init",
                    "ctrl_port": _CTRL_PORT,
                    "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
                },
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200 if ready else 503)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/describe":
            stage = omni.usd.get_context().get_stage()
            body = json.dumps(_describe_active_scene_branch(stage), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/hdri":
            try:
                body = _http_get_hdri_state_body_cached()
            except Exception as exc:
                _log_hdri_timing("/scene/hdri", time.time(), end_ts=time.time(), exception=exc, cache_hit=False, phase="status_get")
                raise
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/environment":
            st = omni.usd.get_context().get_stage()
            body = json.dumps({"ok": True, "environment": _environment_public_status(st)}, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/dynamic-sky-presets":
            body = json.dumps(
                _http_dynamic_sky_presets_payload(include_stage_status=False),
                ensure_ascii=False,
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        # MJPEG 连续流 —— 浏览器一次连接持续接收帧（低延迟）
        if self.path.startswith("/stream.mjpeg"):
            if not preview_enabled:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(b"MJPEG preview disabled")
                return
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=ptzframe")
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Connection",    "close")
            self._cors()
            self.end_headers()
            last_fid = -1
            try:
                while _running:
                    with _mjpeg_lock:
                        fid = _mjpeg["frame_id"]
                        jpg = _mjpeg["jpeg"]
                    if fid == last_fid or jpg is None:
                        time.sleep(0.015)   # ~67fps 上限轮询
                        continue
                    last_fid = fid
                    self.wfile.write(
                        b"--ptzframe\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                        + jpg + b"\r\n"
                    )
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            return

        # 单帧快照（Safari 等不支持 MJPEG 流时的降级方案）
        if self.path.startswith("/snapshot.jpg"):
            _t_snap = time.monotonic()
            _snap_route = "/snapshot.jpg"
            _log_ctrl_http("enter", _snap_route, _t_snap)
            jpg = None
            frame_id = 0
            cap_seq = None
            pixel_src_hdr = None
            snap_viewport_align_diag = None
            snapshot_meta: dict = {}
            rtsp_jpg, rtsp_meta = _rtsp_latest_snapshot_jpeg()
            if rtsp_jpg is not None:
                jpg = rtsp_jpg
                snapshot_meta = dict(rtsp_meta or {})
                frame_id = int(snapshot_meta.get("frame_id") or 0)
                cap_seq = snapshot_meta.get("capture_seq")
                pixel_src_hdr = str(snapshot_meta.get("pixel_source") or "rtsp_latest")
            serve_last_good, serve_last_good_reason = _snapshot_should_prefer_last_good(
                _t_snap
            )
            if serve_last_good:
                _log_ctrl_http(
                    "snapshot_last_good_gate",
                    _snap_route,
                    _t_snap,
                    extra=f"reason={serve_last_good_reason}",
                )
            bypass_last_good_hdr = False
            # 优先：主线程 viewport_delegate（与 /diagnostics/snapshot-live-once 同源；不写 last_good/MJPEG/RTSP）
            if jpg is None:
                ev_lv = threading.Event()
                holder_lv: dict = {"event": ev_lv, "response": None}
                deadline_lv = time.monotonic() + float(_SNAPSHOT_JPG_LIVE_VP_QUEUE_WAIT_S)
                queued_lv = False
                while time.monotonic() < deadline_lv:
                    with _snapshot_jpg_live_vp_lock:
                        if _snapshot_jpg_live_vp_holder is None:
                            _snapshot_jpg_live_vp_holder = holder_lv
                            queued_lv = True
                            break
                    time.sleep(0.02)
                if queued_lv:
                    _snapshot_jpg_live_vp_dirty.set()
                    if not ev_lv.wait(4.0):
                        with _snapshot_jpg_live_vp_lock:
                            if _snapshot_jpg_live_vp_holder is holder_lv:
                                _snapshot_jpg_live_vp_holder = None
                    else:
                        resp_lv = holder_lv.get("response")
                        if (
                            isinstance(resp_lv, dict)
                            and resp_lv.get("ok")
                            and resp_lv.get("source") == "viewport_delegate"
                        ):
                            _jb = resp_lv.get("jpg")
                            if isinstance(_jb, (bytes, bytearray)) and len(_jb) > 0:
                                jpg = bytes(_jb)
                                _snapshot_live_viewport_http_frame_id += 1
                                frame_id = int(_snapshot_live_viewport_http_frame_id)
                                cap_seq = None
                                pixel_src_hdr = "live_viewport_delegate"
                                bypass_last_good_hdr = True
                                _ad = resp_lv.get("align_diag")
                                if isinstance(_ad, dict):
                                    snap_viewport_align_diag = _ad
            if jpg is None:
                jpg, frame_id, cap_seq = _get_last_good_snapshot_jpeg()
                if jpg is not None:
                    pixel_src_hdr = "last_good_snapshot_cache"
            if jpg is None:
                with _mjpeg_lock:
                    jpg = _snapshot_cache["jpeg"] or _mjpeg["jpeg"]
                    frame_id = _snapshot_cache["frame_id"] or _mjpeg["frame_id"]
                    cap_seq = _snapshot_cache.get("capture_seq")
                if pixel_src_hdr is None and jpg is not None:
                    pixel_src_hdr = "replicator_service_cache"
            if jpg is None:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("X-PTZ-Snapshot-Ready", "0")
                self._cors()
                self.end_headers()
                self.wfile.write(b"snapshot cache not ready")
                _last_snapshot_http_dt_ms = (time.monotonic() - _t_snap) * 1000.0
                _log_ctrl_http("exit", _snap_route, _t_snap, extra="status=503 no_jpeg")
                return
            self.send_response(200)
            self.send_header("Content-Type",   "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self.send_header("X-PTZ-Snapshot-Ready", "1")
            self.send_header("X-PTZ-Snapshot-Frame-Id", str(frame_id))
            if cap_seq is not None:
                self.send_header("X-PTZ-Snapshot-Capture-Seq", str(cap_seq))
            if pixel_src_hdr is not None:
                self.send_header("X-PTZ-Snapshot-Pixel-Source", pixel_src_hdr)
            if snapshot_meta.get("capture_epoch_ms") is not None:
                self.send_header("X-PTZ-Snapshot-Capture-Epoch-Ms", str(snapshot_meta.get("capture_epoch_ms")))
            if snapshot_meta.get("capture_iso"):
                self.send_header("X-PTZ-Snapshot-Capture-Iso", str(snapshot_meta.get("capture_iso")))
            if snapshot_meta.get("osd_text"):
                self.send_header("X-PTZ-Snapshot-OSD-Text", str(snapshot_meta.get("osd_text")))
            if snapshot_meta.get("frame_age_ms") is not None:
                self.send_header("X-PTZ-Snapshot-Frame-Age-Ms", str(snapshot_meta.get("frame_age_ms")))
            if snapshot_meta.get("randomize_mode"):
                self.send_header("X-PTZ-Snapshot-Randomize-Mode", str(snapshot_meta.get("randomize_mode")))
            if bypass_last_good_hdr:
                self.send_header("X-PTZ-Snapshot-Bypass-Last-Good", "1")
            if isinstance(snap_viewport_align_diag, dict):
                _vad = snap_viewport_align_diag

                def _snap_vp_hdr_str(v):
                    if v is None:
                        return "-"
                    s = str(v)[:220]
                    return s.encode("ascii", "backslashreplace").decode("ascii")

                self.send_header(
                    "X-PTZ-Snapshot-Viewport-Camera-Before",
                    _snap_vp_hdr_str(_vad.get("viewport_camera_before")),
                )
                self.send_header(
                    "X-PTZ-Snapshot-Viewport-Camera-Target",
                    _snap_vp_hdr_str(_vad.get("viewport_camera_target")),
                )
                self.send_header(
                    "X-PTZ-Snapshot-Viewport-Align-Applied",
                    "1" if _vad.get("viewport_align_applied") else "0",
                )
                _rok = _vad.get("viewport_restore_ok")
                self.send_header(
                    "X-PTZ-Snapshot-Viewport-Restore-Ok",
                    "1" if _rok is True else ("0" if _rok is False else "-"),
                )
                self.send_header(
                    "X-PTZ-Snapshot-Viewport-Restore-Skipped",
                    "1" if _vad.get("viewport_restore_skipped_no_previous") else "0",
                )
            self._cors()
            self.end_headers()
            self.wfile.write(jpg)
            _last_snapshot_http_dt_ms = (time.monotonic() - _t_snap) * 1000.0
            _log_ctrl_http("exit", _snap_route, _t_snap, extra=f"status=200 bytes={len(jpg)} src={pixel_src_hdr or '?'}")
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        global _render_capture_probe_holder, _render_capture_ab_probe_holder
        if self.path == "/api/scene/randomize/last":
            try:
                length = int(self.headers.get("Content-Length", 0))
            except (TypeError, ValueError):
                length = 0
            body = self.rfile.read(length) if length else b""
            payload, status = _api_scene_randomize_last_post_payload(body)
            resp = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp)
            return

        if self.path == "/diagnostics/render-capture-probe":
            try:
                try:
                    _plen = int(self.headers.get("Content-Length", 0))
                except (TypeError, ValueError):
                    _plen = 0
                if _plen > 0:
                    self.rfile.read(_plen)
                ev = threading.Event()
                holder: dict = {"event": ev, "response": None}
                deadline = time.monotonic() + 12.0
                queued = False
                while time.monotonic() < deadline:
                    with _render_capture_probe_lock:
                        if _render_capture_probe_holder is None:
                            _render_capture_probe_holder = holder
                            queued = True
                            break
                    time.sleep(0.02)
                if not queued:
                    self.send_response(503)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps({"ok": False, "error": "probe_queue_busy"}, ensure_ascii=False).encode("utf-8")
                    )
                    return
                _render_capture_probe_dirty.set()
                if not ev.wait(12.0):
                    with _render_capture_probe_lock:
                        if _render_capture_probe_holder is holder:
                            _render_capture_probe_holder = None
                    self.send_response(504)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps({"ok": False, "error": "probe_timeout_main_thread"}, ensure_ascii=False).encode(
                            "utf-8"
                        )
                    )
                    return
                resp = holder.get("response")
                if not isinstance(resp, dict):
                    resp = {"ok": False, "error": "probe_response_missing"}
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(
                    json.dumps(resp, ensure_ascii=False, indent=2, default=str).encode("utf-8")
                )
            except Exception as _probe_http_exc:
                try:
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps(
                            {"ok": False, "error": f"{type(_probe_http_exc).__name__}:{_probe_http_exc}"},
                            ensure_ascii=False,
                            default=str,
                        ).encode("utf-8")
                    )
                except Exception:
                    pass
            return

        if self.path == "/diagnostics/render-capture-ab-probe":
            try:
                try:
                    _plen_ab = int(self.headers.get("Content-Length", 0))
                except (TypeError, ValueError):
                    _plen_ab = 0
                if _plen_ab > 0:
                    self.rfile.read(_plen_ab)
                ev_ab = threading.Event()
                holder_ab: dict = {"event": ev_ab, "response": None}
                deadline_ab = time.monotonic() + 20.0
                queued_ab = False
                while time.monotonic() < deadline_ab:
                    with _render_capture_ab_probe_lock:
                        if _render_capture_ab_probe_holder is None:
                            _render_capture_ab_probe_holder = holder_ab
                            queued_ab = True
                            break
                    time.sleep(0.02)
                if not queued_ab:
                    self.send_response(503)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps({"ok": False, "error": "ab_probe_queue_busy"}, ensure_ascii=False).encode("utf-8")
                    )
                    return
                _render_capture_ab_probe_dirty.set()
                if not ev_ab.wait(20.0):
                    with _render_capture_ab_probe_lock:
                        if _render_capture_ab_probe_holder is holder_ab:
                            _render_capture_ab_probe_holder = None
                    self.send_response(504)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps({"ok": False, "error": "ab_probe_timeout_main_thread"}, ensure_ascii=False).encode(
                            "utf-8"
                        )
                    )
                    return
                resp_ab = holder_ab.get("response")
                if not isinstance(resp_ab, dict):
                    resp_ab = {"ok": False, "error": "ab_probe_response_missing"}
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(json.dumps(resp_ab, ensure_ascii=False, indent=2, default=str).encode("utf-8"))
            except Exception as _ab_probe_http_exc:
                try:
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(
                        json.dumps(
                            {"ok": False, "error": f"{type(_ab_probe_http_exc).__name__}:{_ab_probe_http_exc}"},
                            ensure_ascii=False,
                            default=str,
                        ).encode("utf-8")
                    )
                except Exception:
                    pass
            return

        if self.path == "/render/volumetric":
            try:
                global _volumetric_state
                try:
                    _plen = int(self.headers.get("Content-Length", 0))
                except (TypeError, ValueError):
                    _plen = 0
                raw = self.rfile.read(_plen) if _plen else b""
                try:
                    req = json.loads(raw.decode("utf-8") or "{}")
                except Exception as exc:
                    raise ValueError(f"Invalid JSON: {exc}") from exc
                if not isinstance(req, dict):
                    raise ValueError("request body must be an object")

                warnings: list[str] = []
                ignored_fields: list[str] = []
                changed: dict[str, dict] = {}

                with _volumetric_lock:
                    cur = dict(_volumetric_state)
                    next_state = dict(_volumetric_state)

                for k, v in req.items():
                    if k not in _VOLUMETRIC_SETTINGS:
                        ignored_fields.append(str(k))
                        continue

                    if k in ("enabled", "useDetailNoise"):
                        b, _err = _vol_parse_bool(v)
                        if b is None:
                            warnings.append(f"{k} type error (expected bool-like)")
                            continue
                        next_state[k] = bool(b)
                        continue

                    if k in _VOLUMETRIC_FLOAT_RANGES:
                        lo, hi = _VOLUMETRIC_FLOAT_RANGES[k]
                        try:
                            fv = float(v)  # type: ignore[arg-type]
                        except Exception:
                            warnings.append(f"{k} type error (expected number)")
                            continue
                        next_state[k] = _vol_clamp_float(
                            fv, lo, hi, float(next_state.get(k, _VOLUMETRIC_DEFAULT_STATE[k]))
                        )
                        continue

                    if k in ("transmittanceColor", "singleScatteringAlbedo"):
                        c, err = _vol_normalize_color3(v, next_state.get(k) or _VOLUMETRIC_DEFAULT_STATE[k])
                        if err:
                            warnings.append(f"{k} type error (expected [r,g,b])")
                            continue
                        next_state[k] = c
                        continue

                    warnings.append(f"{k} unsupported_field")

                for kk in _VOLUMETRIC_SETTINGS.keys():
                    if cur.get(kk) != next_state.get(kk):
                        changed[kk] = {"old": cur.get(kk), "new": next_state.get(kk)}

                if changed:
                    with _volumetric_lock:
                        _volumetric_state = dict(next_state)
                    _volumetric_dirty.set()
                    _wait_for_volumetric_apply(timeout_s=1.0)

                payload = _vol_snapshot()
                payload["changed"] = changed
                payload["ignored_fields"] = ignored_fields
                if warnings:
                    payload["warnings"] = (payload.get("warnings") or []) + warnings
                resp = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(resp)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False).encode("utf-8")
                self.send_response(400)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(resp)
            return

        if self.path == "/scene/gondola":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body) if body else {}
                stage = omni.usd.get_context().get_stage()
                raw_height_cm = req.get("height_cm", req.get("y"))
                if raw_height_cm is None:
                    raise ValueError("height_cm is required")
                height_cm = max(_GONDOLA_HEIGHT_MIN, min(_GONDOLA_HEIGHT_MAX, float(raw_height_cm)))
                converted_stage_units = _cm_to_stage_units(stage, height_cm)
                with _scene_lock:
                    _scene_state["gondola_y"] = converted_stage_units
                _scene_dirty.set()
                _invalidate_scene_state_http_cache()
                resp = json.dumps(
                    {
                        "ok": True,
                        "input_height_cm": height_cm,
                        "converted_stage_units": converted_stage_units,
                        "state": _scene_state_snapshot(stage),
                    },
                    ensure_ascii=False,
                )
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/workers":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                if "count" not in req:
                    raise ValueError("count is required")
                stage = omni.usd.get_context().get_stage()
                if stage is None:
                    raise RuntimeError("USD stage not ready")
                all_diaolans = scan_diaolan_prims(stage)
                path_set = _diaolan_path_set(all_diaolans)
                with _scene_lock:
                    current_sel = str(
                        _scene_state.get("selected_diaolan_path") or _scene_state.get("active_diaolan_path") or ""
                    ).strip()
                explicit = str(req.get("diaolan_path") or "").strip()
                if explicit and explicit != current_sel:
                    raise ValueError("diaolan_path must match current selected_diaolan_path")
                sel = explicit or current_sel
                if not sel or sel not in path_set:
                    raise ValueError("no selected diaolan or path not found")
                d_match = next((d for d in all_diaolans if str(d.get("path") or "").strip() == sel), None)
                if not d_match:
                    raise ValueError("diaolan prim not found")
                nmax = len(d_match.get("persons") or [])
                requested_count = max(0, min(nmax, int(req["count"])))
                with _scene_lock:
                    wb = dict(_scene_state.get("workers_visible_count_by_diaolan_path") or {})
                    wb[sel] = requested_count
                    _scene_state["workers_visible_count_by_diaolan_path"] = wb
                _sync_worker_scalar_fields_for_control_diaolan(all_diaolans, sel)
                _apply_scene_state(stage)
                for d in all_diaolans:
                    sync_workers_to_group1(stage, d["group1"], d.get("persons") or [], 0)
                _scene_dirty.set()
                _invalidate_scene_state_http_cache()
                resp = json.dumps({"ok": True, "state": _scene_state_snapshot(stage)}, ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/select_diaolan":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                path = str(req.get("selected_diaolan_path") or req.get("active_diaolan_path") or "").strip()
                if not path:
                    raise ValueError("selected_diaolan_path is required")
                resp = json.dumps(_apply_select_diaolan_http(path), ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc), "state": _scene_state_snapshot()}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/safety-components/status":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                _ = json.loads(body) if body else {}
                stage = omni.usd.get_context().get_stage()
                if stage is None:
                    raise RuntimeError("USD stage not ready")
                with _scene_lock:
                    sel = str(
                        _scene_state.get("selected_diaolan_path") or _scene_state.get("active_diaolan_path") or ""
                    ).strip()
                    wc = int(_scene_state.get("workers", 0))
                if not sel:
                    raise ValueError("no selected diaolan (selected_diaolan_path empty)")
                summary = summarize_diaolan_safety_components(stage, sel, wc)
                resp = json.dumps({"ok": True, "data": summary}, ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/safety-components/apply":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                comp = str(req.get("component_type") or req.get("component") or "").strip()
                act = str(req.get("action") or "").strip()
                stage = omni.usd.get_context().get_stage()
                if stage is None:
                    raise RuntimeError("USD stage not ready")
                with _scene_lock:
                    sel = str(
                        _scene_state.get("selected_diaolan_path") or _scene_state.get("active_diaolan_path") or ""
                    ).strip()
                    wc = int(_scene_state.get("workers", 0))
                if not sel:
                    raise ValueError("no selected diaolan (selected_diaolan_path empty)")
                applied = apply_diaolan_safety_component(stage, sel, comp, act, wc)
                _invalidate_scene_state_http_cache()
                # 不在此处跑 summarize（三次全量 USD 扫描）：易阻塞 HTTP 线程与 launcher 代理超时；
                # 前端在 apply 成功后会调用 /scene/safety-components/status 拉摘要。
                resp = json.dumps({"ok": bool(applied.get("ok")), "data": applied}, ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/random-config":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            _t_rc_post = time.monotonic()
            _log_ctrl_http("enter", "/scene/random-config:POST", _t_rc_post)
            try:
                req = json.loads(body) if body else {}
                with _scene_lock:
                    _scene_state["random_config"] = _sanitize_random_config(req)
                    resp = json.dumps({"ok": True, "random_config": _scene_state["random_config"]}, ensure_ascii=False)
                    _invalidate_scene_state_http_cache()
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            _log_ctrl_http("exit", "/scene/random-config:POST", _t_rc_post, extra=f"bytes={len(resp)}")
            return

        if self.path in ("/scene/randomize", "/api/scene/randomize"):
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            api = self.path == "/api/scene/randomize"
            resp = _http_json_scene_randomize(
                body,
                route_source="api" if api else "web",
                api_envelope=api,
            )
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/activate_diaolan":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                active_diaolan_path = str(req.get("active_diaolan_path") or "").strip()
                if not active_diaolan_path:
                    raise ValueError("active_diaolan_path is required")
                meta = {
                    "active_diaolan_path": active_diaolan_path,
                    "trigger": "manual_activate_diaolan",
                    "runtime_random_config": {
                        "random_gondola": False,
                        "random_workers": False,
                        "random_camera": False,
                        "random_hdri": False,
                    },
                }
                _assign_randomize_request_trace(meta, route_source="web")
                queued = _queue_randomize_scene(meta)
                resp = json.dumps(queued, ensure_ascii=False)
                _invalidate_scene_state_http_cache()
                _invalidate_status_http_cache()
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc), "state": _scene_state_snapshot()}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path in ("/scene/hdri/group", "/scene/hdri/select", "/scene/hdri/random"):
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                if self.path == "/scene/hdri/group":
                    req["action"] = "switch_group"
                elif self.path == "/scene/hdri/select":
                    req["action"] = "select_hdri"
                else:
                    req["action"] = "random_hdri"
                resp = json.dumps(_queue_hdri_apply(req, route=self.path), ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc), "state": _scene_state_snapshot()}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/environment":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                resp = json.dumps(_queue_environment_request(req), ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc), "state": _scene_state_snapshot()}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/dynamic-sky-preset":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                req = json.loads(body) if body else {}
                if not isinstance(req, dict):
                    req = {}
                preset_key = str(req.get("preset") or req.get("current_dynamic_sky_preset") or "").strip()
                by_id = {str(p.get("id")): str(p.get("path")) for p in _list_dynamic_sky_web_presets() if isinstance(p, dict)}
                lookup = preset_key
                if lookup.lower().endswith(".usd"):
                    lookup = lookup[:-4]
                preset_path = by_id.get(lookup)
                if not preset_path:
                    raise ValueError("preset 必填且须为 Skies/Dynamic 下已存在的预设 id（如 ClearSky）")
                env_req = {
                    "environment_mode": "dynamic_sky",
                    "dynamic_sky_enabled": True,
                    "dynamic_sky_preset_path": preset_path,
                }
                out = _queue_environment_request(env_req)
                if isinstance(out, dict):
                    stage = omni.usd.get_context().get_stage()
                    snap = _http_dynamic_sky_presets_payload(stage)
                    out["presets"] = snap.get("presets")
                    out["dynamic_sky_presets"] = snap.get("dynamic_sky_presets")
                    out["current_preset"] = snap.get("current_preset")
                    out["current_dynamic_sky_preset"] = snap.get("current_dynamic_sky_preset")
                    out["current_preset_path"] = snap.get("current_preset_path")
                    out["environment_mode"] = snap.get("environment_mode")
                resp = json.dumps(out, ensure_ascii=False)
            except Exception as exc:
                try:
                    stage = omni.usd.get_context().get_stage()
                    snap = _http_dynamic_sky_presets_payload(stage)
                except Exception:
                    snap = {"ok": False, "presets": _list_dynamic_sky_web_presets(), "current_preset": "", "environment_mode": "hdri"}
                snap_err = {**snap, "ok": False, "error": str(exc)}
                resp = json.dumps(snap_err, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/scene/camera_rig":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                req = json.loads(body)
                xyz = req.get("xyz")
                if not isinstance(xyz, list) or len(xyz) != 3:
                    raise ValueError("xyz must be a length-3 list")
                stage = omni.usd.get_context().get_stage()
                applied_xyz = _set_camera_rig_translate_runtime(stage, xyz)
                _refresh_projection_metrics(stage)
                resp = json.dumps({"ok": True, "xyz": applied_xyz, "describe": _describe_active_scene_branch(stage)}, ensure_ascii=False)
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False)
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode("utf-8"))
            return

        if self.path == "/control":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body)
                raw_pan = req.get("pan", None)
                raw_tilt = req.get("tilt", None)
                raw_zoom = req.get("zoom", None)
                with _ptz_lock:
                    if "pan"  in req:
                        _ptz_state["pan"]  = max(-170.0, min(170.0, float(req["pan"])))
                    if "tilt" in req:
                        _ptz_state["tilt"] = max(-90.0,  min(30.0,  float(req["tilt"])))
                    if "zoom" in req:
                        _ptz_state["zoom"] = max(1.0,    min(32.0,  float(req["zoom"])))

                    global _ptz_last_cmd
                    _ptz_last_cmd = {
                        "input": {
                            "pan": raw_pan if raw_pan is not None else _ptz_state["pan"],
                            "tilt": raw_tilt if raw_tilt is not None else _ptz_state["tilt"],
                            "zoom": raw_zoom if raw_zoom is not None else _ptz_state["zoom"],
                        },
                        "applied": dict(_ptz_state),
                        "source": "/control",
                    }
                _ptz_dirty.set()
                _invalidate_status_http_cache()
                with _ptz_lock:
                    resp = json.dumps({"ok": True, "state": dict(_ptz_state)})
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)})
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode())
            return

        if self.path == "/scene/experiment":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            try:
                req = json.loads(body) if body else {}
                action = str(req.get("action", "describe")).strip().lower()
                stage = omni.usd.get_context().get_stage()
                if action == "describe":
                    resp_obj = {"ok": True, "data": _describe_active_scene_branch(stage)}
                elif action == "clear":
                    _restore_scene_experiment_visibility(stage)
                    # 不在这里跑 _apply_scene_state / projection：与 apply_visibility 相同，
                    # 大量 renderable 日志与 bbox 链路会把 HTTP 卡死；下一次 randomize 会 reconcile。
                    _invalidate_scene_state_http_cache()
                    with _scene_lock:
                        se = {"active_hidden_paths": list(_scene_experiment_state["active_hidden_paths"])}
                    resp_obj = {
                        "ok": True,
                        "data": {
                            "scene_experiment": se,
                            "note": "clear: experiment visibility restored; lightweight JSON (no full describe).",
                        },
                    }
                elif action == "apply_visibility":
                    hide_paths = req.get("hide_paths") if isinstance(req.get("hide_paths"), list) else []
                    _restore_scene_experiment_visibility(stage)
                    apply_result = _set_scene_experiment_hidden_paths(stage, hide_paths)
                    # 不在此处调用 _apply_scene_state：其内 _repair_active_gondola_renderables
                    # 会对大量 renderable 逐条 flush 打印，易把 HTTP 线程拖死数分钟。
                    # 实验性隐藏仅写 USD visibility；后续 randomize 会正常 reconcile。
                    _invalidate_scene_state_http_cache()
                    rt = _scene_state_runtime_snapshot()
                    with _scene_lock:
                        se = {"active_hidden_paths": list(_scene_experiment_state["active_hidden_paths"])}
                    resp_obj = {
                        "ok": True,
                        "data": {
                            "apply_result": apply_result,
                            "active_diaolan_path": rt.get("active_diaolan_path"),
                            "target_prim_path": _GONDOLA_PRIM,
                            "scene_experiment": se,
                            "note": "apply_visibility lightweight response (no full describe / projection refresh).",
                        },
                    }
                elif action == "activate_diaolan":
                    path = str(req.get("active_diaolan_path") or "").strip()
                    if not path:
                        raise ValueError("active_diaolan_path is required")
                    all_diaolans = scan_diaolan_prims(stage)
                    picked = pick_active_diaolan(all_diaolans, random.Random(), forced_path=path)
                    if not picked or picked.get("path") != path:
                        raise ValueError(f"active_diaolan_path not found: {path!r}")
                    _apply_active_diaolan_switch(
                        stage, picked, all_diaolans, group1_source="manual_activate_diaolan"
                    )
                    _sync_worker_scalar_fields_for_control_diaolan(all_diaolans, path)
                    _apply_scene_state(stage)
                    for d in all_diaolans:
                        sync_workers_to_group1(stage, d["group1"], d.get("persons") or [], 0)
                    _refresh_projection_metrics(stage)
                    _invalidate_scene_state_http_cache()
                    resp_obj = {"ok": True, "data": _describe_active_scene_branch(stage)}
                else:
                    resp_obj = {"ok": False, "error": f"unknown action: {action}"}
            except Exception as exc:
                resp_obj = {"ok": False, "error": str(exc)}
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(json.dumps(resp_obj, ensure_ascii=False).encode("utf-8"))
            return

        self.send_response(404)
        self.end_headers()


def _start_control_server() -> None:
    """在 daemon 线程中启动 PTZ 控制 HTTP 服务，不阻塞仿真主循环。"""
    srv = _BoundedThreadingHTTPServer(("0.0.0.0", _CTRL_PORT), _PTZHandler)
    t   = threading.Thread(target=srv.serve_forever, daemon=True, name="ptz-ctrl")
    t.start()
    print(f"[PTZ-RTSP] 控制面板已启动 → http://localhost:{_CTRL_PORT}/")
    print(f"[PTZ-RTSP]   用浏览器打开上面的地址即可实时控制云台和变焦")
    print(f"[PTZ-RTSP]   控制面并发上限={_CTRL_HTTP_MAX_INFLIGHT}")


# ============================================================
# MediaMTX 管理
# ============================================================

_MEDIAMTX_DOWNLOAD_URL = (
    "https://github.com/bluenviron/mediamtx/releases/download/"
    "{version}/mediamtx_{version}_linux_amd64.tar.gz"
)

def _ensure_mediamtx() -> str:
    """
    确保 mediamtx 二进制可用。
    若配置路径不存在且 auto_download=true，则自动下载。
    返回二进制绝对路径。
    """
    mtx_path = _resolve_path(mediamtx_cfg.get("path", "./mediamtx"))
    if os.path.isfile(mtx_path) and os.access(mtx_path, os.X_OK):
        print(f"[PTZ-RTSP] 使用已有 mediamtx：{mtx_path}")
        return mtx_path

    if not mediamtx_cfg.get("auto_download", True):
        raise FileNotFoundError(
            f"mediamtx 未找到：{mtx_path}。"
            "请手动放置二进制或设置 auto_download: true。"
        )

    version = mediamtx_cfg.get("version", "v1.17.0")
    url = _MEDIAMTX_DOWNLOAD_URL.format(version=version)
    tar_path = mtx_path + ".tar.gz"

    print(f"[PTZ-RTSP] 正在下载 mediamtx {version}...")
    print(f"[PTZ-RTSP]   URL: {url}")
    try:
        urllib.request.urlretrieve(url, tar_path)
    except Exception as e:
        raise RuntimeError(f"下载 mediamtx 失败：{e}\n请手动下载并放置于 {mtx_path}") from e

    print(f"[PTZ-RTSP] 解压 {tar_path}...")
    with tarfile.open(tar_path, "r:gz") as tf:
        # mediamtx 压缩包内只有一个可执行文件 mediamtx
        tf.extract("mediamtx", path=os.path.dirname(mtx_path))
    os.remove(tar_path)

    extracted = os.path.join(os.path.dirname(mtx_path), "mediamtx")
    if extracted != mtx_path:
        os.rename(extracted, mtx_path)
    os.chmod(mtx_path, 0o755)
    print(f"[PTZ-RTSP] mediamtx 已就绪：{mtx_path}")
    return mtx_path


def _port_in_use(port: int) -> bool:
    """检测 TCP 端口是否已被监听。"""
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def _start_mediamtx(mtx_path: str) -> subprocess.Popen:
    """
    启动 mediamtx 进程，使用脚本同目录的 mediamtx.yml 配置文件。
    若目标端口已在监听（说明 mediamtx 已运行），则跳过启动直接复用。
    返回 Popen 对象（复用时返回 None）。
    """
    port = mediamtx_cfg.get("port", 8554)

    # 若端口已在监听，假定 mediamtx 已运行，直接复用
    if _port_in_use(port):
        print(f"[PTZ-RTSP] 检测到 :{port} 已在监听，复用已有 mediamtx 实例")
        return None

    # 优先使用脚本同目录的 mediamtx.yml
    cfg_file = os.path.join(script_dir, "mediamtx.yml")
    cmd = [mtx_path]
    if os.path.isfile(cfg_file):
        cmd.append(cfg_file)
        print(f"[PTZ-RTSP] 使用配置文件：{cfg_file}")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    time.sleep(2.0)  # 等待 mediamtx 启动并绑定端口
    if proc.poll() is not None:
        err = proc.stderr.read().decode(errors="replace")
        raise RuntimeError(f"mediamtx 启动失败（exit {proc.returncode}）:\n{err}")
    print(f"[PTZ-RTSP] mediamtx 已启动，RTSP 端口 :{port}（pid={proc.pid}）")
    return proc


# ============================================================
# ffmpeg 管道管理（NVENC 优先，自动回退 libx264）
# ============================================================

def _get_ffmpeg_version() -> str:
    """获取 ffmpeg 版本字符串，失败时返回空字符串。"""
    try:
        out = subprocess.check_output(
            ["ffmpeg", "-version"], stderr=subprocess.STDOUT, timeout=5
        ).decode(errors="replace")
        first_line = out.splitlines()[0] if out else ""
        return first_line
    except Exception:
        return ""


def _build_osd_filter() -> str | None:
    """根据配置生成 ffmpeg drawtext 滤镜字符串，不启用时返回 None。
    使用 expansion=strftime 模式：% 由 ffmpeg 解释为 strftime 指令，
    冒号需转义为 \\: 以避免被 drawtext 解析为选项分隔符。
    """
    if not osd_enabled or _OSD_FRAME_CAPTURE_ENABLED:
        return None
    fmt      = _osd_cfg.get("fmt",  "%Y-%m-%d %H:%M:%S")
    x        = _osd_cfg.get("x",    10)
    y        = _osd_cfg.get("y",    10)
    size     = _osd_cfg.get("size", 28)
    fontfile = _osd_cfg.get("font", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    # expansion=strftime 模式：% 直接作为 strftime 指令；冒号转义为 \:
    escaped_fmt = fmt.replace(":", "\\:")
    return (
        f"drawtext=fontfile={fontfile}"
        f":text='{escaped_fmt}'"
        f":expansion=strftime"
        f":fontcolor=white:fontsize={size}"
        f":box=1:boxcolor=black@0.5:boxborderw=4"
        f":x={x}:y={y}"
    )


def _build_nvenc_cmd(rtsp_url: str, width: int, height: int, fps: int, bitrate: str) -> list:
    """构造 h264_nvenc 低延迟参数列表。"""
    gop = max(1, int(round(float(fps) * _RTSP_GOP_SECONDS)))
    osd = _build_osd_filter()
    # nvenc 需要 yuv420p，OSD 存在时拼接 drawtext，再做 format 转换
    vf = f"{osd},format=yuv420p" if osd else "format=yuv420p"
    return [
        "ffmpeg",
        "-loglevel", "warning",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-f", "rawvideo",
        "-pix_fmt", "rgba",
        "-s", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", "pipe:0",
        "-an",
        "-vf", vf,
        "-c:v", "h264_nvenc",
        "-preset", "p1",       # 最快预设（ffmpeg 5.x/6.x+）
        "-tune", "ull",        # ultra-low-latency 模式
        "-rc", "cbr",          # 固定码率，避免 VBR 帧率波动
        "-rc-lookahead", "0",
        "-surfaces", "2",
        "-bf", "0",            # 禁用 B 帧，降延迟
        "-g", str(gop),        # GOP 大小
        "-delay", "0",
        "-zerolatency", "1",
        "-forced-idr", "1",
        "-b:v", bitrate,
        "-flush_packets", "1",
        "-muxdelay", "0",
        "-muxpreload", "0",
        "-f", "rtsp",
        "-rtsp_transport", _RTSP_PUBLISH_TRANSPORT,
        rtsp_url,
    ]


def _build_x264_cmd(rtsp_url: str, width: int, height: int, fps: int, bitrate: str) -> list:
    """构造 libx264 低延迟参数列表（NVENC 不可用时的回退方案）。"""
    gop = max(1, int(round(float(fps) * _RTSP_GOP_SECONDS)))
    osd = _build_osd_filter()
    vf = f"{osd},format=yuv420p" if osd else "format=yuv420p"
    return [
        "ffmpeg",
        "-loglevel", "warning",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-f", "rawvideo",
        "-pix_fmt", "rgba",
        "-s", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", "pipe:0",
        "-an",
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-x264-params", f"keyint={gop}:min-keyint={gop}:scenecut=0",
        "-b:v", bitrate,
        "-vf", vf,
        "-flush_packets", "1",
        "-muxdelay", "0",
        "-muxpreload", "0",
        "-f", "rtsp",
        "-rtsp_transport", _RTSP_PUBLISH_TRANSPORT,
        rtsp_url,
    ]


def _start_ffmpeg(rtsp_url: str, width: int, height: int, fps: int, bitrate: str) -> subprocess.Popen:
    """
    启动 ffmpeg 进程，通过 stdin 接受 rawvideo RGBA 帧，编码为 H.264 并推流。
    策略：优先尝试 h264_nvenc；若 2 秒内进程退出说明 NVENC 不可用，自动回退 libx264。
    bufsize=0：跳过 Python 内部 BufferedWriter 层，大帧写入直通 OS pipe fd。
    """
    # 打印 ffmpeg 版本，方便排查参数兼容性
    ver = _get_ffmpeg_version()
    print(f"[PTZ-RTSP] {ver or 'ffmpeg 版本未知'}")

    def _launch(cmd: list, label: str) -> subprocess.Popen:
        import os as _os
        _env = _os.environ.copy()
        _env.setdefault("TZ", "Asia/Shanghai")  # OSD strftime 时区，系统未配置时兜底
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE,
                                bufsize=0, env=_env)
        # 异步打印 ffmpeg 错误输出，不阻塞主线程
        def _log_stderr():
            for line in proc.stderr:
                txt = line.decode(errors="replace").rstrip()
                if txt:
                    print(f"[ffmpeg/{label}] {txt}")
        threading.Thread(target=_log_stderr, daemon=True).start()
        return proc

    # 先试 NVENC
    nvenc_cmd = _build_nvenc_cmd(rtsp_url, width, height, fps, bitrate)
    print(f"[PTZ-RTSP] 尝试 h264_nvenc 编码器...")
    proc = _launch(nvenc_cmd, "nvenc")
    time.sleep(2.0)
    if proc.poll() is None:
        print(f"[PTZ-RTSP] h264_nvenc 编码器可用，已启动（pid={proc.pid}）")
        return proc

    # NVENC 失败，回退 libx264
    print("[PTZ-RTSP] h264_nvenc 不可用，回退到 libx264...")
    x264_cmd = _build_x264_cmd(rtsp_url, width, height, fps, bitrate)
    proc = _launch(x264_cmd, "x264")
    time.sleep(0.5)
    if proc.poll() is not None:
        raise RuntimeError("ffmpeg 进程意外退出（libx264），请检查 rtsp_url 和 mediamtx 是否正常运行。")
    print(f"[PTZ-RTSP] libx264 编码器已启动（pid={proc.pid}）")
    return proc


# ============================================================
# 相机绑定（Replicator）
# ============================================================

def _bind_camera(camera_prim_path: str, width: int, height: int, *, force_new: bool = False):
    """
    Create a Replicator RenderProduct and attach an RGB annotator to the camera prim.
    Returns (render_product, annotator).
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(camera_prim_path)
    if not prim.IsValid():
        raise ValueError(
            f"?? Prim ????{camera_prim_path}\n"
            "??? ptz_rtsp_config.yaml ?? camera_prim ??????????"
        )

    rp = rep.create.render_product(camera_prim_path, (width, height), force_new=force_new)

    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(rp)
    _render_capture_on_bind(rp, annotator, camera_prim_path, width, height)
    print(
        f"[PTZ-RTSP] ???????{camera_prim_path}  RenderProduct={rp} force_new={force_new}",
        flush=True,
    )
    return rp, annotator


def _recover_render_capture(world, rp, annotator, *, reason: str, camera_prim_path: str, width: int, height: int):
    global _render_recover_in_progress, _last_render_recover_attempt_mono
    global _post_recover_first_capture_pending, _last_render_recover_finish_mono
    now = time.monotonic()
    with _render_recover_state_lock:
        if _render_recover_in_progress:
            return False, rp, annotator, "recover_in_progress"
        if _last_render_recover_attempt_mono > 0.0:
            remaining = _NEAR_BLACK_RECOVER_COOLDOWN_S - (
                now - _last_render_recover_attempt_mono
            )
            if remaining > 0.0:
                return False, rp, annotator, f"recover_cooldown_{remaining:.1f}s"
        _render_recover_in_progress = True
        _last_render_recover_attempt_mono = now
    try:
        print(
            "[snapshot-chain] recover_render_capture begin "
            f"reason={reason} cooldown_s={_NEAR_BLACK_RECOVER_COOLDOWN_S:.1f}",
            flush=True,
        )
        for _ in range(4):
            world.step(render=True)
        rep.orchestrator.step(
            rt_subframes=2,
            delta_time=0.0,
            pause_timeline=False,
        )
        sim_app.update()
        sim_app.update()
        health_check = _run_render_capture_health_check(
            rp,
            annotator,
            camera_prim_path,
            perform_rep_step=True,
            source_label="recover_after_warmup",
        )
        should_rebind, rebind_reason = _should_rebind_render_capture_from_health_check(health_check)
        if should_rebind:
            print(
                "[snapshot-chain] recover_render_capture escalate_to_rebind "
                f"reason={reason} rebind_reason={rebind_reason}",
                flush=True,
            )
            reb_ok, rp, annotator, reb_err, reb_diag = _rebind_render_capture(
                world,
                rp,
                annotator,
                reason=f"{reason}|{rebind_reason}",
                camera_prim_path=camera_prim_path,
                width=width,
                height=height,
            )
            if reb_ok:
                _post_recover_first_capture_pending = True
                return True, rp, annotator, None
            diag_reason = None
            if isinstance(reb_diag, dict):
                diag_reason = (reb_diag.get("comparison") or {}).get("which_source_is_healthier")
            return False, rp, annotator, reb_err or f"rebind_failed:{diag_reason or 'unknown'}"
        _post_recover_first_capture_pending = True
        print(
            "[snapshot-chain] recover_render_capture success (warm-up only) "
            f"reason={reason} decision={rebind_reason} "
            f"{_snapshot_render_capture_identity('recover_done')}",
            flush=True,
        )
        return True, rp, annotator, None
    except Exception as exc:
        print(f"[snapshot-chain] recover_render_capture failed: {exc}", flush=True)
        return False, rp, annotator, f"{type(exc).__name__}:{exc}"
    finally:
        _last_render_recover_finish_mono = time.monotonic()
        with _render_recover_state_lock:
            _render_recover_in_progress = False


# ============================================================
# 优雅退出 + 帧队列写入线程
# ============================================================

_running = True
_ffmpeg_proc = None
_mediamtx_proc = None

# 帧队列：主线程生产，写入线程消费。maxsize=2 保证实时性，满时丢旧帧。
_frame_queue: queue.Queue = queue.Queue(maxsize=2)
_c_local_frame_saved = False
_rtsp_latest_lock = threading.Lock()
_rtsp_latest_cond = threading.Condition(_rtsp_latest_lock)
_rtsp_latest_frame: bytes | None = None
_rtsp_latest_source = "none"
_rtsp_latest_seq = 0
_rtsp_latest_mono = 0.0
_rtsp_latest_capture_epoch_ms: int | None = None
_rtsp_latest_capture_iso: str | None = None
_rtsp_latest_osd_text: str | None = None
_rtsp_latest_osd_draw_ms: float | None = None
_rtsp_latest_randomize_mode: str | None = None
_rtsp_snapshot_jpeg_cache: dict = {"seq": None, "jpeg": None, "meta": None, "encoded_mono": 0.0}
# RTSP 入队来源（供 _pipe_writer 摘要日志；主线程与 randomize 帧服务写入前更新）
_rtsp_last_enqueued_source = "none"
# 队列空时重复写 last_frame 的上限（与 get timeout≈0.5s 配合，避免无限垫旧帧）
_RTSP_PIPE_MAX_REPEAT_STREAK = 30
_RTSP_PIPE_MAX_LAST_FRAME_STALE_MS = 2800.0
# 主线程最后一次成功入队 RTSP 帧（monotonic）；用于诊断与可选恢复
_rtsp_main_last_enqueue_mono = 0.0
# 写入线程最后一次成功 stdin.write 后（monotonic）；用于检测 publisher 假活
_rtsp_writer_last_ok_mono: float | None = None


def _rtsp_put_frame_bytes(raw: bytes, source_tag: str, frame_meta: dict | None = None) -> bool:
    """
    将一帧 RGBA raw 字节入 RTSP 队列（与 ffmpeg -f rawvideo -pix_fmt rgba 一致）。
    返回 True 表示曾触发“丢旧帧再入队”路径。
    """
    global _rtsp_last_enqueued_source, _rtsp_main_last_enqueue_mono
    global _rtsp_latest_frame, _rtsp_latest_source, _rtsp_latest_seq, _rtsp_latest_mono
    global _rtsp_latest_capture_epoch_ms, _rtsp_latest_capture_iso, _rtsp_latest_osd_text, _rtsp_latest_osd_draw_ms
    global _rtsp_latest_randomize_mode
    _rtsp_last_enqueued_source = source_tag
    now = time.monotonic()
    meta = dict(frame_meta) if isinstance(frame_meta, dict) else _capture_time_meta()
    if _RTSP_LOW_LATENCY_MODE:
        with _rtsp_latest_cond:
            _rtsp_latest_frame = raw
            _rtsp_latest_source = source_tag
            _rtsp_latest_seq += 1
            _rtsp_latest_mono = now
            _rtsp_latest_capture_epoch_ms = meta.get("capture_epoch_ms")
            _rtsp_latest_capture_iso = meta.get("capture_iso")
            _rtsp_latest_osd_text = meta.get("osd_text")
            try:
                _rtsp_latest_osd_draw_ms = float(meta.get("osd_draw_ms"))
            except Exception:
                _rtsp_latest_osd_draw_ms = None
            _rtsp_latest_randomize_mode = (
                str(meta.get("randomize_mode"))
                if meta.get("randomize_mode") is not None
                else None
            )
            _rtsp_latest_cond.notify_all()
        _stream_diag_update(
            rtsp_latest_capture_epoch_ms=_rtsp_latest_capture_epoch_ms,
            rtsp_latest_capture_iso=_rtsp_latest_capture_iso,
            rtsp_latest_osd_text=_rtsp_latest_osd_text,
            osd_draw_ms=_rtsp_latest_osd_draw_ms,
            osd_draw_method=meta.get("osd_draw_method"),
            osd_applied=bool(meta.get("osd_applied", False)),
            randomize_stream_mode=_rtsp_latest_randomize_mode or "idle",
        )
        _rtsp_main_last_enqueue_mono = now
        return False
    dropped = False
    try:
        _frame_queue.put_nowait(raw)
    except queue.Full:
        try:
            _frame_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            _frame_queue.put_nowait(raw)
        except queue.Full:
            pass
        dropped = True
    _rtsp_main_last_enqueue_mono = now
    return dropped


def _rtsp_put_rgba_frame(
    rgba_u8,
    source_tag: str,
    *,
    capture_epoch_s: float | None = None,
) -> tuple[bool, dict]:
    raw, meta = _prepare_rtsp_rgba_frame(rgba_u8, capture_epoch_s=capture_epoch_s)
    dropped = _rtsp_put_frame_bytes(raw, source_tag, meta)
    return dropped, meta


def _rtsp_latest_snapshot_jpeg() -> tuple[bytes | None, dict | None]:
    if not _cfg_capture_prefer_rtsp_latest_for_snapshot() or _jpeg_encode_fn is None:
        return None, None
    with _rtsp_latest_cond:
        frame = _rtsp_latest_frame
        seq = int(_rtsp_latest_seq)
        source = str(_rtsp_latest_source or "rtsp_latest")
        mono_ts = float(_rtsp_latest_mono or 0.0)
        meta = {
            "frame_id": seq,
            "capture_seq": None,
            "capture_epoch_ms": _rtsp_latest_capture_epoch_ms,
            "capture_iso": _rtsp_latest_capture_iso,
            "osd_text": _rtsp_latest_osd_text,
            "pixel_source": f"rtsp_latest:{source}",
            "mono_ts": mono_ts,
            "randomize_mode": _rtsp_latest_randomize_mode,
        }
    if not isinstance(frame, (bytes, bytearray)) or len(frame) <= 0 or seq <= 0:
        return None, None
    if len(frame) != int(W) * int(H) * 4:
        return None, None
    with _rtsp_latest_lock:
        cache_seq = _rtsp_snapshot_jpeg_cache.get("seq")
        cached = _rtsp_snapshot_jpeg_cache.get("jpeg")
        cached_meta = _rtsp_snapshot_jpeg_cache.get("meta")
        if cache_seq == seq and isinstance(cached, (bytes, bytearray)) and cached_meta:
            out_meta = dict(cached_meta)
            out_meta["frame_age_ms"] = round(max(0.0, (time.monotonic() - mono_ts) * 1000.0), 1) if mono_ts > 0 else None
            return bytes(cached), out_meta
    try:
        rgba = np.frombuffer(frame, dtype=np.uint8).reshape((int(H), int(W), 4))
        jpg = _jpeg_encode_fn(rgba)
    except Exception as exc:
        _stream_diag_update(snapshot_rtsp_latest_error=f"jpeg_encode:{type(exc).__name__}:{exc}")
        return None, None
    if not isinstance(jpg, (bytes, bytearray)) or len(jpg) <= 0:
        _stream_diag_update(snapshot_rtsp_latest_error="jpeg_encode_empty")
        return None, None
    meta["frame_age_ms"] = round(max(0.0, (time.monotonic() - mono_ts) * 1000.0), 1) if mono_ts > 0 else None
    meta["snapshot_source"] = "rtsp_latest"
    blob = bytes(jpg)
    with _rtsp_latest_lock:
        _rtsp_snapshot_jpeg_cache["seq"] = seq
        _rtsp_snapshot_jpeg_cache["jpeg"] = blob
        _rtsp_snapshot_jpeg_cache["meta"] = dict(meta)
        _rtsp_snapshot_jpeg_cache["encoded_mono"] = time.monotonic()
    _stream_diag_update(
        snapshot_last_source="rtsp_latest",
        snapshot_last_rtsp_seq=seq,
        snapshot_last_capture_epoch_ms=meta.get("capture_epoch_ms"),
        snapshot_last_osd_text=meta.get("osd_text"),
    )
    return blob, meta


def _shutdown(signum=None, frame=None):
    global _running
    print("\n[PTZ-RTSP] 收到退出信号，正在清理...")
    _running = False


signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)


def _cleanup():
    """关闭写入线程（通过哨兵），再关闭 ffmpeg 和 mediamtx 子进程。"""
    # 放 None 哨兵让写入线程退出
    try:
        _frame_queue.put_nowait(None)
    except Exception:
        pass
    try:
        with _rtsp_latest_cond:
            _rtsp_latest_cond.notify_all()
    except Exception:
        pass

    if _ffmpeg_proc and _ffmpeg_proc.poll() is None:
        try:
            _ffmpeg_proc.stdin.close()
        except Exception:
            pass
        try:
            _ffmpeg_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _ffmpeg_proc.kill()
        print("[PTZ-RTSP] ffmpeg 进程已退出。")

    # _mediamtx_proc 为 None 表示复用了外部已有实例，不终止
    if _mediamtx_proc is not None and _mediamtx_proc.poll() is None:
        _mediamtx_proc.terminate()
        try:
            _mediamtx_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            _mediamtx_proc.kill()
        print("[PTZ-RTSP] mediamtx 进程已退出。")


def _pipe_writer(proc: subprocess.Popen) -> None:
    """
    后台写入线程：从 _frame_queue 取帧写入 ffmpeg stdin。
    与渲染主线程解耦，阻塞在 ffmpeg 消费速度上，不影响 Isaac 渲染循环。
    None 哨兵表示退出。
    队列空时仅在有限次数/时间内重复写 last_frame，超时则停止垫帧直至新帧入队。
    """
    t_write_total = 0.0
    write_count = 0
    last_stat_t = time.monotonic()
    last_frame = None
    repeat_streak_cur = 0
    queue_empty_since_stat = 0
    global _rtsp_writer_last_ok_mono
    last_new_frame_mono = time.monotonic()

    while True:
        if _RTSP_LOW_LATENCY_MODE:
            break
        from_queue = True
        try:
            frame = _frame_queue.get(timeout=0.5)
        except queue.Empty:
            from_queue = False
            queue_empty_since_stat += 1
            frame = last_frame
        if from_queue and frame is None:   # 哨兵，退出
            break
        if frame is None:
            if proc.poll() is not None:
                break
            continue
        if proc.poll() is not None:
            break

        skip_write = False
        if not from_queue:
            age_ms = (time.monotonic() - last_new_frame_mono) * 1000.0
            if last_frame is not None and (
                repeat_streak_cur >= _RTSP_PIPE_MAX_REPEAT_STREAK
                or age_ms >= _RTSP_PIPE_MAX_LAST_FRAME_STALE_MS
            ):
                skip_write = True
                # randomize 主线程占用期间：避免完全停写 stdin（ffmpeg 断链 / VLC 需重开）
                try:
                    if bool(_rtsp_randomize_keepalive_active):
                        skip_write = False
                except Exception:
                    pass

        if not skip_write:
            t0 = time.monotonic()
            try:
                proc.stdin.write(frame)
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                print("[PTZ-RTSP] ffmpeg 管道已断开（写入线程退出）")
                break
            _rtsp_writer_last_ok_mono = time.monotonic()
            if from_queue:
                last_frame = frame
                last_new_frame_mono = time.monotonic()
                repeat_streak_cur = 0
            else:
                repeat_streak_cur += 1
            t_write_total += time.monotonic() - t0
            write_count += 1

        now = time.monotonic()
        if now - last_stat_t >= 5.0:
            elapsed = now - last_stat_t
            push_fps = write_count / elapsed if elapsed > 0 else 0.0
            avg_write_ms = (t_write_total / write_count * 1000) if write_count else 0
            age_ms = (now - last_new_frame_mono) * 1000.0
            try:
                ens = _rtsp_last_enqueued_source
            except Exception:
                ens = "?"
            if skip_write:
                src_tag = "repeat_blocked"
            elif from_queue:
                src_tag = str(ens)
            else:
                src_tag = "repeat_last_frame"
            print(
                f"[PTZ-RTSP][write] source={src_tag} repeat_count={repeat_streak_cur} "
                f"queue_empty={queue_empty_since_stat} last_frame_age_ms={age_ms:.0f} "
                f"push_fps={push_fps:.1f} avg_write_ms={avg_write_ms:.1f}",
                flush=True,
            )
            t_write_total = 0.0
            write_count = 0
            queue_empty_since_stat = 0
            last_stat_t = now


def _pipe_writer_low_latency(proc: subprocess.Popen, target_fps: int) -> None:
    """按固定 fps 将最新帧写入 ffmpeg，避免旧帧在 RTSP/NVR 侧排队。"""
    global _rtsp_writer_last_ok_mono
    period_s = 1.0 / max(1, int(target_fps))
    next_deadline = time.monotonic()
    last_seq = -1
    last_frame = None
    last_source = "none"
    last_new_frame_mono = time.monotonic()
    repeat_count = 0
    new_count = 0
    max_gap_ms = 0.0
    write_count = 0
    t_write_total = 0.0
    last_stat_t = time.monotonic()

    while _running:
        if proc.poll() is not None:
            break
        now = time.monotonic()
        wait_s = next_deadline - now
        if wait_s > 0:
            time.sleep(min(wait_s, period_s))
        with _rtsp_latest_cond:
            if _rtsp_latest_frame is None and _running and proc.poll() is None:
                _rtsp_latest_cond.wait(timeout=0.5)
            frame = _rtsp_latest_frame
            seq = _rtsp_latest_seq
            source = _rtsp_latest_source
            frame_mono = _rtsp_latest_mono

        if frame is None:
            continue
        repeated = seq == last_seq
        if repeated and not _RTSP_REPEAT_LATEST_FRAME:
            next_deadline += period_s
            continue

        t0 = time.monotonic()
        try:
            proc.stdin.write(frame)
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            print("[PTZ-RTSP] ffmpeg 管道已断开（低延时写入线程退出）", flush=True)
            break
        _rtsp_writer_last_ok_mono = time.monotonic()
        t_write_total += _rtsp_writer_last_ok_mono - t0
        write_count += 1
        if repeated:
            repeat_count += 1
        else:
            if last_seq >= 0 and frame_mono:
                try:
                    gap_ms = max(0.0, (float(frame_mono) - float(last_new_frame_mono)) * 1000.0)
                    max_gap_ms = max(max_gap_ms, gap_ms)
                except Exception:
                    pass
            new_count += 1
            last_seq = seq
            last_frame = frame
            last_source = source
            last_new_frame_mono = frame_mono or _rtsp_writer_last_ok_mono

        age_ms = (_rtsp_writer_last_ok_mono - float(frame_mono or last_new_frame_mono)) * 1000.0
        _stream_diag_update(
            rtsp_writer_push_fps=None,
            rtsp_writer_repeated_frame=bool(repeated),
            rtsp_writer_frame_age_ms=round(age_ms, 1),
            rtsp_writer_last_source=str(source or last_source),
            rtsp_writer_latest_seq=int(seq),
        )

        now = time.monotonic()
        if now - last_stat_t >= 5.0:
            elapsed = now - last_stat_t
            push_fps = write_count / elapsed if elapsed > 0 else 0.0
            source_new_fps = new_count / elapsed if elapsed > 0 else 0.0
            repeat_ratio = (repeat_count / write_count) if write_count else 0.0
            avg_write_ms = (t_write_total / write_count * 1000.0) if write_count else 0.0
            latest_age_ms = (now - float(_rtsp_latest_mono or last_new_frame_mono)) * 1000.0
            _stream_diag_update(
                rtsp_writer_push_fps=round(push_fps, 2),
                rtsp_writer_frame_age_ms=round(latest_age_ms, 1),
                rtsp_writer_last_source=str(_rtsp_latest_source or last_source),
                rtsp_writer_new_frames=int(new_count),
                rtsp_writer_repeated_frames=int(repeat_count),
                rtsp_source_new_fps=round(source_new_fps, 2),
                rtsp_source_repeat_ratio=round(repeat_ratio, 3),
                rtsp_source_max_gap_ms=round(max_gap_ms, 1),
            )
            print(
                f"[PTZ-RTSP][write] mode=low_latency source={_rtsp_latest_source} "
                f"new={new_count} repeat={repeat_count} latest_age_ms={latest_age_ms:.0f} "
                f"push_fps={push_fps:.1f}/{target_fps} source_new_fps={source_new_fps:.1f} "
                f"repeat_ratio={repeat_ratio:.2f} max_gap_ms={max_gap_ms:.0f} "
                f"avg_write_ms={avg_write_ms:.1f}",
                flush=True,
            )
            write_count = 0
            repeat_count = 0
            new_count = 0
            max_gap_ms = 0.0
            t_write_total = 0.0
            last_stat_t = now

        next_deadline += period_s
        lag_s = time.monotonic() - next_deadline
        if lag_s > period_s:
            next_deadline = time.monotonic() + period_s


# ============================================================
# 主流程
# ============================================================

def main():
    global _ffmpeg_proc, _mediamtx_proc, _jpeg_encode_fn, _c_local_frame_saved, _post_recover_first_capture_pending
    global _rtsp_last_enqueued_source
    global _render_capture_probe_holder, _render_capture_probe_dirty, _render_capture_ab_probe_holder, _render_capture_ab_probe_dirty
    global _snapshot_http_viewport_holder, _snapshot_http_viewport_dirty
    global _diag_live_once_holder, _diag_live_once_dirty
    global _snapshot_jpg_live_vp_holder, _snapshot_jpg_live_vp_dirty, _snapshot_live_viewport_http_frame_id
    c_branch_geometry_only = bool(cfg.get("hydra_c_branch_geometry_only", False))
    _STREAM_INIT_READY.clear()

    # --- 初始化 JPEG 编码器（MJPEG 预览与 snapshot 快照共享）---
    _jpeg_encode_fn = _init_jpeg_encoder(mjpeg_quality)
    if _jpeg_encode_fn is None:
        print("[PTZ-RTSP] ⚠ JPEG 编码器不可用，/snapshot.jpg 将返回 503")
    elif preview_enabled:
        print("[PTZ-RTSP] preview_enabled=true，启用 MJPEG 预览与 snapshot 快照缓存")
    else:
        print(
            f"[PTZ-RTSP] preview_enabled=false，关闭 MJPEG 预览，但保留 snapshot 快照缓存 "
            f"(interval={snapshot_interval_s:.2f}s)"
        )

    if _cfg_capture_prefer_viewport_delegate_for_snapshot():
        print(
            "[PTZ-RTSP] capture_source_prefer_viewport_delegate_for_snapshot=true："
            "GET /snapshot.jpg 与 render-capture-probe/ab-probe 使用 viewport_delegate 像素；"
            "主循环 snapshot 缓存与 MJPEG 仍只用 Replicator；RTSP 入队优先 viewport_delegate（失败回退）。",
            flush=True,
        )

    # --- 启动 PTZ 控制 HTTP API（含 MJPEG 端点）---
    _start_control_server()

    # --- RTSP 管道（可选）---
    if rtsp_enabled:
        mtx_bin = _ensure_mediamtx()
        _mediamtx_proc = _start_mediamtx(mtx_bin)
        _ffmpeg_proc = _start_ffmpeg(rtsp_url, W, H, fps, bitrate)
        # 启动帧队列写入线程，与渲染主线程解耦
        _writer_target = _pipe_writer_low_latency if _RTSP_LOW_LATENCY_MODE else _pipe_writer
        _writer_args = (_ffmpeg_proc, fps) if _RTSP_LOW_LATENCY_MODE else (_ffmpeg_proc,)
        _writer = threading.Thread(
            target=_writer_target, args=_writer_args,
            daemon=True, name="frame-writer-low-latency" if _RTSP_LOW_LATENCY_MODE else "frame-writer"
        )
        _writer.start()
        print(
            f"[PTZ-RTSP] writer_mode={'low_latency' if _RTSP_LOW_LATENCY_MODE else 'queue'} "
            f"target_fps={fps} repeat_latest={_RTSP_REPEAT_LATEST_FRAME}",
            flush=True,
        )
    else:
        print(f"[PTZ-RTSP] RTSP 已禁用，仅 MJPEG 预览")
        print(f"[PTZ-RTSP] 预览地址：http://localhost:{_CTRL_PORT}/stream.mjpeg")

    # --- 加载 USD 场景 ---
    scene_abs = os.path.abspath(scene_path)
    print("[PTZ-RTSP] ========== 场景加载诊断 ==========")
    print(f"[PTZ-RTSP] scene_path(绝对)={scene_abs}")
    print(f"[PTZ-RTSP] scene_basename={os.path.basename(scene_path)}")
    print(
        "[PTZ-RTSP] 提示: 基名含 scene_generated / dataset 一般为生成样本；"
        "V4.0 等为母场景/基础场景"
    )
    print(f"[PTZ-RTSP] camera_prim(将绑定)={camera_prim}")
    _stream_diag_update(
        scene_path=scene_abs,
        scene_basename=os.path.basename(scene_path),
        camera_prim=camera_prim,
        open_stage_detail="open_stage_pending",
    )

    print(f"[PTZ-RTSP] 调用 open_stage …")
    _enforce_renderer_mode("pre_open_stage")
    usd_context = omni.usd.get_context()
    result = usd_context.open_stage(scene_path)
    # open_stage 根据版本返回 bool 或 (bool, str)
    if isinstance(result, tuple):
        ok, err = result
    else:
        ok, err = result, ""
    if not ok:
        detail = str(err) if err else "unknown"
        _stream_diag_update(stage_open_ok=False, open_stage_detail=f"open_stage_failed: {detail}")
        print(f"[PTZ-RTSP] open_stage 失败：{detail}")
        raise RuntimeError(f"加载场景失败：{detail}")
    _stream_diag_update(stage_open_ok=True, open_stage_detail="open_stage_ok")
    print("[PTZ-RTSP] open_stage 成功")

    # 等待场景完全加载
    sim_app.update()
    sim_app.update()

    # --- 读取场景坐标轴（自动适配 Pan/Tilt 旋转轴）---
    _enforce_renderer_mode("post_open_stage", reset_launch_config=True)
    try:
        with _volumetric_lock:
            _vol_enabled = bool(_volumetric_state.get("enabled", False))
        if _vol_enabled:
            _vol_apply_state()
            print("[render-volumetric] stage-load re-apply: enabled=true (applied)", flush=True)
        else:
            print("[render-volumetric] stage-load re-apply: enabled=false (skipped)", flush=True)
    except Exception as _vol_exc:
        print(f"[render-volumetric] stage-load re-apply skipped: {_vol_exc}", flush=True)
    global _scene_up_axis
    try:
        import omni.usd as _ousd
        from pxr import UsdGeom as _UsdGeom
        _scene_up_axis = _UsdGeom.GetStageUpAxis(_ousd.get_context().get_stage())
        print(f"[PTZ-RTSP] 场景 upAxis={_scene_up_axis}  "
              f"Pan={'rotateZ' if _scene_up_axis=='Z' else 'rotateY'}  "
              f"Tilt={'rotateX' if _scene_up_axis=='Z' else 'rotateZ'}")
    except Exception as e:
        print(f"[PTZ-RTSP] upAxis 读取失败（使用默认 Y-up）：{e}")

    # --- 固定相机 rig、解析吊篮 prim、启动时随机一次吊篮高度并写入 stage ---
    _init_gondola_paths_camera_and_random_height_once(usd_context.get_stage())
    clear_wall_mount_candidate_cache()
    _clear_building_context_selection_cache()
    _repair_broken_texture_paths(usd_context.get_stage())
    _sync_hdri_environment_dome_repairs(usd_context.get_stage())
    _ensure_hdri_render_consistency(usd_context.get_stage())
    try:
        _rule11_snapshot_hdri_dome_energy_baseline(usd_context.get_stage())
    except Exception as _bl_exc:
        print(f"[rule11-baseline] startup snapshot failed: {_bl_exc}", flush=True)
    st0 = usd_context.get_stage()
    _rig_world_xyz = _get_world_translation(st0, _CAMERA_RIG_PRIM)
    _startup_orientation_strategy = "legacy_preset_startup"
    if _rig_world_xyz is not None:
        dyn_result = _apply_dynamic_lookat_after_random_camera(
            st0,
            _rig_world_xyz,
            source="startup_init",
            preset_name="default_initial",
            apply_to_stage=False,
            target_xyz=_resolve_startup_lookat_target_xyz(st0, _effective_lookat_target_prim_path(st0)),
        )
        if dyn_result is not None:
            _startup_orientation_strategy = "dynamic_startup_orientation"
    _apply_ptz_state(st0)
    with _ptz_lock:
        _pp, _tt, _zz = _ptz_state["pan"], _ptz_state["tilt"], _ptz_state["zoom"]
    _startup_orientation = dict(_orientation_state)
    _startup_view_state.update({
        "token": "startup",
        "name": "StartupView",
        "pan": float(_pp),
        "tilt": float(_tt),
        "zoom": float(_zz),
        "camera_xyz": _startup_orientation.get("camera_xyz"),
        "target_xyz": _startup_orientation.get("target_xyz"),
        "base_pan": _startup_orientation.get("base_pan"),
        "base_tilt": _startup_orientation.get("base_tilt"),
        "source": _startup_orientation.get("last_source"),
        "preset_name": "default_initial",
    })
    print(
        "[startup-orientation] "
        f"strategy={_startup_orientation_strategy} "
        f"mode={_CAMERA_ORIENTATION_MODE} "
        f"camera_xyz={_startup_orientation.get('camera_xyz')} "
        f"lookat_target_xyz={_startup_orientation.get('target_xyz')} "
        f"base_pan={_startup_orientation.get('base_pan')} "
        f"base_tilt={_startup_orientation.get('base_tilt')} "
        f"applied_pan={_pp} applied_tilt={_tt} "
        "preset_override_after_start=False"
    )
    print(
        f"[PTZ-RTSP] 已应用默认相机姿态 pan={_pp}° tilt={_tt}° zoom={_zz}× "
        f"(rig 位置见 _CAMERA_RIG_TRANSLATE_XYZ / USDA)"
    )
    _log_usd_ptz_debug_snapshot(st0)
    _refresh_projection_metrics(st0)

    # --- 初始化 World ---
    stage_mpu = 0.01 if _scene_up_axis == "Y" else 1.0
    world = World(stage_units_in_meters=stage_mpu)
    world.reset()

    # 让渲染管线稳定（需要几帧预热）
    for _ in range(10):
        world.step(render=True)

    # --- 绑定相机 ---
    try:
        rp, annotator = _bind_camera(camera_prim, W, H)
    except Exception as e:
        _stream_diag_update(
            camera_bound_ok=False,
            open_stage_detail=f"camera_bind_failed: {e}",
        )
        print(f"[PTZ-RTSP] 相机绑定失败：{e}")
        raise
    _stream_diag_update(
        camera_bound_ok=True,
        render_product=str(rp),
        open_stage_detail="streaming_ready",
    )
    print(
        f"[PTZ-RTSP] 推流来源: Replicator RenderProduct={rp}  "
        f"RGB→{'ffmpeg→RTSP' if rtsp_enabled else '仅MJPEG'}"
    )
    print(
        "[render-pipeline] "
        f"renderer={renderer_mode} "
        f"camera_prim={camera_prim} "
        f"render_product={rp} "
        f"rtsp_url={rtsp_url if rtsp_enabled else 'disabled'}"
    )

    # Delegate/settings snapshot for C-branch troubleshooting
    try:
        import carb
        _settings = carb.settings.get_settings()
        keys = [
            "/app/renderer/plugin",
            "/rtx/rendermode",
            "/omni/replicator/RTSubframes",
            "/persistent/app/viewport/displayOptions",
            "/renderer/multiGpu/currentGpu",
        ]
        for k in keys:
            v = _settings.get(k)
            if v is not None:
                print(f"[render-delegate] {k}={v}")
    except Exception as e:
        print(f"[render-delegate] read settings failed: {e}")

    def _log_camera_target_frustum():
        try:
            stage = omni.usd.get_context().get_stage()
            cam_prim = stage.GetPrimAtPath(camera_prim)
            tgt_prim = stage.GetPrimAtPath(_GONDOLA_PRIM) if _GONDOLA_PRIM else None
            if not cam_prim.IsValid() or not tgt_prim or not tgt_prim.IsValid():
                print("[camera-frustum] invalid camera or target prim")
                return
            from pxr import Usd, UsdGeom
            bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
            target_bbox = bbox_cache.ComputeWorldBound(tgt_prim)
            target_range = target_bbox.ComputeAlignedRange()
            cam = UsdGeom.Camera(cam_prim)
            xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
            cam_xf = xcache.GetLocalToWorldTransform(cam_prim)
            gf_cam = cam.GetCamera(Usd.TimeCode.Default())
            frustum = gf_cam.frustum
            frustum.Transform(cam_xf)
            inside = frustum.Intersects(target_bbox)
            clip = gf_cam.clippingRange
            target_mid = target_range.GetMidpoint()
            cam_pos = cam_xf.ExtractTranslation()
            dist = (target_mid - cam_pos).GetLength()
            if hasattr(clip, "GetMin"):
                near_clip = float(clip.GetMin())
                far_clip = float(clip.GetMax())
            else:
                near_clip = float(clip[0])
                far_clip = float(clip[1])
            clip_state = "visible"
            if dist < near_clip:
                clip_state = "clipped_near"
            elif dist > far_clip:
                clip_state = "clipped_far"
            elif not inside:
                clip_state = "out_of_frustum"
            print(
                "[camera-frustum] "
                f"camera_pos=({cam_pos[0]:.2f},{cam_pos[1]:.2f},{cam_pos[2]:.2f}) "
                f"target_bbox_min={target_range.GetMin()} target_bbox_max={target_range.GetMax()} "
                f"target_mid=({target_mid[0]:.2f},{target_mid[1]:.2f},{target_mid[2]:.2f}) "
                f"distance={dist:.2f} near={near_clip:.4f} far={far_clip:.2f} "
                f"in_frustum={inside} result={clip_state}"
            )
        except Exception as e:
            print(f"[camera-frustum] failed: {e}")

    _log_camera_target_frustum()
    _refresh_projection_metrics(omni.usd.get_context().get_stage())

    # 纯 RTXRealTime 路线：让 Replicator 走默认 RTX RenderProduct，预热稍高一点换稳定首帧。
    _warm_sub = 4
    rep.orchestrator.step(rt_subframes=_warm_sub, delta_time=0.0, pause_timeline=False)
    sim_app.update()

    print(f"[PTZ-RTSP] 开始推流 → {rtsp_url}")
    print(f"[PTZ-RTSP] 客户端命令：vlc {rtsp_url}  或  ffplay {rtsp_url} -rtsp_transport tcp")
    snap_u = f"http://127.0.0.1:{_CTRL_PORT}/snapshot.jpg"
    print(f"[PTZ-RTSP] Web/代理快照源: {snap_u}（launcher 8080 会代理到本端口）")
    print(f"[PTZ-RTSP] preview_enabled={preview_enabled}  renderer={renderer_mode}")
    print("[PTZ-RTSP] 按 Ctrl-C 停止。\n")
    if rtsp_enabled and _ffmpeg_proc is not None:
        ff_ok = _ffmpeg_proc.poll() is None
    else:
        ff_ok = False
    _mtx_port = int(mediamtx_cfg.get("port", 8554))
    _stream_diag_update(
        ffmpeg_alive=ff_ok,
        mediamtx_started=bool(_mediamtx_proc is not None or _port_in_use(_mtx_port)),
    )

    # ── 分段耗时统计（Baseline 与优化后共用）────────────────────
    frame_idx        = 0
    last_stat_time   = time.monotonic()

    # 主线程侧计数器
    t_step_total     = 0.0   # world.step() 累计耗时
    t_get_total      = 0.0   # annotator.get_data() 累计耗时
    t_jpeg_total     = 0.0   # JPEG 编码累计耗时
    t_tobytes_total  = 0.0   # rgba.tobytes() 累计耗时
    render_count     = 0     # render=True 帧数
    capture_count    = 0     # 执行采集的帧数
    drop_count       = 0     # 队列满导致的丢帧数

    print(
        "[scene-auto-random-timer] main-loop scheduler enabled "
        "(interval from random_config.auto_random_interval_seconds, gate auto_random_timer_enabled)"
    )

    _capture_black_streak = 0

    while _running and sim_app.is_running():
        # 只在推流帧时渲染，其余帧跳过 GPU 渲染节省资源
        do_render = (frame_idx % skip_frames == 0)

        t0 = time.monotonic()
        try:
            world.step(render=do_render)
        except Exception as _ws_exc:
            if _is_gpu_device_oom_message(str(_ws_exc)):
                _note_gpu_oom_from_exception(_ws_exc, "world.step")
                print(
                    "[render-capture-diag] OOM_WRAP phase=world.step "
                    f"{_snapshot_render_capture_identity('world.step')}",
                    flush=True,
                )
            raise
        t_step_total += time.monotonic() - t0
        _now_loop = time.monotonic()
        _rtsp_age_ms, _rtsp_recovery_mode = _rtsp_update_stale_recovery(_now_loop)
        rtsp_stale_warn_now = _rtsp_age_ms is not None and _rtsp_age_ms > _RTSP_STALE_WARN_MS
        rtsp_recovery_now = _rtsp_recovery_mode == "viewport_only_recovery"
        rtsp_probe_replicator_this_tick = (
            _rtsp_should_probe_replicator()
            if do_render and not rtsp_stale_warn_now and not rtsp_recovery_now
            else False
        )
        global _RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO
        rtsp_fast_capture_due = (
            rtsp_enabled
            and _RTSP_LOW_LATENCY_MODE
            and _rtsp_viewport_primary_enabled()
            and _ffmpeg_proc is not None
            and _ffmpeg_proc.poll() is None
            and (
                _now_loop >= float(_RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO)
                or rtsp_stale_warn_now
            )
        )
        rtsp_fast_viewport_only = (
            rtsp_fast_capture_due
            and rtsp_enabled
            and _RTSP_LOW_LATENCY_MODE
            and _rtsp_viewport_primary_enabled()
            and _ffmpeg_proc is not None
            and _ffmpeg_proc.poll() is None
            and (
                rtsp_stale_warn_now
                or rtsp_recovery_now
                or not (
                _render_capture_probe_dirty.is_set()
                or _render_capture_ab_probe_dirty.is_set()
                or _snapshot_http_viewport_dirty.is_set()
                or _diag_live_once_dirty.is_set()
                or _snapshot_jpg_live_vp_dirty.is_set()
                )
            )
        )
        if rtsp_fast_viewport_only:
            try:
                _t0_fast_vp = time.monotonic()
                _vp_fast, _vp_fast_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
                if _vp_fast is not None:
                    _cap_epoch_s = None
                    if isinstance(_vp_fast_meta, dict):
                        try:
                            _cap_epoch_s = float(_vp_fast_meta.get("capture_epoch_s"))
                        except Exception:
                            _cap_epoch_s = None
                    _randomize_publish_block_active = _randomize_stream_guard_publish_block_active()
                    if _randomize_publish_block_active and _randomize_stream_guard_should_block_publish(
                        _render_capture_rgb_frame_health(_vp_fast)
                    ):
                        _stream_diag_update(randomize_stream_mode="frozen_last_good")
                    else:
                        if _rtsp_put_rgba_frame(
                            _vp_fast,
                            "viewport_delegate_primary_fast" if not rtsp_recovery_now else "viewport_only_recovery",
                            capture_epoch_s=_cap_epoch_s,
                        )[0]:
                            drop_count += 1
                    capture_count += 1
                    t_tobytes_total += time.monotonic() - _t0_fast_vp
                    _stream_diag_update(
                        render_capture_last_frame_source="viewport_only_recovery" if rtsp_recovery_now else "viewport_delegate_primary_fast",
                        render_capture_last_fallback_reason="replicator_probe_deferred",
                    )
                    _period = 1.0 / max(1, int(fps))
                    _RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO = max(
                        _t0_fast_vp + _period,
                        float(_RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO) + _period,
                    )
                else:
                    if not rtsp_recovery_now:
                        _rtsp_set_viewport_primary(False, "viewport_primary_fast_capture_failed")
                    _retry_s = 0.05 if rtsp_recovery_now else 0.5
                    _RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO = time.monotonic() + _retry_s
            except Exception as _fast_vp_exc:
                if not rtsp_recovery_now:
                    _rtsp_set_viewport_primary(False, f"viewport_primary_fast_exception:{type(_fast_vp_exc).__name__}")
                _retry_s = 0.05 if rtsp_recovery_now else 0.5
                _RTSP_VIEWPORT_PRIMARY_NEXT_CAPTURE_MONO = time.monotonic() + _retry_s
                print(f"[PTZ-RTSP][capture-mode] viewport_primary_fast failed: {_fast_vp_exc}", flush=True)

        rtsp_primary_fast_source_active = (
            rtsp_enabled
            and _RTSP_LOW_LATENCY_MODE
            and _rtsp_viewport_primary_enabled()
            and _ffmpeg_proc is not None
            and _ffmpeg_proc.poll() is None
        )

        if do_render and not rtsp_fast_viewport_only:
            render_count += 1

        # 应用来自 Web UI 的 PTZ 指令（检测到脏标志时写入 USD Stage）
        if _ptz_dirty.is_set():
            _ptz_dirty.clear()
            st = omni.usd.get_context().get_stage()
            _apply_ptz_state(st)
            _refresh_projection_metrics(st)
            with _ptz_lock:
                _cmd = _ptz_last_cmd
                _cmd = dict(_cmd) if isinstance(_cmd, dict) else None
                if isinstance(_cmd, dict) and isinstance(_cmd.get("input"), dict):
                    _cmd["input"] = dict(_cmd["input"])
            _log_ptz_applied_debug(st, _cmd)

        # 应用场景控制指令（吊篮高度 + 工人数量）
        if _scene_dirty.is_set():
            _scene_dirty.clear()
            st = omni.usd.get_context().get_stage()
            _apply_scene_state(st)
            _refresh_projection_metrics(st)
            _refresh_status_http_scene_cache_main_thread(force=True)

        if _volumetric_dirty.is_set():
            _volumetric_dirty.clear()
            try:
                _vol_apply_state()
            except Exception as _vol_apply_exc:
                with _volumetric_lock:
                    _volumetric_diag["last_error"] = str(_vol_apply_exc)

        _poll_auto_random_timer_main(frame_idx)

        if _scene_randomize_dirty.is_set():
            _scene_randomize_dirty.clear()
            with _scene_randomize_lock:
                global _scene_randomize_request
                pending_randomize = _scene_randomize_request
                _scene_randomize_request = None
            if isinstance(pending_randomize, dict):
                global _rtsp_randomize_keepalive_active
                randomize_event = pending_randomize.get("event")
                request_meta = pending_randomize.get("request") if isinstance(pending_randomize.get("request"), dict) else {}
                try:
                    _rtsp_randomize_keepalive_active = True
                    randomize_result, rp, annotator = _randomize_scene_runtime(
                        request_meta,
                        annotator=annotator,
                        world_obj=world,
                        rp_obj=rp,
                    )
                    _mark_randomize_render_apply()
                    if isinstance(request_meta, dict):
                        trigger = str(request_meta.get("trigger") or "").strip()
                        if trigger:
                            randomize_result["trigger"] = trigger
                        if "is_auto" in request_meta:
                            randomize_result["is_auto"] = bool(request_meta.get("is_auto"))
                    if str((request_meta or {}).get("trigger") or "") == "auto_random_timer":
                        print(
                            "[scene-auto-random-timer] tick done: main-thread randomize completed",
                            flush=True,
                        )
                    pending_randomize["response"] = {
                        "ok": True,
                        "result": randomize_result,
                        "state": _scene_state_lightweight_snapshot(result=randomize_result)
                            if _RANDOMIZE_FAST_RESPONSE
                            else _scene_state_snapshot(),
                        "state_deferred": bool(_RANDOMIZE_FAST_RESPONSE),
                        "full_state_endpoint": "/scene/state",
                        "timing": randomize_result.get("timing"),
                    }
                except Exception as exc:
                    exc_fields = getattr(exc, "randomize_response_fields", None)
                    exc_fields = dict(exc_fields) if isinstance(exc_fields, dict) else {}
                    _randomize_err_diag = _randomize_stream_guard_diag_snapshot()
                    _randomize_stream_guard_finish(mode="frozen_last_good")
                    pending_randomize["response"] = {
                        "ok": False,
                        "error": str(exc),
                        **({"render_stabilization": exc_fields.get("render_stabilization")} if isinstance(exc_fields.get("render_stabilization"), dict) else {}),
                        "render_commit_status": exc_fields.get("render_commit_status") or "failed",
                        "randomize_stream_freeze_used": bool(
                            exc_fields.get("randomize_stream_freeze_used")
                            if "randomize_stream_freeze_used" in exc_fields
                            else _randomize_err_diag.get("randomize_freeze_active")
                        ),
                        "randomize_stream_commit_source": (
                            exc_fields.get("randomize_stream_commit_source")
                            if "randomize_stream_commit_source" in exc_fields
                            else _randomize_err_diag.get("randomize_last_commit_source")
                        ),
                        "randomize_stream_black_frames_blocked": int(
                            (
                                exc_fields.get("randomize_stream_black_frames_blocked")
                                if "randomize_stream_black_frames_blocked" in exc_fields
                                else _randomize_err_diag.get("randomize_black_frames_blocked_total")
                            )
                            or 0
                        ),
                        "randomize_stream_stable_wait_s": exc_fields.get("randomize_stream_stable_wait_s"),
                        "randomize_stream_recovery_attempted": exc_fields.get("randomize_stream_recovery_attempted"),
                        "state": _scene_state_lightweight_snapshot()
                            if _RANDOMIZE_FAST_RESPONSE
                            else _scene_state_snapshot(),
                        "state_deferred": bool(_RANDOMIZE_FAST_RESPONSE),
                        "full_state_endpoint": "/scene/state",
                    }
                finally:
                    _rtsp_randomize_keepalive_active = False
                    _invalidate_hdri_state_http_cache()
                    _invalidate_scene_state_http_cache()
                    _invalidate_status_http_cache()
                    try:
                        _enforce_renderer_mode("post_scene_randomize")
                    except Exception as _rnd_rend_exc:
                        print(
                            f"[renderer-fix] post_scene_randomize enforce failed: {_rnd_rend_exc}",
                            flush=True,
                        )
                    if isinstance(randomize_event, threading.Event):
                        randomize_event.set()
                    if not _RANDOMIZE_FAST_RESPONSE:
                        _refresh_status_http_scene_cache_main_thread(force=True)

        # 推流帧：采集并输出
        if _scene_hdri_dirty.is_set():
            _scene_hdri_dirty.clear()
            with _scene_hdri_lock:
                global _scene_hdri_request
                pending_hdri = _scene_hdri_request
                _scene_hdri_request = None
            if isinstance(pending_hdri, dict):
                hdri_event = pending_hdri.get("event")
                try:
                    pending_hdri["response"] = _apply_hdri_control_request(
                        omni.usd.get_context().get_stage(),
                        pending_hdri.get("request"),
                        route=str(pending_hdri.get("route") or "/scene/hdri"),
                    )
                except Exception as exc:
                    pending_hdri["response"] = {
                        "ok": False,
                        "error": str(exc),
                        "state": _scene_state_snapshot(),
                    }
                finally:
                    _invalidate_hdri_state_http_cache()
                    _invalidate_scene_state_http_cache()
                    _invalidate_status_http_cache()
                    if isinstance(hdri_event, threading.Event):
                        hdri_event.set()

        if _scene_environment_dirty.is_set():
            _scene_environment_dirty.clear()
            with _scene_environment_lock:
                global _scene_environment_request
                pending_env = _scene_environment_request
                _scene_environment_request = None
            if isinstance(pending_env, dict):
                env_event = pending_env.get("event")
                try:
                    pending_env["response"] = _apply_environment_request_main_thread(
                        omni.usd.get_context().get_stage(),
                        pending_env.get("request"),
                    )
                except Exception as exc:
                    pending_env["response"] = {
                        "ok": False,
                        "error": str(exc),
                        "environment": _environment_public_status(),
                        "state": _scene_state_snapshot(),
                    }
                finally:
                    _invalidate_scene_state_http_cache()
                    _invalidate_status_http_cache()
                    if isinstance(env_event, threading.Event):
                        env_event.set()

        if _pending_hdri_audits:
            now_audit = time.monotonic()
            remaining_hdri_audits = []
            stage_for_audit = omni.usd.get_context().get_stage()
            for audit in _pending_hdri_audits:
                if now_audit < float(audit.get("due_at") or 0.0):
                    remaining_hdri_audits.append(audit)
                    continue
                prim = stage_for_audit.GetPrimAtPath(str(audit.get("prim_path") or ""))
                attr = prim.GetAttribute(_HDRI_TEXTURE_ATTR_NAME) if prim and prim.IsValid() else None
                actual_hdri = _extract_asset_path(attr.Get()) if attr and attr.IsValid() else ""
                expected_hdri = _normalize_hdri_path(audit.get("expected_hdri"))
                overwritten = bool(actual_hdri and expected_hdri and _normalize_hdri_path(actual_hdri) != expected_hdri)
                print(
                    "[HDRI] post_apply_audit "
                    f"prim={audit.get('prim_path')} expected={expected_hdri or '<empty>'} "
                    f"actual={actual_hdri or '<empty>'} overwritten_within_3s={overwritten}",
                    flush=True,
                )
            _pending_hdri_audits[:] = remaining_hdri_audits

        if do_render:
            # POST /diagnostics/render-capture-probe：仅主线程执行抓帧，避免与 Kit 线程打架
            if _render_capture_probe_dirty.is_set():
                _render_capture_probe_dirty.clear()
                _holder = None
                with _render_capture_probe_lock:
                    _holder = _render_capture_probe_holder
                if isinstance(_holder, dict):
                    try:
                        _holder["response"] = _run_render_capture_probe_pipeline(rp, annotator, camera_prim)
                    except Exception as _pbe:
                        _holder["response"] = {"ok": False, "error": f"{type(_pbe).__name__}:{_pbe}"}
                    _ev = _holder.get("event")
                    if isinstance(_ev, threading.Event):
                        _ev.set()
                    with _render_capture_probe_lock:
                        if _render_capture_probe_holder is _holder:
                            _render_capture_probe_holder = None

            if _render_capture_ab_probe_dirty.is_set():
                _render_capture_ab_probe_dirty.clear()
                _hab = None
                with _render_capture_ab_probe_lock:
                    _hab = _render_capture_ab_probe_holder
                if isinstance(_hab, dict):
                    try:
                        _hab["response"] = _run_render_capture_ab_probe_pipeline(rp, annotator, camera_prim)
                    except Exception as _abe:
                        _hab["response"] = {"ok": False, "error": f"{type(_abe).__name__}:{_abe}"}
                    _ev_ab = _hab.get("event")
                    if isinstance(_ev_ab, threading.Event):
                        _ev_ab.set()
                    with _render_capture_ab_probe_lock:
                        if _render_capture_ab_probe_holder is _hab:
                            _render_capture_ab_probe_holder = None

            if _snapshot_http_viewport_dirty.is_set():
                _snapshot_http_viewport_dirty.clear()
                _sh = None
                with _snapshot_http_viewport_lock:
                    _sh = _snapshot_http_viewport_holder
                if isinstance(_sh, dict):
                    try:
                        _sh["response"] = _run_snapshot_http_viewport_jpeg()
                    except Exception as _she:
                        _sh["response"] = {"ok": False, "error": f"{type(_she).__name__}:{_she}"}
                    _ev_sh = _sh.get("event")
                    if isinstance(_ev_sh, threading.Event):
                        _ev_sh.set()
                    with _snapshot_http_viewport_lock:
                        if _snapshot_http_viewport_holder is _sh:
                            _snapshot_http_viewport_holder = None

            if _diag_live_once_dirty.is_set():
                _diag_live_once_dirty.clear()
                _dl = None
                with _diag_live_once_lock:
                    _dl = _diag_live_once_holder
                if isinstance(_dl, dict):
                    try:
                        _dl["response"] = _run_diag_snapshot_live_once_pipeline(annotator, camera_prim)
                    except Exception as _dle:
                        _dl["response"] = {
                            "ok": False,
                            "jpg": None,
                            "source": "error",
                            "error": f"{type(_dle).__name__}:{_dle}",
                        }
                    _ev_dl = _dl.get("event")
                    if isinstance(_ev_dl, threading.Event):
                        _ev_dl.set()
                    with _diag_live_once_lock:
                        if _diag_live_once_holder is _dl:
                            _diag_live_once_holder = None

            if _snapshot_jpg_live_vp_dirty.is_set():
                _snapshot_jpg_live_vp_dirty.clear()
                _dlv = None
                with _snapshot_jpg_live_vp_lock:
                    _dlv = _snapshot_jpg_live_vp_holder
                if isinstance(_dlv, dict):
                    try:
                        _dlv["response"] = _run_diag_snapshot_live_once_pipeline(
                            annotator, camera_prim, replicator_fallback=False
                        )
                    except Exception as _dlve:
                        _dlv["response"] = {
                            "ok": False,
                            "jpg": None,
                            "source": "error",
                            "error": f"{type(_dlve).__name__}:{_dlve}",
                        }
                    _ev_dlv = _dlv.get("event")
                    if isinstance(_ev_dlv, threading.Event):
                        _ev_dlv.set()
                    with _snapshot_jpg_live_vp_lock:
                        if _snapshot_jpg_live_vp_holder is _dlv:
                            _snapshot_jpg_live_vp_holder = None

            # 触发 Replicator 更新 annotator 数据
            _t_rep_step = time.monotonic()
            try:
                rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
            except Exception as _rep_exc:
                if _is_gpu_device_oom_message(str(_rep_exc)):
                    _note_gpu_oom_from_exception(_rep_exc, "rep.orchestrator.step")
                    print(
                        "[render-capture-diag] OOM_WRAP phase=rep.orchestrator.step "
                        f"{_snapshot_render_capture_identity('rep.orchestrator.step')}",
                        flush=True,
                    )
                raise
            _ms_rep_step = (time.monotonic() - _t_rep_step) * 1000.0
            global _last_rep_orchestrator_step_ms
            _last_rep_orchestrator_step_ms = float(_ms_rep_step)
            if _ms_rep_step >= 200.0:
                print(
                    f"[main-tick] SLOW rep.orchestrator.step ms={_ms_rep_step:.1f} frame={frame_idx}",
                    flush=True,
                )

            capture_seq = _next_same_tick_capture_seq()
            tick_mono_start = time.monotonic()
            recover_attempted_this_tick = False
            jpg_used = None
            snapshot_written = False
            snap_fid = None
            snap_cap_seq_out = None

            t0 = time.monotonic()
            try:
                rgba_raw = annotator.get_data()  # shape (H, W, 4)；HydraStorm 等路径可能为 float 线性缓冲
            except Exception as _gd_exc:
                if _is_gpu_device_oom_message(str(_gd_exc)):
                    _note_gpu_oom_from_exception(_gd_exc, "annotator.get_data")
                    print(
                        "[render-capture-diag] OOM_WRAP phase=annotator.get_data "
                        f"{_snapshot_render_capture_identity('annotator.get_data')}",
                        flush=True,
                    )
                raise
            _ms_get_data = (time.monotonic() - t0) * 1000.0
            if _ms_get_data >= 200.0:
                print(
                    f"[main-tick] SLOW annotator.get_data ms={_ms_get_data:.1f} frame={frame_idx}",
                    flush=True,
                )
            rgba = _normalize_replicator_rgba_for_output(rgba_raw)
            t_get_total += time.monotonic() - t0
            capture_count += 1

            if rgba is not None and rgba.size > 0:
                if not _c_local_frame_saved and c_branch_geometry_only:
                    _c_local_frame_saved = True
                    try:
                        debug_encoder = _init_jpeg_encoder(90)
                        if debug_encoder is not None:
                            jpg_local = debug_encoder(rgba)
                            if jpg_local:
                                local_path = os.path.join(os.path.dirname(__file__), "_local_render_buffer.jpg")
                                with open(local_path, "wb") as fp:
                                    fp.write(jpg_local)
                                mean_luma = float(np.mean(rgba[:, :, :3]))
                                print(f"[local-render-buffer] saved={local_path} mean_luma={mean_luma:.2f}")
                    except Exception as e:
                        print(f"[local-render-buffer] save failed: {e}")
                # 验证 shape 与配置分辨率一致，不一致时 resize 避免花屏
                if rgba.shape[:2] != (H, W):
                    rgba = np.ascontiguousarray(
                        np.resize(rgba, (H, W, rgba.shape[2] if rgba.ndim == 3 else 4))
                    )
                _rep_frame_health = _render_capture_rgb_frame_health(rgba)
                _rep_rgb_mean = _rep_frame_health.get("full_mean")
                _vp_rgb_mean = None
                _rtsp_viewport_primary_now = _rtsp_viewport_primary_enabled()
                _frame_source = "viewport_delegate_primary" if _rtsp_viewport_primary_now else "replicator"
                _fallback_reason = None
                _probe_replicator_this_tick = rtsp_probe_replicator_this_tick

                # OOM / RTX 分叉后 annotator 可能长期返回近纯 0；连续多帧则最小自愈（重绑 render product，不暴露新 HTTP 接口）
                try:
                    rgb_ch = rgba[:, :, :3]
                    _mx = int(np.max(rgb_ch))
                    _mn = int(np.min(rgb_ch))
                    _lm = float(np.mean(rgb_ch))
                    _raw_stats = _numpy_buffer_diag_stats(rgba_raw)
                    _recover_state = _render_recover_state()
                    _randomize_stabilizing = _randomize_render_stabilizing()
                    _is_nb = _mx <= 2 and _mn == 0 and _lm < 0.35
                    _is_fb = _mx == 0 and _mn == 0 and _lm < 1e-6
                    if _is_nb:
                        _kind = "full_black" if _is_fb else "near_black"
                        _channel_stats = _render_capture_near_black_channel_stats(rgba_raw)
                        print(
                            "[render-capture-diag] NEAR_BLACK "
                            f"capture_seq={capture_seq} "
                            f"kind={_kind} norm_max={_mx} norm_min={_mn} norm_mean={_lm:.6f} "
                            f"randomize_stabilizing={_randomize_stabilizing} "
                            f"recover_in_progress={_recover_state['in_progress']} "
                            f"recover_cooldown_remaining_s={_recover_state['cooldown_remaining_s']:.1f} "
                            f"raw_stats={_raw_stats} channel_stats={_channel_stats} "
                            f"{_snapshot_render_capture_identity('main_loop')}",
                            flush=True,
                        )
                    if _mx <= 2 and _mn == 0 and _lm < 0.35:
                        if _randomize_stabilizing:
                            _capture_black_streak = 0
                        else:
                            _capture_black_streak += 1
                    else:
                        _capture_black_streak = 0
                    if _capture_black_streak >= _NEAR_BLACK_RECOVER_CONSECUTIVE:
                        _capture_black_streak = 0
                        if _randomize_stabilizing:
                            print(
                                "[snapshot-chain] near_black_recover_suppressed "
                                f"reason=randomize_stabilize_window window_s={_RANDOMIZE_RENDER_STABILIZE_WINDOW_S:.1f}",
                                flush=True,
                            )
                        elif _recover_state["in_progress"]:
                            print(
                                "[snapshot-chain] near_black_recover_suppressed "
                                "reason=recover_in_progress",
                                flush=True,
                            )
                        elif _recover_state["cooldown_remaining_s"] > 0.0:
                            print(
                                "[snapshot-chain] near_black_recover_suppressed "
                                f"reason=recover_cooldown cooldown_remaining_s={_recover_state['cooldown_remaining_s']:.1f}",
                                flush=True,
                            )
                        else:
                            recover_attempted_this_tick = True
                            print(
                                "[snapshot-chain] consecutive_near_black_burst "
                                f"max={_mx} min={_mn} mean={_lm:.4f} -> recover_render_capture",
                                flush=True,
                            )
                            _recover_ok, rp, annotator, _recover_err = _recover_render_capture(
                                world,
                                rp,
                                annotator,
                                reason="annotator_near_black_burst",
                                camera_prim_path=camera_prim,
                                width=W,
                                height=H,
                            )
                            if not _recover_ok and _recover_err:
                                print(
                                    f"[snapshot-chain] rebind_camera_failed: {_recover_err}",
                                    flush=True,
                                )
                except Exception as _bf_exc:
                    print(f"[snapshot-chain] black-burst-detect skipped: {_bf_exc}", flush=True)

                if _probe_replicator_this_tick and not bool(_rep_frame_health.get("should_try_viewport_fallback")):
                    _rtsp_set_viewport_primary(False, "replicator_recovered")

                if (
                    (not _rtsp_viewport_primary_now)
                    and bool(_rep_frame_health.get("should_try_viewport_fallback"))
                    and not rtsp_recovery_now
                ):
                    _vp_np, _vp_path, _vp_err = _try_read_viewport_delegate_rgba_uint8_for_rtsp_camera_aligned()
                    _vp_u8 = _normalize_replicator_rgba_for_output(_vp_np) if _vp_np is not None else None
                    if _vp_u8 is not None and getattr(_vp_u8, "size", 0) > 0 and _vp_u8.shape[:2] != (H, W):
                        _vp_u8 = np.ascontiguousarray(
                            np.resize(_vp_u8, (H, W, _vp_u8.shape[2] if _vp_u8.ndim == 3 else 4))
                        )
                    _vp_frame_health = _render_capture_rgb_frame_health(_vp_u8)
                    _vp_rgb_mean = _vp_frame_health.get("full_mean")
                    if _vp_u8 is not None and bool(_vp_frame_health.get("healthy")):
                        rgba_raw = _vp_np
                        rgba = _vp_u8
                        _frame_source = "viewport_delegate_fallback"
                        _fallback_reason = "replicator_black_viewport_healthy"
                        _rtsp_set_viewport_primary(True, "replicator_black_viewport_healthy")
                        print(
                            "[render-capture-diag] VIEWPORT_FALLBACK_USED "
                            f"reason=replicator_black rep_reason={_rep_frame_health.get('black_reason')} "
                            f"rep_mean={_rep_rgb_mean} rep_nonzero_ratio={_rep_frame_health.get('full_nonzero_ratio')} "
                            f"vp_mean={_vp_rgb_mean} vp_nonzero_ratio={_vp_frame_health.get('full_nonzero_ratio')} "
                            f"vp_path={_vp_path} {_snapshot_render_capture_identity('viewport_fallback_used')}",
                            flush=True,
                        )
                    else:
                        _skip_reason = (
                            str(_vp_err)
                            if _vp_err
                            else (
                                "viewport_not_healthy"
                                if _vp_u8 is not None
                                else "viewport_no_pixels"
                            )
                        )
                        print(
                            "[render-capture-diag] VIEWPORT_FALLBACK_SKIPPED "
                            f"reason={_skip_reason} rep_reason={_rep_frame_health.get('black_reason')} "
                            f"rep_mean={_rep_rgb_mean} vp_mean={_vp_rgb_mean} "
                            f"vp_path={_vp_path} {_snapshot_render_capture_identity('viewport_fallback_skipped')}",
                            flush=True,
                        )

                _stream_diag_update(
                    render_capture_last_frame_source=_frame_source,
                    render_capture_last_fallback_reason=_fallback_reason,
                    render_capture_last_replicator_rgb_mean=_rep_rgb_mean,
                    render_capture_last_viewport_rgb_mean=_vp_rgb_mean,
                )

                _randomize_publish_block_active = _randomize_stream_guard_publish_block_active()
                if rgba is not None and getattr(rgba, "size", 0) > 0:
                    # ① snapshot 快照缓存与 MJPEG 预览共享同一张 JPEG。
                    # preview_enabled=false 时只按较低频率刷新 snapshot，避免恢复高频预览开销。
                    if _jpeg_encode_fn is not None:
                        need_snapshot = preview_enabled
                        if not need_snapshot:
                            now_snapshot = time.monotonic()
                            with _mjpeg_lock:
                                last_snapshot_ts = float(_snapshot_cache["ts"])
                                snapshot_empty = _snapshot_cache["jpeg"] is None
                            need_snapshot = snapshot_empty or (
                                now_snapshot - last_snapshot_ts >= snapshot_interval_s
                            )
                        if need_snapshot:
                            is_black = False
                            try:
                                rmax = int(np.max(rgba[:, :, :3]))
                                if rmax <= 2:
                                    is_black = True
                            except Exception:
                                pass

                            if is_black:
                                pass # skip caching black frame
                            else:
                                t0 = time.monotonic()
                                jpg = _jpeg_encode_fn(rgba)
                                t_jpeg_total += time.monotonic() - t0
                                if jpg:
                                    snap_fid, snap_cap_seq_out = _cache_good_snapshot_jpeg(
                                        jpg,
                                        capture_seq=int(capture_seq),
                                        mirror_mjpeg=preview_enabled,
                                    )
                                    jpg_used = jpg
                                    snapshot_written = True

                    # ② RTSP 推流（可选，供 VLC / NVR 外部接入）
                    # 优先 live viewport_delegate（与 HTTP snapshot 视口一致），失败再用 Replicator rgba。
                    if (not rtsp_primary_fast_source_active) and rtsp_enabled and _ffmpeg_proc is not None \
                            and _ffmpeg_proc.poll() is None:
                        t0 = time.monotonic()
                        rgba_for_rtsp = rgba
                        rtsp_src = "replicator"
                        rtsp_capture_epoch_s = None
                        if _rtsp_viewport_primary_enabled():
                            _vp_rtsp, _vp_rtsp_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
                            if _vp_rtsp is not None:
                                rgba_for_rtsp = _vp_rtsp
                                rtsp_src = "viewport_delegate_primary"
                                try:
                                    rtsp_capture_epoch_s = float((_vp_rtsp_meta or {}).get("capture_epoch_s"))
                                except Exception:
                                    rtsp_capture_epoch_s = None
                            elif str(_frame_source) == "viewport_delegate_fallback":
                                rgba_for_rtsp = rgba
                                rtsp_src = "viewport_delegate"
                        elif str(_frame_source) == "viewport_delegate_fallback":
                            rgba_for_rtsp = rgba
                            rtsp_src = "viewport_delegate"
                        else:
                            _vp_rtsp, _vp_rtsp_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
                            if _vp_rtsp is not None:
                                rgba_for_rtsp = _vp_rtsp
                                rtsp_src = "viewport_delegate"
                                try:
                                    rtsp_capture_epoch_s = float((_vp_rtsp_meta or {}).get("capture_epoch_s"))
                                except Exception:
                                    rtsp_capture_epoch_s = None
                        if _randomize_publish_block_active and _randomize_stream_guard_should_block_publish(
                            _render_capture_rgb_frame_health(rgba_for_rtsp)
                        ):
                            _stream_diag_update(randomize_stream_mode="frozen_last_good")
                        else:
                            if _rtsp_put_rgba_frame(
                                rgba_for_rtsp,
                                rtsp_src,
                                capture_epoch_s=rtsp_capture_epoch_s,
                            )[0]:
                                drop_count += 1
                        t_tobytes_total += time.monotonic() - t0

            else:
                # Replicator 无可用 rgba：RTSP 在主循环内独立回退 viewport（与 HTTP /snapshot 同源取帧能力，不经 HTTP 队列）
                if (
                    rtsp_enabled
                    and not rtsp_primary_fast_source_active
                    and _ffmpeg_proc is not None
                    and _ffmpeg_proc.poll() is None
                ):
                    _vp_sb, _vp_sb_meta = _try_rtsp_rgba_from_viewport_delegate_safe(return_meta=True)
                    if _vp_sb is not None:
                        try:
                            _t0sb = time.monotonic()
                            _randomize_publish_block_active = _randomize_stream_guard_publish_block_active()
                            _cap_epoch_s = None
                            try:
                                _cap_epoch_s = float((_vp_sb_meta or {}).get("capture_epoch_s"))
                            except Exception:
                                _cap_epoch_s = None
                            if _randomize_publish_block_active and _randomize_stream_guard_should_block_publish(
                                _render_capture_rgb_frame_health(_vp_sb)
                            ):
                                _stream_diag_update(randomize_stream_mode="frozen_last_good")
                            else:
                                if _rtsp_put_rgba_frame(
                                    _vp_sb,
                                    "viewport_delegate_standalone",
                                    capture_epoch_s=_cap_epoch_s,
                                )[0]:
                                    drop_count += 1
                            t_tobytes_total += time.monotonic() - _t0sb
                        except Exception:
                            pass

            # 同帧抓图诊断（capture_seq 与 /snapshot.jpg 写入对齐）
            try:
                _is_post_recover = bool(_post_recover_first_capture_pending)
                if _is_post_recover:
                    _post_recover_first_capture_pending = False
                _rebind_recent = (time.monotonic() - _LAST_RENDER_CAPTURE_BIND_MONO) < 2.0
                _rgba_for_diag = rgba if rgba is not None and getattr(rgba, "size", 0) > 0 else None
                _diag_row = _build_same_tick_pipeline_diag(
                    capture_seq=capture_seq,
                    monotonic_start=tick_mono_start,
                    rgba_raw=rgba_raw,
                    rgba_u8=_rgba_for_diag,
                    jpg_bytes=jpg_used,
                    snapshot_cache_written=snapshot_written,
                    snapshot_frame_id=snap_fid,
                    snapshot_cache_capture_seq=snap_cap_seq_out,
                    rp=rp,
                    annotator=annotator,
                    camera_prim_path=camera_prim,
                    whether_rebind_recently=_rebind_recent,
                    is_post_recover_first_frame=_is_post_recover,
                    recover_attempted_this_tick=recover_attempted_this_tick,
                    probe=False,
                )
                _same_tick_store_latest(_diag_row)
            except Exception as _same_tick_exc:
                print(f"[same-tick-diag] emit_failed={_same_tick_exc}", flush=True)

            # 每 5 秒打印一次主线程分段耗时统计
            now = time.monotonic()
            if now - last_stat_time >= 5.0:
                elapsed = now - last_stat_time
                render_fps = render_count / elapsed
                avg_step_ms    = (t_step_total  / render_count   * 1000) if render_count   else 0
                avg_get_ms     = (t_get_total   / capture_count  * 1000) if capture_count  else 0
                avg_jpeg_ms    = (t_jpeg_total  / capture_count  * 1000) if capture_count  else 0
                avg_tobytes_ms = (t_tobytes_total / capture_count * 1000) if capture_count else 0

                print(
                    f"[PTZ-RTSP][main] render_fps={render_fps:.1f}  "
                    f"step={avg_step_ms:.1f}ms  "
                    f"get_data={avg_get_ms:.1f}ms  "
                    f"tobytes={avg_tobytes_ms:.1f}ms  "
                    f"jpeg={avg_jpeg_ms:.1f}ms  "
                    f"drop={drop_count}  sim_frame={frame_idx}"
                )
                if rtsp_enabled:
                    ff_live = _ffmpeg_proc is not None and _ffmpeg_proc.poll() is None
                    _stream_diag_update(
                        ffmpeg_alive=ff_live,
                        mediamtx_started=bool(
                            _mediamtx_proc is not None or _port_in_use(_mtx_port)
                        ),
                    )
                    if ff_live and _rtsp_writer_last_ok_mono is not None:
                        _idle_w = now - float(_rtsp_writer_last_ok_mono)
                        if _idle_w > 30.0:
                            print(
                                "[PTZ-RTSP] WARN: stdin 已超过 30s 无成功写入；"
                                "ffmpeg 可能仍在运行但 mediamtx publisher 可能已失效",
                                flush=True,
                            )
                    if _ffmpeg_proc is not None and _ffmpeg_proc.poll() is not None:
                        print("[PTZ-RTSP] 检测到 ffmpeg 已退出，尝试重启 RTSP 推流...", flush=True)
                        try:
                            while True:
                                try:
                                    _frame_queue.get_nowait()
                                except queue.Empty:
                                    break
                            _ffmpeg_proc = _start_ffmpeg(rtsp_url, W, H, fps, bitrate)
                            _writer_target = _pipe_writer_low_latency if _RTSP_LOW_LATENCY_MODE else _pipe_writer
                            _writer_args = (_ffmpeg_proc, fps) if _RTSP_LOW_LATENCY_MODE else (_ffmpeg_proc,)
                            threading.Thread(
                                target=_writer_target,
                                args=_writer_args,
                                daemon=True,
                                name="frame-writer-low-latency" if _RTSP_LOW_LATENCY_MODE else "frame-writer",
                            ).start()
                        except Exception as _ff_ex:
                            print(f"[PTZ-RTSP] ffmpeg 重启失败：{_ff_ex}", flush=True)
                # 重置计数器
                t_step_total    = 0.0
                t_get_total     = 0.0
                t_jpeg_total    = 0.0
                t_tobytes_total = 0.0
                render_count    = 0
                capture_count   = 0
                drop_count      = 0
                last_stat_time  = now

        _update_ctrl_plane_degraded_main_thread_hint()
        if frame_idx % 30 == 0:
            _status_refresh_age_ms, _status_refresh_mode = _rtsp_update_stale_recovery()
            _status_refresh_skip = (
                _status_refresh_mode == "viewport_only_recovery"
                or (
                    _status_refresh_age_ms is not None
                    and _status_refresh_age_ms > _RTSP_STALE_WARN_MS
                )
            )
            if not _status_refresh_skip:
                try:
                    if (
                        time.monotonic() - float(_LAST_STATUS_SCENE_MAIN_REFRESH_MONO or 0.0)
                        >= _STATUS_SCENE_REFRESH_INTERVAL_S
                    ):
                        _refresh_status_http_scene_cache_main_thread(force=False)
                except Exception:
                    pass

        frame_idx += 1

    # 主循环退出：发哨兵让写入线程退出，再清理子进程
    _cleanup()
    sim_app.close()
    print("[PTZ-RTSP] 已退出。")


if __name__ == "__main__":
    main()
