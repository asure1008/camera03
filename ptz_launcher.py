#!/usr/bin/env python3
"""
PTZ 仿真 ONVIF 服务  ptz_launcher.py
======================================
始终运行的轻量 ONVIF 服务（默认端口 8080），GPU 占用接近 0%。
通过 Web UI 按需启动 / 停止 Isaac Sim 仿真进程（ptz_stream.py）。

使用方法：
    python3 ptz_launcher.py [--config ./ptz_config.yaml]

访问地址：
    http://localhost:8080/                       Web 控制面板（始终可用）
    POST http://localhost:8080/onvif/device_service  ONVIF 设备服务
    POST http://localhost:8080/onvif/media_service   ONVIF 媒体服务
    POST http://localhost:8080/onvif/ptz_service     ONVIF PTZ 服务
    GET  http://localhost:8080/onvif-snap.jpg         ONVIF 快照端点
    GET  ws://localhost:8080/ws                       WebSocket 视频流（JPEG 帧推送）
    POST http://localhost:8080/start              启动 Isaac Sim
    POST http://localhost:8080/stop               停止 Isaac Sim

架构说明：
    launcher（8080）← 代理 → Isaac Sim（8081）
    外部 ONVIF 客户端只需连接 8080，无需感知 8081。

坐标换算：
    pan_deg  = onvif_pan  × 170.0        ONVIF[-1,1] → Isaac[-170°,170°]
    tilt_deg = clamp(-60.0 × onvif_tilt - 30.0, -90°, 30°)
    zoom_x   = 1.0 + onvif_zoom × 31.0  ONVIF[0,1]  → Isaac[1×,32×]
"""

import argparse
import base64
import datetime
import errno
import hashlib
import http.client
import io
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
import urllib.request
import urllib.error
import uuid as _uuid_mod
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import yaml

# ── 配置 ─────────────────────────────────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser(description="PTZ Launcher - ONVIF simulation server")
ap.add_argument("--config", default=os.path.join(script_dir, "ptz_config.yaml"))
args, _ = ap.parse_known_args()

with open(args.config, "r", encoding="utf-8") as f:
    cfg = yaml.safe_load(f)

LAUNCHER_PORT = cfg.get("launcher_port", 8080)
ISAAC_PORT    = cfg.get("ctrl_port",     8081)
PYTHON_SH     = cfg.get("python_sh",
                         "/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh")
STREAM_SCRIPT = os.path.join(script_dir, "ptz_stream.py")
ISAAC_LOG     = os.path.join(script_dir, "isaac_stream.log")
CONFIG_PATH   = args.config

# Isaac 控制面「热路径」探测：短超时 + 硬上限，避免 Web 轮询把 launcher 线程拖死在超大 /status 或慢 JPEG 上
_ISAAC_LITE_HTTP_TIMEOUT_S = max(0.35, float(cfg.get("isaac_launcher_lite_http_timeout_s", 1.15)))
_ISAAC_STARTUP_CTRL_HTTP_TIMEOUT_S = max(
    _ISAAC_LITE_HTTP_TIMEOUT_S, float(cfg.get("isaac_startup_ctrl_http_timeout_s", 4.0))
)
_ISAAC_STATUS_BODY_MAX_BYTES = max(65536, int(cfg.get("isaac_status_body_max_bytes", 6291456)))
_ISAAC_STATUS_FETCH_TIMEOUT_CAP_S = max(0.5, float(cfg.get("isaac_status_fetch_timeout_cap_s", 2.0)))

# conda activate env_isaaclab 会通过 etc/conda/activate.d/setenv.sh source IsaacLab/_isaac_sim/setup_conda_env.sh，
# 注入 ISAAC_PATH / CARB_APP_PATH / EXP_PATH 以及大量 *_isaac_sim 的 LD_LIBRARY_PATH；仅清 PYTHONPATH 仍会让 Kit 走不完整运行时并依赖在线注册表 → exit 55。
_ISAACLAB_RUNTIME_ENV_KEYS: tuple[str, ...] = (
    "ISAAC_PATH",
    "CARB_APP_PATH",
    "EXP_PATH",
)
_LD_PATH_BAN_SUBSTR: tuple[str, ...] = (
    "/_isaac_sim/",
    "IsaacLab/_isaac_sim",
    "/.isaac_sim_unzip/",
)


def _strip_pathlist_with_banned(path_val: str, banned: tuple[str, ...]) -> str:
    if not path_val:
        return ""
    parts: list[str] = []
    for p in path_val.split(os.pathsep):
        if not p:
            continue
        if any(b in p for b in banned):
            continue
        parts.append(p)
    return os.pathsep.join(parts)


def _read_isaac_log_tail(max_lines: int = 48) -> list[str]:
    """启动失败时摘取 isaac_stream.log 末尾若干行（仅 ASCII/UTF-8 文本）。"""
    try:
        with open(ISAAC_LOG, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
        tail = lines[-max_lines:] if len(lines) > max_lines else lines
        return [ln.rstrip("\n") for ln in tail]
    except Exception:
        return []


_OOM_LOG_MARKERS: tuple[str, ...] = (
    "out of memory",
    "cuda error",
    "cuda oom",
    "ran out of memory",
    "oom killer",
    "killed process",
    "xid ",
    "nvrm",
    "segmentation fault",
    "vk::device::device lost",
)


def _isaac_log_oom_hints(max_scan_lines: int = 400, max_hints: int = 8) -> list[str]:
    """只读：从 isaac_stream.log 尾部扫描显存/OOM/驱动崩溃相关关键词，供诊断字段使用。"""
    hints: list[str] = []
    try:
        with open(ISAAC_LOG, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()[-max_scan_lines:]
        for ln in lines:
            low = ln.lower()
            if any(m in low for m in _OOM_LOG_MARKERS):
                hints.append(ln.rstrip("\n")[:500])
    except Exception:
        return []
    return hints[-max_hints:]


def _env_for_ptz_stream_subprocess() -> dict:
    """ptz_stream 子进程环境：剥离 IsaacLab _isaac_sim 注入，使用 conda site-packages 内完整 Isaac Sim。

    可通过 ptz_config.yaml 设置 isaac_child_preserve_isaaclab_runtime: true 恢复旧行为（仅调试）。
    """
    env = os.environ.copy()
    preserve = bool(cfg.get("isaac_child_preserve_isaaclab_runtime", False))
    if not preserve:
        for k in _ISAACLAB_RUNTIME_ENV_KEYS:
            env.pop(k, None)
        if not cfg.get("isaac_child_preserve_pythonpath", False):
            env.pop("PYTHONPATH", None)
        raw_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = _strip_pathlist_with_banned(raw_ld, _LD_PATH_BAN_SUBSTR)
    py_exe = os.path.abspath(PYTHON_SH)
    lib_dir = os.path.normpath(os.path.join(os.path.dirname(py_exe), "..", "lib"))
    if os.path.isdir(lib_dir):
        prev = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = lib_dir + (os.pathsep + prev if prev else "")
    return env


_PRESET_LEFT_PAN_OFFSET_DEG = float(cfg.get("preset_left_pan_offset_deg", 90.0))
_PRESET_RIGHT_PAN_OFFSET_DEG = float(cfg.get("preset_right_pan_offset_deg", -90.0))
_PRESET_OVERLOOK_TILT_DEG = float(cfg.get("preset_overlook_tilt_deg", 30.0))
# 与 ptz_stream._resolve_orientation_profile 中 default_initial 一致：启动/随机后「可见中性朝向」相对几何 look-at 的偏置
_DYNAMIC_STARTUP_PAN_OFFSET_DEG = float(cfg.get("dynamic_startup_pan_offset_deg", 0.0))
_DYNAMIC_STARTUP_TILT_OFFSET_DEG = float(cfg.get("dynamic_startup_tilt_offset_deg", 0.0))


def _read_stream_config_from_disk(config_path: str) -> dict:
    """从磁盘读取 ptz 推流相关字段（与 ptz_stream 启动时读取一致；用于诊断/确认样本 USD）。"""
    out: dict = {
        "scene_path": None,
        "scene_basename": None,
        "camera_prim": None,
        "rtsp_url": None,
        "preview_enabled": None,
        "renderer": None,
        "read_error": None,
    }
    try:
        cp = os.path.abspath(config_path)
        with open(cp, "r", encoding="utf-8") as f:
            y = yaml.safe_load(f) or {}
        sp = y.get("scene_path")
        if sp:
            if not os.path.isabs(str(sp)):
                sp = os.path.join(script_dir, str(sp))
            sp_abs = os.path.abspath(sp)
            out["scene_path"] = sp_abs
            out["scene_basename"] = os.path.basename(sp_abs)
        out["camera_prim"] = y.get("camera_prim")
        out["rtsp_url"] = y.get("rtsp_url")
        out["preview_enabled"] = y.get("preview_enabled")
        out["renderer"] = y.get("renderer")
    except Exception as e:
        out["read_error"] = str(e)
    return out


_PRESET_LIMIT = 5
_STARTUP_PRESET_TOKEN = "startup"
_STARTUP_PRESET_NAME = "StartupView"
_DEFAULT_PRESETS = {
    "1": {"name": "Home", "pan": 0.0, "tilt": -15.0, "zoom": 1.5},
    "2": {"name": "Front", "pan": 0.0, "tilt": -15.0, "zoom": 1.5},
    "3": {"name": "Right90", "pan": 90.0, "tilt": -15.0, "zoom": 1.5},
    "4": {"name": "Left90", "pan": -90.0, "tilt": -15.0, "zoom": 1.5},
    "5": {"name": "TopDown", "pan": 0.0, "tilt": 30.0, "zoom": 1.2},
}


def _default_preset_name(token: str) -> str:
    return {
        "1": "Home",
        "2": "Front",
        "3": "Right90",
        "4": "Left90",
        "5": "TopDown",
    }.get(str(token), f"Preset{token}")
_preset_lock = threading.Lock()


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _preset_token_order() -> list[str]:
    return [str(i) for i in range(1, _PRESET_LIMIT + 1)]


def _normalize_preset(token: str, item: dict | None) -> dict | None:
    if token not in _preset_token_order() or not isinstance(item, dict):
        return None
    return {
        "token": token,
        "name": str(item.get("name") or _default_preset_name(token)),
        "pan": round(_clamp(float(item.get("pan", 0.0)), -170.0, 170.0), 3),
        "tilt": round(_clamp(float(item.get("tilt", -45.0)), -90.0, 30.0), 3),
        "zoom": round(_clamp(float(item.get("zoom", 1.0)), 1.0, 32.0), 3),
    }


def _load_presets_from_cfg() -> dict[str, dict]:
    raw = cfg.get("presets") or {}
    presets: dict[str, dict] = {}
    if isinstance(raw, dict):
        for token in _preset_token_order():
            norm = _normalize_preset(token, raw.get(token))
            if norm is not None:
                presets[token] = norm
    if not presets:
        for token, item in _DEFAULT_PRESETS.items():
            presets[token] = _normalize_preset(token, item)
    return presets


_presets = _load_presets_from_cfg()


def _build_startup_preset() -> dict | None:
    status = _get_status_from_isaac()
    startup_view = status.get("startup_view") if isinstance(status, dict) else None
    if not isinstance(startup_view, dict):
        return None
    try:
        pan = float(startup_view["pan"])
        tilt = float(startup_view["tilt"])
        zoom = float(startup_view["zoom"])
    except Exception:
        return None
    return {
        "token": _STARTUP_PRESET_TOKEN,
        "name": str(startup_view.get("name") or _STARTUP_PRESET_NAME),
        "pan": round(_clamp(pan, -170.0, 170.0), 3),
        "tilt": round(_clamp(tilt, -90.0, 30.0), 3),
        "zoom": round(_clamp(zoom, 1.0, 32.0), 3),
        "source": "startup_view",
        "readonly": True,
    }


def _persist_presets() -> None:
    serializable = {
        token: {
            "name": item["name"],
            "pan": item["pan"],
            "tilt": item["tilt"],
            "zoom": item["zoom"],
        }
        for token, item in sorted(_presets.items(), key=lambda pair: int(pair[0]))
    }
    cfg["presets"] = serializable
    # 必须合并写入：若用启动时内存里的整份 cfg 覆盖磁盘，会把用户已改的 scene_path 等字段打回旧值。
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            disk_cfg = yaml.safe_load(f) or {}
    except Exception:
        disk_cfg = {}
    if not isinstance(disk_cfg, dict):
        disk_cfg = {}
    disk_cfg["presets"] = serializable
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        yaml.safe_dump(disk_cfg, f, allow_unicode=True, sort_keys=False)


def _list_presets() -> list[dict]:
    with _preset_lock:
        items = [dict(_presets[token]) for token in _preset_token_order() if token in _presets]
    startup_item = _build_startup_preset()
    if startup_item is not None:
        items.append(startup_item)
    return items


def _get_preset(token: str) -> dict | None:
    if str(token) == _STARTUP_PRESET_TOKEN:
        return _build_startup_preset()
    with _preset_lock:
        item = _presets.get(token)
        return dict(item) if item else None


def _save_preset(token: str | None, name: str | None, ptz: dict) -> dict:
    with _preset_lock:
        use_token = token
        if use_token is None:
            for candidate in _preset_token_order():
                if candidate not in _presets:
                    use_token = candidate
                    break
            if use_token is None:
                use_token = "1"
        current = _presets.get(use_token, {})
        preset = _normalize_preset(use_token, {
            "name": name or current.get("name") or _default_preset_name(use_token),
            "pan": ptz["pan"],
            "tilt": ptz["tilt"],
            "zoom": ptz["zoom"],
        })
        _presets[use_token] = preset
        _persist_presets()
        return dict(preset)


def _delete_preset(token: str) -> bool:
    if str(token) == _STARTUP_PRESET_TOKEN:
        return False
    with _preset_lock:
        existed = token in _presets
        if existed:
            _presets.pop(token, None)
            _persist_presets()
        return existed


def _preset_to_onvif_xml(preset: dict) -> str:
    return f"""  <tptz:Preset token="{preset['token']}">
    <tt:Name>{preset['name']}</tt:Name>
    <tt:PTZPosition>
      <tt:PanTilt x="{round(_pan_to_norm(preset['pan']), 4)}" y="{round(_tilt_to_norm(preset['tilt']), 4)}" space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{round(_zoom_to_norm(preset['zoom']), 4)}" space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:PTZPosition>
  </tptz:Preset>"""


def _soap_fault(reason: str, code: str = "ter:InvalidArgVal") -> bytes:
    return _soap_wrap(f"""<s:Fault>
  <s:Code>
    <s:Value>s:Sender</s:Value>
    <s:Subcode><s:Value>{code}</s:Value></s:Subcode>
  </s:Code>
  <s:Reason><s:Text xml:lang="zh-CN">{reason}</s:Text></s:Reason>
</s:Fault>""")

# ── Isaac Sim 子进程管理 ─────────────────────────────────────────────
_proc_lock  = threading.Lock()
_isaac_proc: subprocess.Popen | None = None
_start_time: float | None = None
_isaac_state = "stopped"   # "stopped" | "starting" | "running" | "stopping"
_STATUS_CACHE_TTL_S = 0.5
_SNAPSHOT_PROXY_TTL_S = 0.25
_SNAPSHOT_PROXY_STALE_TTL_S = 2.0
_status_cache = {"ts": 0.0, "data": None}
_status_cache_lock = threading.Lock()
_SNAPSHOT_META_HEADERS = (
    "X-PTZ-Snapshot-Ready",
    "X-PTZ-Snapshot-Frame-Id",
    "X-PTZ-Snapshot-Capture-Seq",
    "X-PTZ-Snapshot-Capture-Epoch-Ms",
    "X-PTZ-Snapshot-Capture-Iso",
    "X-PTZ-Snapshot-OSD-Text",
    "X-PTZ-Snapshot-Pixel-Source",
    "X-PTZ-Snapshot-Frame-Age-Ms",
    "X-PTZ-Snapshot-Randomize-Mode",
    "X-PTZ-Snapshot-Bypass-Last-Good",
)
_snapshot_proxy_cache = {"ts": 0.0, "jpeg": None, "source": None, "headers": {}}
_snapshot_proxy_lock = threading.Lock()
_snapshot_proxy_fetch_lock = threading.Lock()

_ISAAC_STARTUP_READY_TIMEOUT_S = float(cfg.get("isaac_startup_ready_timeout_s", 420.0))
_ISAAC_STARTUP_READY_CONSECUTIVE = max(1, int(cfg.get("isaac_startup_ready_consecutive", 3)))
_ISAAC_SNAPSHOT_READY_MIN_BYTES = max(512, int(cfg.get("isaac_snapshot_ready_min_bytes", 2048)))
_isaac_startup_streak = 0


def _http_get_isaac_ex(path: str, timeout: float = 5.0) -> tuple[int | None, bytes, str | None]:
    """GET Isaac 控制面；第三元为失败分类（成功时为 None），便于区分超时/拒连/非 200。"""
    try:
        conn = http.client.HTTPConnection("127.0.0.1", ISAAC_PORT, timeout=timeout)
        conn.request("GET", path)
        resp = conn.getresponse()
        status = int(resp.status)
        data = resp.read()
        conn.close()
        if status != 200:
            return status, data, f"http_status_{status}"
        return status, data, None
    except socket.timeout:
        return None, b"", "socket_timeout"
    except ConnectionRefusedError:
        return None, b"", "connection_refused"
    except OSError as e:
        en = getattr(e, "errno", None)
        if en in (errno.ECONNRESET, errno.ECONNABORTED, errno.EPIPE):
            return None, b"", f"oserrno_{en}"
        return None, b"", f"oserror:{type(e).__name__}:{e}"
    except http.client.HTTPException as e:
        return None, b"", f"http_exception:{type(e).__name__}:{e}"
    except Exception as e:
        return None, b"", f"{type(e).__name__}:{e}"


def _isaac_fetch_health_stream_parallel(
    timeout_s: float,
) -> tuple[tuple[int | None, bytes, str | None], tuple[int | None, bytes, str | None]]:
    """并行 GET 8081 /api/health 与 /api/stream_ready，避免串行超时叠加。"""
    t = float(timeout_s)
    join_cap = min(t + 0.45, _ISAAC_LITE_HTTP_TIMEOUT_S + 0.55)
    with ThreadPoolExecutor(max_workers=2) as pool:
        fut_h = pool.submit(_http_get_isaac_ex, "/api/health", t)
        fut_s = pool.submit(_http_get_isaac_ex, "/api/stream_ready", t)
        try:
            h = fut_h.result(timeout=join_cap)
        except Exception as exc:
            h = (None, b"", f"join_err:{type(exc).__name__}:{exc}")
        try:
            s = fut_s.result(timeout=join_cap)
        except Exception as exc:
            s = (None, b"", f"join_err:{type(exc).__name__}:{exc}")
    return h, s


def _parse_upstream_health_ctrl_hints(body: bytes) -> tuple[bool | None, str | None]:
    """解析 8081 /api/health JSON：degraded / reason（失败时返回 None, None）。"""
    if not body:
        return None, None
    try:
        j = json.loads(body.decode("utf-8", errors="replace"))
    except Exception:
        return None, None
    if not isinstance(j, dict):
        return None, None
    deg = j.get("degraded")
    if deg is None:
        return None, None
    reason = str(j.get("reason") or "").strip() or None
    return bool(deg), reason


def _isaac_ctrl_degraded_probe_detail(http_st: int | None, err: str | None) -> dict:
    """仅从 /ptz_state 探测结果细分 degraded，不额外发 HTTP；不改 start/stop 主逻辑。"""
    sub: list[str] = []
    er = str(err or "")
    el = er.lower()
    if http_st == 503 or "http_status_503" in er:
        sub.append("upstream_503_busy")
    elif isinstance(http_st, int) and http_st != 200 and http_st is not None:
        sub.append(f"upstream_http_{http_st}")
    if err == "socket_timeout" or ("timeout" in el):
        sub.append("read_timeout")
        sub.append("connect_timeout")
    if "connection_refused" in el or err == "connection_refused":
        sub.append("connection_refused")
    if not sub:
        sub.append("port_listening_but_http_unreachable")
    return {
        "ptz_state_http": http_st,
        "ptz_state_err": err,
        "subcodes": list(dict.fromkeys(sub)),
    }


def _http_get_isaac(path: str, timeout: float = 5.0) -> tuple[int | None, bytes]:
    st, data, _err = _http_get_isaac_ex(path, timeout)
    return st, data


def _http_get_isaac_json_small(path: str, timeout: float, max_bytes: int = 262144) -> dict | None:
    """GET 小 JSON（如 /ptz_state）；短超时 + 有界读，避免慢控制面阻塞 launcher。"""
    try:
        eff = min(max(0.25, float(timeout)), _ISAAC_STATUS_FETCH_TIMEOUT_CAP_S)
        conn = http.client.HTTPConnection("127.0.0.1", ISAAC_PORT, timeout=eff)
        conn.request("GET", path)
        resp = conn.getresponse()
        status = int(resp.status)
        body = resp.read(max_bytes + 1)
        conn.close()
        if status != 200 or len(body) > max_bytes:
            return None
        out = json.loads(body.decode("utf-8", errors="replace"))
        return out if isinstance(out, dict) else None
    except Exception:
        return None


def _isaac_api_path_ok(path: str, timeout: float) -> bool:
    """GET JSON 路径并判断顶层 ok==True（用于 /api/health、/api/stream_ready）。"""
    st, body = _http_get_isaac(path, timeout)
    if st != 200:
        return False
    try:
        j = json.loads(body.decode("utf-8", errors="replace"))
        return j.get("ok") is True
    except Exception:
        return False


def _is_isaac_api_health_ok() -> bool:
    return _isaac_api_path_ok("/api/health", _ISAAC_LITE_HTTP_TIMEOUT_S)


def _is_isaac_scene_state_ok() -> bool:
    """与 launcher 代理 /scene/state 对齐：子进程控制面须能返回 200。"""
    st, _body, _err = _http_get_isaac_ex(
        "/scene/state", min(2.0, _ISAAC_LITE_HTTP_TIMEOUT_S + 0.85)
    )
    return st == 200


def _external_ctrl_min_healthy() -> tuple[bool, str]:
    """8081 已被占用时：最小「健康外部」判定（listening + /api/health + /scene/state）。"""
    if not _port_in_use(ISAAC_PORT):
        return False, "port_not_listening"
    if not _is_isaac_api_health_ok():
        h_st, _b, h_err = _http_get_isaac_ex("/api/health", _ISAAC_LITE_HTTP_TIMEOUT_S)
        return False, f"api_health_fail http={h_st} err={h_err!r}"
    if not _is_isaac_scene_state_ok():
        s_st, _b, s_err = _http_get_isaac_ex(
            "/scene/state", min(2.0, _ISAAC_LITE_HTTP_TIMEOUT_S + 0.85)
        )
        return False, f"scene_state_fail http={s_st} err={s_err!r}"
    return True, "listening_health_scene_ok"


def _isaac_child_operational_truth(proc: subprocess.Popen) -> tuple[bool, str]:
    """子进程句柄仍「存活」时，结合端口 + 健康 + scene/state 判断是否真在跑（避免假阳性）。"""
    pid = getattr(proc, "pid", None)
    poll = proc.poll()
    if poll is not None:
        return False, f"proc_exited pid={pid} rc={poll}"
    if not _port_in_use(ISAAC_PORT):
        return False, f"pid={pid} poll=None port={ISAAC_PORT}_not_listening"
    if not _is_isaac_api_health_ok():
        h_st, _b, h_err = _http_get_isaac_ex("/api/health", _ISAAC_LITE_HTTP_TIMEOUT_S)
        return False, f"pid={pid} api_health_fail http={h_st} err={h_err!r}"
    if not _is_isaac_scene_state_ok():
        s_st, _b, s_err = _http_get_isaac_ex(
            "/scene/state", min(2.0, _ISAAC_LITE_HTTP_TIMEOUT_S + 0.85)
        )
        return False, f"pid={pid} scene_state_fail http={s_st} err={s_err!r}"
    return True, f"pid={pid} port_ok health_ok scene_state_ok"


def _wait_prev_isaac_stop_complete(timeout_s: float = 90.0) -> tuple[bool, str]:
    """stop_isaac 异步收尾期间须阻塞 start，避免旧句柄+未释放资源误判。"""
    t0 = time.monotonic()
    last = ""
    while time.monotonic() - t0 < timeout_s:
        with _proc_lock:
            st = _isaac_state
            p = _isaac_proc
            poll = p.poll() if p is not None else None
            last = f"isaac_state={st!r} proc_pid={getattr(p,'pid',None)} poll={poll}"
        if st != "stopping":
            return True, last
        time.sleep(0.2)
    return False, f"timeout waiting stop; last={last}"


def _is_isaac_stream_init_ready() -> bool:
    """ptz_stream 主线程完成相机/吊篮首帧初始化后 /api/stream_ready 才返回 200。"""
    return _isaac_api_path_ok("/api/stream_ready", _ISAAC_LITE_HTTP_TIMEOUT_S)


def _is_isaac_snapshot_jpeg_ready() -> bool:
    st, body = _http_get_isaac("/snapshot.jpg", 12.0)
    if st != 200 or len(body) < _ISAAC_SNAPSHOT_READY_MIN_BYTES:
        return False
    return body[:2] == b"\xff\xd8"


def _is_isaac_http_ready() -> bool:
    """兼容旧逻辑：/status 首包 200（大 JSON，仅作弱信号）。"""
    try:
        conn = http.client.HTTPConnection("127.0.0.1", ISAAC_PORT, timeout=5)
        conn.request("GET", "/status")
        resp = conn.getresponse()
        status_ok = resp.status == 200
        _ = resp.read(4096)
        conn.close()
        return status_ok
    except Exception:
        return False


def _isaac_deemed_ready_for_launcher(preview_enabled: bool) -> bool:
    """端口 + /api/health + /api/stream_ready；避免把 health 卡在 snapshot 链路上。"""
    if not _port_in_use(ISAAC_PORT):
        return False
    # 启动/就绪判定：用较长 HTTP 超时，避免 Kit 首包慢导致「永远达不到连续就绪」
    t0 = float(_ISAAC_STARTUP_CTRL_HTTP_TIMEOUT_S)
    if not _isaac_api_path_ok("/api/health", t0):
        return False
    if not _isaac_api_path_ok("/api/stream_ready", t0):
        return False
    return True


def _launcher_api_health() -> dict:
    """对外统一健康检查（仅走 launcher，不强制拉取 Isaac 大 JSON）。"""
    port_busy = _port_in_use(ISAAC_PORT)
    preview = bool(cfg.get("preview_enabled", True))
    h_st, h_body, h_err = (None, b"", "skipped_port_free")
    s_st, s_body, s_err = (None, b"", "skipped_port_free")
    if port_busy:
        (h_st, h_body, h_err), (s_st, s_body, s_err) = _isaac_fetch_health_stream_parallel(
            _ISAAC_LITE_HTTP_TIMEOUT_S
        )

    def _json_ok_field(st: int | None, body: bytes) -> bool:
        if st != 200:
            return False
        try:
            j = json.loads(body.decode("utf-8", errors="replace"))
            return j.get("ok") is True
        except Exception:
            return False

    health_ok = _json_ok_field(h_st, h_body)
    stream_ok = _json_ok_field(s_st, s_body)
    ctrl_health_ok_no_stream = bool(port_busy and health_ok)
    snap_probe: dict | None = (
        {"skipped": True, "reason": "health_probe_avoids_snapshot"}
        if port_busy and preview
        else None
    )
    if port_busy and (h_err or s_err):
        print(
            "[PTZ-Launcher] isaac_ctrl_probe "
            f"health_http={h_st} health_err={h_err!r} "
            f"stream_ready_http={s_st} stream_ready_err={s_err!r} "
            f"snapshot_probe={snap_probe!r}",
            flush=True,
        )
    http_ready = bool(port_busy and health_ok and stream_ok)
    degraded_busy = bool(port_busy and health_ok and not stream_ok)
    up_deg, up_reason = _parse_upstream_health_ctrl_hints(h_body)
    with _proc_lock:
        proc0 = _isaac_proc
    proc_alive = proc0 is not None and proc0.poll() is None
    proc_alive_or_listener = bool(proc_alive or port_busy)
    ctrl_deg = bool(
        (port_busy and not http_ready)
        or degraded_busy
        or (up_deg is True)
        or (h_err in ("socket_timeout",) or (h_err and "timeout" in str(h_err).lower()))
        or (s_err in ("socket_timeout",) or (s_err and "timeout" in str(s_err).lower()))
    )
    deg_reason = None
    if up_reason:
        deg_reason = up_reason
    elif degraded_busy:
        deg_reason = "stream_not_ready"
    elif port_busy and not health_ok:
        deg_reason = f"api_health_fail:{h_err or h_st}"
    elif port_busy and health_ok and not stream_ok:
        deg_reason = f"stream_ready_fail:{s_err or s_st}"
    elif port_busy and not http_ready:
        deg_reason = "control_plane_not_ready"
    return {
        "ok": True,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
        "launcher": {"listen": "0.0.0.0", "port": LAUNCHER_PORT},
        "isaac_process_running": proc_alive_or_listener,
        "isaac_port_listening": port_busy,
        "isaac_ctrl_ready": http_ready,
        "isaac_ctrl_degraded": ctrl_deg,
        "isaac_ctrl_degraded_reason": deg_reason,
        "isaac": {
            "ctrl_port": ISAAC_PORT,
            "port_listening": port_busy,
            "http_ready": http_ready,
            "ctrl_health_ok": ctrl_health_ok_no_stream,
            "degraded_stream_not_ready": degraded_busy,
            "stream_init_ready": stream_ok if port_busy else False,
            "isaac_state": _isaac_state,
            "isaac_process_running": proc_alive_or_listener,
            "isaac_port_listening": port_busy,
            "isaac_ctrl_ready": http_ready,
            "isaac_ctrl_degraded": ctrl_deg,
            "isaac_ctrl_degraded_reason": deg_reason,
            "ctrl_probe": {
                "health_http": h_st,
                "health_err": h_err,
                "stream_ready_http": s_st,
                "stream_ready_err": s_err,
                "snapshot_jpeg_probe": snap_probe,
            },
        },
    }


def _get_isaac_status_cached(timeout: float = 90.0, max_age: float = _STATUS_CACHE_TTL_S) -> dict | None:
    now = time.monotonic()
    with _status_cache_lock:
        cached = _status_cache["data"]
        cached_ts = float(_status_cache["ts"])
        if isinstance(cached, dict) and now - cached_ts <= max_age:
            return dict(cached)
    eff_timeout = min(max(0.35, float(timeout)), _ISAAC_STATUS_FETCH_TIMEOUT_CAP_S)
    try:
        conn = http.client.HTTPConnection("127.0.0.1", ISAAC_PORT, timeout=eff_timeout)
        conn.request("GET", "/status")
        resp = conn.getresponse()
        status = int(resp.status)
        raw = resp.read(_ISAAC_STATUS_BODY_MAX_BYTES + 1)
        conn.close()
        if status != 200 or len(raw) > _ISAAC_STATUS_BODY_MAX_BYTES:
            return None
        data = json.loads(raw.decode("utf-8", errors="replace"))
        if not isinstance(data, dict):
            return None
    except Exception:
        return None
    with _status_cache_lock:
        _status_cache["data"] = data
        _status_cache["ts"] = time.monotonic()
    return dict(data)


def _read_snapshot_proxy_cache(max_age: float) -> tuple[bytes | None, str | None, dict]:
    now = time.monotonic()
    with _snapshot_proxy_lock:
        jpeg = _snapshot_proxy_cache["jpeg"]
        if jpeg is not None and now - float(_snapshot_proxy_cache["ts"]) <= max_age:
            return jpeg, _snapshot_proxy_cache["source"], dict(_snapshot_proxy_cache.get("headers") or {})
    return None, None, {}


def _cache_snapshot_proxy(jpeg: bytes, source: str, headers: dict | None = None) -> None:
    with _snapshot_proxy_lock:
        _snapshot_proxy_cache["jpeg"] = jpeg
        _snapshot_proxy_cache["source"] = source
        _snapshot_proxy_cache["headers"] = dict(headers or {})
        _snapshot_proxy_cache["ts"] = time.monotonic()


def _is_jpeg_bytes(data: bytes) -> bool:
    return bool(data) and len(data) >= 2 and data[:2] == b"\xff\xd8"


def _fetch_runtime_snapshot_jpeg(
    timeout_s: float = 3.0,
) -> tuple[bytes | None, str | None, dict, str | None]:
    """
    只读拉取 runtime(8081) 的 /snapshot.jpg。
    校验：HTTP 200 + body 非空 + JPEG magic bytes(FF D8)。
    失败时不抛异常，返回 (None, None, err_reason)。
    """
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{ISAAC_PORT}/snapshot.jpg", timeout=float(timeout_s)
        ) as r:
            data = r.read()
            status = getattr(r, "status", 200)
            headers = {
                name: r.headers.get(name)
                for name in _SNAPSHOT_META_HEADERS
                if r.headers.get(name) is not None
            }
    except Exception as e:
        return None, None, {}, f"fetch_error:{type(e).__name__}"
    if status != 200:
        return None, None, {}, f"upstream_http_{status}"
    if not data or len(data) < _ISAAC_SNAPSHOT_READY_MIN_BYTES:
        return None, None, {}, "upstream_empty_or_too_small"
    if not _is_jpeg_bytes(data):
        return None, None, {}, "upstream_not_jpeg"
    return data, "isaac-snapshot", headers, None


def _get_snapshot_proxy_data() -> tuple[bytes | None, str | None, dict]:
    cached_jpeg, cached_source, cached_headers = _read_snapshot_proxy_cache(_SNAPSHOT_PROXY_TTL_S)
    if cached_jpeg is not None:
        return cached_jpeg, cached_source or "launcher-snapshot-cache", cached_headers

    # 允许并发请求等待短时间，避免非阻塞锁导致的偶发 503
    acquired = False
    try:
        acquired = _snapshot_proxy_fetch_lock.acquire(timeout=0.8)
    except Exception:
        acquired = False

    if acquired:
        try:
            data, source, headers, _err = _fetch_runtime_snapshot_jpeg(timeout_s=3.0)
            if data is not None:
                _cache_snapshot_proxy(data, source or "isaac-snapshot", headers)
                return data, source or "isaac-snapshot", headers
        finally:
            try:
                _snapshot_proxy_fetch_lock.release()
            except Exception:
                pass
    else:
        # 其他线程可能正在拉取，尝试再读一次缓存
        cached_jpeg2, cached_source2, cached_headers2 = _read_snapshot_proxy_cache(_SNAPSHOT_PROXY_TTL_S)
        if cached_jpeg2 is not None:
            return cached_jpeg2, cached_source2 or "launcher-snapshot-cache", cached_headers2

    stale_jpeg, stale_source, stale_headers = _read_snapshot_proxy_cache(_SNAPSHOT_PROXY_STALE_TTL_S)
    if stale_jpeg is not None:
        return stale_jpeg, stale_source or "launcher-snapshot-cache", stale_headers
    return None, None, {}


def _send_snapshot_meta_headers(handler, headers: dict | None, *, skip: set[str] | None = None) -> None:
    if not isinstance(headers, dict):
        return
    skip_norm = {str(x).lower() for x in (skip or set())}
    for name in _SNAPSHOT_META_HEADERS:
        if name.lower() in skip_norm:
            continue
        value = headers.get(name)
        if value is None:
            continue
        try:
            handler.send_header(name, str(value))
        except Exception:
            pass


def _terminate_isaac_spawn(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    deadline = time.time() + 14.0
    while time.time() < deadline and proc.poll() is None:
        time.sleep(0.2)
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def _wait_isaac_post_spawn_ready(
    proc: subprocess.Popen | None, preview_enabled: bool
) -> tuple[bool, str, float]:
    """阻塞直到 Isaac 稳定就绪；需连续若干次满足端口 + /api/health + /api/stream_ready。"""
    t0 = time.monotonic()
    deadline = t0 + _ISAAC_STARTUP_READY_TIMEOUT_S
    consecutive = 0
    gap = 0.45
    while time.monotonic() < deadline:
        if proc is not None and proc.poll() is not None:
            code = proc.poll()
            return False, f"Isaac 子进程在就绪等待期退出 exit_code={code}", time.monotonic() - t0
        if _isaac_deemed_ready_for_launcher(preview_enabled):
            consecutive += 1
            if consecutive >= _ISAAC_STARTUP_READY_CONSECUTIVE:
                return True, "stable_ready", time.monotonic() - t0
        else:
            consecutive = 0
        time.sleep(gap)
    tail = (
        "限时内未达到稳定就绪：需连续满足 TCP 监听 + GET /api/health(200) + GET /api/stream_ready(200)"
        + f"；详见 {ISAAC_LOG}"
    )
    return False, tail, time.monotonic() - t0


def _watch_isaac(proc: subprocess.Popen) -> None:
    """启动就绪已在 start_isaac 内同步探测；此处仅监控子进程是否存活。"""
    global _isaac_state, _isaac_proc
    while True:
        rc = proc.poll()
        if rc is not None:
            with _proc_lock:
                if _isaac_proc is proc:
                    _isaac_proc = None
                if _isaac_state != "stopping":
                    _isaac_state = "stopped"
            print(
                f"[PTZ-Launcher] _watch_isaac: 子进程退出 pid={proc.pid} returncode={rc} "
                f"(stopping 中则不抢改 isaac_state，仅回收句柄)",
                flush=True,
            )
            return
        time.sleep(2)


def start_isaac(force_restart: bool = False) -> dict:
    global _isaac_proc, _start_time, _isaac_state, _isaac_startup_streak
    spawned_proc: subprocess.Popen | None = None
    reuse_external = False
    existing_pid = -1

    ok_wait, wait_detail = _wait_prev_isaac_stop_complete(90.0)
    if not ok_wait:
        print(f"[PTZ-Launcher] start_isaac: 等待 stop 收尾失败: {wait_detail}", flush=True)
        return {
            "ok": False,
            "error": "上一路 Isaac stop 尚未完成，请稍后重试",
            "detail": wait_detail,
            "ctrl_port": ISAAC_PORT,
        }

    # 假阳性：poll 仍 None 但 8081/健康/scene 不可用 → 终止并回收后再 spawn
    stale_to_kill: subprocess.Popen | None = None
    with _proc_lock:
        if _isaac_proc is not None and _isaac_proc.poll() is not None:
            _isaac_proc = None

        if _isaac_proc is not None and _isaac_proc.poll() is None:
            if _isaac_state == "starting":
                print(
                    "[PTZ-Launcher] start_isaac: 另一路启动尚未完成，拒绝并发 start",
                    flush=True,
                )
                return {"ok": False, "error": "Isaac Sim 正在启动中"}
            truth_ok, truth_why = _isaac_child_operational_truth(_isaac_proc)
            if truth_ok:
                print(
                    "[PTZ-Launcher] start_isaac: 判定已在运行 "
                    f"({_isaac_child_operational_truth.__name__}: {truth_why})",
                    flush=True,
                )
                return {"ok": False, "error": "Isaac Sim 已在运行"}
            print(
                "[PTZ-Launcher] start_isaac: 进程对象残留但非真运行，将清理后重拉 "
                f"({truth_why})",
                flush=True,
            )
            stale_to_kill = _isaac_proc
            _isaac_proc = None
            _isaac_state = "stopped"

    if stale_to_kill is not None:
        print(
            f"[PTZ-Launcher] start_isaac: terminate 假运行子进程 pid={stale_to_kill.pid} "
            f"pre_terminate_poll={stale_to_kill.poll()}",
            flush=True,
        )
        _terminate_isaac_spawn(stale_to_kill)
        rc_after = stale_to_kill.poll()
        print(
            f"[PTZ-Launcher] start_isaac: 假运行清理后 pid={stale_to_kill.pid} "
            f"poll={rc_after} returncode={stale_to_kill.returncode}",
            flush=True,
        )
        with _proc_lock:
            if _isaac_proc is stale_to_kill:
                _isaac_proc = None
            _isaac_state = "stopped"

    with _proc_lock:
        # stop_isaac 异步收尾：子进程已退出但句柄尚未置 None 时，避免误判「已在运行」从而拒绝 spawn（8081 无人监听）。
        if _isaac_proc is not None and _isaac_proc.poll() is not None:
            _isaac_proc = None

        # 端口已被占用：force_restart / 坏残留须受控回收；健康外部可复用
        if _port_in_use(ISAAC_PORT):
            existing_pids = _get_listening_pids(ISAAC_PORT)
            existing_pid = existing_pids[0] if existing_pids else -1

            if force_restart:
                ok, msg, killed_pids = _force_kill_project_isaac_ctrl(ISAAC_PORT)
                if not ok:
                    print(f"[PTZ-Launcher] [force_restart] refusal: {msg} (pids={existing_pids})", flush=True)
                    return {
                        "ok": False,
                        "ctrl_port": ISAAC_PORT,
                        "port_busy": True,
                        "existing_pid": existing_pid,
                        "log_path": ISAAC_LOG,
                        "action_taken": "force_restart_refused_or_failed",
                        "note": msg,
                    }
                if _port_in_use(ISAAC_PORT):
                    pass
                else:
                    print(
                        f"[PTZ-Launcher] [force_restart] port freed; spawning new ptz_stream (killed={killed_pids}).",
                        flush=True,
                    )

            # 非「最小健康」的外部占口：视为坏残留，避免直接 spawn 子进程在内部 Errno 98 或 reuse 卡死
            if _port_in_use(ISAAC_PORT):
                min_ok, min_why = _external_ctrl_min_healthy()
                print(
                    f"[PTZ-Launcher] start_isaac: ctrl_port_precheck port={ISAAC_PORT} "
                    f"listeners={existing_pids} min_healthy={min_ok} detail={min_why}",
                    flush=True,
                )
                if not min_ok:
                    ok_kill, msg_kill, killed_bad = _force_kill_project_isaac_ctrl(ISAAC_PORT)
                    print(
                        f"[PTZ-Launcher] start_isaac: bad_residual_cleanup ok={ok_kill} "
                        f"msg={msg_kill!r} killed={killed_bad}",
                        flush=True,
                    )
                    if not ok_kill:
                        cmdlines = {p: _read_cmdline(p).strip() for p in existing_pids}
                        return {
                            "ok": False,
                            "error": "控制端口被占用且无法识别为本项目 ptz_stream，未自动杀进程",
                            "ctrl_port": ISAAC_PORT,
                            "port_busy": True,
                            "listeners_pids": existing_pids,
                            "listeners_cmdlines": cmdlines,
                            "action_taken": "port_precheck_refused",
                            "detail": msg_kill,
                            "log_path": ISAAC_LOG,
                        }
                    if _port_in_use(ISAAC_PORT):
                        cmdlines2 = {
                            p: _read_cmdline(p).strip() for p in _get_listening_pids(ISAAC_PORT)
                        }
                        return {
                            "ok": False,
                            "error": "尝试回收坏残留后端口仍被占用",
                            "ctrl_port": ISAAC_PORT,
                            "port_busy": True,
                            "listeners_cmdlines": cmdlines2,
                            "action_taken": "port_precheck_still_busy",
                            "log_path": ISAAC_LOG,
                        }
                    print(
                        "[PTZ-Launcher] start_isaac: bad_residual freed; will spawn new ptz_stream.",
                        flush=True,
                    )

            # 端口仍占用：健康外部实例 → 复用；须通过稳定就绪探测后才可标为 running
            if _port_in_use(ISAAC_PORT):
                print(
                    f"[PTZ-Launcher] Isaac Sim ctrl_port busy: reuse external (port={ISAAC_PORT}, pids={existing_pids}).",
                    flush=True,
                )

                _isaac_state = "starting"
                _start_time  = time.time()
                _isaac_proc  = None  # 无法持有旧进程句柄，置 None
                reuse_external = True
                existing_pid = existing_pids[0] if existing_pids else -1

        if not reuse_external:
            log_file = open(ISAAC_LOG, "w", encoding="utf-8", buffering=1)
            sc = _read_stream_config_from_disk(CONFIG_PATH)
            print("[PTZ-Launcher] ========== 本次 Isaac 子进程将加载（ptz_stream 读盘）==========", flush=True)
            print(f"[PTZ-Launcher] config_path={os.path.abspath(CONFIG_PATH)}", flush=True)
            print(f"[PTZ-Launcher] scene_path={sc.get('scene_path')}", flush=True)
            print(f"[PTZ-Launcher] scene_basename={sc.get('scene_basename')}", flush=True)
            print(f"[PTZ-Launcher] camera_prim={sc.get('camera_prim')}", flush=True)
            print(f"[PTZ-Launcher] renderer={sc.get('renderer')}", flush=True)
            if sc.get("read_error"):
                print(f"[PTZ-Launcher] 读取 YAML 异常: {sc['read_error']}", flush=True)

            config_abs = os.path.abspath(CONFIG_PATH)
            cmd = [
                PYTHON_SH, "-u", STREAM_SCRIPT,
                "--config", config_abs,
                "--ctrl-port", str(ISAAC_PORT),
            ]
            _ov = os.environ.get("PTZ_SCENE_OVERRIDE", "").strip()
            if _ov:
                cmd.extend(["--scene", os.path.abspath(_ov)])
                print(f"[PTZ-Launcher] PTZ_SCENE_OVERRIDE（可被磁盘 scene_path 覆盖）→ {_ov}", flush=True)

            _cam_ov = os.environ.get("PTZ_CAMERA_PRIM_OVERRIDE", "").strip()
            if _cam_ov:
                cmd.extend(["--camera", _cam_ov])
                print(f"[PTZ-Launcher] PTZ_CAMERA_PRIM_OVERRIDE 生效 → {_cam_ov}", flush=True)

            if sc.get("scene_path"):
                cmd.extend(["--scene", sc["scene_path"]])
                print(
                    f"[PTZ-Launcher] 子进程 --scene（与 YAML 一致，最终生效）={sc['scene_path']}",
                    flush=True,
                )

            spawned_proc = subprocess.Popen(
                cmd,
                stdout=log_file,
                stderr=log_file,
                cwd=script_dir,
                start_new_session=True,
                env=_env_for_ptz_stream_subprocess(),
            )
            _isaac_proc = spawned_proc
            _start_time  = time.time()
            _isaac_state = "starting"

    preview = bool(cfg.get("preview_enabled", True))
    ok_ready, ready_msg, ready_elapsed = _wait_isaac_post_spawn_ready(
        spawned_proc if not reuse_external else None, preview
    )
    _isaac_startup_streak = 0
    if not ok_ready:
        if spawned_proc is not None:
            _terminate_isaac_spawn(spawned_proc)
        if reuse_external and _port_in_use(ISAAC_PORT):
            ok_orphan, msg_orphan, killed_orphan = _force_kill_project_isaac_ctrl(ISAAC_PORT)
            print(
                f"[PTZ-Launcher] start_isaac: reuse_external_failed → cleanup project listeners "
                f"ok={ok_orphan} msg={msg_orphan!r} killed={killed_orphan}",
                flush=True,
            )
        with _proc_lock:
            if spawned_proc is not None and _isaac_proc is spawned_proc:
                _isaac_proc = None
            _isaac_state = "stopped"
        print(f"[PTZ-Launcher] Isaac 启动失败: {ready_msg}", flush=True)
        oom_hints = _isaac_log_oom_hints()
        if oom_hints:
            print(
                "[PTZ-Launcher] isaac_stream.log OOM/GPU 相关摘录:\n" + "\n".join(oom_hints),
                flush=True,
            )
        fail: dict = {
            "ok": False,
            "error": ready_msg,
            "ctrl_port": ISAAC_PORT,
            "port_busy": bool(reuse_external),
            "existing_pid": existing_pid if reuse_external else -1,
            "pid": spawned_proc.pid if spawned_proc is not None else -1,
            "log_path": ISAAC_LOG,
            "action_taken": "reused_external_failed" if reuse_external else "spawn_ready_timeout",
            "ready_wait_s": round(float(ready_elapsed), 2),
            "isaac_log_oom_hints": oom_hints,
        }
        if spawned_proc is not None and spawned_proc.poll() is not None:
            fail["isaac_exit_code"] = int(spawned_proc.returncode or 0)
        tail_lines = _read_isaac_log_tail(48)
        if tail_lines:
            fail["isaac_log_tail"] = tail_lines
            print(
                "[PTZ-Launcher] isaac_stream.log 末尾摘录:\n" + "\n".join(tail_lines[-12:]),
                flush=True,
            )
        return fail

    with _proc_lock:
        _isaac_state = "running"

    if reuse_external:
        return {
            "ok": True,
            "ctrl_port": ISAAC_PORT,
            "port_busy": True,
            "existing_pid": existing_pid,
            "pid": -1,
            "log_path": ISAAC_LOG,
            "action_taken": "reused_external_ready",
            "note": "外部占用 ctrl 端口：已通过稳定就绪探测（含 /api/stream_ready）。",
            "ready_wait_s": round(float(ready_elapsed), 2),
        }

    threading.Thread(target=_watch_isaac, args=(spawned_proc,), daemon=True).start()
    return {
        "ok": True,
        "ctrl_port": ISAAC_PORT,
        "port_busy": False,
        "existing_pid": -1,
        "pid": spawned_proc.pid,
        "log_path": ISAAC_LOG,
        "action_taken": "spawned",
        "ready_wait_s": round(float(ready_elapsed), 2),
    }


def _port_free(port: int) -> bool:
    """检测 TCP 端口是否已释放。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) != 0


def stop_isaac() -> dict:
    global _isaac_proc, _isaac_state
    with _proc_lock:
        if _isaac_proc is not None and _isaac_proc.poll() is None:
            pass  # 走下方托管停止
        else:
            if _isaac_proc is not None:
                _isaac_proc = None
            _isaac_state = "stopped"

    if _isaac_proc is None or _isaac_proc.poll() is not None:
        # 无存活托管子进程：处理「launcher 认为未运行但 8081 仍被旧 ptz_stream 占口」
        if not _port_in_use(ISAAC_PORT):
            return {
                "ok": False,
                "error": "Isaac Sim 未在运行",
                "ctrl_port": ISAAC_PORT,
                "port_listening": False,
            }
        pids = _get_listening_pids(ISAAC_PORT)
        cmdlines = {pid: _read_cmdline(pid).strip() for pid in pids}
        project_pids = [pid for pid in pids if _is_project_ptz_stream_pid(pid)]
        print(
            f"[PTZ-Launcher] stop_isaac: no_managed_proc but ctrl_port busy "
            f"pids={pids} project_pids={project_pids} cmdlines={cmdlines!r}",
            flush=True,
        )
        if not project_pids:
            return {
                "ok": False,
                "error": "控制端口已被占用，但监听者不是本项目 ptz_stream，未自动杀进程",
                "ctrl_port": ISAAC_PORT,
                "port_listening": True,
                "listeners_pids": pids,
                "listeners_cmdlines": cmdlines,
                "action_taken": "refused_unknown_listener",
            }
        ok_kill, msg_kill, killed = _force_kill_project_isaac_ctrl(ISAAC_PORT)
        oom_hints = _isaac_log_oom_hints()
        if oom_hints:
            print(
                "[PTZ-Launcher] stop_isaac: isaac_log_oom_hints=\n" + "\n".join(oom_hints),
                flush=True,
            )
        print(
            f"[PTZ-Launcher] stop_isaac: orphan_project_cleanup ok={ok_kill} "
            f"msg={msg_kill!r} killed={killed} port_still_busy={_port_in_use(ISAAC_PORT)}",
            flush=True,
        )
        return {
            "ok": ok_kill,
            "ctrl_port": ISAAC_PORT,
            "port_listening_after": _port_in_use(ISAAC_PORT),
            "action_taken": "orphan_project_ptz_stream_stopped" if ok_kill else "orphan_stop_failed",
            "killed_pids": killed,
            "detail": msg_kill,
            "isaac_log_oom_hints": oom_hints,
            "note": "launcher 未持有子进程句柄，但已按本项目 ptz_stream 证据回收占口进程。"
            if ok_kill
            else "回收失败，请检查权限或手工处理监听进程。",
        }

    with _proc_lock:
        if _isaac_proc is None or _isaac_proc.poll() is not None:
            _isaac_proc = None
            _isaac_state = "stopped"
            return {"ok": False, "error": "Isaac Sim 未在运行", "ctrl_port": ISAAC_PORT}

        _isaac_state = "stopping"
        pid  = _isaac_proc.pid
        proc = _isaac_proc
        try:
            pgid = os.getpgid(pid)
        except OSError:
            pgid = pid
        print(
            f"[PTZ-Launcher] stop_isaac: 进入 stopping pid={pid} pgid={pgid} "
            f"pre_SIGTERM_poll={proc.poll()} port8081_listening={_port_in_use(ISAAC_PORT)}",
            flush=True,
        )

    def _wait():
        global _isaac_proc, _isaac_state
        try:
            os.killpg(pgid, signal.SIGTERM)
        except (ProcessLookupError, OSError) as e:
            print(f"[PTZ-Launcher] stop_isaac[_wait]: SIGTERM killpg 跳过: {e!r}", flush=True)

        deadline = time.time() + 15
        while time.time() < deadline:
            if _port_free(ISAAC_PORT):
                break
            time.sleep(0.5)
        else:
            print(
                f"[PTZ-Launcher] stop_isaac[_wait]: SIGTERM 后 15s 内端口仍不可连 "
                f"pid={pid} poll={proc.poll()} port8081_free={_port_free(ISAAC_PORT)}",
                flush=True,
            )

        print(
            f"[PTZ-Launcher] stop_isaac[_wait]: SIGTERM 阶段结束 pid={pid} poll={proc.poll()} "
            f"returncode={proc.returncode} port8081_free={_port_free(ISAAC_PORT)}",
            flush=True,
        )

        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError) as e:
            print(f"[PTZ-Launcher] stop_isaac[_wait]: SIGKILL killpg 跳过: {e!r}", flush=True)

        try:
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ChildProcessError) as e:
            print(
                f"[PTZ-Launcher] stop_isaac[_wait]: proc.wait 异常 pid={pid} poll={proc.poll()} "
                f"returncode={proc.returncode} err={e!r}",
                flush=True,
            )

        for port in (ISAAC_PORT,):
            for _ in range(20):
                if _port_free(port):
                    break
                time.sleep(0.5)

        global _isaac_proc, _isaac_state
        with _proc_lock:
            _isaac_proc = None
            _isaac_state = "stopped"
        print(
            f"[PTZ-Launcher] stop_isaac[_wait]: 收尾完成 pid={pid} final_poll={proc.poll()} "
            f"final_returncode={proc.returncode} port8081_free={_port_free(ISAAC_PORT)} "
            "launcher 句柄已清空。",
            flush=True,
        )

    threading.Thread(target=_wait, daemon=True, name="isaac-stopper").start()
    return {"ok": True, "stopping_pid": pid}


def get_status() -> dict:
    """返回启动器状态 + 可选的 Isaac Sim PTZ 状态。"""
    global _isaac_state, _isaac_startup_streak, _start_time
    with _proc_lock:
        proc   = _isaac_proc
        state  = _isaac_state
        up_sec = int(time.time() - _start_time) if _start_time else 0
        dead   = (proc is not None and proc.poll() is not None)
        has_proc_handle = proc is not None

    port_alive = _port_in_use(ISAAC_PORT)

    # launcher 认为 stopped 但端口已被健康控制面占用：同步为 starting/running，避免 Web 误报「未运行」
    if state == "stopped" and not has_proc_handle and port_alive and _is_isaac_api_health_ok():
        preview = bool(cfg.get("preview_enabled", True))
        with _proc_lock:
            if _isaac_proc is None and _isaac_state == "stopped":
                if _isaac_deemed_ready_for_launcher(preview):
                    _isaac_state = "running"
                else:
                    _isaac_state = "starting"
                if _start_time is None:
                    _start_time = time.time()
        with _proc_lock:
            state = _isaac_state
            up_sec = int(time.time() - _start_time) if _start_time else 0

    if dead and state not in ("stopped", "stopping"):
        _isaac_state = "stopped"
        state = "stopped"
    elif not has_proc_handle and state in ("running", "starting") and not port_alive:
        _isaac_state = "stopped"
        state = "stopped"

    if state != "starting":
        _isaac_startup_streak = 0
    elif state == "starting" and port_alive:
        preview = bool(cfg.get("preview_enabled", True))
        if _isaac_deemed_ready_for_launcher(preview):
            _isaac_startup_streak += 1
            if _isaac_startup_streak >= _ISAAC_STARTUP_READY_CONSECUTIVE:
                _isaac_state = "running"
                state = "running"
                _isaac_startup_streak = 0
        else:
            _isaac_startup_streak = 0

    result = {
        "isaac_state": state,
        "isaac_port":  ISAAC_PORT,
        "uptime_s":    up_sec if state == "running" else 0,
        "ptz":         None,
        "config_path": os.path.abspath(CONFIG_PATH),
        "stream_config": _read_stream_config_from_disk(CONFIG_PATH),
        "note": "stream_config=磁盘当前值；切换 scene_path 后须 stop 再 start 方生效。"
                "ptz.stream=Isaac 进程内实际加载（仅 running 时有意义）。",
    }

    def _json_ok_body(st: int | None, body: bytes) -> bool:
        if st != 200:
            return False
        try:
            j = json.loads(body.decode("utf-8", errors="replace"))
            return j.get("ok") is True
        except Exception:
            return False

    proc_alive_launcher = bool(has_proc_handle and not dead)

    if state == "running":
        eff_ptz_to = min(0.95, _ISAAC_LITE_HTTP_TIMEOUT_S + 0.1)
        join_cap = max(eff_ptz_to, _ISAAC_LITE_HTTP_TIMEOUT_S) + 0.55
        with ThreadPoolExecutor(max_workers=3) as pool3:
            fut_ptz = pool3.submit(_http_get_isaac_ex, "/ptz_state", eff_ptz_to)
            fut_h = pool3.submit(_http_get_isaac_ex, "/api/health", _ISAAC_LITE_HTTP_TIMEOUT_S)
            fut_s = pool3.submit(_http_get_isaac_ex, "/api/stream_ready", _ISAAC_LITE_HTTP_TIMEOUT_S)
            try:
                st_ptz, raw_ptz, ptz_err = fut_ptz.result(timeout=join_cap)
            except Exception as exc:
                st_ptz, raw_ptz, ptz_err = None, b"", f"{type(exc).__name__}:{exc}"
            try:
                hst, hbody, herr = fut_h.result(timeout=join_cap)
            except Exception as exc:
                hst, hbody, herr = None, b"", f"{type(exc).__name__}:{exc}"
            try:
                sst, sbody, serr = fut_s.result(timeout=join_cap)
            except Exception as exc:
                sst, sbody, serr = None, b"", f"{type(exc).__name__}:{exc}"
        health_ok2 = _json_ok_body(hst, hbody)
        stream_ok2 = _json_ok_body(sst, sbody)
        ctrl_ready = bool(port_alive and health_ok2 and stream_ok2)
        up_deg2, up_reason2 = _parse_upstream_health_ctrl_hints(hbody)
        ctrl_deg2 = bool(
            (port_alive and not ctrl_ready)
            or (up_deg2 is True)
            or (herr in ("socket_timeout",) or (herr and "timeout" in str(herr).lower()))
            or (serr in ("socket_timeout",) or (serr and "timeout" in str(serr).lower()))
        )
        deg_reason2 = up_reason2 or (
            f"health={herr or hst} stream={serr or sst}" if port_alive and not ctrl_ready else None
        )
        result["isaac_process_running"] = bool(proc_alive_launcher or (port_alive and health_ok2))
        result["isaac_port_listening"] = bool(port_alive)
        result["isaac_ctrl_ready"] = ctrl_ready
        result["isaac_ctrl_degraded"] = ctrl_deg2
        result["isaac_ctrl_degraded_reason"] = deg_reason2

        ptz_j = None
        if st_ptz == 200 and raw_ptz and len(raw_ptz) <= 262144:
            try:
                _pj = json.loads(raw_ptz.decode("utf-8", errors="replace"))
                if isinstance(_pj, dict) and _pj:
                    ptz_j = _pj
            except Exception:
                ptz_j = None
        if isinstance(ptz_j, dict) and ptz_j:
            result["ptz"] = ptz_j
        else:
            status = _get_isaac_status_cached(
                timeout=_ISAAC_STATUS_FETCH_TIMEOUT_CAP_S, max_age=2.0
            )
            if status is not None:
                try:
                    result["ptz"] = {
                        "pan": float(status.get("pan", 0.0)),
                        "tilt": float(status.get("tilt", -15.0)),
                        "zoom": float(status.get("zoom", 1.5)),
                    }
                except Exception:
                    result["ptz"] = None
            elif not _port_in_use(ISAAC_PORT):
                _isaac_state = "stopped"
                result["isaac_state"] = "stopped"
            elif _port_in_use(ISAAC_PORT):
                result["isaac_ctrl_degraded"] = True
                result["isaac_ctrl_degraded_reason"] = "ptz_state_and_status_unreachable_but_port_listening"
                result["isaac_ctrl_degraded_detail"] = _isaac_ctrl_degraded_probe_detail(
                    st_ptz, ptz_err
                )
    elif port_alive or state == "starting":
        (hst, hbody, herr), (sst, sbody, serr) = _isaac_fetch_health_stream_parallel(
            _ISAAC_LITE_HTTP_TIMEOUT_S
        )
        health_ok2 = _json_ok_body(hst, hbody)
        stream_ok2 = _json_ok_body(sst, sbody)
        ctrl_ready = bool(port_alive and health_ok2 and stream_ok2)
        up_deg2, up_reason2 = _parse_upstream_health_ctrl_hints(hbody)
        ctrl_deg2 = bool(
            (port_alive and not ctrl_ready)
            or (up_deg2 is True)
            or (herr in ("socket_timeout",) or (herr and "timeout" in str(herr).lower()))
            or (serr in ("socket_timeout",) or (serr and "timeout" in str(serr).lower()))
        )
        deg_reason2 = up_reason2 or (
            f"health={herr or hst} stream={serr or sst}" if port_alive and not ctrl_ready else None
        )
        result["isaac_process_running"] = bool(proc_alive_launcher or (port_alive and health_ok2))
        result["isaac_port_listening"] = bool(port_alive)
        result["isaac_ctrl_ready"] = ctrl_ready
        result["isaac_ctrl_degraded"] = ctrl_deg2
        result["isaac_ctrl_degraded_reason"] = deg_reason2
    else:
        result["isaac_process_running"] = proc_alive_launcher
        result["isaac_port_listening"] = bool(port_alive)
        result["isaac_ctrl_ready"] = False
        result["isaac_ctrl_degraded"] = False
        result["isaac_ctrl_degraded_reason"] = None

    return result


# ══════════════════════════════════════════════════════════════════════
# WebSocket 视频流
# ══════════════════════════════════════════════════════════════════════

# 帧缓存：后台线程写，WS handler 线程读
_ws_cache      = {"jpeg": None, "frame_id": 0}
_ws_cache_lock = threading.Lock()

# 目标推流帧率（从 config 读取，默认 25）
_WS_FPS = 25


def _ws_frame_fetcher() -> None:
    """后台线程：以 _WS_FPS 速率轮询 Isaac Sim 快照并缓存最新帧。"""
    interval = 1.0 / max(1, _WS_FPS)
    while True:
        t0 = time.monotonic()
        # 只要 Isaac Sim 端口可达就尝试拉帧（不依赖 _isaac_state 是否同步）
        if _isaac_state == "running" or _port_in_use(ISAAC_PORT):
            try:
                with urllib.request.urlopen(
                    f"http://127.0.0.1:{ISAAC_PORT}/snapshot.jpg", timeout=8
                ) as r:
                    data = r.read()
                if data and len(data) > 1000:
                    with _ws_cache_lock:
                        _ws_cache["jpeg"]     = data
                        _ws_cache["frame_id"] += 1
            except Exception:
                pass
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, interval - elapsed))


def _ws_send(sock_file, data: bytes) -> None:
    """向 WebSocket 客户端发送一帧 binary frame（RFC 6455，服务端无掩码）。"""
    length = len(data)
    if length <= 125:
        header = bytes([0x82, length])
    elif length <= 65535:
        header = struct.pack("!BBH", 0x82, 126, length)
    else:
        header = struct.pack("!BBQ", 0x82, 127, length)
    sock_file.write(header + data)
    sock_file.flush()


# ══════════════════════════════════════════════════════════════════════
# ONVIF 轻量 SOAP 服务器
# ══════════════════════════════════════════════════════════════════════

_ONVIF_PROFILE_TOKEN = "Profile_1"
_ONVIF_PTZ_TOKEN     = "PTZConfig_1"
_ONVIF_PTZ_NODE      = "PTZNode_1"

# 线程局部变量：记录当前请求的 SOAP 版本，让 _soap_wrap 自动匹配
# True=SOAP 1.2 (application/soap+xml), False=SOAP 1.1 (text/xml)
_soap_tls = threading.local()

# 坐标换算常量
_PAN_SCALE  = 170.0
_TILT_SCALE = 90.0
_ZOOM_SCALE = 31.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _norm_to_pan(norm: float) -> float:
    return _clamp(norm, -1.0, 1.0) * _PAN_SCALE


def _pan_to_norm(pan_deg: float) -> float:
    return _clamp(pan_deg, -170.0, 170.0) / _PAN_SCALE


def _norm_to_tilt(norm: float) -> float:
    norm = _clamp(norm, -1.0, 1.0)
    return _clamp(-60.0 * norm - 30.0, -90.0, 30.0)


def _tilt_to_norm(tilt_deg: float) -> float:
    tilt_deg = _clamp(tilt_deg, -90.0, 30.0)
    return _clamp(-(2.0 * (tilt_deg - (-90.0)) / (30.0 - (-90.0)) - 1.0), -1.0, 1.0)


def _norm_to_zoom(norm: float) -> float:
    return 1.0 + _clamp(norm, 0.0, 1.0) * _ZOOM_SCALE


def _zoom_to_norm(zoom_x: float) -> float:
    return (_clamp(zoom_x, 1.0, 32.0) - 1.0) / _ZOOM_SCALE

# RTSP 端口（来自 mediamtx 配置，默认 8554）
_RTSP_PORT = cfg.get("mediamtx", {}).get("port", 8554)
# 可选：强制 ONVIF GetStreamUri 中的 RTSP 主机名（ODM 跨机时若 Host 为 localhost 可在此写局域网 IP）
_ONVIF_RTSP_HOST_OVERRIDE = str(cfg.get("onvif_rtsp_host") or "").strip()


def _make_offline_jpeg() -> bytes:
    """
    生成"仿真未启动"占位 JPEG。
    优先用 Pillow；不可用时回退内嵌最小 JPEG（8×8 灰色）。
    确保快照端点永远返回 200 而不是 503，VMS/ODM 才能正常显示 NVT 模块。
    """
    try:
        from PIL import Image as _Img, ImageDraw as _Draw, ImageFont as _Font
        img  = _Img.new("RGB", (640, 360), color=(20, 20, 20))
        draw = _Draw.Draw(img)
        draw.rectangle([0, 0, 640, 360], fill=(20, 20, 40))
        draw.text((200, 155), "Simulation Offline", fill=(180, 180, 180))
        draw.text((250, 185), "请在 Web UI 启动仿真", fill=(120, 120, 120))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=50)
        return buf.getvalue()
    except Exception:
        pass
    # 内嵌最小合法 JPEG（8×8 灰色像素，无外部依赖）
    return base64.b64decode(
        "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8U"
        "HRofHh0aHBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAAR"
        "CAAIAAgDASIAAhEBAxEB/8QAFgABAQEAAAAAAAAAAAAAAAAABgUE/8QAIhAA"
        "AgIDAQADAQAAAAAAAAAAAQIDBAUREiExQf/EABQBAQAAAAAAAAAAAAAAAAAA"
        "AAD/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwCw1jiuNZv3"
        "fVc5bU2rxuTq9NGlfCb9ZVJfGkRQA7tezA7G+rEfFmAAAAAAAAAAAAAAAAAA"
        "AAAAAAAAAAB//9k="
    )


# 模块启动时预生成离线占位图（非阻塞，尽早缓存）
_OFFLINE_JPEG: bytes = _make_offline_jpeg()

# ══════════════════════════════════════════════════════════════════════
# WS-Discovery：局域网 ONVIF 设备发现（UDP 多播 239.255.255.250:3702）
# ══════════════════════════════════════════════════════════════════════

_WSD_MCAST_ADDR = "239.255.255.250"
_WSD_PORT       = 3702

# 稳定 UUID：基于主机名 + 端口，重启后不变，客户端可识别同一设备
_DEVICE_UUID = str(_uuid_mod.uuid5(
    _uuid_mod.NAMESPACE_DNS,
    f"onvif-ptz-cam-{socket.gethostname()}-{LAUNCHER_PORT}"
))

_WSD_SCOPES = (
    "onvif://www.onvif.org/location/country/china "
    "onvif://www.onvif.org/name/Isaac-Sim-PTZ-Camera "
    "onvif://www.onvif.org/hardware/PTZ-Camera-V4 "
    "onvif://www.onvif.org/type/video_encoder "
    "onvif://www.onvif.org/Profile/Streaming"
)


def _get_local_ips() -> list:
    """获取本机所有非 loopback IPv4 地址列表（避免 hostname 解析卡死）。"""
    ips: list = []

    # 先用 UDP connect 获取默认出口 IP，通常不依赖 DNS
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
            _s.connect(("8.8.8.8", 80))
            ip = _s.getsockname()[0]
            if ip and not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except Exception:
        pass

    # 若没拿到，再做 hostname getaddrinfo（可能需要 DNS）
    if not ips:
        try:
            for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                ip = info[4][0]
                if ip and not ip.startswith("127.") and ip not in ips:
                    ips.append(ip)
        except Exception:
            pass

    return ips or ["127.0.0.1"]


def _get_sender_local_ip(remote_ip: str) -> str:
    """获取通往 remote_ip 所在网段的本地出口 IP。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
            _s.connect((remote_ip, 1))
            return _s.getsockname()[0]
    except Exception:
        ips = _get_local_ips()
        return ips[0] if ips else "127.0.0.1"


def _parse_host_header_host_only(host_hdr: str) -> str:
    """从 HTTP Host 头解析纯主机名（IPv4:port 不能用 split(':')[0]，否则会截成 '192'）。"""
    s = (host_hdr or "").strip()
    if not s:
        return "127.0.0.1"
    if s.startswith("["):
        end = s.find("]")
        return s[1:end] if end > 1 else s
    if ":" in s:
        return s.rsplit(":", 1)[0]
    return s


def _rtsp_host_for_onvif(http_host: str, client_ip: str) -> str:
    """ONVIF GetStreamUri 中 RTSP 主机：修正 Host 解析；环回时改用通往客户端的网卡 IP。"""
    if _ONVIF_RTSP_HOST_OVERRIDE:
        return _ONVIF_RTSP_HOST_OVERRIDE
    h = _parse_host_header_host_only(http_host)
    if h in ("127.0.0.1", "localhost", "::1"):
        if client_ip and not client_ip.startswith("127.") and client_ip not in ("::1", "::ffff:127.0.0.1"):
            try:
                return _get_sender_local_ip(client_ip)
            except Exception:
                pass
        ips = _get_local_ips()
        return ips[0] if ips else "127.0.0.1"
    return h


def _wsd_probe_match(msg_id: str, local_ip: str) -> bytes:
    """构建 WS-Discovery ProbeMatch 单播响应报文。"""
    xaddr    = f"http://{local_ip}:{LAUNCHER_PORT}/onvif/device_service"
    reply_id = str(_uuid_mod.uuid4())
    return (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<s:Envelope '
        'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
        'xmlns:wsd="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
        'xmlns:dn="http://www.onvif.org/ver10/network/wsdl">'
        '<s:Header>'
        '<wsa:Action>'
        'http://schemas.xmlsoap.org/ws/2005/04/discovery/ProbeMatches'
        '</wsa:Action>'
        f'<wsa:MessageID>urn:uuid:{reply_id}</wsa:MessageID>'
        f'<wsa:RelatesTo>{msg_id}</wsa:RelatesTo>'
        '<wsa:To>'
        'http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anonymous'
        '</wsa:To>'
        '</s:Header>'
        '<s:Body>'
        '<wsd:ProbeMatches>'
        '<wsd:ProbeMatch>'
        '<wsa:EndpointReference>'
        f'<wsa:Address>urn:uuid:{_DEVICE_UUID}</wsa:Address>'
        '</wsa:EndpointReference>'
        '<wsd:Types>dn:NetworkVideoTransmitter</wsd:Types>'
        f'<wsd:Scopes>{_WSD_SCOPES}</wsd:Scopes>'
        f'<wsd:XAddrs>{xaddr}</wsd:XAddrs>'
        '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
        '</wsd:ProbeMatch>'
        '</wsd:ProbeMatches>'
        '</s:Body>'
        '</s:Envelope>'
    ).encode()


def _wsd_hello(sock: socket.socket) -> None:
    """服务启动时向多播组广播 Hello，主动宣告设备上线。"""
    local_ips = _get_local_ips()
    xaddrs = " ".join(
        f"http://{ip}:{LAUNCHER_PORT}/onvif/device_service" for ip in local_ips
    )
    reply_id = str(_uuid_mod.uuid4())
    msg = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<s:Envelope '
        'xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
        'xmlns:wsa="http://schemas.xmlsoap.org/ws/2004/08/addressing" '
        'xmlns:wsd="http://schemas.xmlsoap.org/ws/2005/04/discovery" '
        'xmlns:dn="http://www.onvif.org/ver10/network/wsdl">'
        '<s:Header>'
        '<wsa:Action>'
        'http://schemas.xmlsoap.org/ws/2005/04/discovery/Hello'
        '</wsa:Action>'
        f'<wsa:MessageID>urn:uuid:{reply_id}</wsa:MessageID>'
        '<wsa:To>urn:schemas-xmlsoap-org:ws:2005:04:discovery</wsa:To>'
        '</s:Header>'
        '<s:Body>'
        '<wsd:Hello>'
        '<wsa:EndpointReference>'
        f'<wsa:Address>urn:uuid:{_DEVICE_UUID}</wsa:Address>'
        '</wsa:EndpointReference>'
        '<wsd:Types>dn:NetworkVideoTransmitter</wsd:Types>'
        f'<wsd:Scopes>{_WSD_SCOPES}</wsd:Scopes>'
        f'<wsd:XAddrs>{xaddrs}</wsd:XAddrs>'
        '<wsd:MetadataVersion>1</wsd:MetadataVersion>'
        '</wsd:Hello>'
        '</s:Body>'
        '</s:Envelope>'
    ).encode()
    try:
        sock.sendto(msg, (_WSD_MCAST_ADDR, _WSD_PORT))
    except Exception:
        pass


def _wsd_listener() -> None:
    """
    WS-Discovery UDP 多播监听线程。
    监听 239.255.255.250:3702，响应局域网 ONVIF 客户端的 Probe 请求。
    失败时仅打印警告，不影响 ONVIF HTTP 服务正常运行。
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except (AttributeError, OSError):
            pass  # 部分内核不支持 SO_REUSEPORT，忽略
        sock.bind(("", _WSD_PORT))

        # 加入 IPv4 多播组（所有网卡）
        mreq = struct.pack("4sL", socket.inet_aton(_WSD_MCAST_ADDR),
                           socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(5.0)

        print(f"[PTZ-Launcher] WS-Discovery 已启动（{_WSD_MCAST_ADDR}:{_WSD_PORT}），"
              f"设备 UUID={_DEVICE_UUID}")

        # 启动时广播 Hello，主动宣告上线
        _wsd_hello(sock)

        while True:
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                text = data.decode("utf-8", errors="replace")
                # 只处理 Probe 请求
                if "Probe" not in text or "ProbeMatch" in text:
                    continue

                # 解析 MessageID 和 Types
                root   = ET.fromstring(data)
                msg_id = ""
                for el in root.iter():
                    lname = el.tag.split("}")[-1] if "}" in el.tag else el.tag
                    if lname == "MessageID" and el.text:
                        msg_id = el.text.strip()
                        break

                # 检查 Types 过滤：若指定类型中不含 NVT 则跳过
                accept = True
                for el in root.iter():
                    lname = el.tag.split("}")[-1] if "}" in el.tag else el.tag
                    if lname == "Types" and el.text and el.text.strip():
                        accept = "NetworkVideoTransmitter" in el.text
                        break

                if not accept:
                    continue

                local_ip = _get_sender_local_ip(addr[0])
                response = _wsd_probe_match(msg_id, local_ip)
                sock.sendto(response, addr)

            except Exception:
                pass

    except OSError as exc:
        print(f"[PTZ-Launcher] ⚠ WS-Discovery 绑定失败（端口 {_WSD_PORT} 可能被占用）："
              f"{exc}  局域网自动发现不可用，仍可手动填写 IP 连接。")
    except Exception as exc:
        print(f"[PTZ-Launcher] ⚠ WS-Discovery 异常退出：{exc}")


# ══════════════════════════════════════════════════════════════════════
# WebSocket FLV+H.264 广播器
# ══════════════════════════════════════════════════════════════════════

def _port_in_use(port: int) -> bool:
    """检测 TCP 端口是否已被监听。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.5)
        if s.connect_ex(("127.0.0.1", port)) == 0:
            return True
    return bool(_get_listening_pids(port))


def _get_listening_pids(port: int) -> list[int]:
    """返回监听指定 TCP 端口的 pid 列表（尽力而为）。"""
    try:
        out = subprocess.check_output(
            ["lsof", "-t", f"-iTCP:{port}", "-sTCP:LISTEN", "-nP"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
        pids: list[int] = []
        for line in out.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                pids.append(int(line))
            except ValueError:
                continue
        return pids
    except Exception:
        return []


def _read_cmdline(pid: int) -> str:
    """读取 /proc/<pid>/cmdline（尽力而为），用于判断是否属于本项目 ptz_stream。"""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            b = f.read()
        return b.replace(b"\x00", b" ").decode("utf-8", errors="ignore")
    except Exception:
        return ""


def _proc_cwd_resolved(pid: int) -> str:
    try:
        return os.path.realpath(f"/proc/{pid}/cwd")
    except Exception:
        return ""


def _is_project_ptz_stream_pid(pid: int) -> bool:
    cmd = _read_cmdline(pid)
    script_abs = os.path.realpath(STREAM_SCRIPT)
    dir_abs = os.path.realpath(script_dir)
    # 启动 argv 含本项目 ptz_stream.py 绝对路径
    if script_abs in cmd or STREAM_SCRIPT in cmd:
        return True
    if "ptz_stream.py" not in cmd:
        return False
    # 兜底：脚本名匹配且 cwd 为本项目目录（避免误杀其它目录下同名的 python 任务）
    cwd = _proc_cwd_resolved(pid)
    if cwd == dir_abs:
        return True
    if script_dir in cmd or dir_abs in cmd:
        return True
    return False


def _force_kill_project_isaac_ctrl(port: int) -> tuple[bool, str, list[int]]:
    """当 force_restart=true 时，尽力杀掉本项目的 ptz_stream（只在监听者确认为本项目时）。"""
    pids = _get_listening_pids(port)
    if not pids:
        return (True, "port not listening anymore", [])

    project_pids = [pid for pid in pids if _is_project_ptz_stream_pid(pid)]
    if not project_pids:
        return (False, "port busy but listeners not recognized as project ptz_stream", pids)

    # SIGTERM -> 等待 -> SIGKILL（只针对本项目 ptz_stream）
    for pid in project_pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass

    deadline = time.time() + 10
    while time.time() < deadline:
        if not _port_in_use(port):
            return (True, "project ptz_stream killed; port freed", project_pids)
        time.sleep(0.3)

    for pid in project_pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except Exception:
            pass

    ok = not _port_in_use(port)
    return (
        ok,
        "project ptz_stream killed; port freed by SIGKILL" if ok else "force_kill failed",
        project_pids,
    )


def get_diagnostics() -> dict:
    """合并：磁盘配置 + Launcher 状态 + Isaac /diagnostics（若可达）。"""
    disk = _read_stream_config_from_disk(CONFIG_PATH)
    out: dict = {
        "diagnostics_version": 1,
        "config_path": os.path.abspath(CONFIG_PATH),
        "configured_for_next_start": disk,
        "launcher_port": LAUNCHER_PORT,
        "isaac_ctrl_port": ISAAC_PORT,
        "isaac_state": None,
        "isaac_diagnostics": None,
        "urls": {
            "web_panel": f"http://127.0.0.1:{LAUNCHER_PORT}/",
            "launcher_status": f"http://127.0.0.1:{LAUNCHER_PORT}/status",
            "launcher_diagnostics": f"http://127.0.0.1:{LAUNCHER_PORT}/diagnostics",
            "isaac_diagnostics": f"http://127.0.0.1:{ISAAC_PORT}/diagnostics",
            "isaac_snapshot": f"http://127.0.0.1:{ISAAC_PORT}/snapshot.jpg",
        },
        "howto_switch_sample": (
            "修改 ptz_config.yaml 的 scene_path 后 POST /stop 再 POST /start；"
            "或 ./start_abnormal_web.sh <样本.usd>；"
            "PTZ_SCENE_OVERRIDE 仅作兼容，启动时仍以当前 YAML 中 scene_path 为准（子进程 argv 中后写 --scene）。"
        ),
    }
    with _proc_lock:
        st = _isaac_state
    out["isaac_state"] = st
    if _port_in_use(ISAAC_PORT):
        try:
            with urllib.request.urlopen(
                f"http://127.0.0.1:{ISAAC_PORT}/diagnostics", timeout=3
            ) as r:
                out["isaac_diagnostics"] = json.loads(r.read().decode())
        except Exception as e:
            out["isaac_diagnostics"] = {"error": str(e), "hint": "端口已占用但 HTTP 未就绪或进程异常"}
    return out


_flv_init_buf: bytes | None = None       # FLV 文件头 + metadata + AVC seq header
_flv_init_ready = threading.Event()      # init_buf 就绪信号
_flv_clients: dict = {}                  # cid → {sock_file, lock, state}
_flv_clients_lock = threading.Lock()
_flv_cid_counter = 0


def _flv_tag_size(data: bytes, pos: int) -> int:
    """返回 pos 处 FLV tag 的总字节数（含末尾 PreviousTagSize 4 字节），不足则返回 0。"""
    if pos + 11 > len(data):
        return 0
    data_size = (data[pos + 1] << 16) | (data[pos + 2] << 8) | data[pos + 3]
    total = 11 + data_size + 4
    return total if pos + total <= len(data) else 0


def _flv_is_keyframe(tag: bytes) -> bool:
    """判断是否为 H.264 视频关键帧 tag（FrameType=1, CodecID=7）。"""
    return len(tag) >= 12 and tag[0] == 0x09 and (tag[11] >> 4) == 1


def _flv_is_avc_seqhdr(tag: bytes) -> bool:
    """判断是否为 AVC Sequence Header（解码必须先收到此包）。"""
    return (len(tag) >= 13 and tag[0] == 0x09
            and tag[11] == 0x17 and tag[12] == 0x00)


def _flv_broadcast(tag: bytes, is_kf: bool) -> None:
    """将一个 FLV tag 广播给所有已连接客户端；新客户端等待首个关键帧后才开始接收。"""
    dead = []
    with _flv_clients_lock:
        items = list(_flv_clients.items())
    for cid, c in items:
        if c["state"] == "waiting_kf":
            if not is_kf:
                continue
            c["state"] = "live"
        with c["lock"]:
            try:
                _ws_send(c["sock_file"], tag)
            except Exception:
                dead.append(cid)
    if dead:
        with _flv_clients_lock:
            for cid in dead:
                _flv_clients.pop(cid, None)


def _flv_broadcaster() -> None:
    """后台线程：Isaac Sim 运行时启动 ffmpeg 读 RTSP，解析 FLV tag 并广播给客户端。"""
    global _flv_init_buf, _flv_cid_counter

    FLV_FILE_HDR_LEN = 13   # 9 字节 FLV 文件头 + 4 字节 PreviousTagSize0

    while True:
        if _isaac_state != "running" or not _port_in_use(8554):
            time.sleep(0.5)
            continue

        time.sleep(0.5)   # 等待 MediaMTX 稳定

        cmd = [
            "ffmpeg", "-loglevel", "quiet",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "0",
            "-probesize", "32",
            "-rtsp_transport", "tcp",
            "-i", "rtsp://localhost:8554/ptz_cam",
            "-c:v", "copy",
            "-an",
            "-f", "flv", "pipe:1",
        ]
        proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )

        buf        = bytearray()
        init_done  = False
        local_init = bytearray()

        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf.extend(chunk)

                # 收集 FLV 文件头（固定 13 字节）
                if not local_init and len(buf) >= FLV_FILE_HDR_LEN:
                    if buf[:3] != b'FLV':
                        break
                    local_init.extend(buf[:FLV_FILE_HDR_LEN])
                    buf = buf[FLV_FILE_HDR_LEN:]

                if not local_init:
                    continue

                # 逐 tag 解析
                pos = 0
                while True:
                    sz = _flv_tag_size(bytes(buf), pos)
                    if sz == 0:
                        break
                    tag = bytes(buf[pos: pos + sz])
                    pos += sz

                    if not init_done:
                        local_init.extend(tag)
                        if _flv_is_avc_seqhdr(tag):
                            # 收到 AVC 序列头，初始化包完整
                            _flv_init_buf = bytes(local_init)
                            _flv_init_ready.set()
                            init_done = True
                    else:
                        _flv_broadcast(tag, _flv_is_keyframe(tag))

                buf = buf[pos:]

        except Exception:
            pass
        finally:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                pass

        # 流中断：清空状态，断开所有客户端
        _flv_init_ready.clear()
        _flv_init_buf = None
        with _flv_clients_lock:
            _flv_clients.clear()
        time.sleep(5)


def _soap_wrap(body_content: str) -> bytes:
    """
    将内容包裹在 SOAP Envelope 中，自动匹配当前请求的 SOAP 版本：
      - 请求用 application/soap+xml → SOAP 1.2（_soap_tls.soap12 = True）
      - 请求用 text/xml 或未指定   → SOAP 1.1（_soap_tls.soap12 = False）
    这样可以同时兼容：
      · 中文 ONVIF 工具（SOAP 1.1，text/xml）
      · .NET WCF / 现代 ONVIF 客户端（SOAP 1.2，application/soap+xml）
    """
    if getattr(_soap_tls, "soap12", False):
        s_ns = "http://www.w3.org/2003/05/soap-envelope"
    else:
        s_ns = "http://schemas.xmlsoap.org/soap/envelope/"
    return (
        f'<?xml version="1.0" encoding="UTF-8"?>'
        f'<s:Envelope '
        f'xmlns:s="{s_ns}" '
        f'xmlns:tt="http://www.onvif.org/ver10/schema" '
        f'xmlns:tds="http://www.onvif.org/ver10/device/wsdl" '
        f'xmlns:trt="http://www.onvif.org/ver10/media/wsdl" '
        f'xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl" '
        f'xmlns:timg="http://www.onvif.org/ver20/imaging/wsdl">'
        f'<s:Body>{body_content}</s:Body>'
        f'</s:Envelope>'
    ).encode()


def _parse_soap_action(data: bytes) -> tuple[str, ET.Element | None]:
    """解析 SOAP Body，返回 (action_name, body_child_element)。"""
    try:
        root = ET.fromstring(data)
        body = (
            root.find("{http://www.w3.org/2003/05/soap-envelope}Body") or
            root.find("{http://schemas.xmlsoap.org/soap/envelope/}Body")
        )
        if body is not None and len(body):
            el  = body[0]
            tag = el.tag
            action = tag.split("}")[1] if "}" in tag else tag
            return action, el
    except Exception:
        pass
    return "", None


def _find_attr(el: ET.Element, local_name: str, attr: str) -> str | None:
    """忽略命名空间，按本地名查找元素属性值。"""
    if el is None:
        return None
    for child in el.iter():
        lname = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if lname == local_name and attr in child.attrib:
            return child.attrib[attr]
    return None


def _find_text(el: ET.Element, *local_names: str) -> str | None:
    """忽略命名空间，按本地名查找元素文本内容。"""
    if el is None:
        return None
    for child in el.iter():
        lname = child.tag.split("}")[-1] if "}" in child.tag else child.tag
        if lname in local_names and child.text:
            return child.text.strip()
    return None


def _get_ptz_from_isaac() -> dict:
    """从 Isaac Sim 获取当前 PTZ 状态；优先短 TTL 缓存，避免在 /status 极慢时 1s 超时误读。"""
    st = _get_isaac_status_cached(timeout=_ISAAC_STATUS_FETCH_TIMEOUT_CAP_S, max_age=1.5)
    if isinstance(st, dict):
        try:
            return {
                "pan": float(st["pan"]),
                "tilt": float(st["tilt"]),
                "zoom": float(st["zoom"]),
            }
        except Exception:
            pass
    return {"pan": 0.0, "tilt": -15.0, "zoom": 1.5}




def _get_status_from_isaac() -> dict:
    status = _get_isaac_status_cached(timeout=2.0)
    return status or {}


def _resolve_preset_semantic_ptz(preset: dict) -> dict:
    token = str(preset.get("token") or "")
    if token == _STARTUP_PRESET_TOKEN:
        return {
            "pan": float(preset["pan"]),
            "tilt": float(preset["tilt"]),
            "zoom": float(preset["zoom"]),
            "base_pan": None,
            "base_tilt": None,
            "fallback": False,
        }

    status = _get_status_from_isaac()
    orientation = status.get("orientation") or {}
    base_pan = orientation.get("base_pan")
    base_tilt = orientation.get("base_tilt")
    if base_pan is None or base_tilt is None:
        print(
            f"[PTZ-Launcher][preset-semantic] token={preset.get('token')} name={preset.get('name')} fallback=absolute reason=missing_base_orientation",
            flush=True,
        )
        return {
            "pan": float(preset["pan"]),
            "tilt": float(preset["tilt"]),
            "zoom": float(preset["zoom"]),
            "base_pan": None,
            "base_tilt": None,
            "fallback": True,
        }

    mode = str(orientation.get("mode") or "").strip().lower()
    # dynamic_lookat：Isaac 侧 default_initial 使用 base + dynamic_startup_* 作为「对目标的中性可见位」；
    # 预置位 Home/Front 及相对左右/俯视必须以该中性角为锚，不能用裸 base（否则与启动姿态错位 → 主体出画/剪影）。
    if mode == "dynamic_lookat":
        ref_pan = _clamp(
            float(base_pan) + _DYNAMIC_STARTUP_PAN_OFFSET_DEG,
            -170.0,
            170.0,
        )
        ref_tilt = _clamp(
            float(base_tilt) + _DYNAMIC_STARTUP_TILT_OFFSET_DEG,
            -90.0,
            30.0,
        )
    else:
        ref_pan = float(base_pan)
        ref_tilt = float(base_tilt)

    if token == "1":
        pan = float(ref_pan)
        tilt = float(ref_tilt)
    elif token == "2":
        pan = float(ref_pan)
        tilt = float(ref_tilt)
    elif token == "3":
        pan = float(ref_pan) + _PRESET_LEFT_PAN_OFFSET_DEG
        tilt = float(ref_tilt)
    elif token == "4":
        pan = float(ref_pan) + _PRESET_RIGHT_PAN_OFFSET_DEG
        tilt = float(ref_tilt)
    elif token == "5":
        pan = float(ref_pan)
        tilt = float(_PRESET_OVERLOOK_TILT_DEG)
    else:
        pan = float(preset["pan"])
        tilt = float(preset["tilt"])

    pan = _clamp(pan, -170.0, 170.0)
    tilt = _clamp(tilt, -90.0, 30.0)
    print(
        "[PTZ-Launcher][preset-semantic] "
        f"token={token} name={preset.get('name')} base_pan={base_pan} base_tilt={base_tilt} "
        f"applied_pan={pan} applied_tilt={tilt} zoom={preset.get('zoom')} fallback=False",
        flush=True,
    )
    return {
        "pan": float(pan),
        "tilt": float(tilt),
        "zoom": float(preset["zoom"]),
        "base_pan": float(base_pan),
        "base_tilt": float(base_tilt),
        "fallback": False,
    }

def _set_ptz_to_isaac(pan_deg: float, tilt_deg: float, zoom_x: float) -> bool:
    """向 Isaac Sim 发送 PTZ 绝对位置命令。成功返回 True，失败返回 False 并打日志（不再静默吞掉）。"""
    payload = json.dumps({
        "pan":  round(_clamp(pan_deg, -170.0, 170.0), 3),
        "tilt": round(_clamp(tilt_deg, -90.0, 30.0), 3),
        "zoom": round(_clamp(zoom_x, 1.0, 32.0), 3),
    }).encode()
    try:
        req = urllib.request.Request(
            f"http://127.0.0.1:{ISAAC_PORT}/control",
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=25) as r:
            raw = r.read()
            status = getattr(r, "status", 200)
        if status != 200:
            print(
                f"[PTZ-Launcher] Isaac /control 非 200: status={status} body={raw[:500]!r}",
                flush=True,
            )
            return False
        try:
            body = json.loads(raw.decode("utf-8", errors="ignore")) if raw else {}
        except Exception:
            body = {}
        if isinstance(body, dict) and body.get("ok") is False:
            print(
                f"[PTZ-Launcher] Isaac /control 返回 ok=False: {body.get('error', body)!r}",
                flush=True,
            )
            return False
        return True
    except Exception as exc:
        print(f"[PTZ-Launcher] Isaac /control 下发失败: {exc!r} payload={payload.decode()}", flush=True)
        return False


def _onvif_host(handler: "BaseHTTPRequestHandler") -> str:
    """从 Host 请求头提取地址（含端口），用于 ONVIF XAddr 拼接。"""
    return handler.headers.get("Host", f"localhost:{LAUNCHER_PORT}")


# ── Device Service ────────────────────────────────────────────────────

def _onvif_device(action: str, el: ET.Element | None, host: str) -> bytes:
    base = f"http://{host}"

    if action == "GetSystemDateAndTime":
        now = datetime.datetime.utcnow()
        return _soap_wrap(f"""<tds:GetSystemDateAndTimeResponse>
  <tds:SystemDateAndTime>
    <tt:DateTimeType>Manual</tt:DateTimeType>
    <tt:DaylightSavings>false</tt:DaylightSavings>
    <tt:TimeZone><tt:TZ>UTC</tt:TZ></tt:TimeZone>
    <tt:UTCDateTime>
      <tt:Time>
        <tt:Hour>{now.hour}</tt:Hour>
        <tt:Minute>{now.minute}</tt:Minute>
        <tt:Second>{now.second}</tt:Second>
      </tt:Time>
      <tt:Date>
        <tt:Year>{now.year}</tt:Year>
        <tt:Month>{now.month}</tt:Month>
        <tt:Day>{now.day}</tt:Day>
      </tt:Date>
    </tt:UTCDateTime>
  </tds:SystemDateAndTime>
</tds:GetSystemDateAndTimeResponse>""")

    if action == "GetCapabilities":
        return _soap_wrap(f"""<tds:GetCapabilitiesResponse>
  <tds:Capabilities>
    <tt:Device>
      <tt:XAddr>{base}/onvif/device_service</tt:XAddr>
      <tt:Network>
        <tt:IPFilter>false</tt:IPFilter>
        <tt:ZeroConfiguration>false</tt:ZeroConfiguration>
        <tt:IPVersion6>false</tt:IPVersion6>
        <tt:DynDNS>false</tt:DynDNS>
      </tt:Network>
      <tt:System>
        <tt:DiscoveryResolve>true</tt:DiscoveryResolve>
        <tt:DiscoveryBye>false</tt:DiscoveryBye>
        <tt:RemoteDiscovery>false</tt:RemoteDiscovery>
        <tt:SystemBackup>false</tt:SystemBackup>
        <tt:SystemLogging>false</tt:SystemLogging>
        <tt:FirmwareUpgrade>false</tt:FirmwareUpgrade>
      </tt:System>
      <tt:IO>
        <tt:InputConnectors>0</tt:InputConnectors>
        <tt:RelayOutputs>0</tt:RelayOutputs>
      </tt:IO>
      <tt:Security>
        <tt:TLS1.2>false</tt:TLS1.2>
        <tt:OnboardKeyGeneration>false</tt:OnboardKeyGeneration>
        <tt:AccessPolicyConfig>false</tt:AccessPolicyConfig>
        <tt:X.509Token>false</tt:X.509Token>
        <tt:SAMLToken>false</tt:SAMLToken>
        <tt:KerberosToken>false</tt:KerberosToken>
        <tt:RELToken>false</tt:RELToken>
      </tt:Security>
    </tt:Device>
    <tt:Media>
      <tt:XAddr>{base}/onvif/media_service</tt:XAddr>
      <tt:StreamingCapabilities>
        <tt:RTPMulticast>false</tt:RTPMulticast>
        <tt:RTP_TCP>true</tt:RTP_TCP>
        <tt:RTP_RTSP_TCP>true</tt:RTP_RTSP_TCP>
      </tt:StreamingCapabilities>
    </tt:Media>
    <tt:PTZ>
      <tt:XAddr>{base}/onvif/ptz_service</tt:XAddr>
    </tt:PTZ>
    <tt:Imaging>
      <tt:XAddr>{base}/onvif/imaging_service</tt:XAddr>
    </tt:Imaging>
  </tds:Capabilities>
</tds:GetCapabilitiesResponse>""")

    if action == "GetServices":
        return _soap_wrap(f"""<tds:GetServicesResponse>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/device/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/device_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver10/media/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/media_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver20/ptz/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/ptz_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>6</tt:Minor></tds:Version>
  </tds:Service>
  <tds:Service>
    <tds:Namespace>http://www.onvif.org/ver20/imaging/wsdl</tds:Namespace>
    <tds:XAddr>{base}/onvif/imaging_service</tds:XAddr>
    <tds:Version><tt:Major>2</tt:Major><tt:Minor>0</tt:Minor></tds:Version>
  </tds:Service>
</tds:GetServicesResponse>""")

    if action == "GetDeviceInformation":
        return _soap_wrap("""<tds:GetDeviceInformationResponse>
  <tds:Manufacturer>Isaac-Sim</tds:Manufacturer>
  <tds:Model>PTZ-Camera-V4</tds:Model>
  <tds:FirmwareVersion>4.0.0</tds:FirmwareVersion>
  <tds:SerialNumber>CAM-03-SIM</tds:SerialNumber>
  <tds:HardwareId>1.0</tds:HardwareId>
</tds:GetDeviceInformationResponse>""")

    if action == "GetScopes":
        return _soap_wrap("""<tds:GetScopesResponse>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/location/country/china</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/name/Isaac-Sim-PTZ-Camera</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/hardware/PTZ-Camera-V4</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/type/video_encoder</tt:ScopeItem>
  </tds:Scopes>
  <tds:Scopes>
    <tt:ScopeDef>Fixed</tt:ScopeDef>
    <tt:ScopeItem>onvif://www.onvif.org/Profile/Streaming</tt:ScopeItem>
  </tds:Scopes>
</tds:GetScopesResponse>""")

    if action == "GetNetworkInterfaces":
        local_ips = _get_local_ips()
        ifaces_xml = ""
        for i, ip in enumerate(local_ips):
            ifaces_xml += f"""  <tds:NetworkInterfaces token="eth{i}">
    <tt:Enabled>true</tt:Enabled>
    <tt:Info>
      <tt:Name>eth{i}</tt:Name>
      <tt:HwAddress>00:00:00:00:00:0{i}</tt:HwAddress>
      <tt:MTU>1500</tt:MTU>
    </tt:Info>
    <tt:IPv4>
      <tt:Enabled>true</tt:Enabled>
      <tt:Config>
        <tt:Manual>
          <tt:Address>{ip}</tt:Address>
          <tt:PrefixLength>24</tt:PrefixLength>
        </tt:Manual>
        <tt:DHCP>true</tt:DHCP>
      </tt:Config>
    </tt:IPv4>
  </tds:NetworkInterfaces>
"""
        return _soap_wrap(
            f"<tds:GetNetworkInterfacesResponse>\n{ifaces_xml}"
            "</tds:GetNetworkInterfacesResponse>"
        )

    return _soap_wrap(f"<tds:{action}Response/>")


# ── Media Service ─────────────────────────────────────────────────────

def _onvif_media(action: str, el: ET.Element | None, host: str, client_ip: str = "") -> bytes:
    base     = f"http://{host}"
    snap_url = f"{base}/onvif-snap.jpg"

    if action in ("GetProfiles", "GetProfile"):
        # GetProfile（单数）返回 GetProfileResponse；GetProfiles（复数）返回 GetProfilesResponse
        resp_tag  = "GetProfileResponse"  if action == "GetProfile"  else "GetProfilesResponse"
        item_tag  = "Profile"             if action == "GetProfile"  else "Profiles"
        profile_xml = f"""<trt:{item_tag} token="{_ONVIF_PROFILE_TOKEN}" fixed="true">
    <tt:Name>PTZ-Sim-H264-Profile</tt:Name>
    <tt:VideoSourceConfiguration token="VSC_1">
      <tt:Name>VideoSource_1</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:SourceToken>VST_1</tt:SourceToken>
      <tt:Bounds x="0" y="0" width="1920" height="1080"/>
    </tt:VideoSourceConfiguration>
    <tt:VideoEncoderConfiguration token="VEC_1">
      <tt:Name>H264-1080p-25fps</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:Encoding>H264</tt:Encoding>
      <tt:Resolution>
        <tt:Width>1920</tt:Width>
        <tt:Height>1080</tt:Height>
      </tt:Resolution>
      <tt:Quality>80</tt:Quality>
      <tt:RateControl>
        <tt:FrameRateLimit>25</tt:FrameRateLimit>
        <tt:EncodingInterval>1</tt:EncodingInterval>
        <tt:BitrateLimit>4000</tt:BitrateLimit>
      </tt:RateControl>
      <tt:H264>
        <tt:GovLength>50</tt:GovLength>
        <tt:H264Profile>High</tt:H264Profile>
      </tt:H264>
      <tt:Multicast>
        <tt:Address>
          <tt:Type>IPv4</tt:Type>
          <tt:IPv4Address>0.0.0.0</tt:IPv4Address>
        </tt:Address>
        <tt:Port>0</tt:Port>
        <tt:TTL>0</tt:TTL>
        <tt:AutoStart>false</tt:AutoStart>
      </tt:Multicast>
      <tt:SessionTimeout>PT60S</tt:SessionTimeout>
    </tt:VideoEncoderConfiguration>
    <tt:PTZConfiguration token="{_ONVIF_PTZ_TOKEN}">
      <tt:Name>PTZConfig</tt:Name>
      <tt:UseCount>1</tt:UseCount>
      <tt:NodeToken>{_ONVIF_PTZ_NODE}</tt:NodeToken>
      <tt:DefaultAbsolutePantTiltPositionSpace>
        http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace
      </tt:DefaultAbsolutePantTiltPositionSpace>
      <tt:DefaultAbsoluteZoomPositionSpace>
        http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace
      </tt:DefaultAbsoluteZoomPositionSpace>
      <tt:DefaultRelativePanTiltTranslationSpace>
        http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace
      </tt:DefaultRelativePanTiltTranslationSpace>
      <tt:PanTiltLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
          <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
        </tt:Range>
      </tt:PanTiltLimits>
      <tt:ZoomLimits>
        <tt:Range>
          <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
          <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        </tt:Range>
      </tt:ZoomLimits>
    </tt:PTZConfiguration>
  </trt:{item_tag}>"""
        return _soap_wrap(f"<trt:{resp_tag}>{profile_xml}</trt:{resp_tag}>")

    if action == "GetStreamUri":
        rtsp_host = _rtsp_host_for_onvif(host, client_ip)
        rtsp_url  = f"rtsp://{rtsp_host}:{_RTSP_PORT}/ptz_cam"
        print(
            f"[ONVIF] GetStreamUri Host={host!r} client={client_ip!r} → {rtsp_url}",
            flush=True,
        )
        return _soap_wrap(f"""<trt:GetStreamUriResponse>
  <trt:MediaUri>
    <tt:Uri>{rtsp_url}</tt:Uri>
    <tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>
    <tt:Timeout>PT0S</tt:Timeout>
  </trt:MediaUri>
</trt:GetStreamUriResponse>""")

    if action == "GetSnapshotUri":
        return _soap_wrap(f"""<trt:GetSnapshotUriResponse>
  <trt:MediaUri>
    <tt:Uri>{snap_url}</tt:Uri>
    <tt:InvalidAfterConnect>false</tt:InvalidAfterConnect>
    <tt:InvalidAfterReboot>false</tt:InvalidAfterReboot>
    <tt:Timeout>PT30S</tt:Timeout>
  </trt:MediaUri>
</trt:GetSnapshotUriResponse>""")

    if action == "GetServiceCapabilities":
        return _soap_wrap("""<trt:GetServiceCapabilitiesResponse>
  <trt:Capabilities SnapshotUri="true" Rotation="false"
    VideoSourceMode="false" OSD="false"/>
</trt:GetServiceCapabilitiesResponse>""")

    if action == "GetVideoSources":
        return _soap_wrap("""<trt:GetVideoSourcesResponse>
  <trt:VideoSources token="VST_1">
    <tt:Framerate>25</tt:Framerate>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Imaging/>
  </trt:VideoSources>
</trt:GetVideoSourcesResponse>""")

    if action in ("GetVideoSourceConfigurations", "GetVideoSourceConfiguration"):
        # VideoSourceConfiguration（软件绑定层）与 VideoSources（物理源）是不同类型
        resp_tag = ("GetVideoSourceConfigurationResponse"
                    if action == "GetVideoSourceConfiguration"
                    else "GetVideoSourceConfigurationsResponse")
        item_tag = ("Configuration"
                    if action == "GetVideoSourceConfiguration"
                    else "Configurations")
        return _soap_wrap(f"""<trt:{resp_tag}>
  <trt:{item_tag} token="VSC_1">
    <tt:Name>VideoSource_1</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:SourceToken>VST_1</tt:SourceToken>
    <tt:Bounds x="0" y="0" width="1920" height="1080"/>
  </trt:{item_tag}>
</trt:{resp_tag}>""")

    if action in ("GetVideoEncoderConfigurations", "GetVideoEncoderConfiguration"):
        return _soap_wrap("""<trt:GetVideoEncoderConfigurationsResponse>
  <trt:Configurations token="VEC_1">
    <tt:Name>H264-1080p</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:Encoding>H264</tt:Encoding>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Quality>80</tt:Quality>
    <tt:RateControl>
      <tt:FrameRateLimit>25</tt:FrameRateLimit>
      <tt:EncodingInterval>1</tt:EncodingInterval>
      <tt:BitrateLimit>4000</tt:BitrateLimit>
    </tt:RateControl>
    <tt:H264>
      <tt:GovLength>50</tt:GovLength>
      <tt:H264Profile>High</tt:H264Profile>
    </tt:H264>
    <tt:Multicast>
      <tt:Address>
        <tt:Type>IPv4</tt:Type>
        <tt:IPv4Address>0.0.0.0</tt:IPv4Address>
      </tt:Address>
      <tt:Port>0</tt:Port>
      <tt:TTL>0</tt:TTL>
      <tt:AutoStart>false</tt:AutoStart>
    </tt:Multicast>
    <tt:SessionTimeout>PT60S</tt:SessionTimeout>
  </trt:Configurations>
</trt:GetVideoEncoderConfigurationsResponse>""")

    if action in ("GetVideoEncoderConfigurationOptions",
                  "GetCompatibleVideoEncoderConfigurationOptions"):
        return _soap_wrap("""<trt:GetVideoEncoderConfigurationOptionsResponse>
  <trt:Options>
    <tt:QualityRange>
      <tt:Min>1</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:QualityRange>
    <tt:H264>
      <tt:ResolutionsAvailable>
        <tt:Width>1920</tt:Width>
        <tt:Height>1080</tt:Height>
      </tt:ResolutionsAvailable>
      <tt:ResolutionsAvailable>
        <tt:Width>1280</tt:Width>
        <tt:Height>720</tt:Height>
      </tt:ResolutionsAvailable>
      <tt:GovLengthRange>
        <tt:Min>1</tt:Min>
        <tt:Max>300</tt:Max>
      </tt:GovLengthRange>
      <tt:FrameRateRange>
        <tt:Min>1</tt:Min>
        <tt:Max>30</tt:Max>
      </tt:FrameRateRange>
      <tt:EncodingIntervalRange>
        <tt:Min>1</tt:Min>
        <tt:Max>10</tt:Max>
      </tt:EncodingIntervalRange>
      <tt:H264ProfilesSupported>Baseline</tt:H264ProfilesSupported>
      <tt:H264ProfilesSupported>Main</tt:H264ProfilesSupported>
      <tt:H264ProfilesSupported>High</tt:H264ProfilesSupported>
    </tt:H264>
    <tt:Extension/>
  </trt:Options>
</trt:GetVideoEncoderConfigurationOptionsResponse>""")

    if action == "GetCompatibleVideoEncoderConfigurations":
        return _soap_wrap("""<trt:GetCompatibleVideoEncoderConfigurationsResponse>
  <trt:Configurations token="VEC_1">
    <tt:Name>H264-1080p</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:Encoding>H264</tt:Encoding>
    <tt:Resolution>
      <tt:Width>1920</tt:Width>
      <tt:Height>1080</tt:Height>
    </tt:Resolution>
    <tt:Quality>80</tt:Quality>
    <tt:RateControl>
      <tt:FrameRateLimit>25</tt:FrameRateLimit>
      <tt:EncodingInterval>1</tt:EncodingInterval>
      <tt:BitrateLimit>4000</tt:BitrateLimit>
    </tt:RateControl>
    <tt:H264>
      <tt:GovLength>50</tt:GovLength>
      <tt:H264Profile>High</tt:H264Profile>
    </tt:H264>
  </trt:Configurations>
</trt:GetCompatibleVideoEncoderConfigurationsResponse>""")

    return _soap_wrap(f"<trt:{action}Response/>")


# ── PTZ Service ───────────────────────────────────────────────────────

def _onvif_ptz(action: str, el: ET.Element | None) -> bytes:

    if action == "AbsoluteMove":
        pan_x  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_y  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_z = float(_find_attr(el, "Zoom",    "x") or 0.0)
        if not _set_ptz_to_isaac(
            _norm_to_pan(pan_x),
            _norm_to_tilt(pan_y),
            _norm_to_zoom(zoom_z),
        ):
            return _soap_fault("Isaac PTZ 下发失败（详见 PTZ-Launcher 日志）", "ter:OperationProhibited")
        return _soap_wrap("<tptz:AbsoluteMoveResponse/>")

    if action == "RelativeMove":
        pan_dx  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        pan_dy  = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_dz = float(_find_attr(el, "Zoom",    "x") or 0.0)
        cur = _get_ptz_from_isaac()
        if not _set_ptz_to_isaac(
            cur["pan"]  + _clamp(pan_dx, -1.0, 1.0) * _PAN_SCALE,
            cur["tilt"] - _clamp(pan_dy, -1.0, 1.0) * 60.0,
            cur["zoom"] + _clamp(zoom_dz, -1.0, 1.0) * _ZOOM_SCALE,
        ):
            return _soap_fault("Isaac PTZ 下发失败（详见 PTZ-Launcher 日志）", "ter:OperationProhibited")
        return _soap_wrap("<tptz:RelativeMoveResponse/>")

    if action == "ContinuousMove":
        # 简化实现：按速度比例做单步位移（无持续运动定时器）
        pan_v  = float(_find_attr(el, "PanTilt", "x") or 0.0)
        tilt_v = float(_find_attr(el, "PanTilt", "y") or 0.0)
        zoom_v = float(_find_attr(el, "Zoom",    "x") or 0.0)
        cur = _get_ptz_from_isaac()
        if not _set_ptz_to_isaac(
            cur["pan"]  + pan_v  * 5.0,
            cur["tilt"] + tilt_v * 5.0,
            cur["zoom"] + zoom_v * 1.0,
        ):
            return _soap_fault("Isaac PTZ 下发失败（详见 PTZ-Launcher 日志）", "ter:OperationProhibited")
        return _soap_wrap("<tptz:ContinuousMoveResponse/>")

    if action == "Stop":
        return _soap_wrap("<tptz:StopResponse/>")

    if action == "GotoPreset":
        token = _find_text(el, "PresetToken") or "1"
        preset = _get_preset(token)
        if preset is None:
            return _soap_fault(f"PresetToken 不存在: {token}")
        resolved = _resolve_preset_semantic_ptz(preset)
        if not _set_ptz_to_isaac(resolved["pan"], resolved["tilt"], float(preset["zoom"])):
            return _soap_fault("Isaac PTZ 下发失败（详见 PTZ-Launcher 日志）", "ter:OperationProhibited")
        return _soap_wrap("<tptz:GotoPresetResponse/>")

    if action == "SetPreset":
        token = _find_text(el, "PresetToken")
        name = _find_text(el, "PresetName")
        if token and token not in _preset_token_order():
            return _soap_fault(f"PresetToken 超出范围: {token}")
        preset = _save_preset(token, name, _get_ptz_from_isaac())
        return _soap_wrap(f"""<tptz:SetPresetResponse>
  <tptz:PresetToken>{preset["token"]}</tptz:PresetToken>
</tptz:SetPresetResponse>""")

    if action == "RemovePreset":
        token = _find_text(el, "PresetToken") or ""
        if token not in _preset_token_order():
            return _soap_fault(f"PresetToken 超出范围: {token}")
        if not _delete_preset(token):
            return _soap_fault(f"PresetToken 不存在: {token}")
        return _soap_wrap("<tptz:RemovePresetResponse/>")

    if action == "GetPresets":
        preset_xml = "\n".join(_preset_to_onvif_xml(item) for item in _list_presets())
        return _soap_wrap(f"""<tptz:GetPresetsResponse>
{preset_xml}
</tptz:GetPresetsResponse>""")

    if action == "GetStatus":
        st         = _get_ptz_from_isaac()
        onvif_pan  = round(_pan_to_norm(st["pan"]), 4)
        onvif_tilt = round(_tilt_to_norm(st["tilt"]), 4)
        onvif_zoom = round(_zoom_to_norm(st["zoom"]), 4)
        return _soap_wrap(f"""<tptz:GetStatusResponse>
  <tptz:PTZStatus>
    <tt:Position>
      <tt:PanTilt x="{onvif_pan}" y="{onvif_tilt}"
        space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace"/>
      <tt:Zoom x="{onvif_zoom}"
        space="http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"/>
    </tt:Position>
    <tt:MoveStatus>
      <tt:PanTilt>IDLE</tt:PanTilt>
      <tt:Zoom>IDLE</tt:Zoom>
    </tt:MoveStatus>
  </tptz:PTZStatus>
</tptz:GetStatusResponse>""")

    if action == "GetNodes":
        return _soap_wrap(f"""<tptz:GetNodesResponse>
  <tptz:PTZNode token="{_ONVIF_PTZ_NODE}" FixedHomePosition="false">
    <tt:Name>PTZ Node</tt:Name>
    <tt:SupportedPTZSpaces>
      <tt:AbsolutePanTiltPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:AbsolutePanTiltPositionSpace>
      <tt:AbsoluteZoomPositionSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:AbsoluteZoomPositionSpace>
      <tt:RelativePanTiltTranslationSpace>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/TranslationGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:RelativePanTiltTranslationSpace>
    </tt:SupportedPTZSpaces>
    <tt:MaximumNumberOfPresets>5</tt:MaximumNumberOfPresets>
    <tt:HomeSupported>false</tt:HomeSupported>
  </tptz:PTZNode>
</tptz:GetNodesResponse>""")

    if action in ("GetConfigurations", "GetConfiguration"):
        return _soap_wrap(f"""<tptz:GetConfigurationsResponse>
  <tptz:PTZConfiguration token="{_ONVIF_PTZ_TOKEN}">
    <tt:Name>PTZConfig</tt:Name>
    <tt:UseCount>1</tt:UseCount>
    <tt:NodeToken>{_ONVIF_PTZ_NODE}</tt:NodeToken>
    <tt:DefaultPTZSpeed>
      <tt:PanTilt x="0.5" y="0.5"
        space="http://www.onvif.org/ver10/tptz/PanTiltSpaces/GenericSpeedSpace"/>
      <tt:Zoom x="0.5"
        space="http://www.onvif.org/ver10/tptz/ZoomSpaces/ZoomGenericSpeedSpace"/>
    </tt:DefaultPTZSpeed>
    <tt:DefaultPTZTimeout>PT5S</tt:DefaultPTZTimeout>
    <tt:PanTiltLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:XRange>
        <tt:YRange><tt:Min>-1</tt:Min><tt:Max>1</tt:Max></tt:YRange>
      </tt:Range>
    </tt:PanTiltLimits>
    <tt:ZoomLimits>
      <tt:Range>
        <tt:URI>http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace</tt:URI>
        <tt:XRange><tt:Min>0</tt:Min><tt:Max>1</tt:Max></tt:XRange>
      </tt:Range>
    </tt:ZoomLimits>
  </tptz:PTZConfiguration>
</tptz:GetConfigurationsResponse>""")

    return _soap_wrap(f"<tptz:{action}Response/>")


# ── Imaging Service ───────────────────────────────────────────────────
# NVT 设备的必要服务：ONVIF 客户端通过它确认设备是完整的 NVT 摄像头，
# 并据此显示 "Live Video" 模块。即使参数是只读存根也必须能正常响应。

def _onvif_imaging(action: str, el: ET.Element | None) -> bytes:

    if action == "GetServiceCapabilities":
        return _soap_wrap("""<timg:GetServiceCapabilitiesResponse>
  <timg:Capabilities ImageStabilization="false" Presets="false" AdaptablePreset="false"/>
</timg:GetServiceCapabilitiesResponse>""")

    if action == "GetImagingSettings":
        return _soap_wrap("""<timg:GetImagingSettingsResponse>
  <timg:ImagingSettings>
    <tt:BacklightCompensation>
      <tt:Mode>OFF</tt:Mode>
      <tt:Level>0</tt:Level>
    </tt:BacklightCompensation>
    <tt:Brightness>50</tt:Brightness>
    <tt:ColorSaturation>50</tt:ColorSaturation>
    <tt:Contrast>50</tt:Contrast>
    <tt:Exposure>
      <tt:Mode>AUTO</tt:Mode>
      <tt:MinExposureTime>1</tt:MinExposureTime>
      <tt:MaxExposureTime>100000</tt:MaxExposureTime>
      <tt:MinGain>0</tt:MinGain>
      <tt:MaxGain>100</tt:MaxGain>
    </tt:Exposure>
    <tt:Focus>
      <tt:AutoFocusMode>AUTO</tt:AutoFocusMode>
    </tt:Focus>
    <tt:IrCutFilter>AUTO</tt:IrCutFilter>
    <tt:Sharpness>50</tt:Sharpness>
    <tt:WideDynamicRange>
      <tt:Mode>OFF</tt:Mode>
      <tt:Level>0</tt:Level>
    </tt:WideDynamicRange>
    <tt:WhiteBalance>
      <tt:Mode>AUTO</tt:Mode>
    </tt:WhiteBalance>
  </timg:ImagingSettings>
</timg:GetImagingSettingsResponse>""")

    if action == "GetOptions":
        return _soap_wrap("""<timg:GetOptionsResponse>
  <timg:ImagingOptions>
    <tt:BacklightCompensation>
      <tt:Mode>OFF</tt:Mode>
      <tt:Mode>ON</tt:Mode>
    </tt:BacklightCompensation>
    <tt:Brightness>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Brightness>
    <tt:ColorSaturation>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:ColorSaturation>
    <tt:Contrast>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Contrast>
    <tt:Sharpness>
      <tt:Min>0</tt:Min>
      <tt:Max>100</tt:Max>
    </tt:Sharpness>
  </timg:ImagingOptions>
</timg:GetOptionsResponse>""")

    if action == "GetMoveOptions":
        return _soap_wrap("""<timg:GetMoveOptionsResponse>
  <timg:MoveOptions/>
</timg:GetMoveOptionsResponse>""")

    if action == "GetStatus":
        return _soap_wrap("""<timg:GetStatusResponse>
  <timg:Status>
    <tt:FocusStatus20>
      <tt:Position>0</tt:Position>
      <tt:MoveStatus>IDLE</tt:MoveStatus>
    </tt:FocusStatus20>
  </timg:Status>
</timg:GetStatusResponse>""")

    return _soap_wrap(f"<timg:{action}Response/>")


# ══════════════════════════════════════════════════════════════════════
# HTTP Handler
# ══════════════════════════════════════════════════════════════════════

class _Handler(BaseHTTPRequestHandler):

    def log_message(self, *_):
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type,SOAPAction,Authorization")

    def _json(self, data: dict, code: int = 200):
        body = json.dumps(data, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    # ── GET ──────────────────────────────────────────────────────────
    def do_GET(self):
        path = self.path.split("?")[0]

        if path in ("/", "/index.html"):
            html_path = os.path.join(script_dir, "ptz_web_control.html")
            if not os.path.isfile(html_path):
                self.send_response(404); self.end_headers(); return
            data = open(html_path, "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self._cors()
            self.end_headers()
            self.wfile.write(data)
            return

        if path in ("/status", "/api/status"):
            self._json(get_status())
            return

        if path == "/api/health":
            self._json(_launcher_api_health())
            return

        if path == "/diagnostics":
            self._json(get_diagnostics())
            return

        if path == "/presets":
            self._json({"items": _list_presets(), "capacity": _PRESET_LIMIT})
            return

        if path == "/log":
            if os.path.isfile(ISAAC_LOG):
                with open(ISAAC_LOG, "rb") as f:
                    size = os.path.getsize(ISAAC_LOG)
                    f.seek(max(0, size - 8192))
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self._cors()
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404); self.end_headers()
            return

        # ONVIF 快照端点（优先实时快照，失败时回退缓存帧）
        if path == "/onvif-snap.jpg":
            self._snap_handle()
            return

        # 快照端点：代理到 runtime(8081) /snapshot.jpg（与 onvif-snap 复用同一上游拉取逻辑）
        if path == "/snapshot.jpg":
            jpeg, source, snap_headers = _get_snapshot_proxy_data()
            if jpeg is not None and _is_jpeg_bytes(jpeg):
                try:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-PTZ-Snapshot-Source", source or "unknown")
                    _send_snapshot_meta_headers(self, snap_headers)
                    self._cors()
                    self.end_headers()
                    self.wfile.write(jpeg)
                except Exception:
                    pass
                return
            try:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-PTZ-Snapshot-Source", "unavailable")
                self._cors()
                self.end_headers()
                self.wfile.write(b"snapshot upstream unavailable")
            except Exception:
                pass
            return

        # WebSocket JPEG 帧流（轻量，浏览器自定义解码）
        if path == "/ws":
            self._ws_handle()
            return

        # WebSocket FLV+H.264 流（兼容 flv.js / jessibuca 等标准播放器）
        if path == "/ws-flv":
            self._ws_flv_handle()
            return

        # 场景状态代理（Web UI 用）
        if path == "/scene/state":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/state")
            return

        if path == "/scene/random-config":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/random-config")
            return

        if path == "/api/scene/randomize/last":
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json({"ok": False, "error": "Isaac Sim 未就绪", "last": None}, 503)
                return
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/api/scene/randomize/last")
            return

        if path == "/api/scene/random-config":
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json({"ok": False, "error": "Isaac Sim 未就绪"}, 503)
                return
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/api/scene/random-config")
            return

        if path == "/scene/hdri":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/hdri")
            return

        if path == "/scene/environment":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/environment")
            return

        if path == "/scene/dynamic-sky-presets":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/dynamic-sky-presets")
            return

        if path == "/scene/describe":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/scene/describe")
            return

        if path == "/ptz_state":
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/ptz_state")
            return

        if path == "/render/volumetric":
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json({"ok": False, "error": "Isaac Sim 未就绪"}, 503)
                return
            self._proxy_once(f"http://127.0.0.1:{ISAAC_PORT}/render/volumetric")
            return

        self.send_response(404); self.end_headers()

    # ── POST ─────────────────────────────────────────────────────────
    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)

        if path == "/api/scene/randomize/last":
            try:
                length = int(self.headers.get("Content-Length", 0))
            except (TypeError, ValueError):
                length = 0
            body = self.rfile.read(length) if length else b""
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json(
                    {
                        "ok": False,
                        "api_version": "v1",
                        "method": "POST",
                        "project_id": "diaolan",
                        "scene_type": "gondola",
                        "request_id": None,
                        "data": None,
                        "error": "Isaac Sim 未就绪",
                    },
                    503,
                )
                return
            self._proxy_post(path, body)
            return

        if path == "/start":
            force_restart = False
            v = (qs.get("force") or [None])[0]
            if isinstance(v, str) and v.strip().lower() in ("1", "true", "yes", "y", "on"):
                force_restart = True

            # 可选：POST body: {"force": true}
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            if body:
                try:
                    data = json.loads(body.decode("utf-8", errors="ignore")) or {}
                    if isinstance(data, dict) and bool(data.get("force", False)):
                        force_restart = True
                except Exception:
                    pass

            self._json(start_isaac(force_restart=force_restart))
            return

        if path == "/stop":
            self._json(stop_isaac())
            return

        if path.startswith("/presets/"):
            self._preset_post(path)
            return

        # 透明代理 POST 到 Isaac Sim 的 /control 和 /scene/* 端点（Web UI 用）
        if path in (
            "/control",
            "/scene/gondola",
            "/scene/workers",
            "/scene/randomize",
            "/api/scene/randomize",
            "/scene/activate_diaolan",
            "/scene/select_diaolan",
            "/scene/random-config",
            "/scene/hdri/group",
            "/scene/hdri/select",
            "/scene/hdri/random",
            "/scene/environment",
            "/scene/dynamic-sky-preset",
            "/scene/experiment",
            "/scene/safety-components/status",
            "/scene/safety-components/apply",
            "/render/volumetric",
        ):
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            if _isaac_state != "running" and not _port_in_use(ISAAC_PORT):
                self._json({"ok": False, "error": "Isaac Sim 未就绪"}, 503)
                return
            self._proxy_post(path, body)
            return

        # ONVIF SOAP 服务端点
        if path == "/onvif/device_service":
            self._onvif_handle("device")
            return

        if path == "/onvif/media_service":
            self._onvif_handle("media")
            return

        if path == "/onvif/ptz_service":
            self._onvif_handle("ptz")
            return

        if path == "/onvif/imaging_service":
            self._onvif_handle("imaging")
            return

        if path == "/onvif-proxy":
            self._onvif_proxy_handle()
            return

        self.send_response(404); self.end_headers()

    def do_DELETE(self):
        path = self.path.split("?")[0]
        if path.startswith("/presets/"):
            parts = [p for p in path.split("/") if p]
            if len(parts) == 2 and parts[0] == "presets" and parts[1] in _preset_token_order():
                if not _delete_preset(parts[1]):
                    self._json({"ok": False, "error": "预置位不存在"}, 404)
                    return
                self._json({"ok": True, "token": parts[1], "items": _list_presets()})
                return
        self.send_response(404); self.end_headers()

    # ── ONVIF 分发 ───────────────────────────────────────────────────
    def _onvif_handle(self, svc: str) -> None:
        length = int(self.headers.get("Content-Length", 0))
        body   = self.rfile.read(length) if length else b""

        # 检测请求 SOAP 版本，同步到线程局部变量供 _soap_wrap 使用
        req_ct = self.headers.get("Content-Type", "")
        _soap_tls.soap12 = "application/soap+xml" in req_ct

        action, el = _parse_soap_action(body)
        host = _onvif_host(self)

        # 调试日志：打印每一次 ONVIF 请求，方便排查 NVT 空白问题
        soap_ver = "SOAP1.2" if _soap_tls.soap12 else "SOAP1.1"
        client_ip = self.client_address[0]
        print(f"[ONVIF] {client_ip} → {svc}.{action}  [{soap_ver}]  CT={req_ct[:40] if req_ct else '(none)'}", flush=True)

        if svc == "device":
            resp = _onvif_device(action, el, host)
        elif svc == "media":
            resp = _onvif_media(action, el, host, self.client_address[0])
        elif svc == "imaging":
            resp = _onvif_imaging(action, el)
        else:
            resp = _onvif_ptz(action, el)

        # Content-Type 与请求 SOAP 版本保持一致：
        #   SOAP 1.2 客户端（.NET WCF 等） → application/soap+xml
        #   SOAP 1.1 客户端（中文 ONVIF 工具等） → text/xml
        resp_ct = ("application/soap+xml; charset=utf-8"
                   if _soap_tls.soap12 else "text/xml; charset=utf-8")
        print(f"[ONVIF] → 响应 {svc}.{action}  {len(resp)}B  CT={resp_ct}", flush=True)
        self.send_response(200)
        self.send_header("Content-Type",   resp_ct)
        self.send_header("Content-Length", str(len(resp)))
        self._cors()
        self.end_headers()
        self.wfile.write(resp)

    # ── 代理工具 ─────────────────────────────────────────────────────
    def _preset_post(self, path: str) -> None:
        parts = [p for p in path.split("/") if p]
        if len(parts) < 2 or parts[0] != "presets":
            self.send_response(404); self.end_headers(); return
        token = parts[1]
        if token != _STARTUP_PRESET_TOKEN and token not in _preset_token_order():
            self._json({"ok": False, "error": "token 超出范围"}, 400)
            return

        if len(parts) == 3 and parts[2] == "goto":
            preset = _get_preset(token)
            if preset is None:
                self._json({"ok": False, "error": "预置位不存在"}, 404)
                return
            resolved = _resolve_preset_semantic_ptz(preset)
            if not _set_ptz_to_isaac(resolved["pan"], resolved["tilt"], resolved["zoom"]):
                self._json({"ok": False, "error": "Isaac PTZ 下发失败，详见 PTZ-Launcher 控制台日志"}, 502)
                return
            self._json({"ok": True, "item": preset, "resolved": resolved})
            return

        if token == _STARTUP_PRESET_TOKEN:
            self._json({"ok": False, "error": "StartupView is readonly"}, 405)
            return

        if len(parts) != 2:
            self.send_response(404); self.end_headers(); return

        length = int(self.headers.get("Content-Length", 0))
        req = {}
        if length:
            try:
                req = json.loads(self.rfile.read(length))
            except Exception:
                self._json({"ok": False, "error": "JSON 无效"}, 400)
                return
        ptz = req.get("ptz") if isinstance(req.get("ptz"), dict) else _get_ptz_from_isaac()
        try:
            ptz_norm = {
                "pan": float(ptz["pan"]),
                "tilt": float(ptz["tilt"]),
                "zoom": float(ptz["zoom"]),
            }
        except Exception:
            self._json({"ok": False, "error": "ptz 参数无效"}, 400)
            return
        preset = _save_preset(token, req.get("name"), ptz_norm)
        self._json({"ok": True, "item": preset, "items": _list_presets()})

    def _proxy_post(self, path: str, body: bytes) -> None:
        try:
            if path in ("/scene/randomize", "/api/scene/randomize", "/scene/activate_diaolan"):
                timeout_s = 360
            elif path == "/scene/experiment":
                timeout_s = 60
            elif path in ("/scene/safety-components/status", "/scene/safety-components/apply"):
                timeout_s = 120
            elif path in (
                "/control",
                "/scene/hdri/group",
                "/scene/hdri/select",
                "/scene/hdri/random",
                "/scene/environment",
                "/scene/dynamic-sky-preset",
            ):
                timeout_s = 25
            else:
                timeout_s = 12
            req = urllib.request.Request(
                f"http://127.0.0.1:{ISAAC_PORT}{path}",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=timeout_s) as r:
                resp = r.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(resp)
        except urllib.error.HTTPError as e:
            # POST /api/scene/randomize/last：上游 4xx/5xx 的 JSON 须原样透传，避免把 HTTPError 包装成 502
            if path != "/api/scene/randomize/last":
                self._json({"ok": False, "error": str(e)}, 502)
                return
            try:
                err_body = e.read()
            except Exception:
                err_body = b""
            code = e.code if isinstance(e.code, int) else 502
            self.send_response(code)
            ct = ""
            if e.headers:
                ct = (e.headers.get("Content-Type") or "").strip()
            if not ct:
                ct = "application/json; charset=utf-8"
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(err_body)))
            self._cors()
            self.end_headers()
            self.wfile.write(err_body)
        except Exception as e:
            self._json({"ok": False, "error": str(e)}, 502)

    def _onvif_proxy_handle(self) -> None:
        """
        POST /onvif-proxy
        body(JSON):
          - mode: "soap" | "snapshot"
          - target_url/url: 目标地址（必填）
          - body/soap: SOAP XML 字符串（mode=soap 时建议提供）
          - headers: 透传请求头
          - username/password: 可选，若未显式提供 Authorization，则仅自动补 Basic
        说明：
          - 当前代理仅低成本支持 Basic 认证透传
          - Digest / WS-Security UsernameToken 暂未实现
        """
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b""
        try:
            reqj = json.loads(raw.decode("utf-8", errors="ignore")) if raw else {}
        except Exception:
            self._json({"ok": False, "error": "JSON 无效"}, 400)
            return

        mode = str(reqj.get("mode") or "").strip().lower()
        target_url = str(reqj.get("target_url") or reqj.get("url") or "").strip()
        body_txt = reqj.get("body")
        if body_txt is None:
            body_txt = reqj.get("soap")
        pass_headers = reqj.get("headers") if isinstance(reqj.get("headers"), dict) else {}
        username = str(reqj.get("username") or "").strip()
        password = str(reqj.get("password") or "")

        if mode not in ("soap", "snapshot"):
            self._json({"ok": False, "error": "mode 必须为 soap 或 snapshot"}, 400)
            return
        if not (target_url.startswith("http://") or target_url.startswith("https://")):
            self._json({"ok": False, "error": "target_url 必须是 http/https 地址"}, 400)
            return

        if mode == "soap":
            out_body = (body_txt or "").encode("utf-8", errors="ignore")
            method = "POST"
            default_ct = "application/soap+xml; charset=utf-8"
        else:
            out_body = None
            method = "GET"
            default_ct = ""

        out_headers = {}
        auth = pass_headers.get("Authorization")
        if isinstance(auth, str) and auth.strip():
            out_headers["Authorization"] = auth.strip()
        elif username or password:
            basic_token = base64.b64encode(f"{username}:{password}".encode("utf-8")).decode("ascii")
            out_headers["Authorization"] = f"Basic {basic_token}"

        content_type = pass_headers.get("Content-Type")
        if mode == "soap":
            out_headers["Content-Type"] = (
                content_type.strip() if isinstance(content_type, str) and content_type.strip() else default_ct
            )
            soap_action = pass_headers.get("SOAPAction")
            if isinstance(soap_action, str) and soap_action.strip():
                out_headers["SOAPAction"] = soap_action.strip()

        try:
            req = urllib.request.Request(
                target_url,
                data=out_body,
                headers=out_headers,
                method=method,
            )
            with urllib.request.urlopen(req, timeout=8) as r:
                data = r.read()
                ct = r.headers.get("Content-Type", "application/octet-stream")
                status = r.status
            self.send_response(status)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)
        except urllib.error.HTTPError as e:
            try:
                err_body = e.read()
            except Exception:
                err_body = str(e).encode("utf-8", errors="ignore")
            self.send_response(e.code or 502)
            self.send_header("Content-Type", e.headers.get("Content-Type", "text/plain; charset=utf-8"))
            self.send_header("Content-Length", str(len(err_body)))
            www_auth = e.headers.get("WWW-Authenticate", "")
            if "digest" in str(www_auth).lower():
                self.send_header("X-PTZ-Proxy-Auth-Limit", "basic-only")
            self._cors()
            self.end_headers()
            self.wfile.write(err_body)
        except Exception as e:
            self._json({"ok": False, "error": str(e)}, 502)

    def _snap_handle(self) -> None:
        """
        返回最新真实 JPEG 快照：
          1. 优先向 Isaac Sim 实时拉取（timeout=3s）
          2. 失败时回退到 launcher 内的最近缓存帧（若存在）
          3. 两者均无时明确返回 503，不再用占位图伪装成功
        """
        jpeg = None
        source = None
        snap_headers = {}

        # ① 实时拉取
        jpeg, source, snap_headers = _get_snapshot_proxy_data()

        # ② 回退缓存帧
        if jpeg is None:
            with _ws_cache_lock:
                jpeg = _ws_cache.get("jpeg")
            if jpeg:
                source = "launcher-ws-cache"

        if jpeg is None:
            try:
                self.send_response(503)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Cache-Control", "no-cache, no-store")
                self.send_header("X-PTZ-Snapshot-Ready", "0")
                self.send_header("X-PTZ-Snapshot-Source", "unavailable")
                self._cors()
                self.end_headers()
                self.wfile.write(b"snapshot upstream unavailable")
            except Exception:
                pass
            return

        try:
            self.send_response(200)
            self.send_header("Content-Type",   "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self.send_header("X-PTZ-Snapshot-Ready", "1")
            self.send_header("X-PTZ-Snapshot-Source", source or "unknown")
            _send_snapshot_meta_headers(self, snap_headers, skip={"X-PTZ-Snapshot-Ready"})
            self._cors()
            self.end_headers()
            self.wfile.write(jpeg)
        except Exception:
            pass

    def _proxy_once(self, url: str) -> None:
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                data = r.read()
                ct   = r.headers.get("Content-Type", "application/octet-stream")
            self.send_response(200)
            self.send_header("Content-Type",   ct)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(data)
        except Exception as e:
            try:
                el = str(type(e).__name__).lower()
                em = str(e).lower()
                is_to = bool(
                    isinstance(e, TimeoutError)
                    or isinstance(e, socket.timeout)
                    or "timeout" in el
                    or "timed out" in em
                )
                err_body = json.dumps(
                    {
                        "ok": False,
                        "error": "upstream_unreachable",
                        "upstream": "8081",
                        "upstream_timeout": is_to,
                        "control_degraded": True,
                        "advice": "8081_control_plane_unreachable",
                    },
                    ensure_ascii=False,
                ).encode("utf-8")
                self.send_response(503)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(err_body)))
                self.send_header("Cache-Control", "no-store")
                self._cors()
                self.end_headers()
                self.wfile.write(err_body)
            except Exception:
                pass

    # ── WebSocket 升级 & 推流 ────────────────────────────────────────
    def _ws_handle(self) -> None:
        """完成 WebSocket 握手，持续推送 JPEG 帧直到客户端断开。"""
        ws_key = self.headers.get("Sec-WebSocket-Key", "")
        if not ws_key:
            self.send_response(400); self.end_headers()
            return

        # RFC 6455 握手
        magic   = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept  = base64.b64encode(
            hashlib.sha1((ws_key + magic).encode()).digest()
        ).decode()

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade",              "websocket")
        self.send_header("Connection",           "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        last_fid = -1
        sock_file = self.connection.makefile("wb", buffering=0)
        try:
            while True:
                with _ws_cache_lock:
                    fid  = _ws_cache["frame_id"]
                    jpeg = _ws_cache["jpeg"]
                if fid == last_fid or jpeg is None:
                    time.sleep(0.015)
                    continue
                last_fid = fid
                _ws_send(sock_file, jpeg)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            try:
                sock_file.close()
            except Exception:
                pass

    # ── WebSocket FLV+H.264 推流 ─────────────────────────────────────
    def _ws_flv_handle(self) -> None:
        """完成 WebSocket 握手，向客户端推送 FLV+H.264 流（兼容 flv.js / jessibuca）。"""
        global _flv_cid_counter

        ws_key = self.headers.get("Sec-WebSocket-Key", "")
        if not ws_key:
            self.send_response(400); self.end_headers()
            return

        magic  = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        accept = base64.b64encode(
            hashlib.sha1((ws_key + magic).encode()).digest()
        ).decode()

        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade",              "websocket")
        self.send_header("Connection",           "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()

        sock_file = self.connection.makefile("wb", buffering=0)
        lock = threading.Lock()

        # 等待 FLV 初始化包就绪（最多 20 秒）
        if not _flv_init_ready.wait(timeout=20) or _flv_init_buf is None:
            try: sock_file.close()
            except Exception: pass
            return

        # 先发送初始化包（FLV 文件头 + metadata + AVC 序列头）
        try:
            _ws_send(sock_file, _flv_init_buf)
        except Exception:
            try: sock_file.close()
            except Exception: pass
            return

        # 注册为活跃客户端，等下一个关键帧后开始接收实时数据
        with _flv_clients_lock:
            _flv_cid_counter += 1
            cid = _flv_cid_counter
            _flv_clients[cid] = {"sock_file": sock_file, "lock": lock, "state": "waiting_kf"}

        try:
            while True:
                with _flv_clients_lock:
                    if cid not in _flv_clients:
                        break
                time.sleep(1)
        finally:
            with _flv_clients_lock:
                _flv_clients.pop(cid, None)
            try: sock_file.close()
            except Exception: pass


# ── 主函数 ───────────────────────────────────────────────────────────
def _sync_isaac_state_on_startup() -> None:
    """Launcher 启动时探测 Isaac Sim 是否已在运行，同步内部状态。"""
    global _isaac_state, _start_time
    if _port_in_use(ISAAC_PORT):
        preview = bool(cfg.get("preview_enabled", True))
        if _isaac_deemed_ready_for_launcher(preview):
            _isaac_state = "running"
            _start_time  = time.time()
            print(f"[PTZ-Launcher] 检测到 Isaac Sim 已稳定就绪（端口 {ISAAC_PORT}），状态同步为 running")
        else:
            _isaac_state = "starting"
            _start_time  = time.time()
            print(
                f"[PTZ-Launcher] 检测到端口 {ISAAC_PORT} 占用但尚未满足稳定就绪条件，"
                "状态同步为 starting（需 /api/stream_ready 等）"
            )


def main():
    global _WS_FPS
    _WS_FPS = cfg.get("fps", 25)

    # Launcher 重启后同步 Isaac Sim 实际运行状态
    _sync_isaac_state_on_startup()

    # preview_enabled=false 时关闭预览线程，减少不必要的轮询与 GIL 竞争
    _preview_enabled = cfg.get("preview_enabled", True)
    if _preview_enabled:
        # 启动 WebSocket JPEG 帧拉取后台线程
        threading.Thread(target=_ws_frame_fetcher, daemon=True, name="ws-fetcher").start()
        # 启动 WebSocket FLV+H.264 广播器线程
        threading.Thread(target=_flv_broadcaster, daemon=True, name="flv-broadcaster").start()
    else:
        print("[PTZ-Launcher] preview_enabled=false，ws-fetcher 和 flv-broadcaster 已禁用")
    # 启动 WS-Discovery UDP 多播监听线程（局域网 ONVIF 自动发现）
    threading.Thread(target=_wsd_listener, daemon=True, name="wsd-discovery").start()

    srv = ThreadingHTTPServer(("0.0.0.0", LAUNCHER_PORT), _Handler)
    local_ips = _get_local_ips()
    lan_ip    = local_ips[0] if local_ips else "localhost"
    print(f"[PTZ-Launcher] ONVIF 仿真服务运行中 → http://localhost:{LAUNCHER_PORT}/")
    print(f"[PTZ-Launcher] ONVIF 设备服务（局域网）：http://{lan_ip}:{LAUNCHER_PORT}/onvif/device_service")
    print(f"[PTZ-Launcher] ONVIF 快照端点：http://{lan_ip}:{LAUNCHER_PORT}/onvif-snap.jpg")
    print(f"[PTZ-Launcher] RTSP 视频流：rtsp://{lan_ip}:{_RTSP_PORT}/ptz_cam  (VLC / NVR)")
    print(f"[PTZ-Launcher] WS-FLV 流：ws://localhost:{LAUNCHER_PORT}/ws-flv  (flv.js / jessibuca)")
    print(f"[PTZ-Launcher] WS-Discovery：{_WSD_MCAST_ADDR}:{_WSD_PORT}（设备 UUID={_DEVICE_UUID}）")
    print(f"[PTZ-Launcher] Isaac Sim 内部端口：{ISAAC_PORT}（按需启动）")
    print(f"[PTZ-Launcher] 在 Web 界面点击 '启动仿真' 启动 Isaac Sim")
    print(f"[PTZ-Launcher] 按 Ctrl-C 停止")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[PTZ-Launcher] 正在停止...")
        stop_isaac()
        time.sleep(2)


if __name__ == "__main__":
    main()
