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
import os
import sys

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

scene_path   = _resolve_path(args.scene   or cfg["scene_path"])
camera_prim  = args.camera  or cfg["camera_prim"]
rtsp_url     = args.rtsp    or cfg["rtsp_url"]
fps          = args.fps     or cfg.get("fps", 25)
resolution   = tuple(cfg.get("resolution", [1920, 1080]))
bitrate      = cfg.get("bitrate", "4M")
sim_hz       = cfg.get("sim_hz", 60)
mediamtx_cfg  = cfg.get("mediamtx", {})
rtsp_enabled    = cfg.get("rtsp_enabled",    True)
mjpeg_quality   = cfg.get("mjpeg_quality",   80)
FOCAL_LENGTH_1X = float(cfg.get("focal_length_1x", 18.14756))

W, H = resolution
skip_frames = max(1, round(sim_hz / fps))

print(f"[PTZ-RTSP] 配置：场景={scene_path}")
print(f"[PTZ-RTSP]       相机Prim={camera_prim}")
print(f"[PTZ-RTSP]       RTSP URL={rtsp_url}  分辨率={W}x{H}  fps={fps}")

# 启动 SimulationApp（headless 模式）
from isaacsim import SimulationApp

sim_app = SimulationApp({
    "headless": True,
    "renderer": "RaytracedLighting",
    "width": W,
    "height": H,
})

# ============================================================
# 第二阶段：所有 omni.* 导入在 SimulationApp 启动后进行
# ============================================================
import io as _io
import json
import signal
import subprocess
import tarfile
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer

import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World

# ============================================================
# PTZ 控制 HTTP API（内嵌在仿真进程中，port 8080）
# ============================================================


_CTRL_PORT = args.ctrl_port or cfg.get("ctrl_port", 8080)

_ptz_state = {
    "pan":  0.0,     # -170 ~ +170 度
    "tilt": -45.0,   # -90 ~ +30 度
    "zoom": 1.0,     # 1× ~ 32×
}
_ptz_lock    = threading.Lock()
_ptz_dirty   = threading.Event()
_scene_up_axis = "Y"   # 加载场景后自动更新；Z-up 时改为 "Z"

# ── 场景控制状态（吊篮 + 工人）──────────────────────────────
# 吊篮和工人 prim 路径（内部 Y-up cm 坐标，Y 轴为高度方向）
_GONDOLA_PRIM  = "/World/DiaoLan/Model/Group1"
_WORKER1_PRIM  = "/World/DiaoLan/Model/node______1"
_WORKER2_PRIM  = "/World/DiaoLan/Model/node______2"

_scene_state = {
    "gondola_y": 0.0,    # 吊篮高度（0 ~ 3300，单位：cm 内部坐标）
    "workers":   2,      # 工人数量（0 / 1 / 2）
}
_scene_lock  = threading.Lock()
_scene_dirty = threading.Event()
_chosen_worker: int = 1   # 当 workers=1 时随机显示的工人编号（1 或 2）

# ── MJPEG 帧缓冲（主循环写，HTTP handler 读）────────────────
_mjpeg = {"jpeg": None, "frame_id": 0}
_mjpeg_lock = threading.Lock()
_jpeg_encode_fn = None   # 在 main() 里初始化


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


def _apply_ptz_state(stage) -> None:
    """将 _ptz_state 写入 USD Stage：Pan 旋转、Tilt 旋转、相机焦距。

    自动适配坐标轴：
      Z-up 场景（V4.0/V3.0）：Pan → rotateZ，Tilt → rotateY
      Y-up 场景（PTZ_SecurityDome）：Pan → rotateY，Tilt → rotateZ
    """
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

    # Z-up：水平旋转绕 Z，俯仰绕 Y
    # Y-up：水平旋转绕 Y，俯仰绕 Z
    if _scene_up_axis == "Z":
        pan_attr_name  = "xformOp:rotateZ"
        tilt_attr_name = "xformOp:rotateY"
    else:
        pan_attr_name  = "xformOp:rotateY"
        tilt_attr_name = "xformOp:rotateZ"

    if pan_p.IsValid():
        attr = pan_p.GetAttribute(pan_attr_name)
        if attr and attr.IsValid():
            attr.Set(float(-pan_deg))
    if tilt_p.IsValid():
        attr = tilt_p.GetAttribute(tilt_attr_name)
        if attr and attr.IsValid():
            attr.Set(float(-tilt_deg))
    if cam_p.IsValid():
        attr = cam_p.GetAttribute("focalLength")
        if attr and attr.IsValid():
            attr.Set(float(FOCAL_LENGTH_1X * zoom))


def _set_prim_y(stage, prim_path: str, y_val: float) -> None:
    """将 prim 的 xformOp:translate Y 分量设为 y_val，X/Z 保持不变。"""
    from pxr import Gf as _Gf
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    attr = prim.GetAttribute("xformOp:translate")
    if not (attr and attr.IsValid()):
        return
    cur = attr.Get()
    if cur is not None:
        attr.Set(_Gf.Vec3d(cur[0], float(y_val), cur[2]))
    else:
        attr.Set(_Gf.Vec3d(0.0, float(y_val), 0.0))


def _set_prim_visibility(stage, prim_path: str, visible: bool) -> None:
    """设置 prim 的 visibility 属性（invisible / inherited）。"""
    from pxr import UsdGeom as _UG
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    attr = prim.GetAttribute("visibility")
    if attr and attr.IsValid():
        attr.Set(_UG.Tokens.inherited if visible else _UG.Tokens.invisible)


def _apply_scene_state(stage) -> None:
    """将 _scene_state 写入 USD Stage：吊篮高度 + 工人显示。"""
    global _chosen_worker
    with _scene_lock:
        gondola_y = _scene_state["gondola_y"]
        workers   = _scene_state["workers"]

    # 吊篮高度（Y 方向）
    _set_prim_y(stage, _GONDOLA_PRIM, gondola_y)

    # 工人可见性 + Y 位置跟随吊篮
    if workers == 0:
        _set_prim_visibility(stage, _WORKER1_PRIM, False)
        _set_prim_visibility(stage, _WORKER2_PRIM, False)
    elif workers == 1:
        import random
        _chosen_worker = random.choice([1, 2])
        if _chosen_worker == 1:
            _set_prim_visibility(stage, _WORKER1_PRIM, True)
            _set_prim_visibility(stage, _WORKER2_PRIM, False)
            _set_prim_y(stage, _WORKER1_PRIM, gondola_y)
        else:
            _set_prim_visibility(stage, _WORKER1_PRIM, False)
            _set_prim_visibility(stage, _WORKER2_PRIM, True)
            _set_prim_y(stage, _WORKER2_PRIM, gondola_y)
    else:  # workers == 2
        _set_prim_visibility(stage, _WORKER1_PRIM, True)
        _set_prim_visibility(stage, _WORKER2_PRIM, True)
        _set_prim_y(stage, _WORKER1_PRIM, gondola_y)
        _set_prim_y(stage, _WORKER2_PRIM, gondola_y)


class _PTZHandler(BaseHTTPRequestHandler):
    """轻量 HTTP handler：提供控制面板 HTML 和 REST 接口。"""

    def log_message(self, fmt, *args):  # 静默访问日志
        pass

    def _cors(self):
        self.send_header("Access-Control-Allow-Origin",  "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
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

        if self.path == "/status":
            with _ptz_lock:
                body = json.dumps(_ptz_state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/scene/state":
            with _scene_lock:
                body = json.dumps(_scene_state).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(body)
            return

        # MJPEG 连续流 —— 浏览器一次连接持续接收帧（低延迟）
        if self.path.startswith("/stream.mjpeg"):
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
            with _mjpeg_lock:
                jpg = _mjpeg["jpeg"]
            if jpg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type",   "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.send_header("Cache-Control",  "no-cache, no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(jpg)
            return

        self.send_response(404)
        self.end_headers()

    def do_POST(self):
        if self.path == "/scene/gondola":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body)
                with _scene_lock:
                    if "y" in req:
                        _scene_state["gondola_y"] = max(0.0, min(3300.0, float(req["y"])))
                _scene_dirty.set()
                with _scene_lock:
                    resp = json.dumps({"ok": True, "state": dict(_scene_state)})
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)})
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode())
            return

        if self.path == "/scene/workers":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body)
                with _scene_lock:
                    if "count" in req:
                        _scene_state["workers"] = max(0, min(2, int(req["count"])))
                _scene_dirty.set()
                with _scene_lock:
                    resp = json.dumps({"ok": True, "state": dict(_scene_state)})
            except Exception as exc:
                resp = json.dumps({"ok": False, "error": str(exc)})
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self._cors()
            self.end_headers()
            self.wfile.write(resp.encode())
            return

        if self.path == "/control":
            length = int(self.headers.get("Content-Length", 0))
            body   = self.rfile.read(length)
            try:
                req = json.loads(body)
                with _ptz_lock:
                    if "pan"  in req:
                        _ptz_state["pan"]  = max(-170.0, min(170.0, float(req["pan"])))
                    if "tilt" in req:
                        _ptz_state["tilt"] = max(-90.0,  min(30.0,  float(req["tilt"])))
                    if "zoom" in req:
                        _ptz_state["zoom"] = max(1.0,    min(32.0,  float(req["zoom"])))
                _ptz_dirty.set()
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

        self.send_response(404)
        self.end_headers()


def _start_control_server() -> None:
    """在 daemon 线程中启动 PTZ 控制 HTTP 服务，不阻塞仿真主循环。"""
    srv = ThreadingHTTPServer(("0.0.0.0", _CTRL_PORT), _PTZHandler)
    t   = threading.Thread(target=srv.serve_forever, daemon=True, name="ptz-ctrl")
    t.start()
    print(f"[PTZ-RTSP] 控制面板已启动 → http://localhost:{_CTRL_PORT}/")
    print(f"[PTZ-RTSP]   用浏览器打开上面的地址即可实时控制云台和变焦")


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
# ffmpeg 管道管理
# ============================================================

def _start_ffmpeg(rtsp_url: str, width: int, height: int, fps: int, bitrate: str) -> subprocess.Popen:
    """
    启动 ffmpeg 进程，通过 stdin 接受 rawvideo RGBA 帧，
    编码为 H.264 并以 RTSP 推流到 MediaMTX。
    """
    cmd = [
        "ffmpeg",
        "-loglevel", "warning",
        # 输入：原始视频（来自 stdin）
        "-f", "rawvideo",
        "-pix_fmt", "rgba",
        "-s", f"{width}x{height}",
        "-r", str(fps),
        "-i", "pipe:0",
        # 无音频
        "-an",
        # 编码：H.264，超快预设 + 零延迟调优
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-b:v", bitrate,
        # 强制转换到 yuv420p（标准 H.264 High Profile，兼容所有播放器）
        "-vf", "format=yuv420p",
        # 输出：RTSP（推流默认使用 TCP ANNOUNCE）
        "-f", "rtsp",
        rtsp_url,
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    # 异步打印 ffmpeg 错误输出，不阻塞主线程
    def _log_stderr():
        for line in proc.stderr:
            txt = line.decode(errors="replace").rstrip()
            if txt:
                print(f"[ffmpeg] {txt}")
    threading.Thread(target=_log_stderr, daemon=True).start()

    time.sleep(0.5)
    if proc.poll() is not None:
        raise RuntimeError("ffmpeg 进程意外退出，请检查 rtsp_url 和 mediamtx 是否正常运行。")
    print(f"[PTZ-RTSP] ffmpeg 推流进程已启动（pid={proc.pid}）")
    return proc


# ============================================================
# 相机绑定（Replicator）
# ============================================================

def _bind_camera(camera_prim_path: str, width: int, height: int):
    """
    创建 Replicator RenderProduct 并绑定 RGB Annotator 到指定相机 Prim。
    返回 (render_product, annotator)。
    """
    # 确认 Prim 存在
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(camera_prim_path)
    if not prim.IsValid():
        raise ValueError(
            f"相机 Prim 不存在：{camera_prim_path}\n"
            "请检查 ptz_rtsp_config.yaml 中的 camera_prim 路径是否与场景一致。"
        )

    rp = rep.create.render_product(camera_prim_path, (width, height))
    annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    annotator.attach(rp)
    print(f"[PTZ-RTSP] 相机绑定成功：{camera_prim_path}  RenderProduct={rp}")
    return rp, annotator


# ============================================================
# 优雅退出处理
# ============================================================

_running = True
_ffmpeg_proc = None
_mediamtx_proc = None


def _shutdown(signum=None, frame=None):
    global _running
    print("\n[PTZ-RTSP] 收到退出信号，正在清理...")
    _running = False


signal.signal(signal.SIGINT,  _shutdown)
signal.signal(signal.SIGTERM, _shutdown)


def _cleanup():
    """关闭 ffmpeg 和 mediamtx 子进程。"""
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


# ============================================================
# 主流程
# ============================================================

def main():
    global _ffmpeg_proc, _mediamtx_proc, _jpeg_encode_fn

    # --- 初始化 MJPEG JPEG 编码器 ---
    _jpeg_encode_fn = _init_jpeg_encoder(mjpeg_quality)

    # --- 启动 PTZ 控制 HTTP API（含 MJPEG 端点）---
    _start_control_server()

    # --- RTSP 管道（可选）---
    if rtsp_enabled:
        mtx_bin = _ensure_mediamtx()
        _mediamtx_proc = _start_mediamtx(mtx_bin)
        _ffmpeg_proc = _start_ffmpeg(rtsp_url, W, H, fps, bitrate)
    else:
        print(f"[PTZ-RTSP] RTSP 已禁用，仅 MJPEG 预览")
        print(f"[PTZ-RTSP] 预览地址：http://localhost:{_CTRL_PORT}/stream.mjpeg")

    # --- 加载 USD 场景 ---
    print(f"[PTZ-RTSP] 加载场景：{scene_path}")
    usd_context = omni.usd.get_context()
    result = usd_context.open_stage(scene_path)
    # open_stage 根据版本返回 bool 或 (bool, str)
    if isinstance(result, tuple):
        ok, err = result
    else:
        ok, err = result, ""
    if not ok:
        raise RuntimeError(f"加载场景失败：{err}")

    # 等待场景完全加载
    sim_app.update()
    sim_app.update()

    # --- 读取场景坐标轴（自动适配 Pan/Tilt 旋转轴）---
    global _scene_up_axis
    try:
        import omni.usd as _ousd
        from pxr import UsdGeom as _UsdGeom
        _scene_up_axis = _UsdGeom.GetStageUpAxis(_ousd.get_context().get_stage())
        print(f"[PTZ-RTSP] 场景 upAxis={_scene_up_axis}  "
              f"Pan={'rotateZ' if _scene_up_axis=='Z' else 'rotateY'}  "
              f"Tilt={'rotateY' if _scene_up_axis=='Z' else 'rotateZ'}")
    except Exception as e:
        print(f"[PTZ-RTSP] upAxis 读取失败（使用默认 Y-up）：{e}")

    # --- 初始化 World ---
    stage_mpu = 0.01 if _scene_up_axis == "Y" else 1.0
    world = World(stage_units_in_meters=stage_mpu)
    world.reset()

    # 让渲染管线稳定（需要几帧预热）
    for _ in range(10):
        world.step(render=True)

    # --- 绑定相机 ---
    rp, annotator = _bind_camera(camera_prim, W, H)

    # 预热 Replicator（确保第一帧数据有效）
    rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
    sim_app.update()

    print(f"[PTZ-RTSP] 开始推流 → {rtsp_url}")
    print(f"[PTZ-RTSP] 客户端命令：vlc {rtsp_url}  或  ffplay {rtsp_url} -rtsp_transport tcp")
    print("[PTZ-RTSP] 按 Ctrl-C 停止。\n")

    # --- 主仿真 + 推流循环 ---
    frame_idx = 0
    last_stat_time = time.time()
    pushed_frames = 0

    while _running and sim_app.is_running():
        world.step(render=True)

        # 应用来自 Web UI 的 PTZ 指令（检测到脏标志时写入 USD Stage）
        if _ptz_dirty.is_set():
            _ptz_dirty.clear()
            _apply_ptz_state(omni.usd.get_context().get_stage())

        # 应用场景控制指令（吊篮高度 + 工人数量）
        if _scene_dirty.is_set():
            _scene_dirty.clear()
            _apply_scene_state(omni.usd.get_context().get_stage())

        # 按 skip_frames 跳帧，使推流帧率 ≈ fps
        if frame_idx % skip_frames == 0:
            # 触发 Replicator 更新 annotator 数据
            rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)

            rgba = annotator.get_data()  # shape (H, W, 4), dtype uint8

            if rgba is not None and rgba.size > 0:
                # 验证 shape 与配置分辨率一致，不一致时 resize 避免花屏
                if rgba.shape[:2] != (H, W):
                    rgba = np.ascontiguousarray(
                        np.resize(rgba, (H, W, rgba.shape[2] if rgba.ndim == 3 else 4))
                    )

                # ① MJPEG 帧缓冲更新（低延迟浏览器预览）
                if _jpeg_encode_fn is not None:
                    jpg = _jpeg_encode_fn(rgba)
                    if jpg:
                        with _mjpeg_lock:
                            _mjpeg["jpeg"]     = jpg
                            _mjpeg["frame_id"] += 1

                # ② RTSP 推流（可选，供 VLC / NVR 外部接入）
                if rtsp_enabled and _ffmpeg_proc is not None \
                        and _ffmpeg_proc.poll() is None:
                    raw = np.ascontiguousarray(rgba, dtype=np.uint8).tobytes()
                    try:
                        _ffmpeg_proc.stdin.write(raw)
                        _ffmpeg_proc.stdin.flush()
                        pushed_frames += 1
                    except BrokenPipeError:
                        print("[PTZ-RTSP] ffmpeg 管道已断开（RTSP 停止，MJPEG 继续）")
                        _ffmpeg_proc = None

                # 每 5 秒打印一次统计
                now = time.time()
                if now - last_stat_time >= 5.0:
                    elapsed = now - last_stat_time
                    actual_fps = pushed_frames / elapsed if rtsp_enabled else \
                        _mjpeg["frame_id"] / elapsed
                    print(f"[PTZ-RTSP] 运行中... 帧率={actual_fps:.1f} fps  仿真帧={frame_idx}")
                    pushed_frames = 0
                    last_stat_time = now

        frame_idx += 1

    # ffmpeg 管道可能已关闭，不再写入
    _cleanup()
    sim_app.close()
    print("[PTZ-RTSP] 已退出。")


if __name__ == "__main__":
    main()
