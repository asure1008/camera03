from __future__ import annotations

import io as _io
import json
import os
import random
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import carb
import numpy as np
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api import World

from .stream_media import ensure_mediamtx, start_ffmpeg, start_mediamtx


def resize_rgba_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    src_h, src_w = frame.shape[:2]
    if (src_h, src_w) == (height, width):
        return np.ascontiguousarray(frame)
    y_idx = np.linspace(0, src_h - 1, height).astype(np.int64)
    x_idx = np.linspace(0, src_w - 1, width).astype(np.int64)
    return np.ascontiguousarray(frame[y_idx][:, x_idx])


class StreamRuntime:
    def __init__(self, sim_app: Any, settings: dict[str, Any]) -> None:
        self.sim_app = sim_app
        self.settings = settings

        self.script_dir = settings["script_dir"]
        self.scene_path = settings["scene_path"]
        self.camera_prim = settings["camera_prim"]
        self.rtsp_url = settings["rtsp_url"]
        self.fps = int(settings["fps"])
        self.width, self.height = settings["resolution"]
        self.bitrate = settings["bitrate"]
        self.sim_hz = int(settings["sim_hz"])
        self.mediamtx_cfg = settings["mediamtx_cfg"]
        self.rtsp_enabled = bool(settings["rtsp_enabled"])
        self.mjpeg_quality = int(settings["mjpeg_quality"])
        self.focal_length_1x = float(settings["focal_length_1x"])
        self.ctrl_port = int(settings["ctrl_port"])

        self._running = True
        self._scene_up_axis = "Y"

        self._ptz_state = {"pan": 0.0, "tilt": 0.0, "zoom": 1.0}
        self._ptz_ref = {
            "pan_zero_op": 0.0,
            "tilt_zero_op": 0.0,
            "pan_sign": -1.0,
            "tilt_sign": 1.0,
        }
        self._ptz_lock = threading.Lock()
        self._ptz_dirty = threading.Event()

        self._gondola_prim = "/World/DiaoLan/Model/Group1"
        self._worker1_prim = "/World/DiaoLan/Model/node______1"
        self._worker2_prim = "/World/DiaoLan/Model/node______2"
        self._scene_state = {"gondola_y": 0.0, "workers": 2}
        self._scene_lock = threading.Lock()
        self._scene_dirty = threading.Event()
        self._chosen_worker = 1

        self._mjpeg = {"jpeg": None, "frame_id": 0}
        self._mjpeg_lock = threading.Lock()
        self._jpeg_encode_fn = None

        self._ffmpeg_proc = None
        self._mediamtx_proc = None
        self._ready = False
        self._last_good_rgba: np.ndarray | None = None
        self._pan_attr_name_override: str | None = None
        self._tilt_attr_name_override: str | None = None
        self._runtime_cache_root = os.path.join(self.script_dir, ".runtime_cache")
        self._runtime_ov_cache = os.path.join(self._runtime_cache_root, "ov_cache")
        self._runtime_kit_cache = os.path.join(self._runtime_cache_root, "kit_cache")

    def _apply_render_throttle(self) -> None:
        settings = carb.settings.get_settings()
        target_fps = max(1, self.fps)
        settings.set("/rtx/ecoMode/enabled", False)
        settings.set("/rtx/ecoMode/maxFramesWithoutChange", 1)
        settings.set("/app/asyncRendering", False)
        settings.set("/app/asyncRenderingLowLatency", False)
        settings.set("/rtx/rendermode", "PathTracing")
        settings.set(
            "/rtx-transient/resourcemanager/localTextureCachePath",
            os.path.join(self._runtime_ov_cache, "texturecache").replace("\\", "/"),
        )
        for loop in ("main", "rendering_0", "rendering_1", "present"):
            settings.set(f"/app/runLoops/{loop}/rateLimitEnabled", True)
            settings.set(f"/app/runLoops/{loop}/rateLimitFrequency", target_fps)
        settings.set("/rtx/ambientOcclusion/enabled", False)
        settings.set("/rtx/reflections/enabled", False)
        settings.set("/rtx/translucency/enabled", False)
        settings.set("/rtx/post/aa/op", 0)
        settings.set("/rtx/directLighting/sampledLighting/enabled", False)
        settings.set("/exts/isaacsim.core.throttling/enable_async", False)
        settings.set("/exts/isaacsim.core.throttling/enable_manualmode", False)

    def _configure_runtime_cache(self) -> None:
        os.makedirs(self._runtime_cache_root, exist_ok=True)
        os.makedirs(self._runtime_ov_cache, exist_ok=True)
        os.makedirs(self._runtime_kit_cache, exist_ok=True)
        os.makedirs(os.path.join(self._runtime_ov_cache, "texturecache"), exist_ok=True)
        try:
            import carb.tokens

            tokens = carb.tokens.get_tokens_interface()
            tokens.set_value("omni_global_cache", self._runtime_ov_cache.replace("\\", "/"))
            tokens.set_value("cache", self._runtime_kit_cache.replace("\\", "/"))
            print(f"[PTZ-RTSP] cache redirected: omni_global_cache={tokens.resolve('${omni_global_cache}')}")
            print(f"[PTZ-RTSP] cache redirected: cache={tokens.resolve('${cache}')}")
        except Exception as exc:
            print(f"[PTZ-RTSP] warning: cache token override failed: {exc}")

    def _init_jpeg_encoder(self):
        try:
            from PIL import Image as PILImage

            def _encode(rgba: np.ndarray) -> bytes:
                image = PILImage.fromarray(rgba[:, :, :3])
                buffer = _io.BytesIO()
                image.save(buffer, "JPEG", quality=self.mjpeg_quality, optimize=False)
                return buffer.getvalue()

            return _encode
        except ImportError:
            pass

        try:
            import cv2

            params = [cv2.IMWRITE_JPEG_QUALITY, self.mjpeg_quality]

            def _encode(rgba: np.ndarray) -> bytes | None:
                bgr = rgba[:, :, :3][:, :, ::-1].copy()
                ok, buffer = cv2.imencode(".jpg", bgr, params)
                return buffer.tobytes() if ok else None

            return _encode
        except ImportError:
            return None

    def _default_ptz_op_names(self) -> tuple[str, str]:
        if self._scene_up_axis == "Z":
            return "xformOp:rotateZ", "xformOp:rotateY"
        return "xformOp:rotateY", "xformOp:rotateZ"

    def _ptz_op_names(self) -> tuple[str, str]:
        if self._pan_attr_name_override and self._tilt_attr_name_override:
            return self._pan_attr_name_override, self._tilt_attr_name_override
        return self._default_ptz_op_names()

    def _ptz_paths(self) -> tuple[str, str]:
        parts = [part for part in self.camera_prim.split("/") if part]
        if not parts:
            return "/", "/"

        # Typical PTZ rigs use .../Pan/Tilt/Camera. Some external scenes expose
        # shallow camera prims like /World/Camera_Follow; keep those valid too.
        if len(parts) >= 3:
            pan_parts = parts[:-2]
            tilt_parts = parts[:-1]
        else:
            pan_parts = parts
            tilt_parts = parts

        return "/" + "/".join(pan_parts), "/" + "/".join(tilt_parts)

    @staticmethod
    def _has_attr(prim, attr_name: str) -> bool:
        if not prim.IsValid():
            return False
        attr = prim.GetAttribute(attr_name)
        return bool(attr and attr.IsValid())

    @staticmethod
    def _ensure_rotate_attr(prim, attr_name: str) -> None:
        if not prim.IsValid():
            return
        if StreamRuntime._has_attr(prim, attr_name):
            return

        try:
            from pxr import UsdGeom

            xformable = UsdGeom.Xformable(prim)
            if attr_name == "xformOp:rotateX":
                xformable.AddRotateXOp()
            elif attr_name == "xformOp:rotateY":
                xformable.AddRotateYOp()
            elif attr_name == "xformOp:rotateZ":
                xformable.AddRotateZOp()
        except Exception:
            return

    @staticmethod
    def _read_attr_scalar(prim, attr_name: str, default: float = 0.0) -> float:
        if not prim.IsValid():
            return default
        attr = prim.GetAttribute(attr_name)
        if not (attr and attr.IsValid()):
            return default
        try:
            return float(attr.Get())
        except Exception:
            return default

    @staticmethod
    def _norm3(vec) -> float:
        return float((vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]) ** 0.5)

    def _normalize3(self, vec):
        norm = self._norm3(vec)
        if norm < 1e-8:
            return None
        return (float(vec[0] / norm), float(vec[1] / norm), float(vec[2] / norm))

    @staticmethod
    def _dot3(a, b) -> float:
        return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])

    @staticmethod
    def _cross3(a, b):
        return (
            float(a[1] * b[2] - a[2] * b[1]),
            float(a[2] * b[0] - a[0] * b[2]),
            float(a[0] * b[1] - a[1] * b[0]),
        )

    @staticmethod
    def _sub3(a, b):
        return (float(a[0] - b[0]), float(a[1] - b[1]), float(a[2] - b[2]))

    def _project_to_horizontal(self, vec, up):
        dot = self._dot3(vec, up)
        flat = self._sub3(vec, (up[0] * dot, up[1] * dot, up[2] * dot))
        return self._normalize3(flat)

    def _camera_forward_world(self, stage):
        from pxr import Gf, UsdGeom

        camera_prim = stage.GetPrimAtPath(self.camera_prim)
        if not camera_prim.IsValid():
            return None
        cache = UsdGeom.XformCache()
        matrix = cache.GetLocalToWorldTransform(camera_prim)
        forward = matrix.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
        return self._normalize3(forward)

    def _detect_pan_sign(self, stage, pan_prim, pan_attr_name: str, pan_zero: float, default_sign: float = -1.0) -> float:
        if not pan_prim.IsValid():
            return default_sign
        attr = pan_prim.GetAttribute(pan_attr_name)
        if not (attr and attr.IsValid()):
            return default_sign

        up = (0.0, 0.0, 1.0) if self._scene_up_axis == "Z" else (0.0, 1.0, 0.0)
        probe = 5.0

        def _forward_horizontal():
            forward = self._camera_forward_world(stage)
            if forward is None:
                return None
            return self._project_to_horizontal(forward, up)

        old_value = self._read_attr_scalar(pan_prim, pan_attr_name, pan_zero)
        try:
            attr.Set(float(pan_zero))
            base = _forward_horizontal()
            if base is None:
                return default_sign

            attr.Set(float(pan_zero + probe))
            probe_forward = _forward_horizontal()
            if probe_forward is None:
                return default_sign

            right = self._normalize3(self._cross3(base, up))
            if right is None:
                return default_sign

            delta = self._sub3(probe_forward, base)
            score = self._dot3(delta, right)
            if abs(score) < 1e-6:
                return default_sign
            return 1.0 if score > 0 else -1.0
        finally:
            try:
                attr.Set(float(old_value))
            except Exception:
                pass

    def _detect_tilt_sign(self, stage, tilt_prim, tilt_attr_name: str, tilt_zero: float, default_sign: float = 1.0) -> float:
        if not tilt_prim.IsValid():
            return default_sign
        attr = tilt_prim.GetAttribute(tilt_attr_name)
        if not (attr and attr.IsValid()):
            return default_sign

        up = (0.0, 0.0, 1.0) if self._scene_up_axis == "Z" else (0.0, 1.0, 0.0)
        probe = 5.0
        old_value = self._read_attr_scalar(tilt_prim, tilt_attr_name, tilt_zero)
        try:
            attr.Set(float(tilt_zero))
            base = self._camera_forward_world(stage)
            if base is None:
                return default_sign
            base_up = self._dot3(base, up)

            attr.Set(float(tilt_zero + probe))
            probe_forward = self._camera_forward_world(stage)
            if probe_forward is None:
                return default_sign
            probe_up = self._dot3(probe_forward, up)

            delta = probe_up - base_up
            if abs(delta) < 1e-6:
                return default_sign
            return 1.0 if delta > 0 else -1.0
        finally:
            try:
                attr.Set(float(old_value))
            except Exception:
                pass

    def _auto_select_shallow_axes(self, stage, camera_prim) -> tuple[str, str]:
        default_pan_attr, default_tilt_attr = self._default_ptz_op_names()
        candidates = ("xformOp:rotateX", "xformOp:rotateY", "xformOp:rotateZ")

        for attr_name in candidates:
            self._ensure_rotate_attr(camera_prim, attr_name)

        up = (0.0, 0.0, 1.0) if self._scene_up_axis == "Z" else (0.0, 1.0, 0.0)
        base_forward = self._camera_forward_world(stage)
        if base_forward is None:
            return default_pan_attr, default_tilt_attr
        base_h = self._project_to_horizontal(base_forward, up)
        right = self._normalize3(self._cross3(base_h, up)) if base_h is not None else None
        base_up = self._dot3(base_forward, up)

        scores_pan: dict[str, float] = {}
        scores_tilt: dict[str, float] = {}
        probe = 5.0
        original_values = {name: self._read_attr_scalar(camera_prim, name, 0.0) for name in candidates}

        for attr_name in candidates:
            attr = camera_prim.GetAttribute(attr_name)
            if not (attr and attr.IsValid()):
                continue

            old_value = original_values.get(attr_name, 0.0)
            try:
                attr.Set(float(old_value + probe))
                probe_forward = self._camera_forward_world(stage)
            finally:
                try:
                    attr.Set(float(old_value))
                except Exception:
                    pass

            if probe_forward is None:
                continue

            probe_h = self._project_to_horizontal(probe_forward, up)
            if base_h is not None and probe_h is not None and right is not None:
                delta_h = self._sub3(probe_h, base_h)
                scores_pan[attr_name] = abs(self._dot3(delta_h, right))
            else:
                scores_pan[attr_name] = 0.0
            scores_tilt[attr_name] = abs(self._dot3(probe_forward, up) - base_up)

        pan_attr = max(scores_pan, key=scores_pan.get, default=default_pan_attr)
        pan_score = scores_pan.get(pan_attr, 0.0)
        if pan_score < 1e-4:
            pan_attr = default_pan_attr

        tilt_candidates = {k: v for k, v in scores_tilt.items() if k != pan_attr}
        tilt_attr = max(tilt_candidates, key=tilt_candidates.get, default=default_tilt_attr)
        tilt_score = tilt_candidates.get(tilt_attr, 0.0)
        if tilt_score < 1e-4:
            tilt_attr = default_tilt_attr if default_tilt_attr != pan_attr else tilt_attr

        print(
            f"[PTZ-RTSP] shallow camera axis select: pan_attr={pan_attr} pan_score={pan_score:.6f}, "
            f"tilt_attr={tilt_attr} tilt_score={scores_tilt.get(tilt_attr, 0.0):.6f}"
        )
        return pan_attr, tilt_attr

    def _init_ptz_reference(self, stage) -> None:
        pan_path, tilt_path = self._ptz_paths()
        pan_prim = stage.GetPrimAtPath(pan_path)
        tilt_prim = stage.GetPrimAtPath(tilt_path)
        camera_prim = stage.GetPrimAtPath(self.camera_prim)

        self._pan_attr_name_override = None
        self._tilt_attr_name_override = None
        pan_attr_name, tilt_attr_name = self._default_ptz_op_names()
        if pan_path == self.camera_prim and tilt_path == self.camera_prim and camera_prim.IsValid():
            pan_attr_name, tilt_attr_name = self._auto_select_shallow_axes(stage, camera_prim)
            self._pan_attr_name_override = pan_attr_name
            self._tilt_attr_name_override = tilt_attr_name

        # External scenes may only expose a bare camera prim. In that case, try
        # to attach rotation ops directly on the resolved control prim(s) so Pan/Tilt
        # writes are effective instead of becoming no-op state updates.
        self._ensure_rotate_attr(pan_prim, pan_attr_name)
        self._ensure_rotate_attr(tilt_prim, tilt_attr_name)

        pan_zero = self._read_attr_scalar(pan_prim, pan_attr_name, 0.0)
        tilt_zero = self._read_attr_scalar(tilt_prim, tilt_attr_name, 0.0)
        pan_sign = self._detect_pan_sign(stage, pan_prim, pan_attr_name, pan_zero, default_sign=float(self._ptz_ref["pan_sign"]))
        tilt_sign = self._detect_tilt_sign(stage, tilt_prim, tilt_attr_name, tilt_zero, default_sign=float(self._ptz_ref["tilt_sign"]))

        zoom_x = 1.0
        if camera_prim.IsValid():
            focal_length = self._read_attr_scalar(camera_prim, "focalLength", self.focal_length_1x)
            if self.focal_length_1x > 1e-6:
                zoom_x = max(1.0, min(32.0, focal_length / self.focal_length_1x))

        with self._ptz_lock:
            self._ptz_ref["pan_zero_op"] = pan_zero
            self._ptz_ref["tilt_zero_op"] = tilt_zero
            self._ptz_ref["pan_sign"] = pan_sign
            self._ptz_ref["tilt_sign"] = tilt_sign
            self._ptz_state["pan"] = 0.0
            self._ptz_state["tilt"] = 0.0
            self._ptz_state["zoom"] = zoom_x
        self._ptz_dirty.clear()

    def _apply_ptz_state(self, stage) -> None:
        with self._ptz_lock:
            pan_cmd = self._ptz_state["pan"]
            tilt_cmd = self._ptz_state["tilt"]
            zoom = self._ptz_state["zoom"]
            pan_zero = self._ptz_ref["pan_zero_op"]
            tilt_zero = self._ptz_ref["tilt_zero_op"]
            pan_sign = self._ptz_ref["pan_sign"]
            tilt_sign = self._ptz_ref["tilt_sign"]

        pan_prim_path, tilt_prim_path = self._ptz_paths()
        pan_prim = stage.GetPrimAtPath(pan_prim_path)
        tilt_prim = stage.GetPrimAtPath(tilt_prim_path)
        camera_prim = stage.GetPrimAtPath(self.camera_prim)
        pan_attr_name, tilt_attr_name = self._ptz_op_names()

        pan_op = pan_zero + pan_sign * pan_cmd
        tilt_op = tilt_zero + tilt_sign * tilt_cmd

        if pan_prim.IsValid():
            attr = pan_prim.GetAttribute(pan_attr_name)
            if attr and attr.IsValid():
                attr.Set(float(pan_op))
        if tilt_prim.IsValid():
            attr = tilt_prim.GetAttribute(tilt_attr_name)
            if attr and attr.IsValid():
                attr.Set(float(tilt_op))
        if camera_prim.IsValid():
            attr = camera_prim.GetAttribute("focalLength")
            if attr and attr.IsValid():
                attr.Set(float(self.focal_length_1x * zoom))

    def _collect_ptz_selfcheck(self, stage) -> dict[str, Any]:
        pan_attr_name, tilt_attr_name = self._ptz_op_names()
        pan_path, tilt_path = self._ptz_paths()
        pan_prim = stage.GetPrimAtPath(pan_path)
        tilt_prim = stage.GetPrimAtPath(tilt_path)
        camera_prim = stage.GetPrimAtPath(self.camera_prim)

        with self._ptz_lock:
            state = dict(self._ptz_state)
            pan_zero = float(self._ptz_ref["pan_zero_op"])
            tilt_zero = float(self._ptz_ref["tilt_zero_op"])
            pan_sign = float(self._ptz_ref["pan_sign"])
            tilt_sign = float(self._ptz_ref["tilt_sign"])

        def _read_actual(prim, attr_name: str):
            if not prim.IsValid():
                return None
            attr = prim.GetAttribute(attr_name)
            if not (attr and attr.IsValid()):
                return None
            try:
                return float(attr.Get())
            except Exception:
                return None

        return {
            "ok": True,
            "scene_up_axis": self._scene_up_axis,
            "ptz_dirty_pending": self._ptz_dirty.is_set(),
            "camera_prim": self.camera_prim,
            "pan": {
                "prim_path": pan_path,
                "attr_name": pan_attr_name,
                "cmd_deg": float(state["pan"]),
                "zero_op_deg": pan_zero,
                "sign": pan_sign,
                "expected_op_deg": pan_zero + pan_sign * float(state["pan"]),
                "actual_op_deg": _read_actual(pan_prim, pan_attr_name),
            },
            "tilt": {
                "prim_path": tilt_path,
                "attr_name": tilt_attr_name,
                "cmd_deg": float(state["tilt"]),
                "zero_op_deg": tilt_zero,
                "sign": tilt_sign,
                "expected_op_deg": tilt_zero + tilt_sign * float(state["tilt"]),
                "actual_op_deg": _read_actual(tilt_prim, tilt_attr_name),
            },
            "zoom": {
                "cmd_x": float(state["zoom"]),
                "focal_length_1x_mm": float(self.focal_length_1x),
                "expected_focal_length_mm": float(self.focal_length_1x * float(state["zoom"])),
                "actual_focal_length_mm": _read_actual(camera_prim, "focalLength"),
            },
            "rule": "Pan+ means rotate right, Tilt+ means pitch up",
            "timestamp_s": time.time(),
        }

    @staticmethod
    def _set_prim_y(stage, prim_path: str, y_value: float) -> None:
        from pxr import Gf

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        attr = prim.GetAttribute("xformOp:translate")
        if not (attr and attr.IsValid()):
            return
        current = attr.Get()
        if current is None:
            attr.Set(Gf.Vec3d(0.0, float(y_value), 0.0))
            return
        attr.Set(Gf.Vec3d(current[0], float(y_value), current[2]))

    @staticmethod
    def _set_prim_visibility(stage, prim_path: str, visible: bool) -> None:
        from pxr import UsdGeom

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        attr = prim.GetAttribute("visibility")
        if attr and attr.IsValid():
            attr.Set(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)

    def _apply_scene_state(self, stage) -> None:
        with self._scene_lock:
            gondola_y = self._scene_state["gondola_y"]
            workers = self._scene_state["workers"]
            chosen_worker = self._chosen_worker

        self._set_prim_y(stage, self._gondola_prim, gondola_y)

        if workers == 0:
            self._set_prim_visibility(stage, self._worker1_prim, False)
            self._set_prim_visibility(stage, self._worker2_prim, False)
            return

        if workers == 1:
            if chosen_worker == 1:
                self._set_prim_visibility(stage, self._worker1_prim, True)
                self._set_prim_visibility(stage, self._worker2_prim, False)
                self._set_prim_y(stage, self._worker1_prim, gondola_y)
            else:
                self._set_prim_visibility(stage, self._worker1_prim, False)
                self._set_prim_visibility(stage, self._worker2_prim, True)
                self._set_prim_y(stage, self._worker2_prim, gondola_y)
            return

        self._set_prim_visibility(stage, self._worker1_prim, True)
        self._set_prim_visibility(stage, self._worker2_prim, True)
        self._set_prim_y(stage, self._worker1_prim, gondola_y)
        self._set_prim_y(stage, self._worker2_prim, gondola_y)

    def _choose_single_worker_if_needed(self, previous_count: int, new_count: int) -> None:
        if new_count == 1 and previous_count != 1:
            self._chosen_worker = random.choice([1, 2])

    def _wait_for_ptz_apply(self, timeout_s: float = 1.0) -> None:
        deadline = time.time() + timeout_s
        while self._ptz_dirty.is_set() and time.time() < deadline:
            time.sleep(0.01)

    def _wait_for_scene_apply(self, timeout_s: float = 1.0) -> None:
        deadline = time.time() + timeout_s
        while self._scene_dirty.is_set() and time.time() < deadline:
            time.sleep(0.01)

    def _make_handler(self):
        runtime = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args) -> None:
                pass

            def _cors(self) -> None:
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def _json(self, data: dict[str, Any], code: int = 200) -> None:
                body = json.dumps(data, ensure_ascii=False).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self._cors()
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:
                self.send_response(204)
                self._cors()
                self.end_headers()

            def do_GET(self) -> None:
                path = self.path.split("?")[0]

                if path in ("/", "/index.html"):
                    html_path = os.path.join(runtime.script_dir, "ptz_web_control.html")
                    if not os.path.isfile(html_path):
                        self.send_response(404)
                        self.end_headers()
                        self.wfile.write(b"ptz_web_control.html not found")
                        return
                    with open(html_path, "rb") as handle:
                        data = handle.read()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(data)))
                    self._cors()
                    self.end_headers()
                    self.wfile.write(data)
                    return

                if path == "/status":
                    with runtime._ptz_lock:
                        state = dict(runtime._ptz_state)
                    state["ready"] = bool(runtime._ready)
                    self._json(state)
                    return

                if path == "/scene/state":
                    with runtime._scene_lock:
                        state = dict(runtime._scene_state)
                    self._json(state)
                    return

                if path == "/ptz/selfcheck":
                    try:
                        stage = omni.usd.get_context().get_stage()
                        self._json(runtime._collect_ptz_selfcheck(stage))
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 500)
                    return

                if path.startswith("/stream.mjpeg"):
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=ptzframe")
                    self.send_header("Cache-Control", "no-cache, no-store")
                    self.send_header("Connection", "close")
                    self._cors()
                    self.end_headers()
                    last_frame_id = -1
                    try:
                        while runtime._running:
                            with runtime._mjpeg_lock:
                                frame_id = runtime._mjpeg["frame_id"]
                                jpg = runtime._mjpeg["jpeg"]
                            if frame_id == last_frame_id or jpg is None:
                                time.sleep(0.015)
                                continue
                            last_frame_id = frame_id
                            self.wfile.write(
                                b"--ptzframe\r\n"
                                b"Content-Type: image/jpeg\r\n"
                                b"Content-Length: "
                                + str(len(jpg)).encode("ascii")
                                + b"\r\n\r\n"
                                + jpg
                                + b"\r\n"
                            )
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        pass
                    return

                if path.startswith("/snapshot.jpg"):
                    with runtime._mjpeg_lock:
                        jpg = runtime._mjpeg["jpeg"]
                    if jpg is None:
                        self.send_response(503)
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.send_header("Cache-Control", "no-cache, no-store")
                    self._cors()
                    self.end_headers()
                    self.wfile.write(jpg)
                    return

                self.send_response(404)
                self.end_headers()

            def do_POST(self) -> None:
                if self.path == "/scene/gondola":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        with runtime._scene_lock:
                            if "y" in request:
                                runtime._scene_state["gondola_y"] = max(0.0, min(3300.0, float(request["y"])))
                            state = dict(runtime._scene_state)
                        runtime._scene_dirty.set()
                        runtime._wait_for_scene_apply()
                        self._json({"ok": True, "state": state})
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                if self.path == "/scene/workers":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        with runtime._scene_lock:
                            previous = int(runtime._scene_state["workers"])
                            if "count" in request:
                                new_count = max(0, min(2, int(request["count"])))
                                runtime._choose_single_worker_if_needed(previous, new_count)
                                runtime._scene_state["workers"] = new_count
                            state = dict(runtime._scene_state)
                        runtime._scene_dirty.set()
                        runtime._wait_for_scene_apply()
                        self._json({"ok": True, "state": state})
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                if self.path == "/control":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        with runtime._ptz_lock:
                            if "pan" in request:
                                runtime._ptz_state["pan"] = max(-90.0, min(90.0, float(request["pan"])))
                            if "tilt" in request:
                                runtime._ptz_state["tilt"] = max(-90.0, min(90.0, float(request["tilt"])))
                            if "zoom" in request:
                                runtime._ptz_state["zoom"] = max(1.0, min(32.0, float(request["zoom"])))
                            state = dict(runtime._ptz_state)
                        runtime._ptz_dirty.set()
                        runtime._wait_for_ptz_apply()
                        self._json({"ok": True, "state": state})
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                self.send_response(404)
                self.end_headers()

        return Handler

    def _start_control_server(self) -> None:
        server = ThreadingHTTPServer(("0.0.0.0", self.ctrl_port), self._make_handler())
        threading.Thread(target=server.serve_forever, daemon=True, name="ptz-ctrl").start()
        print(f"[PTZ-RTSP] control server on http://localhost:{self.ctrl_port}/")

    def _bind_camera(self):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.camera_prim)
        if not prim.IsValid():
            raise ValueError(f"Camera prim not found: {self.camera_prim}")

        render_product = rep.create.render_product(self.camera_prim, (self.width, self.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach(render_product)
        return render_product, annotator

    def _shutdown(self, _signum=None, _frame=None) -> None:
        self._running = False

    def _cleanup(self) -> None:
        self._ready = False
        if self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
            try:
                self._ffmpeg_proc.stdin.close()
            except Exception:
                pass
            try:
                self._ffmpeg_proc.wait(timeout=5)
            except Exception:
                self._ffmpeg_proc.kill()

        if self._mediamtx_proc is not None and self._mediamtx_proc.poll() is None:
            self._mediamtx_proc.terminate()
            try:
                self._mediamtx_proc.wait(timeout=5)
            except Exception:
                self._mediamtx_proc.kill()

    def run(self) -> None:
        signal.signal(signal.SIGINT, self._shutdown)
        signal.signal(signal.SIGTERM, self._shutdown)

        self._configure_runtime_cache()
        self._apply_render_throttle()
        self._jpeg_encode_fn = self._init_jpeg_encoder()
        self._start_control_server()

        if self.rtsp_enabled:
            try:
                mediamtx_bin = ensure_mediamtx(self.script_dir, self.mediamtx_cfg)
                self._mediamtx_proc = start_mediamtx(self.script_dir, self.mediamtx_cfg, mediamtx_bin)
                self._ffmpeg_proc = start_ffmpeg(self.rtsp_url, self.width, self.height, self.fps, self.bitrate)
            except FileNotFoundError as exc:
                self._cleanup()
                self.rtsp_enabled = False
                self._mediamtx_proc = None
                self._ffmpeg_proc = None
                print(f"[PTZ-RTSP] RTSP disabled automatically: {exc}")
                print(f"[PTZ-RTSP] MJPEG available on http://localhost:{self.ctrl_port}/stream.mjpeg")
            except Exception:
                self._cleanup()
                raise
        else:
            print(f"[PTZ-RTSP] RTSP disabled, MJPEG available on http://localhost:{self.ctrl_port}/stream.mjpeg")

        usd_context = omni.usd.get_context()
        result = usd_context.open_stage(self.scene_path)
        if isinstance(result, tuple):
            ok, error = result
        else:
            ok, error = result, ""
        if not ok:
            raise RuntimeError(f"Failed to open stage: {error}")

        self.sim_app.update()
        self.sim_app.update()

        try:
            from pxr import UsdGeom

            self._scene_up_axis = UsdGeom.GetStageUpAxis(usd_context.get_stage())
        except Exception:
            self._scene_up_axis = "Y"

        stage = usd_context.get_stage()
        self._init_ptz_reference(stage)
        self._apply_scene_state(stage)

        stage_units_in_meters = 0.01 if self._scene_up_axis == "Y" else 1.0
        world = World(stage_units_in_meters=stage_units_in_meters)
        world.reset()

        for _ in range(10):
            world.step(render=True)

        _render_product, annotator = self._bind_camera()
        for _ in range(4):
            world.step(render=True)
            self.sim_app.update()
        self._ready = True

        frame_idx = 0
        pushed_frames = 0
        last_stat_time = time.time()
        last_mjpeg_frame_id = 0
        capture_dt = 1.0 / max(1, self.fps)
        loop_t0 = time.perf_counter()

        while self._running and self.sim_app.is_running():
            world.step(render=True)

            if self._ptz_dirty.is_set():
                self._ptz_dirty.clear()
                self._apply_ptz_state(stage)

            if self._scene_dirty.is_set():
                self._scene_dirty.clear()
                self._apply_scene_state(stage)

            rgba = annotator.get_data()

            if rgba is not None and getattr(rgba, "size", 0) > 0:
                rgba = resize_rgba_frame(np.asarray(rgba), self.width, self.height)
                if rgba.dtype != np.uint8:
                    rgba = np.clip(rgba, 0, 255).astype(np.uint8, copy=False)

                # Guard against intermittent all-black frames from the renderer path.
                # Reuse the last valid frame to prevent visible flash-to-black loops.
                if np.max(rgba[:, :, :3]) == 0 and self._last_good_rgba is not None:
                    rgba = self._last_good_rgba
                else:
                    self._last_good_rgba = np.ascontiguousarray(rgba)

                if self._jpeg_encode_fn is not None:
                    jpg = self._jpeg_encode_fn(rgba)
                    if jpg:
                        with self._mjpeg_lock:
                            self._mjpeg["jpeg"] = jpg
                            self._mjpeg["frame_id"] += 1

                if self.rtsp_enabled and self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
                    raw = np.ascontiguousarray(rgba, dtype=np.uint8).tobytes()
                    try:
                        self._ffmpeg_proc.stdin.write(raw)
                        self._ffmpeg_proc.stdin.flush()
                        pushed_frames += 1
                    except BrokenPipeError:
                        print("[PTZ-RTSP] ffmpeg pipe broke; RTSP will stop while MJPEG continues")
                        self._ffmpeg_proc = None

                now = time.time()
                if now - last_stat_time >= 5.0:
                    elapsed = now - last_stat_time
                    with self._mjpeg_lock:
                        current_mjpeg_id = self._mjpeg["frame_id"]
                    if self.rtsp_enabled:
                        actual_fps = pushed_frames / elapsed if elapsed > 0 else 0.0
                    else:
                        actual_fps = (current_mjpeg_id - last_mjpeg_frame_id) / elapsed if elapsed > 0 else 0.0
                    print(f"[PTZ-RTSP] running... fps={actual_fps:.1f} frame={frame_idx}")
                    pushed_frames = 0
                    last_stat_time = now
                    last_mjpeg_frame_id = current_mjpeg_id

            frame_idx += 1

            now_perf = time.perf_counter()
            sleep_for = capture_dt - (now_perf - loop_t0)
            if sleep_for > 0.001:
                time.sleep(sleep_for)
            loop_t0 = time.perf_counter()

        self._cleanup()
        self.sim_app.close()


def run_stream_runtime(sim_app: Any, settings: dict[str, Any]) -> None:
    runtime = StreamRuntime(sim_app, settings)
    runtime.run()
