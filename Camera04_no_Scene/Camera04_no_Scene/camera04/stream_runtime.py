from __future__ import annotations

import io as _io
import json
import math
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

from .common import simulation_renderer_name
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
        self.ffmpeg_path = settings.get("ffmpeg_path")
        self.rtsp_enabled = bool(settings["rtsp_enabled"])
        self.mjpeg_quality = int(settings["mjpeg_quality"])
        self.focal_length_1x = float(settings["focal_length_1x"])
        self.ctrl_port = int(settings["ctrl_port"])
        self.renderer_mode = str(settings.get("renderer_mode", "PathTracing"))
        self.renderer_backend_mode = simulation_renderer_name(self.renderer_mode)
        self.rt_subframes = max(0, min(16, int(settings.get("rt_subframes", 2))))
        self.path_tracing_spp = self._clamp_path_tracing_spp(settings.get("path_tracing_spp", 1), default=1)
        self._path_tracing_lock = threading.Lock()
        self._requested_camera_prim = str(self.camera_prim)
        self._resolved_camera_reason = "configured"
        self._resolved_camera_prim = str(self.camera_prim)

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
        self._instance_id = f"{os.getpid()}-{int(time.time() * 1000)}"
        self._ffmpeg_last_retry_ts = 0.0
        self._ffmpeg_retry_interval_s = 2.0
        self._camera_tuning = {"ae_enabled": True, "ae_value": 0.0, "iso": 100.0}
        self._camera_tuning_lock = threading.Lock()
        self._camera_tuning_dirty = threading.Event()
        self._camera_tuning_diag: dict[str, Any] = {
            "target_camera_prim": self.camera_prim,
            "target_camera_type": "",
            "available_auto_attrs": [],
            "available_exposure_attrs": [],
            "available_iso_attrs": [],
            "applied_auto_attrs": [],
            "applied_exposure_attrs": [],
            "applied_iso_attrs": [],
            "applied_setting_keys": [],
            "last_error": "",
        }
        self._volumetric = {
            "enabled": False,
            "fog_height": 1000.0,
            "fog_height_falloff": 10.0,
            "max_distance": 50000.0,
            "density_mult": 1.0,
            "transmittance_measurement_distance": 10000.0,
            "transmittance_color": [0.5, 0.5, 0.5],
            "single_scattering_albedo": [0.9, 0.9, 0.9],
            "anisotropy_factor": 0.0,
            "apply_density_noise": False,
            "detail_noise_scale": 0.2,
            "noise_animation_speed_x": 0.0,
            "noise_animation_speed_y": 0.0,
            "noise_animation_speed_z": 0.0,
            "noise_scale_min": 0.0,
            "noise_scale_max": 1.0,
            "noise_octave_count": 3,
        }
        self._volumetric_lock = threading.Lock()
        self._volumetric_dirty = threading.Event()
        self._volumetric_diag: dict[str, Any] = {
            "applied_setting_keys": [],
            "last_error": "",
        }
        self._frame_format_logged = False

    @staticmethod
    def _clamp_path_tracing_spp(value: object, default: int = 1) -> int:
        try:
            n = int(value)  # type: ignore[arg-type]
        except Exception:
            n = int(default)
        return max(1, min(512, n))

    def _apply_path_tracing_spp(self, spp: int | None = None) -> int:
        target = self._clamp_path_tracing_spp(self.path_tracing_spp if spp is None else spp, default=1)
        settings = carb.settings.get_settings()
        settings.set("/rtx/pathtracing/spp", int(target))
        settings.set("/rtx/pathtracing/totalSpp", int(target))
        settings.set("/rtx/pathtracing/clampSpp", int(target))
        return int(target)

    def _path_tracing_snapshot(self) -> dict[str, Any]:
        with self._path_tracing_lock:
            configured = int(self.path_tracing_spp)
        live_spp = self._safe_read_setting("/rtx/pathtracing/spp")
        live_total = self._safe_read_setting("/rtx/pathtracing/totalSpp")
        live_clamp = self._safe_read_setting("/rtx/pathtracing/clampSpp")
        return {
            "ok": True,
            "renderer_mode": self.renderer_mode,
            "active": self.renderer_mode == "PathTracing",
            "configured_spp": configured,
            "settings": {
                "/rtx/pathtracing/spp": live_spp,
                "/rtx/pathtracing/totalSpp": live_total,
                "/rtx/pathtracing/clampSpp": live_clamp,
            },
        }

    @staticmethod
    def _to_uint8_rgba(frame: np.ndarray) -> np.ndarray:
        arr = np.asarray(frame)
        if arr.ndim != 3:
            raise ValueError(f"unexpected frame shape: {arr.shape}")

        # Normalize channel count to RGBA.
        if arr.shape[2] == 3:
            if arr.dtype == np.uint8:
                alpha = np.full((arr.shape[0], arr.shape[1], 1), 255, dtype=np.uint8)
            else:
                alpha = np.ones((arr.shape[0], arr.shape[1], 1), dtype=arr.dtype)
            arr = np.concatenate([arr, alpha], axis=2)
        elif arr.shape[2] > 4:
            arr = arr[:, :, :4]
        elif arr.shape[2] < 3:
            raise ValueError(f"unexpected channel count: {arr.shape[2]}")

        if arr.dtype == np.uint8:
            return np.ascontiguousarray(arr)

        if np.issubdtype(arr.dtype, np.floating):
            arr = np.nan_to_num(arr, nan=0.0, posinf=255.0, neginf=0.0)
            max_val = float(np.max(arr)) if arr.size else 0.0
            # RTX real-time pipelines may output normalized floats in [0, 1].
            if max_val <= 1.5:
                arr = arr * 255.0
            arr = np.clip(arr, 0.0, 255.0)
            return np.ascontiguousarray(arr.astype(np.uint8))

        arr = np.clip(arr, 0, 255)
        return np.ascontiguousarray(arr.astype(np.uint8))

    def _apply_render_throttle(self) -> None:
        settings = carb.settings.get_settings()
        target_fps = max(1, self.fps)
        settings.set("/rtx/ecoMode/enabled", False)
        settings.set("/rtx/ecoMode/maxFramesWithoutChange", 1)
        settings.set("/app/asyncRendering", False)
        settings.set("/app/asyncRenderingLowLatency", False)
        render_mode_token = "PathTracing" if self.renderer_mode == "PathTracing" else "RaytracedLighting"
        settings.set("/rtx/rendermode", render_mode_token)
        settings.set(
            "/rtx-transient/resourcemanager/localTextureCachePath",
            os.path.join(self._runtime_ov_cache, "texturecache").replace("\\", "/"),
        )
        for loop in ("main", "rendering_0", "rendering_1", "present"):
            settings.set(f"/app/runLoops/{loop}/rateLimitEnabled", True)
            settings.set(f"/app/runLoops/{loop}/rateLimitFrequency", target_fps)
        if self.renderer_mode == "PathTracing":
            settings.set("/rtx/ambientOcclusion/enabled", False)
            settings.set("/rtx/reflections/enabled", False)
            settings.set("/rtx/translucency/enabled", False)
            settings.set("/rtx/post/aa/op", 0)
            settings.set("/rtx/directLighting/sampledLighting/enabled", False)
            self._apply_path_tracing_spp()
        else:
            # Ensure real-time chain uses the expected lighting path and allow a few
            # subframes for heavy scenes/materials to settle after camera or mode changes.
            settings.set("/rtx/directLighting/enabled", True)
            settings.set("/rtx/directLighting/domeLight/enabled", True)
            settings.set("/rtx/directLighting/sampledLighting/enabled", True)
            settings.set("/rtx/indirectDiffuse/enabled", True)
            settings.set("/rtx/shadows/enabled", True)
            settings.set("/omni/replicator/RTSubframes", int(self.rt_subframes))
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
    def _read_attr_bool(prim, attr_name: str) -> bool | None:
        if not prim.IsValid():
            return None
        attr = prim.GetAttribute(attr_name)
        if not (attr and attr.IsValid()):
            return None
        try:
            value = attr.Get()
        except Exception:
            return None
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("1", "true", "yes", "on", "enabled"):
                return True
            if lowered in ("0", "false", "no", "off", "disabled"):
                return False
        return None

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
    def _safe_read_setting(key: str) -> Any:
        try:
            settings = carb.settings.get_settings()
            return settings.get(key)
        except Exception:
            return None

    def _collect_render_diag(self, stage) -> dict[str, Any]:
        setting_keys = (
            "/rtx/rendermode",
            "/rtx/directLighting/enabled",
            "/rtx/directLighting/domeLight/enabled",
            "/rtx/directLighting/sampledLighting/enabled",
            "/rtx/indirectDiffuse/enabled",
            "/rtx/shadows/enabled",
            "/omni/replicator/RTSubframes",
            "/rtx/pathtracing/spp",
            "/rtx/pathtracing/totalSpp",
            "/rtx/pathtracing/clampSpp",
            "/rtx/post/histogram/enabled",
            "/rtx/post/histogram/whiteScale",
            "/rtx/post/tonemap/filmIso",
            "/rtx/raytracing/globalVolumetricEffects/enabled",
            "/rtx/raytracing/inscattering/densityMult",
            "/rtx/raytracing/inscattering/maxDistance",
            "/rtx/raytracing/inscattering/anisotropyFactor",
            "/rtx/raytracing/inscattering/useDetailNoise",
        )
        settings_snapshot: dict[str, Any] = {key: self._safe_read_setting(key) for key in setting_keys}

        light_types = {
            "DomeLight",
            "DistantLight",
            "RectLight",
            "DiskLight",
            "SphereLight",
            "CylinderLight",
        }
        light_counts: dict[str, int] = {k: 0 for k in sorted(light_types)}
        light_samples: list[dict[str, Any]] = []
        try:
            iterator = stage.Traverse()
        except Exception:
            iterator = []
        for prim in iterator:
            type_name = str(prim.GetTypeName() or "")
            if type_name not in light_types:
                continue
            light_counts[type_name] = light_counts.get(type_name, 0) + 1
            if len(light_samples) >= 12:
                continue
            intensity = None
            exposure = None
            try:
                i_attr = prim.GetAttribute("intensity")
                if i_attr and i_attr.IsValid():
                    intensity = float(i_attr.Get())
            except Exception:
                intensity = None
            try:
                e_attr = prim.GetAttribute("exposure")
                if e_attr and e_attr.IsValid():
                    exposure = float(e_attr.Get())
            except Exception:
                exposure = None
            light_samples.append(
                {
                    "path": str(prim.GetPath()),
                    "type": type_name,
                    "intensity": intensity,
                    "exposure": exposure,
                }
            )

        with self._mjpeg_lock:
            frame_id = int(self._mjpeg.get("frame_id", 0))
            has_jpeg = self._mjpeg.get("jpeg") is not None

        warnings: list[str] = []
        live_mode = str(settings_snapshot.get("/rtx/rendermode") or "").strip()
        if self.renderer_mode == "RTXRealTime" and live_mode.lower() not in ("raytracedlighting", "rt"):
            warnings.append(
                f"renderer mismatch: requested RTXRealTime but /rtx/rendermode is {live_mode or '(empty)'}"
            )
        if light_counts.get("DistantLight", 0) == 0 and light_counts.get("DomeLight", 0) > 0:
            warnings.append(
                "lighting risk: no DistantLight found; DomeLight-only setups can render geometry near-black in real-time"
            )

        return {
            "ok": True,
            "renderer_mode": self.renderer_mode,
            "renderer_backend_mode": self.renderer_backend_mode,
            "camera_requested": self._requested_camera_prim,
            "camera_resolved": self._resolved_camera_prim,
            "camera_resolve_reason": self._resolved_camera_reason,
            "scene_path": self.scene_path,
            "rt_subframes": int(self.rt_subframes),
            "path_tracing_spp": int(self.path_tracing_spp),
            "settings": settings_snapshot,
            "lights": {
                "counts": light_counts,
                "samples": light_samples,
            },
            "warnings": warnings,
            "stream_state": {
                "ready": bool(self._ready),
                "mjpeg_frame_id": frame_id,
                "has_jpeg": bool(has_jpeg),
            },
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

    def _wait_for_camera_tuning_apply(self, timeout_s: float = 1.0) -> None:
        deadline = time.time() + timeout_s
        while self._camera_tuning_dirty.is_set() and time.time() < deadline:
            time.sleep(0.01)

    def _wait_for_volumetric_apply(self, timeout_s: float = 1.0) -> None:
        deadline = time.time() + timeout_s
        while self._volumetric_dirty.is_set() and time.time() < deadline:
            time.sleep(0.01)

    _VOLUMETRIC_SETTINGS = {
        "enabled": "/rtx/raytracing/globalVolumetricEffects/enabled",
        "fog_height": "/rtx/raytracing/inscattering/atmosphereHeight",
        "fog_height_falloff": "/rtx/pathtracing/ptvol/fogHeightFallOff",
        "max_distance": "/rtx/raytracing/inscattering/maxDistance",
        "density_mult": "/rtx/raytracing/inscattering/densityMult",
        "transmittance_measurement_distance": "/rtx/raytracing/inscattering/transmittanceMeasurementDistance",
        "transmittance_color": "/rtx/raytracing/inscattering/transmittanceColor",
        "single_scattering_albedo": "/rtx/raytracing/inscattering/singleScatteringAlbedo",
        "anisotropy_factor": "/rtx/raytracing/inscattering/anisotropyFactor",
        "apply_density_noise": "/rtx/raytracing/inscattering/useDetailNoise",
        "detail_noise_scale": "/rtx/raytracing/inscattering/detailNoiseScale",
        "noise_animation_speed_x": "/rtx/raytracing/inscattering/noiseAnimationSpeedX",
        "noise_animation_speed_y": "/rtx/raytracing/inscattering/noiseAnimationSpeedY",
        "noise_animation_speed_z": "/rtx/raytracing/inscattering/noiseAnimationSpeedZ",
        "noise_scale_min": "/rtx/raytracing/inscattering/noiseScaleRangeMin",
        "noise_scale_max": "/rtx/raytracing/inscattering/noiseScaleRangeMax",
        "noise_octave_count": "/rtx/raytracing/inscattering/noiseNumOctaves",
    }
    _VOLUMETRIC_FLOAT_RANGES = {
        "fog_height": (-2000.0, 100000.0),
        "fog_height_falloff": (10.0, 2000.0),
        "max_distance": (10.0, 1000000.0),
        "density_mult": (0.0, 2.0),
        "transmittance_measurement_distance": (0.0001, 1000000.0),
        "anisotropy_factor": (-0.999, 0.999),
        "detail_noise_scale": (0.0, 1.0),
        "noise_animation_speed_x": (-1.0, 1.0),
        "noise_animation_speed_y": (-1.0, 1.0),
        "noise_animation_speed_z": (-1.0, 1.0),
        "noise_scale_min": (-1.0, 5.0),
        "noise_scale_max": (-1.0, 5.0),
    }
    _VOLUMETRIC_INT_RANGES = {
        "noise_octave_count": (1, 8),
    }

    @staticmethod
    def _clamp_float(value: object, lo: float, hi: float, default: float) -> float:
        try:
            n = float(value)  # type: ignore[arg-type]
        except Exception:
            n = float(default)
        return max(float(lo), min(float(hi), float(n)))

    @staticmethod
    def _clamp_int(value: object, lo: int, hi: int, default: int) -> int:
        try:
            n = int(value)  # type: ignore[arg-type]
        except Exception:
            n = int(default)
        return max(int(lo), min(int(hi), int(n)))

    @staticmethod
    def _normalize_color3(value: object, default: list[float] | tuple[float, float, float]) -> list[float]:
        base = [float(default[0]), float(default[1]), float(default[2])]
        if isinstance(value, (list, tuple)) and len(value) >= 3:
            out: list[float] = []
            for i in range(3):
                try:
                    c = float(value[i])  # type: ignore[index]
                except Exception:
                    c = base[i]
                out.append(max(0.0, min(1.0, c)))
            return out
        return base

    @staticmethod
    def _read_carb_setting_color3(setting_key: str, default: list[float] | tuple[float, float, float]) -> list[float]:
        try:
            settings = carb.settings.get_settings()
            value = settings.get(setting_key)
        except Exception:
            value = None
        if value is None:
            return [float(default[0]), float(default[1]), float(default[2])]
        if isinstance(value, (list, tuple)):
            return StreamRuntime._normalize_color3(value, default)
        xyz = []
        for name in ("x", "y", "z"):
            if not hasattr(value, name):
                xyz = []
                break
            try:
                xyz.append(float(getattr(value, name)))
            except Exception:
                xyz = []
                break
        if len(xyz) == 3:
            return StreamRuntime._normalize_color3(xyz, default)
        return [float(default[0]), float(default[1]), float(default[2])]

    @staticmethod
    def _set_carb_setting_color3(setting_key: str, value: list[float] | tuple[float, float, float]) -> bool:
        color = StreamRuntime._normalize_color3(value, [0.5, 0.5, 0.5])
        try:
            settings = carb.settings.get_settings()
            if hasattr(settings, "set_float_array"):
                settings.set_float_array(setting_key, color)
            else:
                settings.set(setting_key, color)
            return True
        except Exception:
            return False

    def _volumetric_snapshot(self) -> dict[str, Any]:
        with self._volumetric_lock:
            state = dict(self._volumetric)
            diag = dict(self._volumetric_diag)
        live_settings = {
            key: self._safe_read_setting(path)
            for key, path in self._VOLUMETRIC_SETTINGS.items()
        }
        live_settings["transmittance_color"] = self._read_carb_setting_color3(
            self._VOLUMETRIC_SETTINGS["transmittance_color"],
            state["transmittance_color"],
        )
        live_settings["single_scattering_albedo"] = self._read_carb_setting_color3(
            self._VOLUMETRIC_SETTINGS["single_scattering_albedo"],
            state["single_scattering_albedo"],
        )
        return {
            "ok": True,
            "renderer_mode": self.renderer_mode,
            "state": state,
            "ranges": {
                "float": dict(self._VOLUMETRIC_FLOAT_RANGES),
                "int": dict(self._VOLUMETRIC_INT_RANGES),
                "color": [0.0, 1.0],
            },
            "settings": live_settings,
            "diag": diag,
        }

    def _init_volumetric_state(self) -> None:
        with self._volumetric_lock:
            defaults = dict(self._volumetric)
            for key, (lo, hi) in self._VOLUMETRIC_FLOAT_RANGES.items():
                setting_key = self._VOLUMETRIC_SETTINGS[key]
                raw = self._safe_read_setting(setting_key)
                self._volumetric[key] = self._clamp_float(raw, lo, hi, defaults[key])
            for key, (lo, hi) in self._VOLUMETRIC_INT_RANGES.items():
                setting_key = self._VOLUMETRIC_SETTINGS[key]
                raw = self._safe_read_setting(setting_key)
                self._volumetric[key] = self._clamp_int(raw, lo, hi, defaults[key])
            self._volumetric["enabled"] = self._read_carb_setting_bool(
                self._VOLUMETRIC_SETTINGS["enabled"],
                bool(defaults["enabled"]),
            )
            self._volumetric["apply_density_noise"] = self._read_carb_setting_bool(
                self._VOLUMETRIC_SETTINGS["apply_density_noise"],
                bool(defaults["apply_density_noise"]),
            )
            self._volumetric["transmittance_color"] = self._read_carb_setting_color3(
                self._VOLUMETRIC_SETTINGS["transmittance_color"],
                defaults["transmittance_color"],
            )
            self._volumetric["single_scattering_albedo"] = self._read_carb_setting_color3(
                self._VOLUMETRIC_SETTINGS["single_scattering_albedo"],
                defaults["single_scattering_albedo"],
            )
            self._volumetric_diag.update(
                {
                    "applied_setting_keys": [],
                    "last_error": "",
                }
            )
        self._volumetric_dirty.set()

    def _apply_volumetric_state(self) -> None:
        with self._volumetric_lock:
            state = dict(self._volumetric)
        applied_settings: list[str] = []

        bool_fields = ("enabled", "apply_density_noise")
        for key in bool_fields:
            setting_key = self._VOLUMETRIC_SETTINGS[key]
            if self._set_carb_setting_bool(setting_key, bool(state[key])):
                applied_settings.append(setting_key)

        for key, _range in self._VOLUMETRIC_FLOAT_RANGES.items():
            setting_key = self._VOLUMETRIC_SETTINGS[key]
            if self._set_carb_setting_scalar(setting_key, float(state[key])):
                applied_settings.append(setting_key)

        for key, _range in self._VOLUMETRIC_INT_RANGES.items():
            setting_key = self._VOLUMETRIC_SETTINGS[key]
            if self._set_carb_setting_int(setting_key, int(state[key])):
                applied_settings.append(setting_key)

        for key in ("transmittance_color", "single_scattering_albedo"):
            setting_key = self._VOLUMETRIC_SETTINGS[key]
            if self._set_carb_setting_color3(setting_key, state[key]):
                applied_settings.append(setting_key)

        last_error = ""
        if not applied_settings:
            last_error = "no volumetric setting key was applied"

        with self._volumetric_lock:
            self._volumetric_diag.update(
                {
                    "applied_setting_keys": applied_settings,
                    "last_error": last_error,
                }
            )

    _AE_BOOL_ATTRS = (
        "enableAutoExposure",
        "autoExposure",
        "cameraAutoExposure",
        "useAutoExposure",
        "exposure:auto",
        "rtx:post:tonemap:enableAutoExposure",
        "inputs:enableAutoExposure",
        "inputs:autoExposure",
        "inputs:cameraAutoExposure",
        "inputs:useAutoExposure",
        "inputs:rtx:post:tonemap:enableAutoExposure",
    )
    _EXPOSURE_ATTRS = (
        "exposure",
        "cameraExposure",
        "exposureCompensation",
        "exposureCompensationEV",
        "exposure:compensation",
        "exposure:ev100",
        "inputs:exposure",
        "inputs:cameraExposure",
        "inputs:exposureCompensation",
        "inputs:exposureCompensationEV",
        "inputs:exposure:compensation",
        "inputs:exposure:ev100",
    )
    _ISO_ATTRS = (
        "iso",
        "cameraIso",
        "sensorSensitivity",
        "exposure:iso",
        "inputs:iso",
        "inputs:cameraIso",
        "inputs:sensorSensitivity",
        "inputs:exposure:iso",
    )
    # Isaac viewport auto exposure controls:
    # AE switch: /rtx/post/histogram/enabled
    # AE value:  /rtx/post/histogram/whiteScale (0..20 in viewport menu)
    # ISO value: /rtx/post/tonemap/filmIso (50..1600 in viewport menu)
    _AE_SETTING_KEYS = (
        "/rtx/post/histogram/whiteScale",
    )
    _AE_ENABLE_SETTING_KEYS = (
        "/rtx/post/histogram/enabled",
    )
    _ISO_SETTING_KEYS = (
        "/rtx/post/tonemap/filmIso",
    )
    _AE_MIN = 0.0
    _AE_MAX = 20.0
    _ISO_MIN = 50.0
    _ISO_MAX = 1600.0

    def _set_auto_exposure_flag(self, camera_prim, attr_names: tuple[str, ...], enabled: bool) -> list[str]:
        applied: list[str] = []
        for attr_name in attr_names:
            attr = camera_prim.GetAttribute(attr_name)
            if not (attr and attr.IsValid()):
                continue
            try:
                attr.Set(bool(enabled))
                applied.append(attr_name)
            except Exception:
                continue
        return applied

    def _set_scalar_attrs(self, camera_prim, attr_names: tuple[str, ...], value: float) -> list[str]:
        applied: list[str] = []
        for attr_name in attr_names:
            attr = camera_prim.GetAttribute(attr_name)
            if not (attr and attr.IsValid()):
                continue
            try:
                attr.Set(float(value))
                applied.append(attr_name)
            except Exception:
                continue
        return applied

    @staticmethod
    def _set_carb_setting_scalar(setting_key: str, value: float) -> bool:
        try:
            settings = carb.settings.get_settings()
            settings.set(setting_key, float(value))
            return True
        except Exception:
            return False

    @staticmethod
    def _set_carb_setting_int(setting_key: str, value: int) -> bool:
        try:
            settings = carb.settings.get_settings()
            settings.set(setting_key, int(value))
            return True
        except Exception:
            return False

    @staticmethod
    def _set_carb_setting_bool(setting_key: str, value: bool) -> bool:
        try:
            settings = carb.settings.get_settings()
            settings.set(setting_key, bool(value))
            return True
        except Exception:
            return False

    @staticmethod
    def _read_carb_setting_scalar(setting_key: str, default: float) -> float:
        try:
            settings = carb.settings.get_settings()
            value = settings.get(setting_key)
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _read_carb_setting_bool(setting_key: str, default: bool) -> bool:
        try:
            settings = carb.settings.get_settings()
            value = settings.get(setting_key)
            if value is None:
                return bool(default)
            return bool(value)
        except Exception:
            return bool(default)

    @staticmethod
    def _dedupe_attr_names(names: list[str] | tuple[str, ...]) -> tuple[str, ...]:
        seen: set[str] = set()
        out: list[str] = []
        for name in names:
            if name in seen:
                continue
            seen.add(name)
            out.append(name)
        return tuple(out)

    @staticmethod
    def _looks_like_camera_prim(prim) -> bool:
        if not prim or not prim.IsValid():
            return False
        type_name = str(prim.GetTypeName() or "")
        if type_name.lower() == "camera":
            return True
        # Some rigs wrap camera schemas but still expose focalLength.
        return StreamRuntime._has_attr(prim, "focalLength")

    def _all_camera_prims(self, stage) -> list[Any]:
        cameras: list[Any] = []
        try:
            iterator = stage.Traverse()
        except Exception:
            return cameras
        for prim in iterator:
            if self._looks_like_camera_prim(prim):
                cameras.append(prim)
        return cameras

    @staticmethod
    def _path_tail(path: str) -> str:
        parts = [p for p in str(path).split("/") if p]
        return parts[-1] if parts else ""

    def _resolve_camera_prim(self, stage):
        requested = str(self.camera_prim or "").strip()
        requested_prim = stage.GetPrimAtPath(requested) if requested else None
        if requested_prim and requested_prim.IsValid() and self._looks_like_camera_prim(requested_prim):
            return requested_prim, "configured"

        if requested_prim and requested_prim.IsValid():
            stack = list(requested_prim.GetChildren())
            while stack:
                cur = stack.pop(0)
                if self._looks_like_camera_prim(cur):
                    return cur, "descendant_of_configured"
                for child in cur.GetChildren():
                    stack.append(child)

        cameras = self._all_camera_prims(stage)
        if not cameras:
            return requested_prim, "not_found"

        if requested:
            req_lower = requested.lower()
            for cam in cameras:
                cam_path = str(cam.GetPath())
                if cam_path.lower() == req_lower:
                    return cam, "case_insensitive_path_match"

            req_tail = self._path_tail(requested).lower()
            if req_tail:
                for cam in cameras:
                    cam_tail = self._path_tail(str(cam.GetPath())).lower()
                    if cam_tail == req_tail:
                        return cam, "basename_match"

        return cameras[0], "first_camera_fallback"

    def _ensure_valid_camera_prim(self, stage) -> None:
        resolved, reason = self._resolve_camera_prim(stage)
        if not resolved or not resolved.IsValid():
            raise ValueError(f"Camera prim not found and no fallback camera available: {self.camera_prim}")
        resolved_path = str(resolved.GetPath())
        self._resolved_camera_reason = str(reason)
        self._resolved_camera_prim = resolved_path
        if resolved_path != self.camera_prim:
            print(
                f"[PTZ-RTSP] warning: camera_prim '{self.camera_prim}' unavailable, "
                f"fallback to '{resolved_path}' ({reason})"
            )
            self.camera_prim = resolved_path

    def _resolve_camera_for_tuning(self, stage):
        resolved, _reason = self._resolve_camera_prim(stage)
        return resolved

    def _collect_dynamic_tuning_attrs(
        self, camera_prim
    ) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...], list[str], list[str], list[str]]:
        auto_attrs = list(self._AE_BOOL_ATTRS)
        exp_attrs = list(self._EXPOSURE_ATTRS)
        iso_attrs = list(self._ISO_ATTRS)
        available_auto: list[str] = []
        available_exp: list[str] = []
        available_iso: list[str] = []

        try:
            attrs = list(camera_prim.GetAttributes())
        except Exception:
            attrs = []

        for attr in attrs:
            name = str(attr.GetName())
            lname = name.lower()
            if "auto" in lname and "exposure" in lname:
                if name not in auto_attrs:
                    auto_attrs.append(name)
                available_auto.append(name)
                continue
            if ("iso" in lname) or ("sensitivity" in lname):
                if name not in iso_attrs:
                    iso_attrs.append(name)
                available_iso.append(name)
                continue
            if "exposure" in lname and "auto" not in lname:
                if name not in exp_attrs:
                    exp_attrs.append(name)
                available_exp.append(name)

        return (
            self._dedupe_attr_names(auto_attrs),
            self._dedupe_attr_names(exp_attrs),
            self._dedupe_attr_names(iso_attrs),
            available_auto,
            available_exp,
            available_iso,
        )

    def _read_first_scalar_attr(self, camera_prim, attr_names: tuple[str, ...], default: float) -> float:
        for attr_name in attr_names:
            attr = camera_prim.GetAttribute(attr_name)
            if not (attr and attr.IsValid()):
                continue
            try:
                return float(attr.Get())
            except Exception:
                continue
        return default

    def _detect_auto_exposure_flag(self, camera_prim, attr_names: tuple[str, ...] | None = None) -> bool:
        names = attr_names or self._AE_BOOL_ATTRS
        for attr_name in names:
            value = self._read_attr_bool(camera_prim, attr_name)
            if value is not None:
                return bool(value)
        return True

    def _camera_tuning_snapshot(self) -> dict[str, Any]:
        with self._camera_tuning_lock:
            ae_enabled = bool(self._camera_tuning.get("ae_enabled", True))
            ae_value = float(self._camera_tuning["ae_value"])
            iso = float(self._camera_tuning["iso"])
            diag = dict(self._camera_tuning_diag)
        ae_clamped = max(self._AE_MIN, min(self._AE_MAX, ae_value))
        iso_clamped = max(self._ISO_MIN, min(self._ISO_MAX, iso))
        rtx_ae_enabled = self._read_carb_setting_bool(self._AE_ENABLE_SETTING_KEYS[0], ae_enabled)
        rtx_ae_value = max(
            self._AE_MIN,
            min(self._AE_MAX, self._read_carb_setting_scalar(self._AE_SETTING_KEYS[0], ae_clamped)),
        )
        rtx_iso = max(
            self._ISO_MIN,
            min(self._ISO_MAX, self._read_carb_setting_scalar(self._ISO_SETTING_KEYS[0], iso_clamped)),
        )
        if ae_enabled:
            applied_exposure = ae_clamped
        else:
            applied_exposure = math.log2(max(1.0, iso_clamped) / 100.0)
        return {
            "ae_enabled": bool(ae_enabled),
            "ae_value": float(ae_clamped),
            "exposure_ev": float(ae_clamped),  # legacy key for compatibility
            "iso": float(iso_clamped),
            "exposure_applied": float(applied_exposure),
            "ae_range": [float(self._AE_MIN), float(self._AE_MAX)],
            "iso_range": [float(self._ISO_MIN), float(self._ISO_MAX)],
            "rtx_ae_enabled": bool(rtx_ae_enabled),
            "rtx_ae_value": float(rtx_ae_value),
            "rtx_iso": float(rtx_iso),
            "tuning_diag": diag,
        }

    def _init_camera_tuning(self, stage) -> None:
        camera_prim = self._resolve_camera_for_tuning(stage)
        if not camera_prim.IsValid():
            with self._camera_tuning_lock:
                self._camera_tuning_diag.update(
                    {
                        "target_camera_prim": self.camera_prim,
                        "target_camera_type": "",
                        "last_error": f"camera prim not found: {self.camera_prim}",
                    }
                )
            return

        auto_attrs, exp_attrs, iso_attrs, available_auto, available_exp, available_iso = self._collect_dynamic_tuning_attrs(
            camera_prim
        )

        ae_value = self._read_first_scalar_attr(camera_prim, exp_attrs, 0.0)
        ae_enabled = self._detect_auto_exposure_flag(camera_prim, auto_attrs)
        iso = max(self._ISO_MIN, min(self._ISO_MAX, self._read_first_scalar_attr(camera_prim, iso_attrs, 100.0)))
        # Fallback/read-through from official Isaac RTX settings.
        ae_enabled = self._read_carb_setting_bool(self._AE_ENABLE_SETTING_KEYS[0], ae_enabled)
        ae_value = self._read_carb_setting_scalar(self._AE_SETTING_KEYS[0], ae_value)
        iso = self._read_carb_setting_scalar(self._ISO_SETTING_KEYS[0], iso)

        with self._camera_tuning_lock:
            self._camera_tuning["ae_enabled"] = bool(ae_enabled)
            self._camera_tuning["ae_value"] = float(max(self._AE_MIN, min(self._AE_MAX, ae_value)))
            self._camera_tuning["iso"] = float(max(self._ISO_MIN, min(self._ISO_MAX, iso)))
            self._camera_tuning_diag.update(
                {
                    "target_camera_prim": str(camera_prim.GetPath()),
                    "target_camera_type": str(camera_prim.GetTypeName() or ""),
                        "available_auto_attrs": available_auto[:40],
                        "available_exposure_attrs": available_exp[:40],
                        "available_iso_attrs": available_iso[:40],
                        "applied_setting_keys": [],
                        "last_error": "",
                    }
                )
        self._camera_tuning_dirty.set()

    def _apply_camera_tuning(self, stage) -> None:
        camera_prim = self._resolve_camera_for_tuning(stage)
        if not camera_prim.IsValid():
            with self._camera_tuning_lock:
                self._camera_tuning_diag.update(
                    {
                        "target_camera_prim": self.camera_prim,
                        "target_camera_type": "",
                        "applied_auto_attrs": [],
                        "applied_exposure_attrs": [],
                        "applied_iso_attrs": [],
                        "applied_setting_keys": [],
                        "last_error": f"camera prim not found: {self.camera_prim}",
                    }
                )
            return

        tuning = self._camera_tuning_snapshot()
        ae_enabled = bool(tuning["ae_enabled"])
        exposure_applied = float(tuning["exposure_applied"])
        iso_value = float(tuning["iso"])
        auto_attrs, exp_attrs, iso_attrs, available_auto, available_exp, available_iso = self._collect_dynamic_tuning_attrs(
            camera_prim
        )

        applied_auto = self._set_auto_exposure_flag(camera_prim, auto_attrs, ae_enabled)
        if ae_enabled:
            applied_exp = self._set_scalar_attrs(camera_prim, exp_attrs, exposure_applied)
            applied_iso: list[str] = []
        else:
            # Prefer direct ISO attrs. Only fallback to exposure mapping if ISO attrs do not exist.
            applied_iso = self._set_scalar_attrs(camera_prim, iso_attrs, iso_value)
            applied_exp = self._set_scalar_attrs(camera_prim, exp_attrs, exposure_applied) if not applied_iso else []

        # Always apply official Isaac RTX exposure controls (not image post-multiply).
        applied_settings: list[str] = []
        for key in self._AE_ENABLE_SETTING_KEYS:
            if self._set_carb_setting_bool(key, ae_enabled):
                applied_settings.append(key)
        if ae_enabled:
            for key in self._AE_SETTING_KEYS:
                if self._set_carb_setting_scalar(key, exposure_applied):
                    applied_settings.append(key)
        else:
            for key in self._ISO_SETTING_KEYS:
                if self._set_carb_setting_scalar(key, iso_value):
                    applied_settings.append(key)

        last_error = ""
        if not applied_auto and not applied_exp and not applied_iso and not applied_settings:
            last_error = "no matching AE/ISO camera attrs or RTX setting keys were found"
            print(
                f"[PTZ-RTSP] warning: camera tuning not applied on {camera_prim.GetPath()} "
                f"(type={camera_prim.GetTypeName()})"
            )

        with self._camera_tuning_lock:
            self._camera_tuning_diag.update(
                {
                    "target_camera_prim": str(camera_prim.GetPath()),
                    "target_camera_type": str(camera_prim.GetTypeName() or ""),
                    "available_auto_attrs": available_auto[:40],
                    "available_exposure_attrs": available_exp[:40],
                    "available_iso_attrs": available_iso[:40],
                    "applied_auto_attrs": applied_auto,
                    "applied_exposure_attrs": applied_exp,
                    "applied_iso_attrs": applied_iso,
                    "applied_setting_keys": applied_settings,
                    "last_error": last_error,
                }
            )

    def _ensure_rtsp_publisher(self) -> None:
        if not self.rtsp_enabled:
            return
        if self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
            return
        now = time.time()
        if now - self._ffmpeg_last_retry_ts < self._ffmpeg_retry_interval_s:
            return
        self._ffmpeg_last_retry_ts = now
        try:
            self._ffmpeg_proc = start_ffmpeg(
                self.rtsp_url,
                self.width,
                self.height,
                self.fps,
                self.bitrate,
                ffmpeg_path=self.ffmpeg_path,
            )
            print("[PTZ-RTSP] ffmpeg publisher restarted")
        except Exception as exc:
            print(f"[PTZ-RTSP] ffmpeg restart failed: {exc}")
            self._ffmpeg_proc = None

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
                    state["instance_id"] = runtime._instance_id
                    state["scene_path"] = runtime.scene_path
                    state["camera_prim"] = runtime.camera_prim
                    state.update(runtime._camera_tuning_snapshot())
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

                if path == "/render/diag":
                    try:
                        stage = omni.usd.get_context().get_stage()
                        self._json(runtime._collect_render_diag(stage))
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 500)
                    return

                if path == "/render/pathtracing":
                    try:
                        self._json(runtime._path_tracing_snapshot())
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 500)
                    return

                if path == "/render/volumetric":
                    try:
                        self._json(runtime._volumetric_snapshot())
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 500)
                    return

                if path == "/camera/tuning":
                    payload = {"ok": True}
                    payload.update(runtime._camera_tuning_snapshot())
                    self._json(payload)
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
                        # Return immediately and let render loop apply the latest PTZ state.
                        # This avoids request queue buildup during high-frequency control updates.
                        self._json({"ok": True, "state": state})
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                if self.path == "/camera/tuning":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        with runtime._camera_tuning_lock:
                            if "ae_enabled" in request:
                                runtime._camera_tuning["ae_enabled"] = bool(request["ae_enabled"])
                            if "ae_value" in request:
                                runtime._camera_tuning["ae_value"] = max(
                                    runtime._AE_MIN, min(runtime._AE_MAX, float(request["ae_value"]))
                                )
                            elif "exposure_ev" in request:
                                runtime._camera_tuning["ae_value"] = max(
                                    runtime._AE_MIN, min(runtime._AE_MAX, float(request["exposure_ev"]))
                                )
                            if "iso" in request:
                                runtime._camera_tuning["iso"] = max(
                                    runtime._ISO_MIN, min(runtime._ISO_MAX, float(request["iso"]))
                                )
                        runtime._camera_tuning_dirty.set()
                        runtime._wait_for_camera_tuning_apply()
                        payload = {"ok": True}
                        payload.update(runtime._camera_tuning_snapshot())
                        self._json(payload)
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                if self.path == "/render/pathtracing":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        if "spp" not in request:
                            raise ValueError("spp is required")
                        target_spp = runtime._clamp_path_tracing_spp(request.get("spp"), default=1)
                        with runtime._path_tracing_lock:
                            runtime.path_tracing_spp = int(target_spp)
                            applied = runtime._apply_path_tracing_spp(int(target_spp))
                        payload = runtime._path_tracing_snapshot()
                        payload["requested_spp"] = int(target_spp)
                        payload["applied_spp"] = int(applied)
                        if runtime.renderer_mode != "PathTracing":
                            payload["warning"] = "renderer_mode is not PathTracing; spp updated but not active now"
                        self._json(payload)
                    except Exception as exc:
                        self._json({"ok": False, "error": str(exc)}, 400)
                    return

                if self.path == "/render/volumetric":
                    length = int(self.headers.get("Content-Length", 0))
                    try:
                        request = json.loads(self.rfile.read(length) or b"{}")
                        if not isinstance(request, dict):
                            raise ValueError("request body must be an object")

                        with runtime._volumetric_lock:
                            cur = runtime._volumetric
                            changed = False
                            for key, (lo, hi) in runtime._VOLUMETRIC_FLOAT_RANGES.items():
                                if key in request:
                                    cur[key] = runtime._clamp_float(request.get(key), lo, hi, cur[key])
                                    changed = True
                            for key, (lo, hi) in runtime._VOLUMETRIC_INT_RANGES.items():
                                if key in request:
                                    cur[key] = runtime._clamp_int(request.get(key), lo, hi, cur[key])
                                    changed = True
                            if "enabled" in request:
                                cur["enabled"] = bool(request.get("enabled"))
                                changed = True
                            if "apply_density_noise" in request:
                                cur["apply_density_noise"] = bool(request.get("apply_density_noise"))
                                changed = True
                            if "transmittance_color" in request:
                                cur["transmittance_color"] = runtime._normalize_color3(
                                    request.get("transmittance_color"),
                                    cur["transmittance_color"],
                                )
                                changed = True
                            if "single_scattering_albedo" in request:
                                cur["single_scattering_albedo"] = runtime._normalize_color3(
                                    request.get("single_scattering_albedo"),
                                    cur["single_scattering_albedo"],
                                )
                                changed = True

                        if changed:
                            runtime._volumetric_dirty.set()
                            runtime._wait_for_volumetric_apply()
                        payload = runtime._volumetric_snapshot()
                        payload["changed"] = bool(changed)
                        if not changed:
                            payload["message"] = "no recognized volumetric fields"
                        self._json(payload)
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
        self._ensure_valid_camera_prim(stage)
        prim = stage.GetPrimAtPath(self.camera_prim)
        if not prim.IsValid():
            raise ValueError(f"Camera prim not found after fallback: {self.camera_prim}")

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
                self._ffmpeg_proc = start_ffmpeg(
                    self.rtsp_url,
                    self.width,
                    self.height,
                    self.fps,
                    self.bitrate,
                    ffmpeg_path=self.ffmpeg_path,
                )
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

        # Stage load can override carb RTX settings; re-apply the runtime profile.
        self._apply_render_throttle()

        self.sim_app.update()
        self.sim_app.update()

        try:
            from pxr import UsdGeom

            self._scene_up_axis = UsdGeom.GetStageUpAxis(usd_context.get_stage())
        except Exception:
            self._scene_up_axis = "Y"

        stage = usd_context.get_stage()
        self._ensure_valid_camera_prim(stage)
        self._init_ptz_reference(stage)
        self._apply_scene_state(stage)
        self._init_camera_tuning(stage)
        self._apply_camera_tuning(stage)
        self._init_volumetric_state()
        self._apply_volumetric_state()
        self._volumetric_dirty.clear()

        stage_units_in_meters = 0.01 if self._scene_up_axis == "Y" else 1.0
        world = World(stage_units_in_meters=stage_units_in_meters)
        world.reset()
        self._apply_render_throttle()

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
            self._ensure_rtsp_publisher()
            world.step(render=True)

            if self._ptz_dirty.is_set():
                self._ptz_dirty.clear()
                self._apply_ptz_state(stage)

            if self._scene_dirty.is_set():
                self._scene_dirty.clear()
                self._apply_scene_state(stage)

            if self._camera_tuning_dirty.is_set():
                self._camera_tuning_dirty.clear()
                self._apply_camera_tuning(stage)

            if self._volumetric_dirty.is_set():
                self._volumetric_dirty.clear()
                self._apply_volumetric_state()

            rgba = annotator.get_data()

            if rgba is not None and getattr(rgba, "size", 0) > 0:
                rgba_raw = resize_rgba_frame(np.asarray(rgba), self.width, self.height)
                if not self._frame_format_logged:
                    try:
                        raw_min = float(np.min(rgba_raw))
                        raw_max = float(np.max(rgba_raw))
                        print(
                            f"[PTZ-RTSP] frame format: dtype={rgba_raw.dtype} "
                            f"shape={tuple(rgba_raw.shape)} min={raw_min:.4f} max={raw_max:.4f}"
                        )
                    except Exception:
                        pass
                    self._frame_format_logged = True

                rgba = self._to_uint8_rgba(rgba_raw)

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
