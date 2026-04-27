from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import threading
import time
import traceback
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .common import (
    CREATE_NEW_PROCESS_GROUP,
    IS_WINDOWS,
    default_python_launcher,
    load_yaml_config,
    normalize_renderer_mode,
    resolve_python_launcher,
    resolve_repo_path,
    tcp_port_free,
)
from .launcher_onvif import OnvifService


class LauncherApp:
    @staticmethod
    def _clamp_path_tracing_spp(value: object, default: int = 1) -> int:
        try:
            spp = int(value)
        except Exception:
            spp = int(default)
        return max(1, min(512, spp))

    def __init__(self, config_path: str, script_dir: str) -> None:
        self.script_dir = script_dir
        self.config_path = config_path
        self.cfg = load_yaml_config(config_path)

        self.launcher_port = int(self.cfg.get("launcher_port", 8080))
        self.isaac_port = int(self.cfg.get("ctrl_port", 8081))
        self.python_launcher = self.cfg.get("python_sh", default_python_launcher())
        self.stream_script = os.path.join(script_dir, "ptz_rtsp_stream.py")
        self.isaac_log = os.path.join(script_dir, "isaac_stream.log")

        self._runtime_cfg_lock = threading.Lock()
        self._runtime_scene_path = self.cfg.get("scene_path")
        self._runtime_camera_prim = self.cfg.get("camera_prim")
        try:
            self._runtime_renderer_mode = normalize_renderer_mode(self.cfg.get("renderer_mode", "PathTracing"))
        except ValueError:
            self._runtime_renderer_mode = "PathTracing"
            print("[PTZ-Launcher] invalid renderer_mode in config, fallback to PathTracing")
        self._runtime_path_tracing_spp = self._clamp_path_tracing_spp(self.cfg.get("path_tracing_spp", 1), default=1)
        self._last_good_runtime_cfg = {
            "scene_path": self._runtime_scene_path,
            "camera_prim": self._runtime_camera_prim,
            "renderer_mode": self._runtime_renderer_mode,
            "path_tracing_spp": self._runtime_path_tracing_spp,
        }

        self._proc_lock = threading.Lock()
        self._isaac_proc: subprocess.Popen | None = None
        self._start_time: float | None = None
        self._isaac_state = "stopped"
        self._startup_error: str | None = None

        self._restart_lock = threading.Lock()
        self._restart_in_progress = False
        self._auto_renderer_fallback_used = False

        self.onvif = OnvifService(self.isaac_port)

    @staticmethod
    def _loopback_url(port: int, path: str) -> str:
        return f"http://127.0.0.1:{port}{path}"

    def _resolve_scene_path(self, scene_path: str | None) -> str | None:
        return resolve_repo_path(self.script_dir, scene_path)

    def _snapshot_runtime_cfg(self) -> dict[str, object]:
        with self._runtime_cfg_lock:
            scene_path = self._runtime_scene_path
            camera_prim = self._runtime_camera_prim
            renderer_mode = self._runtime_renderer_mode
            path_tracing_spp = self._runtime_path_tracing_spp
        return {
            "scene_path": scene_path,
            "scene_path_abs": self._resolve_scene_path(scene_path),
            "camera_prim": camera_prim,
            "renderer_mode": renderer_mode,
            "path_tracing_spp": int(path_tracing_spp),
        }

    def _apply_runtime_cfg_dict(self, cfg: dict[str, object]) -> None:
        with self._runtime_cfg_lock:
            if cfg.get("scene_path") is not None:
                self._runtime_scene_path = str(cfg.get("scene_path"))
            if cfg.get("camera_prim") is not None:
                self._runtime_camera_prim = str(cfg.get("camera_prim"))
            if cfg.get("renderer_mode") is not None:
                self._runtime_renderer_mode = normalize_renderer_mode(str(cfg.get("renderer_mode")))
            if cfg.get("path_tracing_spp") is not None:
                self._runtime_path_tracing_spp = self._clamp_path_tracing_spp(cfg.get("path_tracing_spp"), default=1)

    def _validate_runtime_cfg(
        self,
        scene_path: str | None = None,
        camera_prim: str | None = None,
        renderer_mode: str | None = None,
        path_tracing_spp: int | None = None,
        validate_scene_camera_pair: bool = False,
    ) -> list[str]:
        errors: list[str] = []
        if scene_path is not None:
            if not isinstance(scene_path, str) or not scene_path.strip():
                errors.append("scene_path cannot be empty")
            else:
                path = scene_path.strip()
                if "://" not in path:
                    ext = os.path.splitext(path)[1].lower()
                    if ext not in (".usd", ".usda", ".usdc", ".usdz"):
                        errors.append("scene_path must point to a USD file")
                    resolved = self._resolve_scene_path(path)
                    if not resolved or not os.path.isfile(resolved):
                        errors.append(f"scene_path file not found: {resolved}")

        if camera_prim is not None:
            if not isinstance(camera_prim, str) or not camera_prim.strip():
                errors.append("camera_prim cannot be empty")
            elif not camera_prim.strip().startswith("/"):
                errors.append("camera_prim must be an absolute USD prim path")
        if renderer_mode is not None:
            try:
                normalize_renderer_mode(renderer_mode)
            except ValueError as exc:
                errors.append(str(exc))
        if path_tracing_spp is not None:
            if not isinstance(path_tracing_spp, int):
                errors.append("path_tracing_spp must be an integer")
            elif not (1 <= path_tracing_spp <= 512):
                errors.append("path_tracing_spp must be between 1 and 512")

        if validate_scene_camera_pair and scene_path and camera_prim and not errors:
            resolved = self._resolve_scene_path(scene_path)
            if resolved and os.path.isfile(resolved):
                try:
                    from pxr import Usd

                    stage = Usd.Stage.Open(resolved)
                    if stage is None:
                        errors.append(f"unable to open scene: {resolved}")
                    else:
                        prim = stage.GetPrimAtPath(camera_prim)
                        if not prim.IsValid():
                            errors.append(f"camera_prim not found in scene: {camera_prim}")
                except ModuleNotFoundError as exc:
                    if "pxr" in str(exc):
                        # Some launcher environments do not ship pxr in the runtime.
                        # Skip strict scene graph validation and let stream startup validate it.
                        print("[PTZ-Launcher] warning: pxr unavailable, skip scene/camera prim validation")
                    else:
                        errors.append(f"scene/camera validation failed: {exc}")
                except Exception as exc:
                    errors.append(f"scene/camera validation failed: {exc}")
        return errors

    def _fetch_isaac_status(self) -> dict[str, object] | None:
        try:
            with urllib.request.urlopen(
                self._loopback_url(self.isaac_port, "/status"),
                timeout=1,
            ) as response:
                if response.status != 200:
                    return None
                payload = json.loads(response.read() or b"{}")
                if isinstance(payload, dict):
                    return payload
                return None
        except Exception:
            return None

    def _is_isaac_http_ready(self) -> bool:
        status = self._fetch_isaac_status()
        return bool(status and status.get("ready", False))

    def _tail_log_line(self, max_bytes: int = 4096) -> str:
        if not os.path.isfile(self.isaac_log):
            return ""
        try:
            with open(self.isaac_log, "rb") as handle:
                size = os.path.getsize(self.isaac_log)
                handle.seek(max(0, size - max_bytes))
                data = handle.read().decode("utf-8", errors="replace")
        except Exception:
            return ""
        lines = [line.strip() for line in data.splitlines() if line.strip()]
        if not lines:
            return ""
        return lines[-1][:320]

    def _build_exit_error(self, returncode: int | None, context: str) -> str:
        tail = self._tail_log_line()
        base = f"{context} (exit={returncode})"
        return f"{base}: {tail}" if tail else base

    def _watch_isaac(self, proc: subprocess.Popen) -> None:
        timeout_s = 300
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if proc.poll() is not None:
                should_auto_fallback = False
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        prev_state = self._isaac_state
                        current_renderer = str(self._runtime_renderer_mode or "")
                        self._isaac_proc = None
                        self._isaac_state = "stopped"
                        if (
                            prev_state not in ("stopping", "stopped")
                            and current_renderer == "PathTracing"
                            and not self._auto_renderer_fallback_used
                        ):
                            self._auto_renderer_fallback_used = True
                            self._runtime_renderer_mode = "RTXRealTime"
                            self._startup_error = (
                                "PathTracing startup crashed; auto-switched to RTX Real-Time, restarting"
                            )
                            should_auto_fallback = True
                        elif prev_state not in ("stopping", "stopped"):
                            self._startup_error = self._build_exit_error(proc.returncode, "Isaac exited before ready")
                if should_auto_fallback:
                    print("[PTZ-Launcher] auto fallback: PathTracing -> RTX Real-Time")
                    self._schedule_restart(reason="auto renderer fallback after crash")
                return
            if self._is_isaac_http_ready():
                with self._runtime_cfg_lock:
                    last_good = {
                        "scene_path": self._runtime_scene_path,
                        "camera_prim": self._runtime_camera_prim,
                        "renderer_mode": self._runtime_renderer_mode,
                        "path_tracing_spp": self._runtime_path_tracing_spp,
                    }
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        self._isaac_state = "running"
                        self._startup_error = None
                        self._auto_renderer_fallback_used = False
                self._last_good_runtime_cfg = last_good
                break
            time.sleep(1)
        else:
            with self._proc_lock:
                if self._isaac_proc is proc and proc.poll() is None:
                    self._isaac_state = "starting"
                    self._startup_error = f"Isaac HTTP did not become ready within {timeout_s} seconds"

        while True:
            if proc.poll() is not None:
                should_auto_fallback = False
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        prev_state = self._isaac_state
                        current_renderer = str(self._runtime_renderer_mode or "")
                        self._isaac_proc = None
                        self._isaac_state = "stopped"
                        if (
                            prev_state not in ("stopping", "stopped")
                            and current_renderer == "PathTracing"
                            and not self._auto_renderer_fallback_used
                        ):
                            self._auto_renderer_fallback_used = True
                            self._runtime_renderer_mode = "RTXRealTime"
                            self._startup_error = (
                                "PathTracing runtime crashed; auto-switched to RTX Real-Time, restarting"
                            )
                            should_auto_fallback = True
                        elif prev_state not in ("stopping", "stopped"):
                            self._startup_error = self._build_exit_error(proc.returncode, "Isaac exited unexpectedly")
                if should_auto_fallback:
                    print("[PTZ-Launcher] auto fallback: PathTracing crash detected, restarting with RTX Real-Time")
                    self._schedule_restart(reason="auto renderer fallback after runtime crash")
                return
            time.sleep(2)

    def start_isaac(self) -> dict[str, object]:
        with self._proc_lock:
            if self._isaac_proc is not None:
                if self._isaac_proc.poll() is None:
                    return {
                        "ok": True,
                        "already_running": True,
                        "message": "Isaac Sim is already running",
                        "pid": self._isaac_proc.pid,
                    }
                # Cleanup stale process handle before re-launch.
                self._isaac_proc = None
                if self._isaac_state != "stopping":
                    self._isaac_state = "stopped"

        if not tcp_port_free(self.isaac_port, timeout_s=0.5):
            cleanup = self._force_release_runtime_ports()
            time.sleep(0.8)
            if not tcp_port_free(self.isaac_port, timeout_s=0.5):
                detail = cleanup.get("killed_pids") or []
                return {
                    "ok": False,
                    "error": f"Isaac control port {self.isaac_port} is still occupied after cleanup (killed={detail})",
                }

        runtime_cfg = self._snapshot_runtime_cfg()
        scene_path = runtime_cfg.get("scene_path")
        camera_prim = runtime_cfg.get("camera_prim")
        renderer_mode = runtime_cfg.get("renderer_mode")
        path_tracing_spp = int(runtime_cfg.get("path_tracing_spp", 1))
        python_launcher = resolve_python_launcher(self.python_launcher)
        if not python_launcher:
            return {"ok": False, "error": "python_sh is not configured"}
        if not os.path.isfile(python_launcher):
            return {"ok": False, "error": f"Isaac Python launcher not found: {python_launcher}"}
        if not os.path.isfile(self.stream_script):
            return {"ok": False, "error": f"Stream entrypoint not found: {self.stream_script}"}

        command = [
            python_launcher,
            "-u",
            self.stream_script,
            "--config",
            self.config_path,
            "--ctrl-port",
            str(self.isaac_port),
        ]
        if scene_path:
            command.extend(["--scene", str(scene_path)])
        if camera_prim:
            command.extend(["--camera", str(camera_prim)])
        if renderer_mode:
            command.extend(["--renderer-mode", str(renderer_mode)])
        command.extend(["--path-tracing-spp", str(path_tracing_spp)])

        popen_kwargs: dict[str, object] = {
            "stdout": None,
            "stderr": None,
            "cwd": self.script_dir,
        }
        try:
            log_file = open(self.isaac_log, "w", encoding="utf-8", buffering=1)
        except OSError as exc:
            return {"ok": False, "error": f"Unable to open log file: {exc}"}

        popen_kwargs["stdout"] = log_file
        popen_kwargs["stderr"] = log_file
        if IS_WINDOWS:
            popen_kwargs["creationflags"] = CREATE_NEW_PROCESS_GROUP
        else:
            popen_kwargs["start_new_session"] = True

        try:
            proc = subprocess.Popen(command, **popen_kwargs)
        except OSError as exc:
            log_file.close()
            return {"ok": False, "error": f"Failed to launch Isaac Sim: {exc}"}

        log_file.close()
        with self._proc_lock:
            self._isaac_proc = proc
            self._start_time = time.time()
            self._isaac_state = "starting"
            self._startup_error = None

        threading.Thread(target=self._watch_isaac, args=(proc,), daemon=True, name="isaac-watch").start()
        return {
            "ok": True,
            "pid": proc.pid,
            "log": self.isaac_log,
            "scene_path": scene_path,
            "camera_prim": camera_prim,
            "renderer_mode": renderer_mode,
            "path_tracing_spp": path_tracing_spp,
        }

    def _is_isaac_alive(self) -> bool:
        with self._proc_lock:
            return self._isaac_proc is not None and self._isaac_proc.poll() is None

    def _wait_stopped(self, timeout_s: float = 45.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._proc_lock:
                alive = self._isaac_proc is not None and self._isaac_proc.poll() is None
                state = self._isaac_state
            port_free = tcp_port_free(self.isaac_port, timeout_s=0.3)
            if (not alive) and state == "stopped" and port_free:
                return True
            time.sleep(0.3)
        return False

    def _pids_listening_on_port(self, port: int) -> set[int]:
        if not IS_WINDOWS:
            return set()
        try:
            proc = subprocess.run(
                ["netstat", "-ano", "-p", "tcp"],
                capture_output=True,
                text=True,
                timeout=5,
                encoding="utf-8",
                errors="ignore",
            )
        except Exception:
            return set()
        if proc.returncode != 0:
            return set()

        pattern = re.compile(rf":{int(port)}$")
        pids: set[int] = set()
        for line in proc.stdout.splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            local_addr = parts[1].strip()
            state = parts[3].strip().upper()
            pid_raw = parts[4].strip()
            if state != "LISTENING":
                continue
            if not pattern.search(local_addr):
                continue
            try:
                pid = int(pid_raw)
            except ValueError:
                continue
            if pid > 0 and pid != os.getpid():
                pids.add(pid)
        return pids

    def _force_release_runtime_ports(self) -> dict[str, object]:
        ports = (int(self.isaac_port), 8554, 8888)
        killed: list[int] = []
        errors: list[str] = []
        for port in ports:
            for pid in sorted(self._pids_listening_on_port(port)):
                try:
                    proc = subprocess.run(
                        ["taskkill", "/PID", str(pid), "/T", "/F"],
                        capture_output=True,
                        timeout=8,
                        text=True,
                        encoding="utf-8",
                        errors="ignore",
                    )
                    if proc.returncode == 0:
                        killed.append(pid)
                    else:
                        detail = (proc.stderr or proc.stdout or "").strip()
                        errors.append(f"pid={pid}: taskkill failed ({detail})")
                except Exception as exc:
                    errors.append(f"pid={pid}: {exc}")
        return {"killed_pids": sorted(set(killed)), "errors": errors}

    def _wait_running(self, timeout_s: float = 180.0) -> tuple[bool, str | None]:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._proc_lock:
                proc = self._isaac_proc
                state = self._isaac_state
            if proc is None:
                return False, f"Isaac process handle missing (state={state})"
            if proc.poll() is not None:
                return False, self._build_exit_error(proc.returncode, "Isaac exited during restart")

            if self._is_isaac_http_ready():
                with self._proc_lock:
                    self._isaac_state = "running"
                    self._startup_error = None
                self._last_good_runtime_cfg = {
                    "scene_path": self._snapshot_runtime_cfg().get("scene_path"),
                    "camera_prim": self._snapshot_runtime_cfg().get("camera_prim"),
                    "renderer_mode": self._snapshot_runtime_cfg().get("renderer_mode"),
                    "path_tracing_spp": self._snapshot_runtime_cfg().get("path_tracing_spp"),
                }
                return True, None
            time.sleep(1.0)
        return False, f"Isaac did not become ready within {int(timeout_s)} seconds"

    def _schedule_restart(self, reason: str = "runtime config updated", fallback_cfg: dict[str, object] | None = None) -> dict[str, object]:
        with self._restart_lock:
            if self._restart_in_progress:
                return {"ok": False, "error": "Restart already in progress"}
            self._restart_in_progress = True

        def _job() -> None:
            try:
                print(f"[PTZ-Launcher] restarting Isaac Sim ({reason})")
                state_before = self.get_local_state()
                if state_before in ("running", "starting", "stopping") or self._is_isaac_alive():
                    self.stop_isaac(clear_startup_error=False)

                if not self._wait_stopped(timeout_s=120.0):
                    print("[PTZ-Launcher] restart wait timeout, retrying force stop")
                    self.stop_isaac(clear_startup_error=False)
                    if not self._wait_stopped(timeout_s=60.0):
                        cleanup = self._force_release_runtime_ports()
                        time.sleep(1.0)
                        if not self._wait_stopped(timeout_s=12.0):
                            msg = (
                                "Restart failed: previous Isaac process did not stop in time; "
                                f"forced cleanup killed={cleanup.get('killed_pids', [])}"
                            )
                            print(f"[PTZ-Launcher] {msg}")
                            with self._proc_lock:
                                self._startup_error = msg
                            return

                result = self.start_isaac()
                if not result.get("ok", False):
                    print(f"[PTZ-Launcher] restart failed: {result}")
                    with self._proc_lock:
                        self._startup_error = str(result.get("error", "restart failed"))
                    return

                running_ok, running_err = self._wait_running(timeout_s=180.0)
                if running_ok:
                    return

                fail_msg = running_err or "restart failed while waiting for ready"
                print(f"[PTZ-Launcher] restart verification failed: {fail_msg}")
                with self._proc_lock:
                    self._startup_error = fail_msg

                if not fallback_cfg:
                    return

                current_cfg = self._snapshot_runtime_cfg()
                fallback_scene = fallback_cfg.get("scene_path")
                fallback_camera = fallback_cfg.get("camera_prim")
                fallback_renderer = fallback_cfg.get("renderer_mode")
                fallback_spp = fallback_cfg.get("path_tracing_spp")
                if (
                    fallback_scene == current_cfg.get("scene_path")
                    and fallback_camera == current_cfg.get("camera_prim")
                    and fallback_renderer == current_cfg.get("renderer_mode")
                    and fallback_spp == current_cfg.get("path_tracing_spp")
                ):
                    return

                print("[PTZ-Launcher] attempting rollback to last known good runtime model")
                try:
                    self._apply_runtime_cfg_dict(
                        {
                            "scene_path": fallback_scene,
                            "camera_prim": fallback_camera,
                            "renderer_mode": fallback_renderer,
                            "path_tracing_spp": fallback_spp,
                        }
                    )
                except Exception as exc:
                    with self._proc_lock:
                        self._startup_error = f"rollback config apply failed: {exc}"
                    return

                if self._is_isaac_alive() or self.get_local_state() in ("running", "starting", "stopping"):
                    self.stop_isaac(clear_startup_error=False)
                    self._wait_stopped(timeout_s=60.0)

                rollback_result = self.start_isaac()
                if not rollback_result.get("ok", False):
                    with self._proc_lock:
                        self._startup_error = str(rollback_result.get("error", "rollback restart failed"))
                    print(f"[PTZ-Launcher] rollback restart failed: {rollback_result}")
                    return

                rollback_ok, rollback_err = self._wait_running(timeout_s=180.0)
                if rollback_ok:
                    print("[PTZ-Launcher] rollback recovery succeeded")
                    return

                with self._proc_lock:
                    self._startup_error = rollback_err or "rollback restart failed"
                print(f"[PTZ-Launcher] rollback recovery failed: {self._startup_error}")
            except Exception as exc:
                detail = "".join(traceback.format_exception_only(type(exc), exc)).strip()
                with self._proc_lock:
                    self._startup_error = f"restart thread exception: {detail}"
                print(f"[PTZ-Launcher] restart thread exception: {detail}")
            finally:
                with self._restart_lock:
                    self._restart_in_progress = False

        threading.Thread(target=_job, daemon=True, name="isaac-restart").start()
        return {"ok": True}

    def _is_restart_in_progress(self) -> bool:
        with self._restart_lock:
            return self._restart_in_progress

    def stop_isaac(self, *, clear_startup_error: bool = True) -> dict[str, object]:
        with self._proc_lock:
            proc = self._isaac_proc
            if proc is None or proc.poll() is not None:
                self._isaac_proc = None
                self._isaac_state = "stopped"
                return {"ok": False, "error": "Isaac Sim is not running"}

            self._isaac_state = "stopping"
            pid = proc.pid
            pgid = None
            if not IS_WINDOWS:
                try:
                    pgid = os.getpgid(pid)
                except OSError:
                    pgid = pid

        def _wait() -> None:
            if IS_WINDOWS:
                try:
                    proc.terminate()
                except Exception:
                    pass
            else:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass

            deadline = time.time() + 15
            while time.time() < deadline:
                if tcp_port_free(self.isaac_port, timeout_s=0.3):
                    break
                time.sleep(0.5)

            if IS_WINDOWS:
                try:
                    subprocess.run(
                        ["taskkill", "/PID", str(pid), "/T", "/F"],
                        capture_output=True,
                        timeout=6,
                    )
                except Exception:
                    pass
            else:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass

            try:
                proc.wait(timeout=5)
            except Exception:
                pass

            for port in (self.isaac_port, 8554, 8888):
                for _ in range(20):
                    if tcp_port_free(port, timeout_s=0.3):
                        break
                    time.sleep(0.5)

            with self._proc_lock:
                if self._isaac_proc is proc:
                    self._isaac_proc = None
                self._isaac_state = "stopped"
                if clear_startup_error:
                    self._startup_error = None

        threading.Thread(target=_wait, daemon=True, name="isaac-stop").start()
        return {"ok": True, "stopping_pid": pid}

    def get_status(self) -> dict[str, object]:
        with self._proc_lock:
            proc = self._isaac_proc
            state = self._isaac_state
            start_time = self._start_time
            startup_error = self._startup_error
            dead = proc is not None and proc.poll() is not None
            missing = proc is None and state in ("starting", "running", "stopping")

        if dead and state not in ("stopped", "stopping"):
            with self._proc_lock:
                self._isaac_proc = None
                self._isaac_state = "stopped"
                if not self._startup_error:
                    self._startup_error = self._build_exit_error(proc.returncode if proc else None, "Isaac process exited")
                startup_error = self._startup_error
                state = "stopped"
        elif missing:
            with self._proc_lock:
                self._isaac_state = "stopped"
                if not self._startup_error and state in ("starting", "running"):
                    self._startup_error = "Isaac process handle missing (possible crash or external kill)"
                startup_error = self._startup_error
                state = "stopped"

        result: dict[str, object] = {
            "isaac_state": state,
            "isaac_port": self.isaac_port,
            "uptime_s": int(time.time() - start_time) if (start_time and state == "running") else 0,
            "ptz": None,
            "runtime_model": self._snapshot_runtime_cfg(),
            "restart_in_progress": self._is_restart_in_progress(),
            "startup_error": startup_error,
        }

        if state in ("running", "starting"):
            isaac_status = self._fetch_isaac_status()
            if isaac_status is not None:
                result["ptz"] = isaac_status
                if bool(isaac_status.get("ready", False)):
                    if state == "starting":
                        result["isaac_state"] = "running"
                    with self._proc_lock:
                        self._isaac_state = "running"
                        self._startup_error = None
                    result["startup_error"] = None
        return result

    def get_local_state(self) -> str:
        with self._proc_lock:
            proc = self._isaac_proc
            state = self._isaac_state
            dead = proc is not None and proc.poll() is not None
            missing = proc is None and state in ("starting", "running", "stopping")
            if dead and state not in ("stopped", "stopping"):
                self._isaac_proc = None
                self._isaac_state = "stopped"
                if not self._startup_error:
                    self._startup_error = self._build_exit_error(proc.returncode if proc else None, "Isaac process exited")
                return "stopped"
            if missing:
                self._isaac_state = "stopped"
                if not self._startup_error and state in ("starting", "running"):
                    self._startup_error = "Isaac process handle missing (possible crash or external kill)"
                return "stopped"
            return state

    def update_runtime_model(
        self,
        *,
        scene_path: str | None = None,
        camera_prim: str | None = None,
        renderer_mode: str | None = None,
        path_tracing_spp: int | None = None,
        restart_if_running: bool = False,
    ) -> dict[str, object]:
        if scene_path is None and camera_prim is None and renderer_mode is None and path_tracing_spp is None:
            return {"ok": False, "error": "scene_path or camera_prim or renderer_mode or path_tracing_spp is required"}

        old_cfg = self._snapshot_runtime_cfg()
        next_scene = old_cfg.get("scene_path") if scene_path is None else scene_path.strip()
        next_camera = old_cfg.get("camera_prim") if camera_prim is None else camera_prim.strip()
        try:
            next_renderer = (
                old_cfg.get("renderer_mode")
                if renderer_mode is None
                else normalize_renderer_mode(renderer_mode)
            )
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        try:
            next_spp = (
                int(old_cfg.get("path_tracing_spp", 1))
                if path_tracing_spp is None
                else int(path_tracing_spp)
            )
        except Exception:
            return {"ok": False, "error": "path_tracing_spp must be an integer"}

        errors = self._validate_runtime_cfg(
            scene_path=next_scene,
            camera_prim=next_camera,
            renderer_mode=next_renderer,
            path_tracing_spp=next_spp,
            validate_scene_camera_pair=True,
        )
        if errors:
            return {"ok": False, "error": "; ".join(errors)}
        if (
            next_scene == old_cfg.get("scene_path")
            and next_camera == old_cfg.get("camera_prim")
            and next_renderer == old_cfg.get("renderer_mode")
            and int(next_spp) == int(old_cfg.get("path_tracing_spp", 1))
        ):
            return {
                "ok": True,
                "old": old_cfg,
                "new": old_cfg,
                "isaac_state": self.get_local_state(),
                "restart_scheduled": False,
                "message": "Runtime model unchanged",
            }

        with self._runtime_cfg_lock:
            self._runtime_scene_path = next_scene
            self._runtime_camera_prim = next_camera
            self._runtime_renderer_mode = next_renderer
            self._runtime_path_tracing_spp = int(next_spp)
        new_cfg = self._snapshot_runtime_cfg()

        state_now = self.get_local_state()
        restart_scheduled = False
        if restart_if_running and state_now == "running":
            fallback_cfg = dict(self._last_good_runtime_cfg)
            scheduled = self._schedule_restart("runtime model changed", fallback_cfg=fallback_cfg)
            if not scheduled.get("ok", False):
                return {
                    "ok": False,
                    "error": scheduled.get("error", "Unable to schedule restart"),
                    "old": old_cfg,
                    "new": new_cfg,
                }
            restart_scheduled = True
        elif restart_if_running and state_now == "starting":
            return {
                "ok": True,
                "old": old_cfg,
                "new": new_cfg,
                "isaac_state": state_now,
                "restart_scheduled": False,
                "message": "Isaac is still starting; config saved and will apply on next start",
            }

        return {
            "ok": True,
            "old": old_cfg,
            "new": new_cfg,
            "isaac_state": state_now,
            "restart_scheduled": restart_scheduled,
            "message": "Runtime model updated and restart scheduled" if restart_scheduled else "Runtime model saved for the next start",
        }


def make_handler(app: LauncherApp):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args) -> None:
            pass

        def _cors(self) -> None:
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")

        def _json(self, data: dict[str, object], code: int = 200) -> None:
            body = json.dumps(data, ensure_ascii=False).encode("utf-8")
            try:
                self.send_response(code)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self._cors()
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                # Client disconnected before response was fully written.
                return

        def do_OPTIONS(self) -> None:
            self.send_response(204)
            self._cors()
            self.end_headers()

        def do_GET(self) -> None:
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                html_path = os.path.join(app.script_dir, "ptz_web_control.html")
                if not os.path.isfile(html_path):
                    self.send_response(404)
                    self.end_headers()
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
                self._json(app.get_status())
                return

            if path == "/log":
                if not os.path.isfile(app.isaac_log):
                    self.send_response(404)
                    self.end_headers()
                    return
                with open(app.isaac_log, "rb") as handle:
                    size = os.path.getsize(app.isaac_log)
                    handle.seek(max(0, size - 8192))
                    data = handle.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self._cors()
                self.end_headers()
                self.wfile.write(data)
                return

            if path == "/runtime/model":
                self._json(
                    {
                        "ok": True,
                        "model": app._snapshot_runtime_cfg(),
                        "isaac_state": app.get_status().get("isaac_state", "stopped"),
                        "restart_in_progress": app._is_restart_in_progress(),
                    }
                )
                return

            if path == "/stream.mjpeg":
                self._proxy_stream(app._loopback_url(app.isaac_port, "/stream.mjpeg"))
                return

            if path == "/snapshot.jpg":
                self._proxy_once(app._loopback_url(app.isaac_port, "/snapshot.jpg"))
                return

            if path == "/onvif-snap.jpg":
                self._proxy_once(app._loopback_url(app.isaac_port, "/snapshot.jpg"))
                return

            if path == "/scene/state":
                if app.get_local_state() != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_once(app._loopback_url(app.isaac_port, "/scene/state"))
                return

            if path == "/camera/tuning":
                if app.get_local_state() != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_once(app._loopback_url(app.isaac_port, "/camera/tuning"))
                return

            if path == "/ptz/selfcheck":
                if app.get_local_state() != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_once(app._loopback_url(app.isaac_port, "/ptz/selfcheck"))
                return

            if path == "/render/diag":
                if app.get_local_state() != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_once(app._loopback_url(app.isaac_port, "/render/diag"))
                return

            if path == "/render/pathtracing":
                if app.get_local_state() != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_once(app._loopback_url(app.isaac_port, "/render/pathtracing"))
                return

            self.send_response(404)
            self.end_headers()

        def do_POST(self) -> None:
            path = self.path.split("?")[0]

            if path == "/start":
                result = app.start_isaac()
                self._json(result, 200 if result.get("ok") else 400)
                return

            if path == "/stop":
                result = app.stop_isaac()
                self._json(result, 200 if result.get("ok") else 400)
                return

            if path == "/runtime/model":
                try:
                    length = int(self.headers.get("Content-Length", 0))
                    raw = self.rfile.read(length) if length > 0 else b"{}"
                    req = json.loads(raw.decode("utf-8") or "{}")
                except Exception as exc:
                    self._json({"ok": False, "error": f"Invalid JSON: {exc}"}, 400)
                    return

                result = app.update_runtime_model(
                    scene_path=req.get("scene_path"),
                    camera_prim=req.get("camera_prim"),
                    renderer_mode=req.get("renderer_mode"),
                    path_tracing_spp=req.get("path_tracing_spp"),
                    restart_if_running=bool(req.get("restart_if_running", False)),
                )
                self._json(result, 200 if result.get("ok") else 400)
                return

            if path.startswith("/onvif/"):
                length = int(self.headers.get("Content-Length", 0))
                xml_body = self.rfile.read(length)
                host_port = self.headers.get("Host", f"localhost:{app.launcher_port}")

                if path == "/onvif/device_service":
                    response = app.onvif.handle_device_service(xml_body, host_port)
                elif path == "/onvif/media_service":
                    response = app.onvif.handle_media_service(xml_body, host_port)
                elif path == "/onvif/ptz_service":
                    response = app.onvif.handle_ptz_service(xml_body)
                else:
                    self.send_response(404)
                    self.end_headers()
                    return

                self.send_response(response.status_code)
                self.send_header("Content-Type", "application/soap+xml; charset=utf-8")
                self.send_header("Content-Length", str(len(response.body)))
                self._cors()
                self.end_headers()
                self.wfile.write(response.body)
                return

            if path in ("/control", "/scene/gondola", "/scene/workers", "/camera/tuning", "/render/pathtracing"):
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                state = app.get_local_state()
                if state != "running":
                    self._json({"ok": False, "error": "Isaac Sim is not ready"}, 503)
                    return
                self._proxy_post(path, body)
                return

            self.send_response(404)
            self.end_headers()

        def _proxy_stream(self, url: str) -> None:
            try:
                with urllib.request.urlopen(urllib.request.Request(url), timeout=5) as upstream:
                    content_type = upstream.headers.get(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=ptzframe",
                    )
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Cache-Control", "no-cache, no-store")
                    self.send_header("Connection", "close")
                    self._cors()
                    self.end_headers()
                    while True:
                        chunk = upstream.read(65536)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            except urllib.error.HTTPError as exc:
                body = exc.read()
                self.send_response(exc.code)
                self.send_header("Content-Type", exc.headers.get("Content-Type", "text/plain; charset=utf-8"))
                self.send_header("Content-Length", str(len(body)))
                self._cors()
                self.end_headers()
                self.wfile.write(body)
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, 503)

        def _proxy_post(self, path: str, body: bytes) -> None:
            request = urllib.request.Request(
                app._loopback_url(app.isaac_port, path),
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=3) as response:
                    data = response.read()
                    code = response.status
                    content_type = response.headers.get("Content-Type", "application/json")
            except urllib.error.HTTPError as exc:
                data = exc.read()
                code = exc.code
                content_type = exc.headers.get("Content-Type", "application/json")
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, 502)
                return

            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self._cors()
            self.end_headers()
            self.wfile.write(data)

        def _proxy_once(self, url: str) -> None:
            try:
                with urllib.request.urlopen(url, timeout=3) as response:
                    data = response.read()
                    code = response.status
                    content_type = response.headers.get("Content-Type", "application/octet-stream")
            except urllib.error.HTTPError as exc:
                data = exc.read()
                code = exc.code
                content_type = exc.headers.get("Content-Type", "application/octet-stream")
                if not data and (
                    "/camera/tuning" in url
                    or "/render/pathtracing" in url
                    or "application/json" in str(content_type).lower()
                ):
                    state = app.get_local_state()
                    if state != "running":
                        message = f"Isaac Sim is not ready (state={state})"
                    else:
                        message = f"Upstream API returned HTTP {code}"
                    data = json.dumps({"ok": False, "error": message}, ensure_ascii=False).encode("utf-8")
                    content_type = "application/json; charset=utf-8"
            except Exception as exc:
                detail = str(exc).strip() or repr(exc)
                self._json({"ok": False, "error": detail}, 503)
                return

            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store")
            self._cors()
            self.end_headers()
            self.wfile.write(data)

    return Handler


def build_arg_parser(script_dir: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PTZ Launcher - lightweight controller")
    parser.add_argument("--config", default=os.path.join(script_dir, "ptz_rtsp_config.yaml"))
    return parser


def main(argv: list[str] | None = None) -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    script_dir = os.path.dirname(script_dir)
    parser = build_arg_parser(script_dir)
    args, _ = parser.parse_known_args(argv)

    app = LauncherApp(config_path=os.path.abspath(args.config), script_dir=script_dir)
    server = ThreadingHTTPServer(("0.0.0.0", app.launcher_port), make_handler(app))

    print(f"[PTZ-Launcher] listening on http://localhost:{app.launcher_port}/")
    print(f"[PTZ-Launcher] internal Isaac control port: {app.isaac_port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[PTZ-Launcher] stopping...")
        app.stop_isaac()
        time.sleep(2)
