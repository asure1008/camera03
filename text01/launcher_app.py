from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .common import (
    CREATE_NEW_PROCESS_GROUP,
    IS_WINDOWS,
    default_python_launcher,
    load_yaml_config,
    resolve_python_launcher,
    resolve_repo_path,
    tcp_port_free,
)
from .launcher_onvif import OnvifService


class LauncherApp:
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

        self._proc_lock = threading.Lock()
        self._isaac_proc: subprocess.Popen | None = None
        self._start_time: float | None = None
        self._isaac_state = "stopped"
        self._startup_error: str | None = None

        self._restart_lock = threading.Lock()
        self._restart_in_progress = False

        self.onvif = OnvifService(self.isaac_port)

    def _resolve_scene_path(self, scene_path: str | None) -> str | None:
        return resolve_repo_path(self.script_dir, scene_path)

    def _snapshot_runtime_cfg(self) -> dict[str, str | None]:
        with self._runtime_cfg_lock:
            scene_path = self._runtime_scene_path
            camera_prim = self._runtime_camera_prim
        return {
            "scene_path": scene_path,
            "scene_path_abs": self._resolve_scene_path(scene_path),
            "camera_prim": camera_prim,
        }

    def _validate_runtime_cfg(self, scene_path: str | None = None, camera_prim: str | None = None) -> list[str]:
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
        return errors

    def _fetch_isaac_status(self) -> dict[str, object] | None:
        try:
            with urllib.request.urlopen(
                f"http://localhost:{self.isaac_port}/status",
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

    def _watch_isaac(self, proc: subprocess.Popen) -> None:
        timeout_s = 300
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if proc.poll() is not None:
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        self._isaac_proc = None
                        self._isaac_state = "stopped"
                return
            if self._is_isaac_http_ready():
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        self._isaac_state = "running"
                        self._startup_error = None
                break
            time.sleep(1)
        else:
            with self._proc_lock:
                if self._isaac_proc is proc and proc.poll() is None:
                    self._isaac_state = "starting"
                    self._startup_error = f"Isaac HTTP did not become ready within {timeout_s} seconds"

        while True:
            if proc.poll() is not None:
                with self._proc_lock:
                    if self._isaac_proc is proc:
                        self._isaac_proc = None
                        self._isaac_state = "stopped"
                return
            time.sleep(2)

    def start_isaac(self) -> dict[str, object]:
        with self._proc_lock:
            if self._isaac_proc is not None and self._isaac_proc.poll() is None:
                return {"ok": False, "error": "Isaac Sim is already running"}

        runtime_cfg = self._snapshot_runtime_cfg()
        scene_path = runtime_cfg.get("scene_path")
        camera_prim = runtime_cfg.get("camera_prim")
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
            if (not alive) and state == "stopped":
                return True
            time.sleep(0.3)
        return False

    def _schedule_restart(self, reason: str = "runtime config updated") -> dict[str, object]:
        with self._restart_lock:
            if self._restart_in_progress:
                return {"ok": False, "error": "Restart already in progress"}
            self._restart_in_progress = True

        def _job() -> None:
            try:
                print(f"[PTZ-Launcher] restarting Isaac Sim ({reason})")
                if self._is_isaac_alive():
                    self.stop_isaac()
                    self._wait_stopped(timeout_s=60.0)
                result = self.start_isaac()
                if not result.get("ok", False):
                    print(f"[PTZ-Launcher] restart failed: {result}")
            finally:
                with self._restart_lock:
                    self._restart_in_progress = False

        threading.Thread(target=_job, daemon=True, name="isaac-restart").start()
        return {"ok": True}

    def _is_restart_in_progress(self) -> bool:
        with self._restart_lock:
            return self._restart_in_progress

    def stop_isaac(self) -> dict[str, object]:
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

        if dead and state not in ("stopped", "stopping"):
            with self._proc_lock:
                self._isaac_proc = None
                self._isaac_state = "stopped"
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
                if state == "starting" and bool(isaac_status.get("ready", False)):
                    result["isaac_state"] = "running"
                    with self._proc_lock:
                        self._isaac_state = "running"
                        self._startup_error = None
        return result

    def update_runtime_model(
        self,
        *,
        scene_path: str | None = None,
        camera_prim: str | None = None,
        restart_if_running: bool = False,
    ) -> dict[str, object]:
        if scene_path is None and camera_prim is None:
            return {"ok": False, "error": "scene_path or camera_prim is required"}

        errors = self._validate_runtime_cfg(scene_path=scene_path, camera_prim=camera_prim)
        if errors:
            return {"ok": False, "error": "; ".join(errors)}

        old_cfg = self._snapshot_runtime_cfg()
        with self._runtime_cfg_lock:
            if scene_path is not None:
                self._runtime_scene_path = scene_path.strip()
            if camera_prim is not None:
                self._runtime_camera_prim = camera_prim.strip()
        new_cfg = self._snapshot_runtime_cfg()

        state_now = str(self.get_status().get("isaac_state", "stopped"))
        restart_scheduled = False
        if restart_if_running and state_now in ("running", "starting"):
            scheduled = self._schedule_restart("scene/camera changed")
            if not scheduled.get("ok", False):
                return {
                    "ok": False,
                    "error": scheduled.get("error", "Unable to schedule restart"),
                    "old": old_cfg,
                    "new": new_cfg,
                }
            restart_scheduled = True

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
                self._proxy_stream(f"http://localhost:{app.isaac_port}/stream.mjpeg")
                return

            if path == "/snapshot.jpg":
                self._proxy_once(f"http://localhost:{app.isaac_port}/snapshot.jpg")
                return

            if path == "/onvif-snap.jpg":
                self._proxy_once(f"http://localhost:{app.isaac_port}/snapshot.jpg")
                return

            if path == "/scene/state":
                self._proxy_once(f"http://localhost:{app.isaac_port}/scene/state")
                return

            if path == "/ptz/selfcheck":
                self._proxy_once(f"http://localhost:{app.isaac_port}/ptz/selfcheck")
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

            if path in ("/control", "/scene/gondola", "/scene/workers"):
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                state = app.get_status().get("isaac_state")
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
                f"http://localhost:{app.isaac_port}{path}",
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
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, 503)
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
