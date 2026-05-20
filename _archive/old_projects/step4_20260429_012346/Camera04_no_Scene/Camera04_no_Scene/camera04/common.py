from __future__ import annotations

import os
import socket
import subprocess
from typing import Any

import yaml


IS_WINDOWS = os.name == "nt"
CREATE_NEW_PROCESS_GROUP = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0x00000200)


def load_yaml_config(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML object in {path}: expected mapping")
    return data


def resolve_repo_path(script_dir: str, raw_path: str | None) -> str | None:
    if not raw_path:
        return raw_path
    if "://" in raw_path:
        return raw_path
    if os.path.isabs(raw_path):
        return raw_path
    return os.path.abspath(os.path.join(script_dir, raw_path))


def default_python_launcher() -> str:
    return "C:/IsaacSim/python.bat" if IS_WINDOWS else "/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh"


def resolve_python_launcher(raw_path: str | None) -> str:
    path = str(raw_path or "").strip()
    if not path:
        return path
    path = os.path.expandvars(os.path.expanduser(path))
    if not IS_WINDOWS:
        return path

    if os.path.isdir(path):
        for name in ("python.bat", "python.exe"):
            candidate = os.path.join(path, name)
            if os.path.isfile(candidate):
                return candidate

    lower = path.lower()
    if lower.endswith("python.sh"):
        candidate = path[: -len("python.sh")] + "python.bat"
        if os.path.isfile(candidate):
            return candidate
    if lower.endswith(".sh"):
        candidate = os.path.splitext(path)[0] + ".bat"
        if os.path.isfile(candidate):
            return candidate
    return path


def tcp_port_in_use(port: int, host: str = "127.0.0.1", timeout_s: float = 0.5) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        return sock.connect_ex((host, port)) == 0


def tcp_port_free(port: int, host: str = "127.0.0.1", timeout_s: float = 0.5) -> bool:
    return not tcp_port_in_use(port, host=host, timeout_s=timeout_s)


def kill_helpers_by_name(*names: str, force: bool = True) -> None:
    for name in names:
        try:
            if IS_WINDOWS:
                image = name if name.lower().endswith(".exe") else f"{name}.exe"
                cmd = ["taskkill", "/IM", image, "/T"]
                if force:
                    cmd.append("/F")
                subprocess.run(cmd, capture_output=True, timeout=3)
            else:
                sig = "-9" if force else "-15"
                subprocess.run(["pkill", sig, "-f", name], capture_output=True, timeout=3)
        except Exception:
            pass


def sanitize_windows_path_env() -> None:
    if not IS_WINDOWS:
        return

    path_value = os.environ.get("PATH", "")
    if not path_value:
        return

    kept_entries: list[str] = []
    seen: set[str] = set()
    for raw_entry in path_value.split(os.pathsep):
        entry = raw_entry.strip()
        if not entry:
            continue
        normalized = os.path.normcase(os.path.normpath(entry))
        if normalized in seen:
            continue
        seen.add(normalized)

        # WindowsApps frequently exists in PATH but cannot be passed to add_dll_directory.
        if normalized.endswith(os.path.normcase(os.path.normpath(r"Microsoft\WindowsApps"))):
            continue

        if hasattr(os, "add_dll_directory"):
            try:
                handle = os.add_dll_directory(entry)
                handle.close()
            except OSError:
                continue

        kept_entries.append(entry)

    os.environ["PATH"] = os.pathsep.join(kept_entries)


def normalize_renderer_mode(raw_mode: str | None) -> str:
    mode = str(raw_mode or "").strip().lower()
    if mode in ("pathtracing", "path_tracing", "pt", "path tracing"):
        return "PathTracing"
    if mode in (
        "raytracedlighting",
        "ray_traced_lighting",
        "realtimepathtracing",
        "real_time_path_tracing",
        "real-time path tracing",
        "real-time2",
        "rt2",
        "rtl",
        "ray traced lighting",
        "rtxrealtime",
        "rtx_realtime",
        "rtx-realtime",
        "rtx realtime",
        "rtx real-time",
        "real-time",
        "realtime",
        "raytracing",
    ):
        return "RTXRealTime"
    raise ValueError(
        "renderer_mode must be PathTracing or RTXRealTime"
    )


def simulation_renderer_name(renderer_mode: str | None) -> str:
    mode = normalize_renderer_mode(renderer_mode)
    if mode == "RTXRealTime":
        # Isaac Sim 5.1 SimulationApp still expects RaytracedLighting as
        # the real-time renderer token.
        return "RaytracedLighting"
    return mode
