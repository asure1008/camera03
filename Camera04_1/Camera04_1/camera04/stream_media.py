from __future__ import annotations

import os
import shutil
import subprocess
import tarfile
import threading
import time
import urllib.request
import zipfile
from typing import Any

from .common import IS_WINDOWS, resolve_repo_path, tcp_port_in_use


MEDIAMTX_DOWNLOAD_URL_LINUX = (
    "https://github.com/bluenviron/mediamtx/releases/download/"
    "{version}/mediamtx_{version}_linux_amd64.tar.gz"
)
MEDIAMTX_DOWNLOAD_URL_WINDOWS = (
    "https://github.com/bluenviron/mediamtx/releases/download/"
    "{version}/mediamtx_{version}_windows_amd64.zip"
)


def resolve_mediamtx_path(script_dir: str, raw_path: str | None) -> str:
    path = resolve_repo_path(script_dir, raw_path or "./mediamtx") or ""
    if IS_WINDOWS and os.path.splitext(path)[1].lower() != ".exe":
        path += ".exe"
    return path


def is_mediamtx_ready(path: str) -> bool:
    if not os.path.isfile(path):
        return False
    if IS_WINDOWS:
        return True
    return os.access(path, os.X_OK)


def ensure_mediamtx(script_dir: str, mediamtx_cfg: dict[str, Any]) -> str:
    mediamtx_path = resolve_mediamtx_path(script_dir, mediamtx_cfg.get("path", "./mediamtx"))
    if is_mediamtx_ready(mediamtx_path):
        return mediamtx_path

    if not mediamtx_cfg.get("auto_download", True):
        raise FileNotFoundError(f"mediamtx binary not found: {mediamtx_path}")

    version = mediamtx_cfg.get("version", "v1.17.0")
    if IS_WINDOWS:
        url = MEDIAMTX_DOWNLOAD_URL_WINDOWS.format(version=version)
        package_path = mediamtx_path + ".zip"
    else:
        url = MEDIAMTX_DOWNLOAD_URL_LINUX.format(version=version)
        package_path = mediamtx_path + ".tar.gz"

    print(f"[PTZ-RTSP] downloading mediamtx {version} from {url}")
    urllib.request.urlretrieve(url, package_path)

    out_dir = os.path.dirname(mediamtx_path) or "."
    if IS_WINDOWS:
        with zipfile.ZipFile(package_path, "r") as archive:
            exe_member = None
            for name in archive.namelist():
                normalized = name.replace("\\", "/").lower()
                if normalized.endswith("/mediamtx.exe") or normalized == "mediamtx.exe":
                    exe_member = name
                    break
            if exe_member is None:
                raise RuntimeError("mediamtx.exe not found in downloaded package")
            archive.extract(exe_member, path=out_dir)
        extracted_path = os.path.join(out_dir, exe_member)
    else:
        with tarfile.open(package_path, "r:gz") as archive:
            archive.extract("mediamtx", path=out_dir)
        extracted_path = os.path.join(out_dir, "mediamtx")

    os.remove(package_path)
    if os.path.normcase(os.path.normpath(extracted_path)) != os.path.normcase(os.path.normpath(mediamtx_path)):
        os.replace(extracted_path, mediamtx_path)
    if not IS_WINDOWS:
        os.chmod(mediamtx_path, 0o755)
    return mediamtx_path


def start_mediamtx(script_dir: str, mediamtx_cfg: dict[str, Any], mediamtx_path: str) -> subprocess.Popen | None:
    port = int(mediamtx_cfg.get("port", 8554))
    if tcp_port_in_use(port):
        print(f"[PTZ-RTSP] reusing existing mediamtx on :{port}")
        return None

    cfg_file = os.path.join(script_dir, "mediamtx.yml")
    command = [mediamtx_path]
    if os.path.isfile(cfg_file):
        command.append(cfg_file)

    process = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    time.sleep(2.0)
    if process.poll() is not None:
        stderr = process.stderr.read().decode(errors="replace") if process.stderr else ""
        raise RuntimeError(f"mediamtx failed to start (exit {process.returncode}):\n{stderr}")
    return process


def resolve_ffmpeg_executable(ffmpeg_path: str | None = None) -> str:
    candidates: list[str] = []
    if ffmpeg_path:
        raw = str(ffmpeg_path).strip()
        if raw:
            candidates.append(raw)
            if IS_WINDOWS and not raw.lower().endswith(".exe"):
                candidates.append(raw + ".exe")

    # PATH lookup.
    candidates.append("ffmpeg")

    for candidate in candidates:
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
        resolved = shutil.which(candidate)
        if resolved:
            return resolved

    # Windows common install locations fallback.
    if IS_WINDOWS:
        common_paths = (
            r"C:\ffmpeg\bin\ffmpeg.exe",
            r"C:\Program Files\ffmpeg\bin\ffmpeg.exe",
            r"C:\Program Files (x86)\ffmpeg\bin\ffmpeg.exe",
            r"D:\ffmpeg\bin\ffmpeg.exe",
        )
        for path in common_paths:
            if os.path.isfile(path):
                return path

    hint = f" (configured path: {ffmpeg_path})" if ffmpeg_path else ""
    raise FileNotFoundError(f"ffmpeg executable not found{hint}")


def start_ffmpeg(
    rtsp_url: str,
    width: int,
    height: int,
    fps: int,
    bitrate: str,
    *,
    ffmpeg_path: str | None = None,
) -> subprocess.Popen:
    executable = resolve_ffmpeg_executable(ffmpeg_path)

    command = [
        executable,
        "-loglevel",
        "warning",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgba",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-g",
        str(max(1, int(fps))),
        "-keyint_min",
        str(max(1, int(fps))),
        "-sc_threshold",
        "0",
        "-b:v",
        bitrate,
        "-vf",
        "format=yuv420p",
        "-f",
        "rtsp",
        rtsp_url,
    ]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def _log_stderr() -> None:
        if process.stderr is None:
            return
        for line in process.stderr:
            text = line.decode(errors="replace").rstrip()
            if text:
                print(f"[ffmpeg] {text}")

    threading.Thread(target=_log_stderr, daemon=True, name="ffmpeg-stderr").start()

    time.sleep(0.5)
    if process.poll() is not None:
        raise RuntimeError("ffmpeg exited unexpectedly during startup")
    return process
