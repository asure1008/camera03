#!/usr/bin/env python3
import datetime as dt
import json
import subprocess
import time
import urllib.request
from pathlib import Path
import yaml
import os

CFG_PATH = Path("/home/uniubi/xuanyuan/camera05/camera03/ptz_config.yaml")
OUT_DIR  = Path("/home/uniubi/xuanyuan/camera05/camera03")

def write_cfg(cfg: dict) -> None:
    CFG_PATH.write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )

def grab_frame(tag: str, rtsp_url: str) -> Path:
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = OUT_DIR / f"_test_4diaolan_{tag}_{ts}.jpg"
    url = rtsp_url.replace("localhost", "127.0.0.1")
    cmd = [
        "ffmpeg", "-y", "-rtsp_transport", "tcp",
        "-i", url, "-frames:v", "1", "-q:v", "2", str(out),
    ]
    for _ in range(5):
        try:
            subprocess.run(cmd, check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=30)
            return out
        except Exception:
            time.sleep(1.5)
    raise RuntimeError("ffmpeg 抓帧失败")

def _http(url: str, method: str = "GET", payload: dict | None = None) -> dict:
    data, headers = None, {}
    if payload is not None:
        data = json.dumps(payload).encode()
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=25) as r:
        return json.loads(r.read().decode())

def get_status() -> dict:
    return _http("http://127.0.0.1:8081/status")

def main():
    print("\n" + "=" * 64)
    print("Phase 2.5D: 恢复 force_* 参数并验证")
    print("=" * 64)

    original_text = CFG_PATH.read_text(encoding="utf-8")
    base_cfg = yaml.safe_load(original_text) or {}
    rtsp_url = str(base_cfg["rtsp_url"])

    cfg = dict(base_cfg)
    cfg["scene_path"] = "/home/uniubi/xuanyuan/camera05/camera03/scene_4diaolan_ptz.usda"
    cfg["renderer"] = "PathTracing"
    
    # 恢复 force_*
    cfg["force_active_diaolan_path"] = "/World/Diaolan_Ver1_0_2026_01"
    cfg["force_workers_count"] = 1
    cfg["force_gondola_height"] = 24.98
    
    # 启用随机事件 (diaolan_camera_sampling_enabled)
    cfg["diaolan_camera_sampling_enabled"] = True
    
    write_cfg(cfg)

    tag = "phase25_restore_verify"
    
    cmd = ["/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3", "/home/uniubi/xuanyuan/camera05/camera03/ptz_stream.py", "--config", "ptz_config.yaml"]
    log_file = open(f"/home/uniubi/xuanyuan/camera05/camera03/_ptz_stream_{tag}.log", "w")
    proc = subprocess.Popen(cmd, stdout=log_file, stderr=subprocess.STDOUT, cwd="/home/uniubi/xuanyuan/camera05/camera03")
    
    deadline = time.time() + 150
    ready = False
    while time.time() < deadline:
        if proc.poll() is not None:
            print(f"ptz_stream.py exited prematurely with code {proc.returncode}")
            break
        try:
            st = get_status()
            stream = st.get("stream") or {}
            if (
                stream.get("camera_bound_ok") is True
                and stream.get("ffmpeg_alive") is True
            ):
                ready = True
                break
        except Exception:
            pass
        time.sleep(1)
    
    if not ready:
        proc.terminate()
        raise RuntimeError(f"Stream not ready after 150s for {tag}")
        
    time.sleep(3.0)
    
    frame_path = grab_frame(tag, rtsp_url)
    print(f"抓取 RTSP 帧成功: {frame_path}")
    
    # Read the log to find scan_diaolan_prims output
    log_file.close()
    with open(f"/home/uniubi/xuanyuan/camera05/camera03/_ptz_stream_{tag}.log", "r") as f:
        log_content = f.read()
        
    for line in log_content.splitlines():
        if "[diaolan-scan]" in line or "[diaolan-active]" in line or "[diaolan-workers]" in line:
            print(line)
            
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        
    write_cfg(yaml.safe_load(original_text) or {})

if __name__ == "__main__":
    main()
