import os
import time
import subprocess
import yaml
import json
import cv2
import numpy as np
import shutil

# Prepare novis version
with open("ptz_stream.py.bak_4quadrant", "r") as f:
    code = f.read()
code_novis = code.replace("apply_diaolan_visibility(stage, active_diaolan, all_diaolans)", "# apply_diaolan_visibility(stage, active_diaolan, all_diaolans)")
with open("ptz_stream_novis.py", "w") as f:
    f.write(code_novis)

base_cfg = {
    "launcher_port": 8080,
    "ctrl_port": 8081,
    "python_sh": "/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3",
    "camera_prim": "/World/CameraRig/CamTilt/Camera",
    "rtsp_url": "rtsp://localhost:8554/ptz_cam",
    "resolution": [960, 540],
    "fps": 10,
    "bitrate": "1M",
    "sim_hz": 60,
    "rtsp_enabled": True,
    "preview_enabled": False,
    "mjpeg_quality": 80,
    "hydra_target_material_mode": "none",
    "hydra_c_branch_geometry_only": False,
    "osd_time": {"enabled": False},
    "focal_length_1x": 6,
    "diaolan_camera_sampling_enabled": False,
    "initial_pan": 0.0,
    "initial_tilt": -15.0,
    "initial_zoom": 1.5,
    "mediamtx": {"path": "./mediamtx", "port": 8554, "auto_download": True, "version": "v1.17.0"},
    "onvif_rtsp_host": "",
    "force_active_diaolan_path": "/World/DiaoLan_Ver1_0_2026_07"
}

c = {
    "name": "5_minimal_2diaolan",
    "scene_path": "/home/uniubi/xuanyuan/camera05/camera03/scene_2diaolan_test.usda",
    "renderer": "PathTracing",
    "vis_enabled": False,
    "camera_pos": [84.2, 10.87, 6.53]
}

def analyze_frame(img_path):
    img = cv2.imread(img_path)
    if img is None: return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    black_pixels = np.sum(gray < 10)
    total_pixels = gray.size
    black_ratio = black_pixels / total_pixels
    h, w = gray.shape
    cy, cx = h//2, w//2
    rh, rw = h//4, w//4
    center_roi = gray[cy-rh:cy+rh, cx-rw:cx+rw]
    std_dev = np.std(center_roi)
    return {"black_ratio": float(black_ratio), "std_dev": float(std_dev), "size": os.path.getsize(img_path)}

try:
    print(f"Running {c['name']}...")
    cfg = base_cfg.copy()
    cfg["scene_path"] = c["scene_path"]
    cfg["renderer"] = c["renderer"]
    cfg["camera_rig_translate_xyz"] = c["camera_pos"]
    
    with open("ptz_config.yaml", "w") as f:
        yaml.dump(cfg, f)
    
    shutil.copy("ptz_stream_novis.py", "ptz_stream.py")
        
    subprocess.run(["./stop_all.sh"], capture_output=True)
    
    log_file = f"_quadrant_{c['name']}.log"
    print(f"Starting {c['name']}...")
    proc = subprocess.run(["./start_safe.sh"], capture_output=True, text=True)
    
    if proc.returncode != 0:
        print(f"Failed to start {c['name']}:\n{proc.stdout}\n{proc.stderr}")
        exit(1)
        
    print(f"Capturing frame for {c['name']}...")
    img_path = f"_quadrant_{c['name']}.jpg"
    success = False
    for i in range(10):
        time.sleep(2)
        ret = subprocess.run(["ffmpeg", "-y", "-rtsp_transport", "tcp", "-i", "rtsp://localhost:8554/ptz_cam", "-vframes", "1", "-q:v", "2", img_path], capture_output=True)
        if ret.returncode == 0 and os.path.exists(img_path):
            success = True
            break
            
    subprocess.run(["./stop_all.sh"], capture_output=True)
    
    if success:
        stat = analyze_frame(img_path)
        stat["name"] = c["name"]
        stat["img_path"] = img_path
        stat["scene_path"] = c["scene_path"]
        stat["renderer"] = c["renderer"]
        stat["vis_enabled"] = c["vis_enabled"]
        print(f"Success: {stat}")
    else:
        print(f"Failed to capture {c['name']}")

finally:
    shutil.copy("ptz_config.yaml.bak_4quadrant", "ptz_config.yaml")
    shutil.copy("ptz_stream.py.bak_4quadrant", "ptz_stream.py")

