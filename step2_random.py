import os
import time
import subprocess
import yaml
import psutil
import cv2
import numpy as np
import shutil

# 1. Backup
shutil.copy("ptz_config.yaml", "ptz_config.yaml.bak_stable_step1")

# 2. Modify config: Only enable diaolan_camera_sampling_enabled
with open("ptz_config.yaml", "r") as f:
    cfg = yaml.safe_load(f)

cfg["diaolan_camera_sampling_enabled"] = True

with open("ptz_config.yaml", "w") as f:
    yaml.dump(cfg, f)

# 3. Stop all
subprocess.run(["./stop_all.sh"], capture_output=True)
time.sleep(2)

# 4. Start
print("Starting...")
proc = subprocess.Popen(["./start_safe.sh"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

# 5. Wait for stream
success = False
img_path = "_step2_frame.jpg"
for i in range(150):
    time.sleep(1)
    ret = subprocess.run(["ffmpeg", "-y", "-rtsp_transport", "tcp", "-i", "rtsp://localhost:8554/ptz_cam", "-vframes", "1", "-q:v", "2", img_path], capture_output=True)
    if ret.returncode == 0 and os.path.exists(img_path):
        success = True
        break

# 6. Check ports and memory
ports_ok = True
for port in [8080, 8081, 8554]:
    ret = subprocess.run(f"netstat -tulpn 2>/dev/null | grep ':{port}'", shell=True, capture_output=True)
    if ret.returncode != 0:
        ports_ok = False

mem_mb = 0
for p in psutil.process_iter(['name', 'cmdline']):
    try:
        cmdline = " ".join(p.info['cmdline'] or [])
        if "ptz_stream.py" in cmdline and "step2" not in cmdline:
            mem_mb = p.memory_info().rss / (1024 * 1024)
            break
    except:
        pass

# 7. Check image
black_ratio = 1.0
if success:
    img = cv2.imread(img_path)
    if img is not None:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        black_ratio = np.sum(gray < 10) / gray.size

# 8. Stop
subprocess.run(["./stop_all.sh"], capture_output=True)

print(f"RESULT: success={success}, ports_ok={ports_ok}, mem_mb={mem_mb:.1f}, black_ratio={black_ratio:.4f}")
