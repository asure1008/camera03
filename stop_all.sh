#!/usr/bin/env bash
set -euo pipefail

# 熔断：立即停止所有相关进程，避免僵尸会话占端口。
pkill -f 'ptz_launcher.py|ptz_stream.py|mediamtx|ffmpeg' 2>/dev/null || true
sleep 1

# 清理 Isaac Sim / Carbonite 遗留的 POSIX 共享内存，
# 防止重启时出现 NamedSemaphore Permission denied (errno=13) 崩溃。
if [[ -d /dev/shm ]]; then
    find /dev/shm -maxdepth 1 \( -name 'carb_*' -o -name 'sem.carb*' -o -name 'sem.carbonite*' -o -name 'omni_*' -o -name 'isaac_*' -o -name 'nv_*' \) \
        -delete 2>/dev/null || true
fi

echo "[stop_all] 已执行熔断停止。"
