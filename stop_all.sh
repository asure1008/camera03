#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

collect_pids() {
    {
        pgrep -u "${USER}" -f "${ROOT_DIR}/ptz_launcher.py|ptz_launcher.py --config ${ROOT_DIR}/ptz_config.yaml" 2>/dev/null || true
        pgrep -u "${USER}" -f "${ROOT_DIR}/ptz_stream.py|ptz_stream.py --config ${ROOT_DIR}/ptz_config.yaml" 2>/dev/null || true
        pgrep -u "${USER}" -f "${ROOT_DIR}/mediamtx .*${ROOT_DIR}/mediamtx.yml|${ROOT_DIR}/./mediamtx ${ROOT_DIR}/mediamtx.yml" 2>/dev/null || true
        pgrep -u "${USER}" -f "ffmpeg .*rtsp://localhost:8554/ptz_cam" 2>/dev/null || true
    } | awk -v self="$$" 'NF && $1 != self {print $1}' | sort -u
}

wait_gone() {
    local timeout="$1"
    local start_ts now
    start_ts="$(date +%s)"
    while true; do
        if [[ -z "$(collect_pids)" ]]; then
            return 0
        fi
        now="$(date +%s)"
        if (( now - start_ts >= timeout )); then
            return 1
        fi
        sleep 0.5
    done
}

# 熔断：先正常退出；Isaac/Omniverse 偶尔清理很慢，超时后强杀，避免旧进程继续占 8081。
mapfile -t pids < <(collect_pids)
if (( ${#pids[@]} > 0 )); then
    echo "[stop_all] TERM pids: ${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true
    if ! wait_gone 12; then
        mapfile -t pids < <(collect_pids)
        if (( ${#pids[@]} > 0 )); then
            echo "[stop_all] KILL pids: ${pids[*]}"
            kill -KILL "${pids[@]}" 2>/dev/null || true
            wait_gone 5 || true
        fi
    fi
fi

# 清理 Isaac Sim / Carbonite 遗留的 POSIX 共享内存，
# 防止重启时出现 NamedSemaphore Permission denied (errno=13) 崩溃。
if [[ -d /dev/shm ]]; then
    find /dev/shm -maxdepth 1 \( -name 'carb_*' -o -name 'sem.carb*' -o -name 'sem.carbonite*' -o -name 'omni_*' -o -name 'isaac_*' -o -name 'nv_*' \) \
        -delete 2>/dev/null || true
fi

echo "[stop_all] 已执行熔断停止。"
