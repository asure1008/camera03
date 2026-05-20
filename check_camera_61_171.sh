#!/usr/bin/env bash
set -euo pipefail

REMOTE="uniubi@192.168.61.171"
REMOTE_DIR="/home/uniubi/xuanyuan/Camera/camera03"
SSH_OPTS=()

if [[ "${1:-}" == "--accept-host-key" ]]; then
  SSH_OPTS+=(-o StrictHostKeyChecking=accept-new)
fi

SSH_CMD=(ssh)
if [[ -n "${SSHPASS:-}" ]]; then
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "[check-61.171] SSHPASS is set but sshpass is not installed" >&2
    exit 1
  fi
  SSH_CMD=(sshpass -e ssh)
fi

"${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "REMOTE_DIR='${REMOTE_DIR}' bash -s" <<'REMOTE_SCRIPT'
set -eu
echo "=== host ==="
hostname
whoami
date -Is
uname -a

echo
echo "=== disk ==="
df -h "$HOME" /tmp 2>/dev/null || df -h

echo
echo "=== target ==="
ls -ld "$REMOTE_DIR" 2>/dev/null || true
find "$REMOTE_DIR" -maxdepth 1 -mindepth 1 -printf "%M %s %TY-%Tm-%Td %TH:%TM %p\n" 2>/dev/null | sort | head -n 120 || true

echo
echo "=== tools ==="
command -v python3 || true
command -v rsync || true
command -v ffmpeg || true
command -v nvidia-smi || true
ls -l /home/uniubi/miniconda3/envs/env_isaaclab/bin/python3 2>/dev/null || true

echo
echo "=== gpu ==="
nvidia-smi --query-gpu=name,memory.total,memory.used,driver_version --format=csv,noheader 2>/dev/null || true

echo
echo "=== ports ==="
ss -ltnp 2>/dev/null | grep -E ":8080|:8081|:8554" || true

echo
echo "=== processes ==="
ps -ef | grep -E "ptz_launcher.py|ptz_stream.py|mediamtx|ffmpeg" | grep -v grep || true

echo
echo "=== target config ==="
if [[ -f "$REMOTE_DIR/ptz_config.yaml" ]]; then
  grep -En "python_sh|scene_path|dynamic_sky_preset_path|onvif_rtsp_host|rtsp_url|launcher_port|ctrl_port|rtsp_publish_transport" "$REMOTE_DIR/ptz_config.yaml" || true
else
  echo "missing $REMOTE_DIR/ptz_config.yaml"
fi
REMOTE_SCRIPT
