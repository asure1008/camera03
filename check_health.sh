#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="${ROOT_DIR}/isaac_stream.log"

echo "=== 端口状态 ==="
ss -ltnp | awk '/:8080|:8081|:8554/ {print}' || true

echo
echo "=== 关键进程 ==="
ps -ef | awk '/ptz_launcher.py|ptz_stream.py|mediamtx|ffmpeg/ && !/awk/ {print}' || true

echo
echo "=== 日志关键字（最近 120 行） ==="
if [[ -f "${LOG_FILE}" ]]; then
  grep -En "Simulation App Startup Complete|相机绑定成功|开始推流|push_fps|收到退出信号|Traceback|Error" "${LOG_FILE}" | tail -n 120 || true
else
  echo "日志不存在: ${LOG_FILE}"
fi

echo
echo "=== 场景与相机校验 ==="
if [[ -f "${LOG_FILE}" ]]; then
  echo "[scene_basename]:"
  grep "scene_basename=" "${LOG_FILE}" | tail -1 || echo "  未找到"
  echo "[tilt-axis（应仅含 rotateX，无 rotateY）]:"
  grep "tilt-axis" "${LOG_FILE}" | tail -1 || echo "  未找到"
  echo "[gondola 路径]:"
  grep "gondola-init" "${LOG_FILE}" | tail -1 | grep -o "group1=[^ ]*" || echo "  未找到"
  echo "[最近 push_fps]:"
  grep "push_fps=" "${LOG_FILE}" | grep -v "push_fps=0" | tail -1 || echo "  未推流"
fi
