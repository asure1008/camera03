#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/uniubi/xuanyuan/camera05/camera03"
CFG_FILE="${ROOT_DIR}/ptz_config.yaml"

# 参数名：异常样本路径（可传第1个参数覆盖）
abnormal_scene_path_default="/home/uniubi/xuanyuan/camera05/camera03/_tmp_runs/v1_hotfix_height_world_10/usd/scene_generated_dataset_dataset_single_airborne_0001.usd"
abnormal_scene_path="${1:-${abnormal_scene_path_default}}"

if [[ ! -f "${abnormal_scene_path}" ]]; then
  echo "[start_abnormal_web] 异常样本不存在: ${abnormal_scene_path}"
  exit 1
fi

if [[ ! -f "${CFG_FILE}" ]]; then
  echo "[start_abnormal_web] 配置文件不存在: ${CFG_FILE}"
  exit 1
fi

echo "[start_abnormal_web] 目标异常样本: ${abnormal_scene_path}"

# 只替换 scene_path 行；保留其余配置
python3 - <<'PY' "${CFG_FILE}" "${abnormal_scene_path}"
import re
import sys

cfg_path = sys.argv[1]
scene = sys.argv[2]

with open(cfg_path, "r", encoding="utf-8") as f:
    text = f.read()

new_text, n = re.subn(r"(?m)^scene_path:\s*.*$", f"scene_path: {scene}", text)
if n == 0:
    raise SystemExit("[start_abnormal_web] 未找到 scene_path 配置行")

with open(cfg_path, "w", encoding="utf-8") as f:
    f.write(new_text)

print("[start_abnormal_web] 已更新 scene_path")
PY

cd "${ROOT_DIR}"
echo "[start_abnormal_web] 启动 Web 控制台 + 推流..."
./start_safe.sh 6 180

echo "[start_abnormal_web] 验证当前加载的 USD / 相机（Isaac 须已 running）："
echo "  curl -s http://127.0.0.1:8080/diagnostics | head -c 2000"
echo "  或浏览器打开 http://127.0.0.1:8080/diagnostics"
echo "  对照字段: configured_for_next_start.scene_basename 与 isaac_diagnostics.stream.scene_basename"
