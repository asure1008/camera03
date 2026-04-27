#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/uniubi/xuanyuan/camera05/camera03"
CFG_BAK="${ROOT_DIR}/ptz_config.yaml.bak_safe_20260326094631"
SCENE_BAK="${ROOT_DIR}/scene_single_with_ptz.usda.bak_camfix_20260326095640"

echo "[rollback_safe] 先熔断..."
"${ROOT_DIR}/stop_all.sh"

if [[ -f "${CFG_BAK}" ]]; then
  cp -f "${CFG_BAK}" "${ROOT_DIR}/ptz_config.yaml"
  echo "[rollback_safe] 已回滚配置: ptz_config.yaml"
else
  echo "[rollback_safe] 未找到配置备份: ${CFG_BAK}"
fi

if [[ -f "${SCENE_BAK}" ]]; then
  cp -f "${SCENE_BAK}" "${ROOT_DIR}/scene_single_with_ptz.usda"
  echo "[rollback_safe] 已回滚场景: scene_single_with_ptz.usda"
else
  echo "[rollback_safe] 未找到场景备份: ${SCENE_BAK}"
fi

echo "[rollback_safe] 完成。"
