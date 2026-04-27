#!/usr/bin/env bash
# 在「可回滚」前提下运行 generate_scene_usd.py：
# - 仅新增/覆盖输出文件 scene_generated.usd；若已存在则先带时间戳备份。
# - 不修改输入 USD（INPUT）；回滚输出：rm scene_generated.usd 或从 .bak.* 恢复。
#
# 可选：建立 /home/uniubi/xuanyuan/scene.usd -> INPUT 的符号链接（默认关闭）：
#   CREATE_SCENE_SYMLINK=1 ./run_generate_scene_usd.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONDA_PREFIX="${CONDA_PREFIX:-/home/uniubi/miniconda3/envs/env_isaaclab}"
PY="${CONDA_PREFIX}/bin/python3"
INPUT="${INPUT:-$SCRIPT_DIR/V4.0.usd}"
OUTPUT="${OUTPUT:-$SCRIPT_DIR/scene_generated.usd}"
SEED="${SEED:-42}"

# 已确认参数：吊篮、相机固定；当前不使用 PERSON_PRIMS。
# 吊篮高度异常最终作用目标：/World/DiaoLan/Model/Group1
BASKET_PRIMS="${BASKET_PRIMS:-/World/DiaoLan/Model/Group1}"
PERSON_PRIMS="${PERSON_PRIMS:-}"
CAMERA_PRIM="${CAMERA_PRIM:-/World/CameraRig/CamTilt/Camera}"

# 相机随机范围（世界坐标，Z-up）：可按场景包围盒再调
CAMERA_REGION="${CAMERA_REGION:-20,80,-60,-10,5,55}"
CAMERA_LOOK_TARGET="${CAMERA_LOOK_TARGET:-10,70,-40,10,0,45}"
FORCE_AIR_TEST="${FORCE_AIR_TEST:-0}"

if [[ ! -f "$INPUT" ]]; then
  echo "错误：找不到输入 USD：$INPUT" >&2
  exit 1
fi

if [[ "${CREATE_SCENE_SYMLINK:-0}" == "1" ]]; then
  ln -sf "$INPUT" /home/uniubi/xuanyuan/scene.usd
  echo "已建立符号链接：/home/uniubi/xuanyuan/scene.usd -> $INPUT"
fi

if [[ -f "$OUTPUT" ]]; then
  BAK="${OUTPUT}.bak.$(date +%Y%m%d%H%M%S)"
  cp -a "$OUTPUT" "$BAK"
  echo "已备份已有输出：$BAK"
fi

CMD=(
  "$PY" "$SCRIPT_DIR/generate_scene_usd.py"
  --input "$INPUT"
  --output "$OUTPUT"
  --seed "$SEED"
  --basket-prims "$BASKET_PRIMS"
  --camera-prim "$CAMERA_PRIM"
  --camera-region "$CAMERA_REGION"
  --camera-look-target "$CAMERA_LOOK_TARGET"
)
if [[ "$FORCE_AIR_TEST" == "1" ]]; then
  # 临时验证模式：强制空中状态（人数>=1，Z 在 air 区间）
  CMD+=(--air-prob "1.0")
fi
if [[ -n "$PERSON_PRIMS" ]]; then
  CMD+=(--person-prims "$PERSON_PRIMS")
fi

echo "执行：${CMD[*]}"
"${CMD[@]}"
echo "完成。输出：$OUTPUT"
