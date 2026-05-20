#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REMOTE=""
REMOTE_DIR=""
APPLY=0
DELETE=0
INCLUDE_CONFIG=0
INCLUDE_ALL_ASSETS=0
RESTART=0
SSH_OPTS=()
SSH_CMD=(ssh)
RSYNC_RSH=(ssh)

usage() {
  cat <<'EOF'
Usage:
  ./publish_to_server.sh --host user@server --dir /remote/path/camera03 [options]

Options:
  --apply             Execute rsync. Default is dry-run.
  --delete            Delete remote files that are not in the selected publish set.
  --include-config    Also publish ptz_config.yaml. The local project root is rewritten to the target dir.
  --all-assets        Include all top-level *.usd/*.usda/*.usdz plus unpacked assets.
  --restart           After --apply, chmod scripts and restart the remote service.
  --accept-host-key   Use StrictHostKeyChecking=accept-new.
  -h, --help          Show this help.

Examples:
  ./publish_to_server.sh --host uniubi@192.168.46.200 --dir /home/uniubi/xuanyuan/camera05/camera03
  ./publish_to_server.sh --host uniubi@192.168.46.200 --dir /home/uniubi/xuanyuan/camera05/camera03 --apply
  ./publish_to_server.sh --host uniubi@192.168.46.200 --dir /home/uniubi/xuanyuan/camera05/camera03 --apply --include-config

Notes:
  - Default mode is dry-run and prints what would change.
  - By default ptz_config.yaml is not overwritten on the target machine.
  - With --include-config, the old target config is backed up and paths from this project root are rewritten.
  - If SSHPASS is set and sshpass is installed, the script uses it for ssh/rsync authentication.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      REMOTE="${2:-}"
      shift 2
      ;;
    --dir)
      REMOTE_DIR="${2:-}"
      shift 2
      ;;
    --apply)
      APPLY=1
      shift
      ;;
    --delete)
      DELETE=1
      shift
      ;;
    --include-config)
      INCLUDE_CONFIG=1
      shift
      ;;
    --all-assets)
      INCLUDE_ALL_ASSETS=1
      shift
      ;;
    --restart)
      RESTART=1
      shift
      ;;
    --accept-host-key)
      SSH_OPTS+=(-o StrictHostKeyChecking=accept-new)
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[publish] Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${REMOTE}" || -z "${REMOTE_DIR}" ]]; then
  usage >&2
  exit 2
fi

if ! command -v rsync >/dev/null 2>&1; then
  echo "[publish] rsync is required" >&2
  exit 1
fi
if [[ -n "${SSHPASS:-}" ]]; then
  if ! command -v sshpass >/dev/null 2>&1; then
    echo "[publish] SSHPASS is set but sshpass is not installed" >&2
    exit 1
  fi
  SSH_CMD=(sshpass -e ssh)
  RSYNC_RSH=(sshpass -e ssh)
fi

RSYNC_ARGS=(
  -az
  --info=stats2,progress2,name
  --human-readable
  --partial
  --protect-args
  -e "$(printf '%q ' "${RSYNC_RSH[@]}" "${SSH_OPTS[@]}")"
)

if [[ "${APPLY}" -eq 0 ]]; then
  RSYNC_ARGS+=(--dry-run)
fi
if [[ "${DELETE}" -eq 1 ]]; then
  RSYNC_ARGS+=(--delete)
fi

INCLUDES=(
  "/ptz_launcher.py"
  "/ptz_stream.py"
  "/diaolan_randomizer.py"
  "/scene_perception.py"
  "/ptz_web_control.html"
  "/mediamtx.yml"
  "/mediamtx"
  "/DEPLOY.md"
  "/README.md"
  "/API.md"
  "/start_safe.sh"
  "/stop_all.sh"
  "/check_health.sh"
  "/rollback_safe.sh"
  "/publish_to_server.sh"
  "/publish_camera_61_171.sh"
  "/check_camera_61_171.sh"
  "/tls_tcp_proxy.py"
  "/test_onvif.py"
  "/scene_4diaolan_ptz.usda"
  "/GaochuzuoyeDiaolan_05.09.usd"
  "/changjing/***"
  "/textures/***"
  "/dynamic_sky_pkg/***"
)

if [[ "${INCLUDE_CONFIG}" -eq 1 ]]; then
  INCLUDES+=("/ptz_config.yaml")
fi

if [[ "${INCLUDE_ALL_ASSETS}" -eq 1 ]]; then
  INCLUDES+=(
    "/*.usd"
    "/*.usda"
    "/*.usdz"
    "/DiaoLan_ChangJing_2026.03.18_unpacked/***"
  )
fi

FILTERS=()
for item in "${INCLUDES[@]}"; do
  FILTERS+=(--include="${item}")
done
FILTERS+=(--exclude="*")

echo "[publish] source=${ROOT_DIR}/"
echo "[publish] target=${REMOTE}:${REMOTE_DIR}/"
if [[ "${APPLY}" -eq 0 ]]; then
  echo "[publish] mode=dry-run; add --apply to execute"
else
  echo "[publish] mode=apply"
fi
if [[ "${INCLUDE_CONFIG}" -eq 0 ]]; then
  echo "[publish] ptz_config.yaml is skipped by default; use --include-config to overwrite it"
else
  echo "[publish] ptz_config.yaml will be published; ${ROOT_DIR} paths will be rewritten to ${REMOTE_DIR}"
fi

"${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "mkdir -p '${REMOTE_DIR}'"
if [[ "${APPLY}" -eq 1 && "${INCLUDE_CONFIG}" -eq 1 ]]; then
  "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "cd '${REMOTE_DIR}' && if [ -f ptz_config.yaml ]; then cp -p ptz_config.yaml ptz_config.yaml.remote_bak_\$(date +%Y%m%d_%H%M%S); fi"
fi
rsync "${RSYNC_ARGS[@]}" "${FILTERS[@]}" "${ROOT_DIR}/" "${REMOTE}:${REMOTE_DIR}/"

if [[ "${APPLY}" -eq 1 ]]; then
  if [[ "${INCLUDE_CONFIG}" -eq 1 ]]; then
    "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "REMOTE_DIR='${REMOTE_DIR}' SRC_ROOT='${ROOT_DIR}' python3 - <<'PY'
import os
from pathlib import Path

cfg = Path(os.environ['REMOTE_DIR']) / 'ptz_config.yaml'
src_root = os.environ['SRC_ROOT']
remote_dir = os.environ['REMOTE_DIR']
text = cfg.read_text(encoding='utf-8')
text = text.replace(src_root, remote_dir)
cfg.write_text(text, encoding='utf-8')
print(f'[publish] rewrote config paths in {cfg}')
PY"
  fi
  "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "cd '${REMOTE_DIR}' && chmod +x start_safe.sh stop_all.sh check_health.sh rollback_safe.sh mediamtx publish_to_server.sh publish_camera_61_171.sh check_camera_61_171.sh"
  if [[ "${RESTART}" -eq 1 ]]; then
    echo "[publish] restarting remote service"
    "${SSH_CMD[@]}" "${SSH_OPTS[@]}" "${REMOTE}" "cd '${REMOTE_DIR}' && ./stop_all.sh || true; cd '${REMOTE_DIR}' && ./start_safe.sh 8 240 && ./check_health.sh"
  fi
fi

cat <<EOF
[publish] done.

Target-side checklist:
  cd ${REMOTE_DIR}
  chmod +x start_safe.sh stop_all.sh check_health.sh rollback_safe.sh mediamtx
  check ptz_config.yaml target-specific values if needed:
    - python_sh
    - onvif_rtsp_host
  ./start_safe.sh 8 240
  ./check_health.sh
EOF
