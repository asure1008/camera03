#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/uniubi/xuanyuan/camera05/camera03"
PY_BIN="/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3"
CFG_FILE="${ROOT_DIR}/ptz_config.yaml"
LAUNCH_LOG="/tmp/camera05_ptz_launcher.log"
STREAM_LOG="${ROOT_DIR}/isaac_stream.log"

startup_wait_sec="${1:-6}"
stream_wait_timeout_sec="${2:-150}"

cd "${ROOT_DIR}"

orientation_mode="$("${PY_BIN}" - <<'PY'
import yaml
from pathlib import Path

cfg = yaml.safe_load(Path("/home/uniubi/xuanyuan/camera05/camera03/ptz_config.yaml").read_text(encoding="utf-8")) or {}
print(str(cfg.get("camera_orientation_mode", "legacy")).strip().lower() or "legacy")
PY
)"

echo "[start_safe] stop existing processes"
"${ROOT_DIR}/stop_all.sh"

echo "[start_safe] launch launcher"
nohup "${PY_BIN}" ptz_launcher.py --config "${CFG_FILE}" > "${LAUNCH_LOG}" 2>&1 &
sleep "${startup_wait_sec}"

echo "[start_safe] POST /start"
if ! env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
  curl --noproxy "*" -fsS -X POST http://127.0.0.1:8080/start >/tmp/camera05_start_resp.json 2>/tmp/camera05_start_err.log; then
  echo "[start_safe] /start failed; stop all"
  "${ROOT_DIR}/stop_all.sh"
  echo "--- curl stderr ---"
  cat /tmp/camera05_start_err.log || true
  exit 1
fi

echo "[start_safe] /start response:"
cat /tmp/camera05_start_resp.json || true
echo
echo "[start_safe] orientation_mode=${orientation_mode}"
echo "[start_safe] waiting for stream readiness timeout=${stream_wait_timeout_sec}s"

SECONDS=0
while (( SECONDS < stream_wait_timeout_sec )); do
  if [[ -f "${STREAM_LOG}" ]] && grep -Eq "开始推流|push_fps=[1-9]" "${STREAM_LOG}"; then
    echo "[start_safe] stream readiness log detected"
    sleep 2

    if [[ "${orientation_mode}" == "dynamic_lookat" ]]; then
      echo "[start_safe] startup_orientation=dynamic startup orientation"
      echo "[start_safe] skip preset 1 override because orientation_mode=dynamic_lookat"
    else
      echo "[start_safe] startup_orientation=legacy preset startup"
      env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
        curl --noproxy "*" -fsS -X POST http://127.0.0.1:8080/presets/1/goto \
        >/dev/null 2>&1 && echo "[start_safe] applied preset 1 for legacy startup" || true
    fi

    env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u all_proxy -u ALL_PROXY \
      curl --noproxy "*" -fsS http://127.0.0.1:8080/status >/tmp/camera05_status_after_start.json 2>/dev/null || true
    if [[ -f /tmp/camera05_status_after_start.json ]]; then
      "${PY_BIN}" - <<'PY'
import json
from pathlib import Path

path = Path("/tmp/camera05_status_after_start.json")
if path.exists():
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        ptz = data.get("ptz") or {}
        stream = ptz.get("stream") or {}
        orientation = stream.get("orientation") or {}
        print(
            "[start_safe] final_ptz="
            f"pan={ptz.get('pan')} tilt={ptz.get('tilt')} zoom={ptz.get('zoom')} "
            f"orientation_mode={orientation.get('mode')} "
            f"orientation_source={orientation.get('last_source')} "
            f"orientation_preset={orientation.get('last_preset_name')}"
        )
    except Exception as exc:
        print(f"[start_safe] WARN: failed to parse startup status: {exc}")
PY
    fi

    "${ROOT_DIR}/check_health.sh" || true
    exit 0
  fi
  sleep 2
done

echo "[start_safe] timeout waiting for stream readiness; stop all"
"${ROOT_DIR}/stop_all.sh"
echo "[start_safe] recent stream log:"
if [[ -f "${STREAM_LOG}" ]]; then
  tail -n 120 "${STREAM_LOG}" || true
else
  echo "missing ${STREAM_LOG}"
fi
exit 2
