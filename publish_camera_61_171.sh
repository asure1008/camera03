#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cat <<'EOF'
[publish-61.171] Target: uniubi@192.168.61.171:~/xuanyuan/Camera/camera03
[publish-61.171] Password is not stored in this script.
[publish-61.171] Usage:
  SSHPASS='***' ./publish_camera_61_171.sh                 # dry-run, no restart
  SSHPASS='***' ./publish_camera_61_171.sh --apply         # publish and restart
  SSHPASS='***' ./publish_camera_61_171.sh --apply --include-config
EOF

exec "${ROOT_DIR}/publish_to_server.sh" \
  --host "uniubi@192.168.61.171" \
  --dir "/home/uniubi/xuanyuan/Camera/camera03" \
  --restart \
  "$@"
