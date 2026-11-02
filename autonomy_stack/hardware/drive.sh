#!/usr/bin/env bash
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose --project-directory "$hardware_dir" --env-file "$hardware_dir/.env" -f "$hardware_dir/compose.drive.yaml")
case "${1:-status}" in
  up) "${compose[@]}" up -d ;;
  status) "${compose[@]}" ps ;;
  logs) "${compose[@]}" logs --tail=100 ;;
  arm|disarm)
    "${compose[@]}" exec -T drive-guard /opt/autonomy/entrypoint.sh \
      python3 /opt/autonomy/drive_guard.py "$1"
    ;;
  down) "${compose[@]}" down ;;
  *) echo 'Usage: bash drive.sh {up|status|logs|arm|disarm|down}' >&2; exit 2 ;;
esac
