#!/usr/bin/env bash
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
compose=(docker compose --project-directory "$hardware_dir" --env-file "$hardware_dir/.env" -f "$hardware_dir/compose.gui.yaml")
case "${1:-status}" in
  pull) "${compose[@]}" pull ;;
  up) "${compose[@]}" up -d ;;
  status) "${compose[@]}" ps ;;
  logs) "${compose[@]}" logs --tail=100 ;;
  down) "${compose[@]}" stop rviz-web goal-preview ;;
  *) echo 'Usage: bash viewer.sh {pull|up|status|logs|down}' >&2; exit 2 ;;
esac
