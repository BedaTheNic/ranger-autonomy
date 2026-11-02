#!/usr/bin/env bash
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$hardware_dir/.env" ]]; then
  cp -- "$hardware_dir/.env.example" "$hardware_dir/.env"
  echo "Created $hardware_dir/.env with the Ranger deployment defaults."
fi
compose=(docker compose --project-directory "$hardware_dir" --env-file "$hardware_dir/.env" -f "$hardware_dir/compose.yaml")
action="${1:-status}"
case "$action" in
  pull) "${compose[@]}" --profile '*' pull ;;
  mapping|localization|external)
    profile="$("${compose[@]}" config --format json | python3 -c 'import json,sys; print(json.load(sys.stdin)["services"]["controller"]["environment"]["NAV_PROFILE"])')"
    profiles=()
    [[ "$action" == external ]] || profiles+=(--profile "$action")
    [[ "$profile" != ackermann ]] || profiles+=(--profile ackermann)
    if [[ "${ENABLE_SCAN_ADAPTER:-0}" == 1 ]]; then profiles+=(--profile scan-adapter); fi
    export RANGER_DEPLOYMENT_ID="${RANGER_DEPLOYMENT_ID:-$(date +%Y%m%d_%H%M%S)_$RANDOM}"
    # Stop only localization services owned by this Compose project. Existing
    # robot drivers/SLAM in another project are never stopped by this script.
    "${compose[@]}" --profile mapping --profile localization stop \
      slam slam-lifecycle map-server amcl localization-lifecycle
    "${compose[@]}" "${profiles[@]}" up -d --remove-orphans
    ;;
  check) "${compose[@]}" --profile tools run --rm --no-deps preflight ;;
  down) "${compose[@]}" --profile '*' down ;;
  status) "${compose[@]}" --profile '*' ps ;;
  logs) shift; "${compose[@]}" --profile '*' logs --tail=100 "$@" ;;
  ros) shift; service="${1:?Specify service, then ROS command}"; shift
    "${compose[@]}" exec "$service" /opt/autonomy/entrypoint.sh "$@" ;;
  *) echo 'Usage: bash robot.sh {pull|mapping|localization|external|check|down|status|logs [service]|ros service command...}' >&2; exit 2 ;;
esac
