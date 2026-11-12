#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export RANGER_BAG_UID="$(id -u)" RANGER_BAG_GID="$(id -g)"
compose=(docker compose --env-file .env -f compose.bag.yaml)

recorder_ids() {
  # An info command can have a temporary Compose container for the same service.
  "${compose[@]}" ps "$@" --format '{{.ID}} {{.Label "com.docker.compose.oneoff"}}' bag-recorder |
    awk '$2 == "False" { print $1 }'
}

case "${1:-help}" in
  start)
    mkdir -p bags
    exec 9>bags/.control.lock
    flock -x 9
    running="$(recorder_ids --status running)"
    if [[ -n "$running" ]]; then
      echo 'Recorder is already running. Use bash bag.sh status or bash bag.sh stop.' >&2
      exit 1
    fi
    "${compose[@]}" up -d --no-deps --force-recreate bag-recorder
    ;;
  stop)
    mkdir -p bags
    exec 9>bags/.control.lock
    flock -x 9
    recorder="$(recorder_ids --all)"
    if [[ -z "$recorder" ]]; then
      echo 'No recorder container. Use bash bag.sh start.' >&2
      exit 1
    fi
    docker stop "$recorder" >/dev/null
    docker logs --tail 15 "$recorder"
    [[ "$(docker inspect -f '{{.State.ExitCode}}' "$recorder")" == 0 ]]
    ;;
  status)
    "${compose[@]}" ps -a bag-recorder
    ;;
  logs)
    "${compose[@]}" logs --no-color bag-recorder
    ;;
  info)
    session="${2:-}"
    if [[ ! "$session" =~ ^ranger-[0-9]{8}T[0-9]{6}Z-[a-f0-9]{8}$ ]]; then
      echo 'Usage: bash bag.sh info ranger-YYYYMMDDTHHMMSSZ-xxxxxxxx' >&2
      exit 2
    fi
    "${compose[@]}" run --rm --no-deps bag-recorder ros2 bag info "/bags/$session"
    ;;
  help|-h|--help)
    echo 'Usage: bash bag.sh start|stop|status|logs|info SESSION'
    ;;
  *)
    echo 'Usage: bash bag.sh start|stop|status|logs|info SESSION' >&2
    exit 2
    ;;
esac
