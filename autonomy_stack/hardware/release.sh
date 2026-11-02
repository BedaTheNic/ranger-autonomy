#!/usr/bin/env bash
# Build context is the workspace src containing autonomy_stack and ugv_sim.
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
stack_dir="$(dirname -- "$hardware_dir")"
context_dir="$(dirname -- "$stack_dir")"
usage() {
  cat <<'EOF'
Usage:
  bash release.sh build IMAGE [PLATFORM]       # local image; default native platform
  bash release.sh publish IMAGE [PLATFORMS] [EXTRA_IMAGE ...]
                                             # default linux/amd64 (robot architecture)
  bash release.sh bundle [OUTPUT_DIRECTORY]   # robot deployment files, no source/build
  bash release.sh source [OUTPUT_DIRECTORY]   # independent GitHub-ready source tree + archive

Examples:
  bash release.sh build ranger-autonomy:jazzy-r3 linux/amd64
  bash release.sh publish docker.io/USER/ranger-autonomy:jazzy-r3
  bash release.sh publish ghcr.io/ORG/ranger-autonomy:jazzy-r3 linux/amd64 registry.example.com/TEAM/ranger-autonomy:jazzy-r3

Authenticate to each registry with docker login before publish.
BUILD_JOBS controls compiler parallelism per platform (default 2).
The robot target is linux/amd64; no ARM image or QEMU is needed on this computer.
EOF
}
action="${1:-help}"
case "$action" in
  build|publish)
    [[ $# -ge 2 ]] || { usage; exit 2; }
    image="$2"
    if [[ "$action" == publish ]]; then
      platforms="${3:-linux/amd64}"
      output=(--push)
    else
      platforms="${3:-linux/amd64}"
      [[ "$platforms" != *,* ]] || { echo 'Use publish for a multi-platform manifest.' >&2; exit 2; }
      output=(--load)
    fi
    tags=(-t "$image")
    if [[ $# -gt 3 ]]; then
      for tag in "${@:4}"; do tags+=(-t "$tag"); done
    fi
    exec docker buildx build --platform "$platforms" "${output[@]}" "${tags[@]}" \
      --build-arg "BUILD_JOBS=${BUILD_JOBS:-2}" \
      -f "$stack_dir/docker/Dockerfile.jazzy" --target runtime "$context_dir"
    ;;
  bundle)
    destination="${2:-$hardware_dir/dist}"
    mkdir -p -- "$destination"
    destination="$(cd -- "$destination" && pwd)"
    stage="$(mktemp -d)"
    trap 'rm -rf -- "$stage"' EXIT
    mkdir -p "$stage/ranger-hardware/maps" "$stage/ranger-hardware/logs"
    cp -- "$hardware_dir/compose.yaml" "$hardware_dir/.env.example" \
      "$hardware_dir/README.md" "$hardware_dir/robot-contract.yaml" \
      "$hardware_dir/robot.sh" "$hardware_dir/inspect_drive.py" \
      "$hardware_dir/compose.gui.yaml" "$hardware_dir/viewer.sh" \
      "$hardware_dir/compose.drive.yaml" "$hardware_dir/drive.sh" \
      "$hardware_dir/drive_guard.py" "$hardware_dir/apply_update.sh" \
      "$hardware_dir/ackermann_check.sh" "$hardware_dir/ackermann_diagnostics.py" "$stage/ranger-hardware/"
    # Robot preset only; registry credentials remain in the host's Docker login.
    cp -- "$hardware_dir/.env.example" "$stage/ranger-hardware/.env"
    if [[ -n "${DEPLOY_IMAGE:-}" ]]; then
      python3 - "$stage/ranger-hardware/.env" "$DEPLOY_IMAGE" <<'PY'
from pathlib import Path
import sys
path = Path(sys.argv[1])
path.write_text('\n'.join('AUTONOMY_IMAGE='+sys.argv[2] if line.startswith('AUTONOMY_IMAGE=')
                          else line for line in path.read_text().splitlines())+'\n')
PY
    fi
    tar -czf "$destination/ranger-hardware-deploy.tar.gz" -C "$stage" ranger-hardware
    echo "$destination/ranger-hardware-deploy.tar.gz"
    ;;
  source)
    if [[ $# -ge 2 ]]; then
      exec python3 "$hardware_dir/export_source.py" --destination "$2"
    fi
    exec python3 "$hardware_dir/export_source.py"
    ;;
  help|-h|--help) usage ;;
  *) usage; exit 2 ;;
esac
