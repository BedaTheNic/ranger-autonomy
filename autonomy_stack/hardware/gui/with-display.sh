#!/usr/bin/env bash
set -euo pipefail
for attempt in {1..100}; do
  if xdpyinfo -display "${DISPLAY:-:99}" >/dev/null 2>&1; then exec "$@"; fi
  sleep 0.1
done
echo 'The virtual display did not become ready' >&2
exit 1
