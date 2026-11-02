#!/usr/bin/env bash
set -euo pipefail
hardware_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
seconds="${1:-20}"
report_dir="$(mktemp -d "$hardware_dir/ackermann_report_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
compose=(docker compose --project-directory "$hardware_dir" --env-file "$hardware_dir/.env" -f "$hardware_dir/compose.yaml")
echo "Passively recording ROS and CAN feedback for $seconds seconds. Report: $report_dir"
"${compose[@]}" --profile tools run --rm --no-deps --name "ranger_hero_ack-check_$$" \
  -e RANGER_LOG_ENABLED=0 \
  -v "$hardware_dir/ackermann_diagnostics.py:/opt/autonomy/ackermann_diagnostics.py:ro" \
  preflight python3 /opt/autonomy/ackermann_diagnostics.py --seconds "$seconds" \
  > "$report_dir/feedback.json" 2> "$report_dir/feedback.stderr"
if docker inspect ranger_hero_jazzy >/dev/null 2>&1; then
  docker exec -i ranger_hero_jazzy python3 - --seconds 0.01 < "$hardware_dir/inspect_drive.py" \
    > "$report_dir/driver-source.json" 2> "$report_dir/driver-source.stderr" || true
fi
"${compose[@]}" logs --no-color --since 10m --tail 150 controller planner \
  > "$report_dir/navigation.log" 2>&1 || true
for service in controller planner; do
  "${compose[@]}" exec -T -e RANGER_LOG_ENABLED=0 "$service" /opt/autonomy/entrypoint.sh \
    timeout 10s ros2 param dump "/${service}_server" \
    > "$report_dir/$service-parameters.yaml" 2> "$report_dir/$service-parameters.stderr" || true
done
archive="$report_dir.tar.gz"
tar -czf "$archive" -C "$(dirname -- "$report_dir")" "$(basename -- "$report_dir")"
echo "Send this archive: $archive"
