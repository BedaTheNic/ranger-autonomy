#!/usr/bin/env bash
set -e
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source /opt/autonomy_ws/install/setup.bash
if [ -f /opt/simulation_ws/install/setup.bash ]; then
  source /opt/simulation_ws/install/setup.bash
fi
if [[ "${HARDWARE_MODE:-false}" == true && "${RANGER_LOG_WRAPPED:-0}" != 1 ]]; then
  log_action="${RANGER_LOG_SERVICE:-command}"
  for arg in "$@"; do
    if [[ "$arg" == component:=* ]]; then log_action="${arg#component:=}"; fi
  done
  log_root="${RANGER_LOG_ROOT:-/logs}/${RANGER_DEPLOYMENT_ID:-manual}"
  if [[ "${RANGER_LOG_ENABLED:-1}" == 1 ]]; then
    # Run the signal-forwarding wrapper directly. An extra `ros2 run` parent
    # can consume Docker's PID-directed SIGINT without forwarding it to us.
    exec python3 -c 'from ranger_logging.runner import main; raise SystemExit(main())' \
      "$log_action" "$log_root" "$@"
  fi
fi
exec "$@"
