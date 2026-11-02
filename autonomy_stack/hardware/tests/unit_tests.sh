#!/usr/bin/env bash
# Runs the existing navigation and logging regressions, not a second controller.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /opt/autonomy_ws/install/setup.bash
python3 -m unittest discover -s /opt/autonomy_ws/src/ranger_logging/test -v
test_workspace="$(mktemp -d /tmp/ranger-regression-XXXXXX)"
mkdir -p "$test_workspace/src/ranger_mini/config"
cp -a /opt/autonomy_ws/src/ranger_nav_plugins "$test_workspace/src/"
cp /opt/autonomy/config/navigate_*.xml "$test_workspace/src/ranger_mini/config/"
cd "$test_workspace"
MAKEFLAGS="-j${BUILD_JOBS:-2}" colcon build --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ranger_nav_plugins --event-handlers console_direct+
colcon test-result --verbose
