#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
import os
from pathlib import Path
import yaml
port=int(os.environ.get('RANGER_GUI_PORT','6080'))
if not 1024 <= port <= 65535:
    raise SystemExit('RANGER_GUI_PORT must be between 1024 and 65535')
data=yaml.safe_load(Path('/opt/autonomy/gui/ranger.rviz').read_text())
if os.environ.get('RANGER_GUI_NAVIGATION_TOOLS','false').lower() == 'true':
    data['Panels'].append({'Class':'nav2_rviz_plugins/Navigation 2','Name':'Navigation 2'})
    data['Visualization Manager']['Tools'].append({'Class':'nav2_rviz_plugins/GoalTool'})
Path('/tmp/ranger.rviz').write_text(yaml.safe_dump(data,sort_keys=False))
PY
exec /usr/bin/supervisord -c /opt/autonomy/gui/supervisord.conf
