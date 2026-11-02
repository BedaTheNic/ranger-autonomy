import os
import subprocess
import urllib.request

port=int(os.environ.get('RANGER_GUI_PORT','6080'))
with urllib.request.urlopen(f'http://127.0.0.1:{port}/vnc.html',timeout=2) as response:
    assert response.status==200
result=subprocess.run(['supervisorctl','-c','/opt/autonomy/gui/supervisord.conf','status'],
                      capture_output=True,text=True,timeout=2,check=True)
assert all('RUNNING' in line for line in result.stdout.splitlines()),result.stdout
