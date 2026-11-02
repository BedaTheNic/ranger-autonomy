#!/usr/bin/env python3
"""Verify RViz/noVNC and goal preview across three isolated containers."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',default='ranger-autonomy:gui-test')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--keep',action='store_true',help='Keep local test containers for visual inspection')
    args=parser.parse_args()
    hardware=Path(__file__).resolve().parents[1]
    output=(args.output or Path(tempfile.mkdtemp(prefix='ranger-gui-test-'))).resolve()
    output.mkdir(parents=True,exist_ok=True)
    environment=dict(ROS_DOMAIN_ID='211',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        FASTDDS_BUILTIN_TRANSPORTS='UDPv4',ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET',
        HARDWARE_MODE='true',USE_SIM_TIME='false',RANGER_LOG_ROOT='/logs',
        RANGER_DEPLOYMENT_ID='gui-test')
    common=dict(image=args.image,init=True,environment=environment,networks=['isolated'],
                volumes=[str(output/'logs')+':/logs'],stop_grace_period='25s')
    spec=dict(name='ranger-gui-test-'+str(os.getpid()),networks={'isolated':{'internal':True}},services={
        'rviz-web':dict(common,shm_size='256m'),
        'goal-preview':dict(common,entrypoint=['/opt/autonomy/entrypoint.sh'],
            command=['python3','/opt/autonomy/gui/plan_preview.py'],
            healthcheck={'disable':True},stop_signal='SIGINT'),
        'fixture':dict(common,entrypoint=['/opt/autonomy/entrypoint.sh'],
            command=['python3','/tmp/gui_fixture.py'],healthcheck={'disable':True},
            volumes=common['volumes']+[str(hardware/'tests/gui_fixture.py')+':/tmp/gui_fixture.py:ro'])})
    config=output/'compose.json';config.write_text(json.dumps(spec,indent=2))
    compose=['docker','compose','-f',str(config)]
    try:
        subprocess.run(compose+['up','-d','rviz-web','goal-preview'],check=True)
        deadline=time.monotonic()+65
        while True:
            result=subprocess.run(compose+['exec','-T','rviz-web','python3','/opt/autonomy/gui/health.py'],
                                  capture_output=True,text=True)
            if result.returncode==0:break
            if time.monotonic()>deadline:raise RuntimeError('GUI not healthy: '+result.stderr)
            time.sleep(1)
        result=subprocess.run(compose+['run','--rm','--no-deps','fixture'],
                              capture_output=True,text=True,timeout=50)
        (output/'fixture.log').write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('Preview fixture failed: '+result.stdout+result.stderr)
        print(result.stdout)
        subprocess.run(compose+['exec','-T','rviz-web','python3','-'],
                       input=(hardware/'tests/capture_rfb.py').read_text(),text=True,check=True,timeout=20)
        container=subprocess.check_output(compose+['ps','-q','rviz-web'],text=True).strip()
        subprocess.run(['docker','cp',container+':/tmp/rviz.png',str(output/'rviz.png')],check=True)
        summary=dict(passed=True,image=args.image,rviz_and_novnc_healthy=True,
                     goal_preview=True,network='internal Docker bridge',physical_robot=False)
        (output/'summary.json').write_text(json.dumps(summary,indent=2))
        print('Validation artifacts:',output)
    finally:
        logs=subprocess.run(compose+['logs','--no-color'],capture_output=True,text=True)
        (output/'console.log').write_text(logs.stdout+logs.stderr)
        if not args.keep:
            subprocess.run(compose+['down','--remove-orphans'],check=True,timeout=45)


if __name__=='__main__':
    main()
