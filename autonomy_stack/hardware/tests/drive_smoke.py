#!/usr/bin/env python3
"""Run the gateway against synthetic ROS nodes on an internal Docker network."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',default='ghcr.io/bedathenic/ranger-autonomy:jazzy-r3')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    hardware=Path(__file__).resolve().parents[1]
    output=(args.output or Path(tempfile.mkdtemp(prefix='ranger-drive-test-'))).resolve()
    output.mkdir(parents=True,exist_ok=True)
    common=dict(image=args.image,init=True,entrypoint=['/opt/autonomy/entrypoint.sh'],
        environment=dict(ROS_DOMAIN_ID='211',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                         FASTDDS_BUILTIN_TRANSPORTS='UDPv4',ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET',
                         HARDWARE_MODE='true',RANGER_LOG_ROOT='/logs',RANGER_DEPLOYMENT_ID='drive-test',
                         DRIVE_ARM_SECONDS='0.1'),
        networks=['isolated'],volumes=[str(output/'logs')+':/logs'],stop_signal='SIGINT',stop_grace_period='10s')
    spec=dict(name='ranger-drive-test-'+str(os.getpid()),networks={'isolated':{'internal':True}},services={
        'guard':dict(common,command=['python3','/tmp/drive_guard.py'],
            volumes=common['volumes']+[str(hardware/'drive_guard.py')+':/tmp/drive_guard.py:ro']),
        'fixture':dict(common,command=['python3','/tmp/drive_fixture.py'],
            volumes=common['volumes']+[str(hardware/'tests/drive_fixture.py')+':/tmp/drive_fixture.py:ro'])})
    config=output/'compose.json';config.write_text(json.dumps(spec,indent=2))
    compose=['docker','compose','-f',str(config)]
    try:
        with (output/'console.log').open('w') as stream:
            result=subprocess.run(compose+['up','--abort-on-container-exit','--exit-code-from','fixture'],
                stdout=stream,stderr=subprocess.STDOUT,timeout=55)
        if result.returncode:raise RuntimeError('Drive test failed: '+str(output/'console.log'))
        (output/'summary.json').write_text(json.dumps(dict(passed=True,image=args.image,
            network='internal Docker bridge',physical_robot=False),indent=2))
        print('PASS drive gateway:',output)
    finally:
        subprocess.run(compose+['down','--remove-orphans'],check=True,timeout=30,
                       stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT)


if __name__=='__main__':
    main()
