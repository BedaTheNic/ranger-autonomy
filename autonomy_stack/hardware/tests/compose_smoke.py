#!/usr/bin/env python3
"""Test the real service commands on an internal Docker network with synthetic inputs."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', default='ranger-autonomy:ci')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--rmw', default='rmw_fastrtps_cpp',
                        choices=['rmw_cyclonedds_cpp', 'rmw_fastrtps_cpp'])
    args = parser.parse_args()
    hardware = Path(__file__).resolve().parents[1]
    workspace = hardware.parents[1]
    output = (args.output or Path(tempfile.mkdtemp(prefix='ranger-compose-test-'))).resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, AUTONOMY_IMAGE=args.image)
    raw = subprocess.check_output(['docker', 'compose', '--env-file', str(hardware/'.env.example'),
                                   '-f', str(hardware/'compose.yaml'), '--profile', 'scan-adapter',
                                   'config', '--format', 'json'], env=env)
    services = {}
    for name, service in json.loads(raw)['services'].items():
        environment = dict(service['environment'])
        environment.update(ROS_DOMAIN_ID='211', RMW_IMPLEMENTATION=args.rmw,
                           NAV_PROFILE='adaptive', OBSTACLE_SCAN_TOPIC='/autonomy/scan',
                           RANGER_DEPLOYMENT_ID='compose-test', RANGER_LOG_MAX_BYTES='65536')
        services[name] = dict(image=args.image, command=service['command'], init=True,
                             environment=environment, volumes=[str(output/'logs')+':/logs'],
                             networks=['isolated'], stop_signal='SIGINT', stop_grace_period='25s')
    services['fixture'] = dict(image=args.image, init=True,
        command=['python3', '/tmp/fixture.py', 'adaptive', '--external'],
        environment=dict(ROS_DOMAIN_ID='211', RMW_IMPLEMENTATION=args.rmw,
                         FASTDDS_BUILTIN_TRANSPORTS='UDPv4'),
        volumes=[str(hardware/'tests/image_smoke.py')+':/tmp/fixture.py:ro'], networks=['isolated'])
    project = 'ranger-test-'+str(os.getpid())
    spec = dict(name=project, services=services, networks={'isolated': {'internal': True}})
    config = output/'compose.json'
    config.write_text(json.dumps(spec, indent=2))
    compose = ['docker', 'compose', '-f', str(config)]
    try:
        with (output/'console.log').open('w') as stream:
            completed = subprocess.run(compose+['up', '--abort-on-container-exit', '--exit-code-from', 'fixture'],
                                       stdout=stream, stderr=subprocess.STDOUT, timeout=150)
        if completed.returncode:
            raise RuntimeError('Fixture failed; see '+str(output/'console.log'))
        raw = subprocess.check_output(compose+['ps', '-a', '--format', 'json']).decode().strip()
        rows = json.loads(raw) if raw.startswith('[') else [json.loads(line) for line in raw.splitlines()]
        statuses = {row['Service']: row['ExitCode'] for row in rows}
        assert set(statuses) == set(services), statuses
        assert all(code in (0, 130, 143) for code in statuses.values()), statuses
        sys.path.insert(0, str(workspace/'ugv_sim/ranger/ranger_logging'))
        from ranger_logging.reader import records
        files = list((output/'logs').rglob('*.jsonl'))
        assert files and all(path.stat().st_size <= 65536 for path in files)
        records_list = list(records(files))
        events = {row.get('event') for row in records_list}
        assert {'global_plan', 'grid_snapshot', 'pointcloud_metadata', 'health',
                'input_file', 'parameters_snapshot', 'process_exit'}.issubset(events), events
        assert not any(row.get('event') == 'forced_shutdown' for row in records_list)
        manifests = list((output/'logs').rglob('manifest.json'))
        assert len(manifests) == len(services)-1, len(manifests)
        assert all(json.loads(path.read_text()).get('status') == 'finished' for path in manifests)
        summary = dict(passed=True, image=args.image, service_exit_codes=statuses,
                       parts=len(files), records=len(records_list), max_part_bytes=65536,
                       events=sorted(events), ros_domain=211, network='internal Docker bridge',
                       rmw=args.rmw, fastdds_builtin_transports='UDPv4',
                       physical_robot=False)
        (output/'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        print('Validation artifacts:', output)
    finally:
        subprocess.run(compose+['down', '--remove-orphans'], timeout=60, check=True,
                       stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()
