#!/usr/bin/env python3
"""Export an independent GitHub-ready build context, excluding private runtime data."""
import argparse
import datetime
import fnmatch
from pathlib import Path
import shutil
import tarfile


def export(workspace, destination):
    destination.mkdir(parents=True, exist_ok=False)
    ignored = shutil.ignore_patterns('.git', '.env', '__pycache__', '*.pyc',
                                     'logs', 'maps', 'dist', '.runtime', 'build', 'install', 'log',
                                     'ackermann_report_*', 'update_backups', '.update.*')
    packages = ('ugv_sim/ranger/ranger_nav_plugins', 'ugv_sim/ranger/four_ws_control',
                'ugv_sim/ranger/ranger_logging', 'ranger_ros2/ranger_msgs',
                'autonomy_stack/hardware', 'autonomy_stack/launch', 'autonomy_stack/config')
    for package in packages:
        shutil.copytree(workspace/package, destination/package, ignore=ignored)
    docker = destination/'autonomy_stack/docker'
    docker.mkdir(parents=True)
    for name in ('Dockerfile.jazzy', 'Dockerfile.jazzy.dockerignore', 'entrypoint.sh'):
        shutil.copy2(workspace/'autonomy_stack/docker'/name, docker/name)
    configs = destination/'ugv_sim/ranger/ranger_mini/config'
    configs.mkdir(parents=True)
    for path in (workspace/'ugv_sim/ranger/ranger_mini/config').iterdir():
        if any(fnmatch.fnmatch(path.name, pattern) for pattern in
               ('navigation_*.yaml', 'navigate_*.xml', 'ranger_omni_lattice.*',
                'amcl_sim.yaml', 'slam_sim.yaml')):
            shutil.copy2(path, configs/path.name)
    shutil.copy2(workspace/'ranger_ros2/LICENSE', destination/'ranger_ros2/LICENSE')
    docs = destination/'docs'
    docs.mkdir()
    source_docs = workspace/'ranger_navigation_startup'
    if not source_docs.is_dir():
        source_docs = workspace/'docs'
    for name in ('ADAPTIVE_NAVIGATION.md', 'LOGGING.md'):
        shutil.copy2(source_docs/name, docs/name)
    workflows = destination/'.github/workflows'
    workflows.mkdir(parents=True)
    for workflow in (workspace/'autonomy_stack/hardware/ci').glob('*.yaml'):
        shutil.copy2(workflow, workflows/workflow.name)
    (destination/'README.md').write_text(
        '# Ranger autonomy — Jazzy\n\n'
        'Adaptive navigation and structured logging, packaged as separate ROS containers.\n\n'
        'Build/publish: [release helper](autonomy_stack/hardware/release.sh).\n\n'
        'Deploy: [robot helper](autonomy_stack/hardware/robot.sh).\n\n'
        'Canonical navigation behavior: [details](docs/ADAPTIVE_NAVIGATION.md).\n\n'
        'Structured logging: [details](docs/LOGGING.md).\n')
    (destination/'.gitignore').write_text(
        '.env\n**/__pycache__/\n**/*.pyc\n**/logs/\n**/maps/\n**/dist/\n**/build/\n**/install/\n**/log/\n'
        '**/ackermann_report_*\n**/update_backups/\n**/.update.*/\n')
    archive = destination.with_suffix('.tar.gz')
    with tarfile.open(archive, 'w:gz') as stream:
        stream.add(destination, arcname='ranger-autonomy')
    print('GitHub-ready directory:', destination)
    print('Source archive:', archive)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    default = Path(__file__).resolve().parent/'dist'/('source_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    parser.add_argument('--destination', type=Path, default=default)
    args = parser.parse_args()
    export(Path(__file__).resolve().parents[2], args.destination.resolve())
