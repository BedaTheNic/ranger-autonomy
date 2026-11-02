#!/usr/bin/env python3
"""Build the Jazzy hardware configuration from the existing navigation profiles.

Physical footprint, scan height slice and dynamics still need robot validation.
No EKF config is generated: the robot supplies /odom and its base transform.
"""
import argparse
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml


def convert(value):
    if isinstance(value, dict):
        result = {k: convert(v) for k, v in value.items()}
        if 'progress_checker_plugin' in result:
            result['progress_checker_plugins'] = [result.pop('progress_checker_plugin')]
        if 'use_sim_time' in result:
            result['use_sim_time'] = False
        return result
    if isinstance(value, list):
        return [convert(v) for v in value]
    if isinstance(value, str):
        if value == 'base_link':
            return 'ranger_base_link'
        if value == '/odometry/filtered':
            return '/odom'
        if value.startswith(('nav2_smac_planner/', 'nav2_behaviors/')):
            return value.replace('/', '::')
    return value


def prepare(source, destination):
    destination.mkdir(parents=True, exist_ok=True)
    for filename in ('navigation_common.yaml', 'navigation_holonomic.yaml',
                     'navigation_ackermann.yaml', 'navigation_adaptive.yaml',
                     'amcl_sim.yaml', 'slam_sim.yaml'):
        data = convert(yaml.safe_load((source / filename).read_text()))
        if filename == 'navigation_common.yaml':
            bt = data['bt_navigator']['ros__parameters']
            # Jazzy automatically registers the standard Nav2 BT libraries.
            bt['plugin_lib_names'] = ['ranger_path_cache_bt']
            bt['navigators'] = ['navigate_to_pose', 'navigate_through_poses']
            bt['navigate_to_pose'] = {'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'}
            bt['navigate_through_poses'] = {'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'}
            behavior = data['behavior_server']['ros__parameters']
            behavior['local_costmap_topic'] = behavior.pop('costmap_topic')
            behavior['local_footprint_topic'] = behavior.pop('footprint_topic')
            for name in ('controller_server', 'behavior_server'):
                data[name]['ros__parameters']['enable_stamped_cmd_vel'] = False
        # Jazzy's smoother can also explicitly select the Twist interface.
        if 'velocity_smoother' in data:
            data['velocity_smoother']['ros__parameters']['enable_stamped_cmd_vel'] = False
        target = {'amcl_sim.yaml': 'amcl.yaml', 'slam_sim.yaml': 'slam.yaml'}.get(filename, filename)
        (destination / target).write_text(
            '# Hardware seed: verify physical geometry and sensor/dynamic limits before driving.\n'
            + yaml.safe_dump(data, sort_keys=False))
    for filename in ('navigate_to_pose.xml', 'navigate_through_poses.xml'):
        tree = ET.parse(source / filename)
        tree.getroot().set('BTCPP_format', '4')
        tree.write(destination / filename, encoding='unicode')
    for path in source.glob('ranger_omni_lattice.*'):
        shutil.copy2(path, destination / path.name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    prepare(args.source, args.destination)
