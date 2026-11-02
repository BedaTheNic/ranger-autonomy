"""Navigation processes split by ownership; supports simulation and hardware."""
import os
from pathlib import Path
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def rewrite_time(value, sim):
    if isinstance(value, dict):
        return {k: sim if k == 'use_sim_time' else rewrite_time(v, sim)
                for k, v in value.items()}
    if isinstance(value, list):
        return [rewrite_time(v, sim) for v in value]
    return value


def setup(context):
    component = LaunchConfiguration('component').perform(context)
    hardware = os.environ.get('HARDWARE_MODE', 'false').lower() == 'true'
    time_value = os.environ.get('USE_SIM_TIME', 'false').lower()
    if time_value not in ('true', 'false'):
        raise RuntimeError('USE_SIM_TIME must be true or false')
    sim = time_value == 'true'
    if hardware and sim:
        raise RuntimeError('Hardware deployment requires USE_SIM_TIME=false')
    profile = os.environ.get('NAV_PROFILE', 'adaptive' if hardware else 'holonomic')
    if profile not in ('holonomic', 'ackermann', 'adaptive'):
        raise RuntimeError('NAV_PROFILE must be holonomic, ackermann or adaptive')
    config = Path(os.environ.get('AUTONOMY_CONFIG', '/config'))
    runtime = Path(tempfile.mkdtemp(prefix='autonomy-params-'))

    def params(filename):
        if hardware:
            filename = {'amcl_sim.yaml': 'amcl.yaml', 'slam_sim.yaml': 'slam.yaml'}.get(filename, filename)
        with (config / filename).open() as stream:
            data = rewrite_time(yaml.safe_load(stream), sim)
        if hardware and filename == 'navigation_common.yaml':
            for costmap in ('local_costmap', 'global_costmap'):
                data[costmap][costmap]['ros__parameters']['obstacle_layer']['scan']['topic'] = os.environ.get(
                    'OBSTACLE_SCAN_TOPIC', '/scan_obstacles')
        if hardware and filename in ('amcl.yaml', 'slam.yaml'):
            name = 'amcl' if filename == 'amcl.yaml' else 'slam_toolbox'
            data[name]['ros__parameters']['scan_topic'] = os.environ.get('LOCALIZATION_SCAN_TOPIC', '/scan')
        if hardware and filename == 'logging.yaml':
            data['project_logger']['ros__parameters']['scan_topic'] = os.environ.get('LOCALIZATION_SCAN_TOPIC', '/scan')
            data['project_logger']['ros__parameters']['obstacle_scan_topic'] = os.environ.get('OBSTACLE_SCAN_TOPIC', '/scan_obstacles')
        target = runtime / filename
        target.write_text(yaml.safe_dump(data))
        if hardware:
            from ranger_logging.launch_support import record_input_file
            record_input_file(str(target), filename)
        return str(target)

    def node(package, executable, name, files=(), extra=None, remappings=()):
        factory = Node
        if hardware:
            from ranger_logging.launch_support import Node as LoggedNode
            factory = LoggedNode
        return factory(package=package, executable=executable, name=name, output='screen',
                    parameters=[*files, {'use_sim_time': sim, **(extra or {})}],
                    remappings=list(remappings))

    nodes = []
    if component == 'telemetry':
        nodes = [node('ranger_logging', 'project_logger.py', 'project_logger',
                      [params('logging.yaml')])]
    elif component == 'scan-adapter':
        if not hardware:
            raise RuntimeError('scan-adapter is only configured in hardware mode')
        nodes = [node('pointcloud_to_laserscan', 'pointcloud_to_laserscan_node',
                      'autonomy_pointcloud_to_laserscan', [params('pointcloud_to_laserscan.yaml')],
                      remappings=[('cloud_in', '/livox/lidar'),
                                  ('scan', os.environ.get('ADAPTER_SCAN_TOPIC', '/autonomy/scan'))])]
    elif component == 'estimation':
        if hardware:
            raise RuntimeError('Hardware uses existing /odom and TF; do not start a second EKF')
        nodes = [node('robot_localization', 'ekf_node', 'ekf_filter_node',
                      [params('ekf_sim.yaml')])]
    elif component in ('global-localization', 'slam', 'slam-lifecycle',
                       'map-server', 'amcl', 'localization-lifecycle'):
        mode = ('mapping' if component in ('slam', 'slam-lifecycle') else
                'localization' if component in ('map-server', 'amcl', 'localization-lifecycle') else
                os.environ.get('LOCALIZATION_MODE', 'localization'))
        if mode == 'mapping':
            nodes = [node('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox',
                          [params('slam_sim.yaml')],
                          {'use_lifecycle_manager': True} if hardware else {})]
            if hardware:
                nodes.append(node('nav2_lifecycle_manager', 'lifecycle_manager',
                                  'lifecycle_manager_slam', extra={
                                      'autostart': True, 'node_names': ['slam_toolbox'],
                                      'bond_timeout': 10.0}))
        elif mode == 'localization':
            path = Path(os.environ.get('MAP_FILE', '/maps/map.yaml'))
            with path.open() as stream:
                metadata = yaml.safe_load(stream)
            if not isinstance(metadata, dict) or not metadata.get('image'):
                raise RuntimeError('Map YAML must include image')
            if not (path.parent / metadata['image']).is_file():
                raise RuntimeError('Map image is missing: ' + str(metadata['image']))
            files = [params('amcl_sim.yaml')]
            nodes = [node('nav2_map_server', 'map_server', 'map_server', files,
                          {'yaml_filename': str(path)}),
                     node('nav2_amcl', 'amcl', 'amcl', files),
                     node('nav2_lifecycle_manager', 'lifecycle_manager',
                          'lifecycle_manager_localization', extra={
                              'autostart': True, 'node_names': ['map_server', 'amcl'],
                              'bond_timeout': 10.0})]
        else:
            raise RuntimeError('LOCALIZATION_MODE must be mapping or localization')
        if component != 'global-localization':
            indices = {'slam': 0, 'slam-lifecycle': 1,
                       'map-server': 0, 'amcl': 1, 'localization-lifecycle': 2}
            nodes = [nodes[indices[component]]]
    elif component in ('controller', 'planner', 'navigator', 'behaviors',
                       'bt-navigator', 'velocity-smoother', 'command-guard'):
        files = [params('navigation_common.yaml'), params('navigation_' + profile + '.yaml')]
        if component == 'controller':
            # Raw navigation output does not directly drive physical hardware.
            nodes = [node('nav2_controller', 'controller_server', 'controller_server',
                          files, remappings=[('cmd_vel', '/cmd_vel_nav_raw' if profile == 'ackermann'
                                             else '/cmd_vel_nav'),
                                             *([('joint_states', os.environ.get(
                                                 'STEERING_FEEDBACK_TOPIC', '/joint_states'))] if hardware else [])])]
            if profile == 'ackermann' and not hardware:
                control = yaml.safe_load((config / 'navigation_ackermann.yaml').read_text())[
                    'controller_server']['ros__parameters']['FollowPath']
                nodes.extend([
                    node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', files,
                         remappings=[('cmd_vel', '/cmd_vel_nav_raw'),
                                     ('cmd_vel_smoothed', '/cmd_vel_ackermann_smoothed')]),
                    node('four_ws_control', 'ackermann_command_guard.py', 'ackermann_command_guard',
                         files, {
                             'minimum_turning_radius': control['AckermannConstraints']['min_turning_r'],
                             'max_forward_speed': control['vx_max'],
                             'max_reverse_speed': abs(control['vx_min']),
                             'max_angular_speed': control['wz_max'],
                         }, remappings=[('cmd_vel', '/cmd_vel_nav')])])
        elif component in ('velocity-smoother', 'command-guard'):
            if profile != 'ackermann':
                raise RuntimeError(component + ' is only used by the standalone ackermann profile')
            if component == 'velocity-smoother':
                nodes = [node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', files,
                              remappings=[('cmd_vel', '/cmd_vel_nav_raw'),
                                          ('cmd_vel_smoothed', '/cmd_vel_ackermann_smoothed')])]
            else:
                control = yaml.safe_load((config / 'navigation_ackermann.yaml').read_text())[
                    'controller_server']['ros__parameters']['FollowPath']
                nodes = [node('four_ws_control', 'ackermann_command_guard.py', 'ackermann_command_guard',
                              files, {
                                  'minimum_turning_radius': control['AckermannConstraints']['min_turning_r'],
                                  'max_forward_speed': control['vx_max'],
                                  'max_reverse_speed': abs(control['vx_min']),
                                  'max_angular_speed': control['wz_max'],
                              }, remappings=[('cmd_vel', '/cmd_vel_nav')])]
        elif component == 'planner':
            extra = {}
            if profile in ('holonomic', 'adaptive'):
                lattice = config / 'ranger_omni_lattice.json'
                if not lattice.is_file():
                    raise RuntimeError('Missing Ranger lattice: ' + str(lattice))
                key = ('GridBased.Holonomic.lattice_filepath' if profile == 'adaptive'
                       else 'GridBased.lattice_filepath')
                extra[key] = str(lattice)
            nodes = [node('nav2_planner', 'planner_server', 'planner_server', files,
                          extra)]
        else:
            nodes = [node('nav2_behaviors', 'behavior_server', 'behavior_server', files,
                          remappings=[('cmd_vel', '/cmd_vel_nav_raw' if profile == 'ackermann'
                                       else '/cmd_vel_nav')]),
                     node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', files, {
                         'default_nav_to_pose_bt_xml': str(config / 'navigate_to_pose.xml'),
                         'default_nav_through_poses_bt_xml': str(config / 'navigate_through_poses.xml')})]
            if component == 'behaviors':
                nodes = nodes[:1]
            elif component == 'bt-navigator':
                nodes = nodes[1:]
    elif component == 'navigation-lifecycle':
        names = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
        if profile == 'ackermann':
            names.append('velocity_smoother')
        nodes = [node('nav2_lifecycle_manager', 'lifecycle_manager',
                      'lifecycle_manager_navigation', extra={
                          'autostart': True,
                          'node_names': names,
                          'bond_timeout': 10.0,
                          'attempt_respawn_reconnection': True,
                          'bond_respawn_max_duration': 60.0})]
    else:
        raise RuntimeError('Unknown component: ' + component)
    # A dead ROS process must stop its container so Docker can restart it.
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=process, on_exit=[EmitEvent(event=Shutdown(
            reason='Autonomy process exited'))])) for process in nodes]
    return [*handlers, *nodes]


def generate_launch_description():
    if os.environ.get('HARDWARE_MODE', 'false').lower() == 'true':
        from ranger_logging.launch_support import configure_launch_logging
        configure_launch_logging()
    return LaunchDescription([DeclareLaunchArgument('component'), OpaqueFunction(function=setup)])
