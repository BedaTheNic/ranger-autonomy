from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ranger_logging.launch_support import Node, configure_launch_logging, logging_actions


def generate_launch_description():
    configure_launch_logging()
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *logging_actions(),
        Node(package='four_ws_control', executable='robot_control.py', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
    ])
