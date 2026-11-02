import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.utilities import perform_substitutions
from ranger_logging.launch_support import GazeboServerSource


class GazeboArgumentsTest(unittest.TestCase):
    def test_native_log_flag_is_separate_argv_not_a_space_joined_string(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'server.launch.py'
            path.write_text('from launch import LaunchDescription\n'
                'from launch.actions import ExecuteProcess\n'
                'def generate_launch_description():\n'
                ' return LaunchDescription([ExecuteProcess(cmd=["gzserver", "world", '
                '"--ros-args", "--params-file", "config", "--"])])\n')
            for enabled in (True, False):
                with self.subTest(enabled=enabled), patch.dict(os.environ,
                        {'RANGER_LOG_ENABLED': str(int(enabled)), 'ROS_LOG_DIR': folder}):
                    context = LaunchContext()
                    description = GazeboServerSource(str(path)).get_launch_description(context)
                    command = [perform_substitutions(context, item) for item in description.entities[0].cmd]
                    original = ['gzserver', 'world', '--ros-args', '--params-file', 'config', '--']
                    expected = original+(['--ros-args', '--disable-external-lib-logs', '--'] if enabled else [])
                    self.assertEqual(command, expected)
