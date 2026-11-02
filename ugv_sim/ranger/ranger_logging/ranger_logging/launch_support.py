"""Integrate bounded launch logs and a single passive telemetry recorder."""
import atexit
import logging
import os
import hashlib
from pathlib import Path

from launch.actions import OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch.logging import launch_config
from launch_ros.actions import Node as ROSNode
from .storage import Writer, atomic_json, create_session, segment_for, times

_writer = None


def enabled():
    return os.environ.get('RANGER_LOG_ENABLED', '1').lower() not in ('0', 'false', 'no')


class LaunchHandler(logging.Handler):
    def __init__(self, filename, **_kwargs):
        super().__init__()
        self.filename = str(filename)

    def setFormatterFor(self, *_args):
        pass  # JSON fields preserve the source; no per-process text prefix is needed.

    def emit(self, record):
        if _writer is not None:
            _writer.emit(segment_for(record.name), 'process_log',
                {'message': record.getMessage(), 'file': record.pathname, 'line': record.lineno},
                source=record.name, level=record.levelname)


def configure_launch_logging():
    global _writer
    if not enabled() or _writer is not None:
        return
    directory = os.environ.get('RANGER_LOG_SESSION_DIR')
    if not directory:
        directory = str(create_session(os.environ.get('RANGER_LOG_ROOT', str(Path.home()/'.ros/ranger_logs')), 'launch'))
        os.environ['RANGER_LOG_SESSION_DIR'] = directory
        atomic_json(Path(directory)/'manifest.json', dict(**times(), action='direct_launch',
                    domain=os.environ.get('ROS_DOMAIN_ID', '0')))
    os.environ['ROS_LOG_DIR'] = str(Path(directory)/'ros_native')
    _writer = Writer(directory, 'launch')
    atexit.register(_writer.close)
    # Launch creates its bootstrap handler before evaluating a launch file.
    # Replace existing handlers too; changing only the factory leaves launch.log unbounded.
    replacements = {}
    for name, old in list(launch_config.file_handlers.items()):
        new = LaunchHandler(name)
        replacements[old] = new
        launch_config.file_handlers[name] = new
    for logger in [logging.root, *logging.Logger.manager.loggerDict.values()]:
        if isinstance(logger, logging.Logger):
            for old, new in replacements.items():
                if old in logger.handlers:
                    logger.removeHandler(old)
                    logger.addHandler(new)
    for old in replacements:
        old.close()
    launch_config.log_handler_factory = LaunchHandler
    _writer.emit('system', 'launch_logging_started', {'session': directory})


def Node(*args, **kwargs):
    if enabled():
        kwargs['ros_arguments'] = [*kwargs.get('ros_arguments', []), '--disable-external-lib-logs']
        kwargs['output'] = 'both'
    return ROSNode(*args, **kwargs)


def record_input_file(filename, label):
    """Record generated/rewritten deployment params with their exact hash."""
    if _writer is None:
        return
    path = Path(filename)
    raw = path.read_bytes()
    _writer.emit('configuration', 'input_file', dict(argument=label,
                 path=str(path), sha256=hashlib.sha256(raw).hexdigest(),
                 text=raw.decode('utf-8', errors='replace')))


def logging_actions():
    def start(context):
        if not enabled() or context.launch_configurations.get('_ranger_logger_started'):
            return []
        context.launch_configurations['_ranger_logger_started'] = 'true'
        config = context.launch_configurations
        _writer.emit('configuration', 'launch_arguments', dict(config))
        for key, value in config.items():
            if not isinstance(value, str) or key.startswith('_'):
                continue
            path = Path(value)
            if path.suffix not in ('.world', '.yaml', '.xml', '.rviz', '.xacro'):
                continue
            try:
                if path.is_file() and path.stat().st_size <= 8*1024*1024:
                    raw = path.read_bytes()
                    _writer.emit('configuration', 'input_file', dict(argument=key,
                        path=str(path.resolve()), sha256=hashlib.sha256(raw).hexdigest(),
                        text=raw.decode('utf-8', errors='replace')))
            except OSError as exc:
                _writer.emit('configuration', 'input_file_error', {'path': value, 'error': str(exc)}, level='WARN')
        return [Node(package='ranger_logging', executable='project_logger.py',
                     parameters=[{'use_sim_time': config.get('use_sim_time', 'true').lower() == 'true',
                                  'expect_odometry': config.get('odometry', 'false').lower() == 'true',
                                  'expect_ekf': config.get('ekf', 'false').lower() == 'true',
                                  'expect_map': config.get('mode') in ('mapping', 'localization')}])]
    return [OpaqueFunction(function=start)]


class GazeboServerSource(PythonLaunchDescriptionSource):
    """Keep upstream Gazebo launch/env, but append separate ROS argv tokens.

    Humble's extra_gazebo_args is passed as ONE argv item, so a string with
    multiple flags there silently fails to disable the native disk logger.
    """
    def _get_launch_description(self, location):
        description = super()._get_launch_description(location)
        if enabled():
            for action in description.entities:
                if isinstance(action, ExecuteProcess):
                    action.cmd.extend([[TextSubstitution(text=arg)] for arg in
                                       ('--ros-args', '--disable-external-lib-logs', '--')])
        return description
