"""Passive ROS diagnostics. Sampling and health timers use steady wall time."""
import base64
from collections import Counter
import hashlib
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import time
import uuid
import zlib
import xml.etree.ElementTree as ET

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_parameter_events
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from nav2_msgs.msg import Costmap, BehaviorTreeLog
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from lifecycle_msgs.msg import TransitionEvent
from rcl_interfaces.msg import Log, ParameterEvent
from rcl_interfaces.srv import ListParameters, GetParameters
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from .storage import Writer, atomic_json, create_session, encoded, segment_for, times


class Recorder(Node):
    def __init__(self):
        super().__init__('project_logger', enable_rosout=False)
        self.declare_parameter('expect_odometry', True)
        self.declare_parameter('expect_ekf', True)
        self.declare_parameter('expect_map', False)
        self.declare_parameter('expect_clock', True)
        self.declare_parameter('expect_joint_states', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_scan_topic', '')
        self.declare_parameter('wheel_odometry_topic', '/wheel/odometry')
        self.declare_parameter('lidar_topic', '')
        self.imu_topic = self.get_parameter('imu_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.obstacle_scan_topic = self.get_parameter('obstacle_scan_topic').value
        self.wheel_odometry_topic = self.get_parameter('wheel_odometry_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.session = Path(os.environ.get('RANGER_LOG_SESSION_DIR') or create_session(
            os.environ.get('RANGER_LOG_ROOT', str(Path.home()/'.ros/ranger_logs')), 'recorder'))
        self.sample_hz = float(os.environ.get('RANGER_LOG_SAMPLE_HZ', '5'))
        if not math.isfinite(self.sample_hz) or not 0 < self.sample_hz <= 100:
            raise ValueError('RANGER_LOG_SAMPLE_HZ must be in (0, 100]')
        self.writer = Writer(self.session, 'telemetry')
        self.counts, self.sampled, self.last_logged, self.last_seen = Counter(), Counter(), {}, {}
        self.latest, self.maps, self.identities, self.tf_parents = {}, {}, {}, {}
        self.parameter_seen, self.parameter_jobs, self.parameter_clients = set(), {}, {}
        self.parameter_retry_after = {}
        self.lifecycle_subscriptions = {}
        self.hardware_subscriptions = {}
        self.start_wall = time.monotonic()
        self.last_health = self.start_wall
        self.last_counts = Counter()
        self.last_snapshot = -math.inf
        self.last_clock = None
        self.fast = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.events = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscribe(Log, '/rosout', self.on_log, QoSProfile(depth=1000,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.subscribe(ParameterEvent, '/parameter_events', lambda m: self.record(
            'configuration', 'parameter_event', message_to_ordereddict(m), m.node), qos_profile_parameter_events)
        self.subscribe(ClockMessage, '/clock', self.on_clock)
        self.subscribe(TFMessage, '/tf', lambda m: self.on_tf(m, False))
        self.subscribe(TFMessage, '/tf_static', lambda m: self.on_tf(m, True), self.latched)
        for typ, topic, segment, rate in [
            (Imu, self.imu_topic, 'sensors', self.sample_hz),
            (LaserScan, self.scan_topic, 'sensors', 1),
            (JointState, '/joint_states', 'base_control', self.sample_hz),
            (Odometry, self.wheel_odometry_topic, 'odometry', self.sample_hz),
            (Odometry, '/odometry/filtered', 'odometry', self.sample_hz),
            (Odometry, '/ground_truth/odom', 'odometry', self.sample_hz),
            (PoseWithCovarianceStamped, '/amcl_pose', 'localization', self.sample_hz),
            (PoseWithCovarianceStamped, '/initialpose', 'localization', 0),
            (PoseStamped, '/goal_pose', 'navigation', 0),
            (Twist, '/cmd_vel', 'control', 10),
            (Twist, '/cmd_vel_nav', 'control', 10),
            (Twist, '/cmd_vel_nav_raw', 'control', 10),
            (Twist, '/cmd_vel_ackermann_smoothed', 'control', 10),
            (Float64MultiArray, '/forward_position_controller/commands', 'base_control', self.sample_hz),
            (Float64MultiArray, '/forward_velocity_controller/commands', 'base_control', self.sample_hz),
            (DiagnosticArray, '/diagnostics', 'diagnostics', 0),
            (NavPath, '/local_plan', 'control', 1),
            (PolygonStamped, '/local_costmap/published_footprint', 'control', 1),
            (PolygonStamped, '/global_costmap/published_footprint', 'planning', 1),
        ]:
            self.subscribe(typ, topic, self.sample_callback(topic, segment, rate),
                           self.events if rate == 0 else self.fast)
        if self.obstacle_scan_topic and self.obstacle_scan_topic != self.scan_topic:
            self.subscribe(LaserScan, self.obstacle_scan_topic,
                           self.sample_callback(self.obstacle_scan_topic, 'sensors', 1))
        if self.lidar_topic:
            self.subscribe(PointCloud2, self.lidar_topic, self.on_cloud)
        for topic in ['/wheel/distance', '/wheel/kinematic_residual', '/odometry/distance',
                      '/odometry/speed', '/odometry/yaw', '/odometry/rotation']:
            self.subscribe(Float64, topic, self.sample_callback(topic, 'odometry', self.sample_hz))
        for topic in ['/navigation_mode', '/navigation_mode_decision', '/navigation_plan_decision']:
            self.subscribe(String, topic, self.sample_callback(topic,
                'planning' if 'plan_decision' in topic else 'control', 0),
                self.events if topic == '/navigation_mode_decision' else self.latched)
        self.subscribe(NavPath, '/plan', self.on_plan, self.events)
        self.subscribe(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, self.events)
        for name in ['navigate_to_pose', 'navigate_through_poses', 'follow_path',
                     'compute_path_to_pose', 'compute_path_through_poses', 'wait']:
            topic = f'/{name}/_action/status'
            self.subscribe(GoalStatusArray, topic, self.status_callback(topic), self.latched)
        for name, action in [('navigate_to_pose', NavigateToPose), ('navigate_through_poses', NavigateThroughPoses)]:
            topic = f'/{name}/_action/feedback'
            self.subscribe(action.Impl.FeedbackMessage, topic, self.sample_callback(topic, 'navigation', 2))
        for typ, topic in [(OccupancyGrid, '/map'), (Costmap, '/global_costmap/costmap_raw'),
                           (Costmap, '/local_costmap/costmap_raw')]:
            self.subscribe(typ, topic, self.map_callback(topic), self.latched if topic == '/map' else self.fast)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(2, self.health, clock=steady)
        self.create_timer(5, self.discover, clock=steady)
        self.create_timer(30, lambda: self.snapshot('periodic'), clock=steady)
        self.record('system', 'recorder_start', dict(sample_hz=self.sample_hz,
                    domain=os.environ.get('ROS_DOMAIN_ID'), pid=os.getpid(), session=str(self.session)))
        atomic_json(self.session/f'telemetry-{os.getpid()}-metadata.json', dict(**times(),
                    sample_hz=self.sample_hz, scan_hz=1, cmd_vel_hz=10, snapshots_seconds=30,
                    note='Sampled diagnostics, not a lossless rosbag recording. No control commands are published.'))
        versions = {}
        for package in ('ranger_logging', 'ranger_mini', 'four_ws_control', 'ranger_nav_plugins',
                        'robot_localization', 'slam_toolbox', 'nav2_amcl', 'nav2_controller',
                        'nav2_planner', 'nav2_smac_planner', 'nav2_mppi_controller', 'gazebo_ros'):
            try:
                path = Path(get_package_share_directory(package))/'package.xml'
                raw = path.read_bytes()
                versions[package] = dict(version=ET.fromstring(raw).findtext('version'),
                    path=str(path), manifest_sha256=hashlib.sha256(raw).hexdigest())
            except (OSError, PackageNotFoundError, ET.ParseError) as exc:
                versions[package] = {'error': str(exc)}
        self.record('configuration', 'runtime_versions', dict(packages=versions,
            ros_distro=os.environ.get('ROS_DISTRO'), rmw=get_rmw_implementation_identifier(),
            python=platform.python_version(), platform=platform.platform()))

    def subscribe(self, typ, topic, callback, qos=None):
        return self.create_subscription(typ, topic, callback, qos or self.fast)

    def seen(self, topic):
        self.counts[topic] += 1
        self.last_seen[topic] = time.monotonic()

    def record(self, segment, event, data, source='', level='INFO'):
        self.writer.emit(segment, event, data, source=source, level=level,
                         ros_time_ns=self.get_clock().now().nanoseconds)

    def sample_callback(self, topic, segment, rate):
        def receive(msg):
            self.seen(topic)
            now = time.monotonic()
            if rate and now-self.last_logged.get(topic, -math.inf) < 1/rate:
                return
            self.last_logged[topic] = now
            data = message_to_ordereddict(msg)
            self.latest[topic] = dict(received_monotonic=now, message=data)
            self.sampled[topic] += 1
            self.record(segment, 'message', data, topic)
        return receive

    def on_clock(self, msg):
        self.seen('/clock')
        stamp = msg.clock.sec*10**9+msg.clock.nanosec
        if self.last_clock is not None and stamp < self.last_clock:
            self.record('diagnostics', 'clock_rewind', {'previous_ns': self.last_clock, 'current_ns': stamp}, '/clock', 'WARN')
        self.last_clock = stamp

    def on_log(self, msg):
        self.seen('/rosout')
        self.record(segment_for(msg.name), 'ros_log', message_to_ordereddict(msg), msg.name,
                    {10:'DEBUG', 20:'INFO', 30:'WARN', 40:'ERROR', 50:'FATAL'}.get(msg.level, str(msg.level)))
        if msg.level >= 40:
            self.snapshot(f'ros_error:{msg.name}')

    def on_tf(self, msg, static):
        topic = '/tf_static' if static else '/tf'
        self.seen(topic)
        now = time.monotonic()
        for transform in msg.transforms:
            child = transform.child_frame_id
            parent = transform.header.frame_id
            self.tf_parents.setdefault(child, {})[parent] = (now, static)
            key = f'{topic}:{parent}:{child}'
            hz = self.sample_hz if child in (self.base_frame, 'odom') else 1
            if static or now-self.last_logged.get(key, -math.inf) >= 1/hz:
                self.record('tf', 'transform', dict(transform=message_to_ordereddict(transform),
                            static=static), topic)
                self.last_logged[key] = now

    def on_plan(self, msg):
        self.seen('/plan')
        data = message_to_ordereddict(msg)
        signature = hashlib.sha256(encoded(data['poses'])).hexdigest()
        self.record('planning', 'global_plan', dict(signature=signature, path=data), '/plan')
        self.latest['/plan'] = dict(signature=signature, poses=len(msg.poses))
        self.snapshot('global_plan')

    def on_cloud(self, msg):
        self.seen(self.lidar_topic)
        now = time.monotonic()
        if now-self.last_logged.get(self.lidar_topic, -math.inf) < 1:
            return
        self.last_logged[self.lidar_topic] = now
        self.sampled[self.lidar_topic] += 1
        # Keep cloud shape, fields and timing without serializing megabytes
        # of point bytes into the sampled diagnostic log on every callback.
        self.record('sensors', 'pointcloud_metadata', dict(
            header=message_to_ordereddict(msg.header), height=msg.height, width=msg.width,
            fields=[message_to_ordereddict(field) for field in msg.fields],
            point_step=msg.point_step, row_step=msg.row_step,
            data_bytes=len(msg.data), is_dense=msg.is_dense), self.lidar_topic)

    def on_bt(self, msg):
        self.seen('/behavior_tree_log')
        self.record('navigation', 'behavior_tree', message_to_ordereddict(msg), '/behavior_tree_log')
        if any(e.current_status == 'FAILURE' for e in msg.event_log):
            self.snapshot('behavior_tree_failure')

    def status_callback(self, topic):
        def receive(msg):
            self.seen(topic)
            statuses = [(bytes(s.goal_info.goal_id.uuid).hex(), s.status) for s in msg.status_list]
            if self.identities.get(topic) != statuses:
                self.identities[topic] = statuses
                self.record('navigation', 'action_status', message_to_ordereddict(msg), topic)
                if any(status in (5, 6) for _, status in statuses):
                    self.snapshot('action_canceled_or_aborted')
        return receive

    def map_callback(self, topic):
        def receive(msg):
            self.seen(topic)
            self.maps[topic] = (msg, time.monotonic())
        return receive

    def snapshot(self, reason):
        now = time.monotonic()
        if now-self.last_snapshot < 1:
            return
        self.last_snapshot = now
        identity = uuid.uuid4().hex
        self.record('diagnostics', 'snapshot', dict(id=identity, reason=reason, latest=dict(self.latest)))
        for topic, (msg, received) in self.maps.items():
            # Compress in the diagnostic process, never inside the control nodes.
            data = message_to_ordereddict(msg)
            values = data.pop('data')
            raw = bytes((v & 255 for v in values))
            self.record('slam' if topic == '/map' else 'planning' if 'global' in topic else 'control',
                'grid_snapshot', dict(id=identity, reason=reason, metadata=data, age_seconds=now-received,
                    encoding='zlib/base64', dtype='int8' if topic == '/map' else 'uint8',
                    sha256=hashlib.sha256(raw).hexdigest(), data=base64.b64encode(zlib.compress(raw)).decode('ascii')), topic)

    def health(self):
        now = time.monotonic()
        elapsed = now-self.last_health
        expected = [self.scan_topic, self.imu_topic]
        if self.get_parameter('expect_clock').value:
            expected += ['/clock']
        if self.get_parameter('expect_joint_states').value:
            expected += ['/joint_states']
        if self.obstacle_scan_topic:
            expected += [self.obstacle_scan_topic]
        if self.lidar_topic:
            expected += [self.lidar_topic]
        if self.get_parameter('expect_odometry').value:
            expected += [self.wheel_odometry_topic]
            if self.get_parameter('expect_ekf').value:
                expected += ['/odometry/filtered']
        if self.get_parameter('expect_map').value:
            expected += ['/map']
        topics = {}
        for topic in sorted(set(self.counts) | set(expected)):
            age = now-self.last_seen[topic] if topic in self.last_seen else None
            topics[topic] = dict(received=self.counts[topic], sampled=self.sampled[topic],
                hz=(self.counts[topic]-self.last_counts[topic])/elapsed, age_seconds=age,
                missing=topic in expected and age is None, stale=topic in expected and topic != '/map' and age is not None and age>2)
        parents = {child: {parent: dict(age=now-v[0], static=v[1])
                   for parent,v in entries.items() if v[1] or now-v[0]<2}
                   for child,entries in self.tf_parents.items()}
        # Humble's Python executor does not pass MessageInfo to subscription callbacks.
        # Keep topic publishers separate: a publisher cannot be attributed to a TF child.
        publishers = {topic: [dict(name=p.node_name, namespace=p.node_namespace,
                         gid=bytes(p.endpoint_gid).hex()) for p in self.get_publishers_info_by_topic(topic)]
                      for topic in ('/tf', '/tf_static')}
        try:
            disk_free = shutil.disk_usage(self.session).free
        except OSError:
            disk_free = None
        self.record('diagnostics', 'health', dict(topics=topics, tf_parents=parents, tf_publishers=publishers,
            conflicting_tf_parents=[child for child,entries in parents.items() if len(entries)>1],
            disk_free_bytes=disk_free,
            writer_queue=self.writer.queue.qsize(), writer_dropped=dict(self.writer.dropped), writer_errors=self.writer.errors))
        self.last_counts, self.last_health = self.counts.copy(), now

    def discover(self):
        now = time.monotonic()
        nodes = sorted({(ns.rstrip('/')+'/'+name) for name, ns in self.get_node_names_and_namespaces()
                        if name != self.get_name()})
        self.record('lifecycle', 'node_graph', {'nodes': nodes})
        for topic, types in self.get_topic_names_and_types():
            if 'lifecycle_msgs/msg/TransitionEvent' in types and topic not in self.lifecycle_subscriptions:
                self.lifecycle_subscriptions[topic] = self.subscribe(TransitionEvent, topic,
                    self.sample_callback(topic, 'lifecycle', 0), self.fast)
            if topic in ('/actuator_state', '/motion_state', '/system_state', '/battery_state') and topic not in self.hardware_subscriptions:
                try:
                    typ = get_message(types[0])
                except (ImportError, AttributeError, ValueError):
                    continue
                self.hardware_subscriptions[topic] = self.subscribe(
                    typ, topic, self.sample_callback(topic, 'base_control', self.sample_hz))
        self.parameter_seen.intersection_update(nodes)
        for name, (started, future) in list(self.parameter_jobs.items()):
            if now-started > 10:
                future.cancel()
                del self.parameter_jobs[name]
                self.parameter_retry_after[name] = now+60
                self.record('configuration', 'parameter_snapshot_timeout', {'node': name}, level='WARN')
        for name in nodes:
            if (name in self.parameter_seen or name in self.parameter_jobs or len(self.parameter_jobs)>=2
                    or now < self.parameter_retry_after.get(name, 0)):
                continue
            clients = self.parameter_clients.get(name)
            if clients is None:
                clients = (self.create_client(ListParameters, name+'/list_parameters'),
                           self.create_client(GetParameters, name+'/get_parameters'))
                self.parameter_clients[name] = clients
            if not all(client.service_is_ready() for client in clients):
                continue
            future = clients[0].call_async(ListParameters.Request())
            self.parameter_jobs[name] = (now, future)
            future.add_done_callback(lambda f, n=name: self.listed_parameters(n, f))

    def listed_parameters(self, name, future):
        if future.cancelled() or name not in self.parameter_jobs:
            return
        try:
            names = future.result().result.names
            request = GetParameters.Request(names=names)
            get = self.parameter_clients[name][1].call_async(request)
            self.parameter_jobs[name] = (time.monotonic(), get)
            get.add_done_callback(lambda f: self.got_parameters(name, names, f))
        except Exception as exc:
            self.parameter_jobs.pop(name, None)
            self.record('configuration', 'parameter_snapshot_error', {'node': name, 'error': str(exc)}, level='WARN')

    def got_parameters(self, name, names, future):
        if future.cancelled() or name not in self.parameter_jobs:
            return
        try:
            values = future.result().values
            self.record('configuration', 'parameters_snapshot',
                        dict(node=name, parameters={k:message_to_ordereddict(v) for k,v in zip(names,values)}))
            self.parameter_seen.add(name)
        except Exception as exc:
            self.record('configuration', 'parameter_snapshot_error', {'node': name, 'error': str(exc)}, level='WARN')
        self.parameter_jobs.pop(name, None)


def main():
    # Launch and the terminal process group may both send SIGINT. Keep repeated
    # signals from interrupting the final queue drain / corrupting its summary.
    stopping = False
    def stop(_signum, _frame):
        nonlocal stopping
        if not stopping:
            stopping = True
            raise KeyboardInterrupt
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    node = None
    try:
        node = Recorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stopping = True
        if node is not None:
            node.writer.emit('system', 'recorder_stop', {'received': dict(node.counts)})
            node.writer.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
