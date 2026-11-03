"""Isolated synthetic inputs: no physical drive commands, tests image startup/planning/logging."""
import array
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PolygonStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField, JointState
from tf2_ros import StaticTransformBroadcaster
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import IsPathValid
from std_srvs.srv import Trigger

profile=sys.argv[1] if len(sys.argv)>1 else 'adaptive'
external='--external' in sys.argv
can_steering='--can-steering' in sys.argv
os.environ.update(HARDWARE_MODE='true', USE_SIM_TIME='false', NAV_PROFILE=profile, ROS_DOMAIN_ID='211', OBSTACLE_SCAN_TOPIC='/autonomy/scan', RANGER_LOG_ROOT='/tmp/image-logs-'+profile, RANGER_LOG_MAX_BYTES='65536')
rclpy.init()
node=rclpy.create_node('isolated_image_fixture')
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
map_pub=node.create_publisher(OccupancyGrid,'/map',latched)
odom_pub=node.create_publisher(Odometry,'/odom',10)
cloud_pub=node.create_publisher(PointCloud2,'/livox/lidar',10)
joint_pub=None if can_steering else node.create_publisher(JointState,'/joint_states',10)
can_sender=None
transforms=[]
for parent,child in [('map','odom'),('odom','ranger_base_link')]:
 t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;t.header.stamp=node.get_clock().now().to_msg();transforms.append(t)
broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform(transforms)
grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05;grid.info.width=200;grid.info.height=200;grid.info.origin.position.x=-5.;grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.;grid.data=array.array('b',[0]*40000)
for i in range(200):
 for j in range(200):
  if i in (0,199) or j in (0,199): grid.data[j*200+i]=100
cloud=PointCloud2();cloud.header.frame_id='ranger_base_link';cloud.height=1;cloud.width=720
cloud.fields=[PointField(name=name,offset=index*4,datatype=PointField.FLOAT32,count=1) for index,name in enumerate(('x','y','z'))];cloud.point_step=12;cloud.row_step=cloud.width*12;cloud.is_dense=True
cloud.data=b''.join(struct.pack('<fff',4*math.cos(i*math.pi/360),4*math.sin(i*math.pi/360),.4) for i in range(720))
odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='ranger_base_link';odom.pose.pose.orientation.w=1.
joints=JointState();joints.name=['fl_steering_wheel','fr_steering_wheel','rl_steering_wheel','rr_steering_wheel'];joints.position=[0.]*4
lidar_tf=TransformStamped();lidar_tf.header.frame_id='ranger_base_link';lidar_tf.child_frame_id='livox_frame';lidar_tf.transform.translation.x=.288;lidar_tf.transform.translation.y=-.007;lidar_tf.transform.translation.z=.046;lidar_tf.transform.rotation.z=math.sin(math.pi/4);lidar_tf.transform.rotation.w=math.cos(math.pi/4);broadcaster.sendTransform(transforms+[lidar_tf])
cloud.header.frame_id='livox_frame'
# The fixture fixes both base transforms at the origin with identity rotation.
footprints={}
for costmap in ('local_costmap','global_costmap'):
 node.create_subscription(PolygonStamped,'/'+costmap+'/published_footprint',
  lambda msg,key=costmap: footprints.update({key:msg.polygon.points}),10)
received_scan=[]
candidate_commands=[];guard_commands=[];publish_cloud=True
from geometry_msgs.msg import Twist
candidate_sub=node.create_subscription(Twist,'/cmd_vel_candidate',lambda msg:candidate_commands.append((time.monotonic(),msg)),10)
guard_sub=node.create_subscription(Twist,'/cmd_vel_nav',lambda msg:guard_commands.append((time.monotonic(),msg)),10)
from sensor_msgs.msg import LaserScan
scan_sub=node.create_subscription(LaserScan,'/autonomy/scan',lambda msg:received_scan.append(msg.header.frame_id),qos_profile_sensor_data)
def publish():
 stamp=node.get_clock().now().to_msg()
 grid.header.stamp=stamp;map_pub.publish(grid)
 cloud.header.stamp=stamp
 if publish_cloud:cloud_pub.publish(cloud)
 odom.header.stamp=stamp;odom_pub.publish(odom)
 if can_sender is not None:
  # A measured straight wheel frame exercises the same transport, decoding and
  # reliable ROS subscription as hardware. Synthetic odometry stays stationary.
  can_sender.send(struct.pack('=IB3x8s',0x271,8,b'\0'*8))
 else:
  joints.header.stamp=stamp;joint_pub.publish(joints)
node.create_timer(.1,publish)
processes=[]
logdir=Path('/tmp/smoke-'+profile);logdir.mkdir(exist_ok=True)
try:
 if can_steering:
  from virtual_can import create
  can_sender=create()
  env=dict(os.environ,STEERING_CAN_INTERFACE='vcan_nav',STEERING_FEEDBACK_TOPIC='/ranger/steering_joint_states')
  stream=(logdir/'steering-feedback.log').open('w')
  p=subprocess.Popen([sys.executable,str(Path(__file__).resolve().parents[1]/'steering_feedback.py')],
                     env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
  processes.append(p)
 components=['scan-adapter','planner','controller','motion-guard','behaviors','bt-navigator','telemetry']
 if profile=='ackermann':components+=['velocity-smoother','command-guard']
 components+=['navigation-lifecycle']
 for component in ([] if external else components):
  stream=(logdir/(component+'.log')).open('w')
  p=subprocess.Popen(['/opt/autonomy/entrypoint.sh','ros2','launch','/opt/autonomy/launch/component.launch.py','component:='+component],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
  processes.append(p)
 clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in ('planner_server','controller_server','behavior_server','bt_navigator')}
 manager=node.create_client(Trigger,'/lifecycle_manager_navigation/is_active')
 bringup=None;bringup_sent=0.;manager_active=False
 pending={};active=set();deadline=time.monotonic()+70
 while time.monotonic()<deadline and (len(active)<4 or not received_scan or len(footprints)<2):
  rclpy.spin_once(node,timeout_sec=.1)
  # Let the lifecycle manager finish its own transitions before this fixture
  # issues GetState requests to the same nodes. Configuration can spend many
  # seconds warming the Hybrid heuristic on a small CI runner.
  if not manager_active:
   if bringup is not None and bringup.done():
    manager_active=bool(bringup.result() and bringup.result().success);bringup=None
   elif bringup is not None and time.monotonic()-bringup_sent>2:
    manager.remove_pending_request(bringup);bringup.cancel();bringup=None
   if not manager_active:
    if bringup is None and manager.service_is_ready():
     bringup=manager.call_async(Trigger.Request());bringup_sent=time.monotonic()
    continue
  for name,client in clients.items():
   if name in active:continue
   if name in pending and pending[name].done():
    response=pending.pop(name).result()
    if response and response.current_state.label=='active':active.add(name);print('ACTIVE',name,flush=True)
   if name not in pending and client.service_is_ready():pending[name]=client.call_async(GetState.Request())
  if any(p.poll() is not None for p in processes):raise RuntimeError('A launch process exited')
 assert len(active)==4,active
 expected={(x,y) for x in (-.369,.369) for y in (-.250,.250)}
 assert len(footprints)==2,footprints.keys()
 for costmap,points in footprints.items():
  actual={(round(p.x,4),round(p.y,4)) for p in points}
  assert actual==expected,(costmap,actual,expected)
 print('Verified both published 0.738 x 0.500 m physical footprints',flush=True)
 assert received_scan and received_scan[-1]=='ranger_base_link',received_scan
 action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
 assert action.wait_for_server(timeout_sec=5)
 goal=ComputePathToPose.Goal();goal.use_start=True;goal.planner_id='GridBased'
 goal.start.header.frame_id='map';goal.start.pose.orientation.w=1.
 goal.goal.header.frame_id='map';goal.goal.pose.position.x=.5;goal.goal.pose.orientation.w=1.
 future=action.send_goal_async(goal)
 rclpy.spin_until_future_complete(node,future,timeout_sec=10)
 handle=future.result();assert handle and handle.accepted
 result=handle.get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=15)
 response=result.result();assert response and response.status==4, response
 assert len(response.result.path.poses)>1
 validity=node.create_client(IsPathValid,'/is_retained_path_valid')
 assert validity.wait_for_service(timeout_sec=5)
 future=validity.call_async(IsPathValid.Request(path=response.result.path))
 rclpy.spin_until_future_complete(node,future,timeout_sec=5)
 assert future.done() and future.result().is_valid,'Retained path validation failed'
 follow=ActionClient(node,FollowPath,'/follow_path');assert follow.wait_for_server(timeout_sec=5)
 request=FollowPath.Goal();request.path=response.result.path;request.controller_id='FollowPath';request.goal_checker_id='goal_checker'
 future=follow.send_goal_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
 follower=future.result();assert follower and follower.accepted
 moving=lambda msg:math.hypot(msg.linear.x,msg.linear.y)>1e-5 or abs(msg.angular.z)>1e-5
 deadline=time.monotonic()+8
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if any(moving(v) for _,v in candidate_commands) and any(moving(v) for _,v in guard_commands):break
 assert any(moving(v) for _,v in candidate_commands),'Controller did not publish a motion candidate'
 assert any(moving(v) for _,v in guard_commands),'Motion guard did not pass a clear-space command'
 publishers=node.get_publishers_info_by_topic('/cmd_vel_nav')
 assert any(p.node_name=='navigation_motion_guard' for p in publishers),publishers
 assert not any(p.node_name=='controller_server' for p in publishers),'Controller bypassed motion guard'
 publish_cloud=False;stopped_at=time.monotonic();deadline=stopped_at+2
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 assert guard_commands and guard_commands[-1][0]>stopped_at+.7 and not moving(guard_commands[-1][1]),'Stale scan must stop guarded output'
 publish_cloud=True
 future=follower.cancel_goal_async();rclpy.spin_until_future_complete(node,future,timeout_sec=5)
 assert future.done() and future.result().goals_canceling,'FollowPath cancellation failed'
 deadline=time.monotonic()+3
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 print('PASS',profile,': scan conversion, Nav2 activation, planning, retained-path validity, controller/guard wiring, stale-scan stop and cancellation; poses=',len(response.result.path.poses),flush=True)
finally:
 from nav2_msgs.srv import ManageLifecycleNodes
 client=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
 if client.service_is_ready():
  future=client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
  rclpy.spin_until_future_complete(node,future,timeout_sec=10)
 for p in processes:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in processes:
  try:p.wait(timeout=10)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 node.destroy_node();rclpy.shutdown()

if external:
 print('PASS external Compose services: planning and scan checks completed',flush=True)
 raise SystemExit(0)
from ranger_logging.reader import records
files=list(Path('/tmp/image-logs-'+profile).rglob('*.jsonl'))
assert files and all(path.stat().st_size<=65536 for path in files)
rows=list(records(files))
events={row.get('event') for row in rows}
assert {'global_plan','pointcloud_metadata','runtime_versions','health','grid_snapshot','process_start','process_exit'}.issubset(events),events
assert not any(row.get('event')=='forced_shutdown' for row in rows)
health=[row for row in rows if row.get('event')=='health']
assert health and '/clock' not in health[-1]['data']['topics'],health[-1] if health else None
assert '/odom' in health[-1]['data']['topics']
print('PASS structured logging:',len(files),'bounded parts;',len(rows),'records; events:',sorted(events),flush=True)
