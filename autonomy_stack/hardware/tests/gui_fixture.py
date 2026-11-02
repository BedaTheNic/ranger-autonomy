"""Synthetic ROS graph for the isolated GUI/goal-preview integration test."""
import array
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionServer
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster


rclpy.init()
node=rclpy.create_node('isolated_gui_fixture')
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
map_pub=node.create_publisher(OccupancyGrid,'/map',latched)
scan_pub=node.create_publisher(LaserScan,'/scan_obstacles',10)
odom_pub=node.create_publisher(Odometry,'/odom',10)
goal_pub=node.create_publisher(PoseStamped,'/autonomy/preview_goal',10)
statuses=[];paths=[];commands=[];navigation=[];requests=[]
subscriptions=[node.create_subscription(String,'/autonomy/preview_status',
    lambda msg:statuses.append(json.loads(msg.data)),latched),
    node.create_subscription(Path,'/autonomy/preview_path',lambda msg:paths.append(msg),latched)]
for name in ('/cmd_vel','/cmd_vel_nav'):
    subscriptions.append(node.create_subscription(Twist,name,lambda msg:commands.append(msg),10))


def plan(handle):
    requests.append(handle.request)
    result=ComputePathToPose.Result()
    assert not handle.request.use_start
    assert handle.request.planner_id=='GridBased'
    if handle.request.goal.pose.position.x>10:
        result.error_code=1;result.error_msg='Synthetic occupied goal';handle.abort();return result
    result.path.header.frame_id='map'
    result.path.header.stamp=node.get_clock().now().to_msg()
    for fraction in (0.,.5,1.):
        pose=PoseStamped();pose.header=result.path.header;pose.pose.orientation.w=1.
        pose.pose.position.x=fraction*handle.request.goal.pose.position.x
        pose.pose.position.y=fraction*handle.request.goal.pose.position.y
        result.path.poses.append(pose)
    handle.succeed();return result


def navigate(handle):
    navigation.append(handle.request);handle.abort();return NavigateToPose.Result()


server=ActionServer(node,ComputePathToPose,'/compute_path_to_pose',plan)
nav_server=ActionServer(node,NavigateToPose,'/navigate_to_pose',navigate)
tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
tf.transform.rotation.w=1.;tf.header.stamp=node.get_clock().now().to_msg()
base=TransformStamped();base.header.frame_id='odom';base.child_frame_id='ranger_base_link'
base.transform.rotation.w=1.;base.header.stamp=tf.header.stamp
broadcaster=StaticTransformBroadcaster(node);broadcaster.sendTransform([tf,base])
grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.05
grid.info.width=200;grid.info.height=200
grid.info.origin.position.x=-5.;grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.
grid.data=array.array('b',[0]*40000)
for y in range(200):
    for x in range(200):
        if x in (0,199) or y in (0,199):grid.data[y*200+x]=100
        if 125 <= x <= 129 and 65 <= y <= 105:grid.data[y*200+x]=100
odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='ranger_base_link'
odom.pose.pose.orientation.w=1.
scan=LaserScan();scan.header.frame_id='ranger_base_link';scan.angle_min=-math.pi
scan.angle_max=math.pi;scan.angle_increment=math.pi/180;scan.range_min=.1;scan.range_max=10.
scan.ranges=[3.]*361


def publish():
    grid.header.stamp=node.get_clock().now().to_msg();map_pub.publish(grid)
    scan.header.stamp=grid.header.stamp;scan_pub.publish(scan)
    odom.header.stamp=grid.header.stamp;odom_pub.publish(odom)


node.create_timer(.1,publish)


def wait(predicate,seconds=15):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if predicate():return
    raise AssertionError({'statuses':statuses,'requests':len(requests),'paths':len(paths)})


def goal(x):
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.position.x=x;msg.pose.position.y=.5;msg.pose.orientation.w=1.
    goal_pub.publish(msg)


try:
    wait(lambda:any(s['state']=='READY' for s in statuses))
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    goal(1.)
    wait(lambda:any(s['state']=='SUCCEEDED' for s in statuses) and paths and len(paths[-1].poses)==3)
    goal(100.)
    wait(lambda:any(s['state']=='FAILED' for s in statuses) and paths and not paths[-1].poses)
    # Leave a valid path visible for the framebuffer capture after testing failure clearing.
    successes=sum(s['state']=='SUCCEEDED' for s in statuses)
    goal(1.)
    wait(lambda:sum(s['state']=='SUCCEEDED' for s in statuses)>successes and paths and len(paths[-1].poses)==3)
    assert not commands and not navigation,(commands,navigation)
    assert len(requests)==3,len(requests)
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    print(json.dumps(dict(passed=True,planning_requests=len(requests),
        navigation_requests=len(navigation),drive_commands=len(commands),
        stale_path_cleared=True,physical_robot=False),indent=2),flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
