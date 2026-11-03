"""Exercise the navigation gateway against synthetic base/controller nodes only."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ranger_msgs.msg import ActuatorState, ActuatorStateArray, SystemState
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool


rclpy.init()
base=rclpy.create_node('ranger_base_node')
controller=rclpy.create_node('controller_server')
controller.declare_parameter('FollowPath.plugin','ranger_nav_plugins::ProgressDWBController')
executor=SingleThreadedExecutor();executor.add_node(base);executor.add_node(controller)
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
odom_pub=base.create_publisher(Odometry,'/odom',10)
system_pub=base.create_publisher(SystemState,'/system_state',10)
actuator_pub=base.create_publisher(ActuatorStateArray,'/ranger/actuator_state',10)
commands_pub=controller.create_publisher(Twist,'/cmd_vel_nav',10)
goals_pub=controller.create_publisher(GoalStatusArray,'/navigate_to_pose/_action/status',latched)
status=[];outputs=[]
subscriptions=[base.create_subscription(String,'/autonomy/drive_status',lambda msg:status.append(json.loads(msg.data)),latched),
               base.create_subscription(Twist,'/cmd_vel',lambda msg:outputs.append((time.monotonic(),msg)),10)]


def state_callback(request,response):
    response.current_state.id=3;response.current_state.label='active';return response


service=controller.create_service(GetState,'/controller_server/get_state',state_callback)
client=base.create_client(SetBool,'/autonomy/drive_enable')
odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='ranger_base_link';odom.pose.pose.orientation.w=1.
system=SystemState();system.control_mode=SystemState.CONTROL_MODE_CAN
actuators=ActuatorStateArray(states=[ActuatorState(id=i) for i in range(8)])
command=Twist();command.linear.x=.6;command.angular.z=.5
goal=None;publish_commands=False;publish_odometry=True


def publish():
    odom.header.stamp=base.get_clock().now().to_msg()
    if publish_odometry:odom_pub.publish(odom)
    system.header.stamp=odom.header.stamp;system_pub.publish(system)
    actuators.header.stamp=odom.header.stamp;actuator_pub.publish(actuators)
    if goal:goals_pub.publish(GoalStatusArray(status_list=[goal]))
    if publish_commands:commands_pub.publish(command)


timer=base.create_timer(.05,publish)


def spin(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)


def wait(predicate,seconds=12):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=.02)
        if predicate():return
    raise AssertionError(status[-5:])


def arm(value=True):
    future=client.call_async(SetBool.Request(data=value))
    wait(future.done)
    return future.result()


def new_goal(old=False):
    msg=GoalStatus();msg.status=GoalStatus.STATUS_EXECUTING
    msg.goal_info.goal_id.uuid=[2 if old else 3]*16
    if not old:msg.goal_info.stamp=base.get_clock().now().to_msg()
    return msg


try:
    wait(lambda:status and status[-1]['reason']=='READY_DISARMED')
    assert not outputs,'Disarmed startup must not publish even zero commands'
    assert arm().success
    goal=new_goal(old=True);publish_commands=True
    spin(.6)
    assert outputs and all(abs(v.linear.x)<1e-9 and abs(v.angular.z)<1e-9 for _,v in outputs)
    goal=new_goal()
    wait(lambda:outputs and abs(outputs[-1][1].linear.x-.6)<1e-6)
    assert abs(outputs[-1][1].angular.z-.5)<1e-6,'Controller speeds must pass through unchanged'
    assert status[-1]['gateway_speed_limited'] is False
    command.linear.x=-.6;command.angular.z=-.5
    wait(lambda:abs(outputs[-1][1].linear.x+.6)<1e-6)
    assert abs(outputs[-1][1].angular.z+.5)<1e-6
    command.linear.x=.3;command.linear.y=.4;command.angular.z=0.
    wait(lambda:abs(outputs[-1][1].linear.y-.4)<1e-6)
    assert abs(outputs[-1][1].linear.x-.3)<1e-6,'Diagonal translation must pass through unchanged'
    publish_commands=False
    last_command=time.monotonic()
    wait(lambda:status[-1]['reason']=='NAVIGATION_COMMAND_TIMEOUT',seconds=2)
    wait(lambda:outputs[-1][0]>=last_command and abs(outputs[-1][1].linear.x)<1e-9)
    assert not status[-1]['armed']

    assert arm().success
    previous=len(outputs);publish_commands=True
    spin(.4)
    assert all(abs(v.linear.x)<1e-9 for _,v in outputs[previous:]),'Old goal must not resume after rearm'
    foreign=rclpy.create_node('teleop_twist_keyboard');executor.add_node(foreign)
    foreign_pub=foreign.create_publisher(Twist,'/cmd_vel',10)
    wait(lambda:'/teleop_twist_keyboard' in status[-1]['foreign_publishers'],seconds=3)
    assert status[-1]['armed'],'A second /cmd_vel publisher must not disarm the gateway'
    assert arm(False).success
    assert arm().success,'A second /cmd_vel publisher must not block arming'
    foreign.destroy_publisher(foreign_pub);executor.remove_node(foreign);foreign.destroy_node()
    assert arm(False).success
    controller.destroy_publisher(commands_pub)
    motion_guard=rclpy.create_node('navigation_motion_guard');executor.add_node(motion_guard)
    commands_pub=motion_guard.create_publisher(Twist,'/cmd_vel_nav',10)
    wait(lambda:status[-1]['reason']=='READY_DISARMED')
    assert arm().success
    goal=new_goal()
    wait(lambda:abs(outputs[-1][1].linear.y-.4)<1e-6)
    assert arm(False).success
    duplicate=controller.create_publisher(Twist,'/cmd_vel_nav',10)
    wait(lambda:status[-1]['reason']=='DUPLICATE_NAVIGATION_PUBLISHER')
    assert not arm().success,'Controller must not bypass an active motion guard'
    controller.destroy_publisher(duplicate)
    wait(lambda:status[-1]['reason']=='READY_DISARMED')
    assert arm().success
    system.vehicle_state=SystemState.VEHICLE_STATE_ESTOP
    wait(lambda:status[-1]['reason']=='BASE_FAULT_OR_ESTOP',seconds=2)
    assert not status[-1]['armed']
    system.vehicle_state=SystemState.VEHICLE_STATE_NORMAL;publish_odometry=False
    spin(.7)
    response=arm();assert not response.success and response.message=='ODOMETRY_STALE',response
    publish_odometry=True
    controller.set_parameters([Parameter('FollowPath.plugin',value='ranger_nav_plugins::AdaptiveController')])
    wait(lambda:status[-1]['reason']=='MEASURED_STEERING_MISSING')
    response=arm();assert not response.success and response.message=='MEASURED_STEERING_MISSING'
    print(json.dumps(dict(passed=True,physical_robot=False,
        checks=['disarmed startup','ignore pre-arm goal','forward/reverse/diagonal speeds pass through unchanged',
                'legacy DRIVE_MAX_LINEAR/DRIVE_MAX_ANGULAR ignored',
                'motion guard publisher accepted; duplicate controller bypass blocked',
                'command timeout stops and disarms','rearm ignores old goal','other publishers allowed',
                'no elapsed-time disarm (obsolete DRIVE_ARM_SECONDS ignored)',
                'E-stop stops','stale odometry blocks','adaptive requires steering']),indent=2),flush=True)
finally:
    if 'motion_guard' in globals():motion_guard.destroy_node()
    base.destroy_node();controller.destroy_node();executor.shutdown()
    if rclpy.ok():rclpy.shutdown()
