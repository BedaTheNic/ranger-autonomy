#!/usr/bin/env python3
"""Limited commissioning gateway. Starts disarmed; requires exclusive /cmd_vel ownership."""
import json
import math
import os
import signal
import sys
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from ranger_msgs.msg import ActuatorStateArray, SystemState
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool


class DriveGuard(Node):
    def __init__(self):
        super().__init__('autonomy_drive_guard_'+uuid.uuid4().hex[:8])
        self.max_linear=float(os.environ.get('DRIVE_MAX_LINEAR','0.05'))
        self.max_angular=float(os.environ.get('DRIVE_MAX_ANGULAR','0.10'))
        self.command_timeout=float(os.environ.get('DRIVE_COMMAND_TIMEOUT','0.30'))
        self.arm_duration=float(os.environ.get('DRIVE_ARM_SECONDS','60'))
        if not all(math.isfinite(v) and v>0 for v in
                   (self.max_linear,self.max_angular,self.command_timeout,self.arm_duration)):
            raise ValueError('Drive limits/timeouts must be positive and finite')
        if self.max_linear>.10 or self.max_angular>.20 or self.arm_duration>120:
            raise ValueError('Commissioning caps: 0.10 m/s, 0.20 rad/s and 120 seconds')
        self.started=time.monotonic();self.last_graph=0;self.graph_ready_since=0
        self.armed=False;self.stopping=False;self.arm_until=0;self.arm_stamp=0
        self.goal_key=None;self.command=None;self.command_received=0
        self.odom=None;self.odom_received=0;self.system=None;self.system_received=0
        self.joints=None;self.joints_received=0;self.controller_active=False
        self.controller_stamp=0;self.plugin='';self.parameters_stamp=0;self.sim_time=True
        self.foreign_publishers=[];self.input_source_issue='NAVIGATION_PUBLISHERS_NOT_DISCOVERED'
        self.last_actuator_log=0;self.state_future=None;self.params_future=None
        self.last_reason=None;self.last_status=0
        self.output=self.create_publisher(Twist,'/cmd_vel',10)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,'/autonomy/drive_status',latched)
        self.drive_subscriptions=[
            self.create_subscription(Twist,'/cmd_vel_nav',self.on_command,10),
            self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data),
            self.create_subscription(SystemState,'/system_state',self.on_system,qos_profile_sensor_data),
            self.create_subscription(JointState,os.environ.get('STEERING_FEEDBACK_TOPIC','/joint_states'),
                                     self.on_joints,qos_profile_sensor_data),
            self.create_subscription(ActuatorStateArray,'/ranger/actuator_state',self.on_actuators,qos_profile_sensor_data),
            self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.on_goals,latched)]
        self.state_client=self.create_client(GetState,'/controller_server/get_state')
        self.params_client=self.create_client(GetParameters,'/controller_server/get_parameters')
        self.arm_service=self.create_service(SetBool,'/autonomy/drive_enable',self.enable)
        self.poll_timer=self.create_timer(.25,self.poll)
        self.output_timer=self.create_timer(.02,self.tick)
        self.publish_status('DISARMED')

    def on_odom(self,msg):
        self.odom=msg;self.odom_received=time.monotonic()

    def on_system(self,msg):
        self.system=msg;self.system_received=time.monotonic()

    def on_joints(self,msg):
        self.joints=msg;self.joints_received=time.monotonic()

    def on_actuators(self,msg):
        now=time.monotonic()
        if now-self.last_actuator_log<.2:return
        self.last_actuator_log=now
        # Retain actual samples, including unchanged angles. No inferred zero or encoder conversion.
        self.get_logger().info('ACTUATOR_SAMPLE '+json.dumps(dict(topic='/ranger/actuator_state',
            stamp_sec=msg.header.stamp.sec,stamp_nanosec=msg.header.stamp.nanosec,
            states=[dict(id=s.id,angle=s.motor.motor_angles,pulse_count=s.motor.pulse_count,
                         rpm=s.motor.rpm) for s in msg.states])))

    def on_command(self,msg):
        # Jazzy's Python MessageInfo exposes timestamps, not the publisher GID.
        # Qualify the topic's complete publisher set in poll(); this is not DDS authentication.
        if not self.armed or self.goal_key is None or self.input_source_issue:
            return
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(abs(v)>1e-6 for v in
                (msg.linear.z,msg.angular.x,msg.angular.y)):
            self.disarm('INVALID_COMMAND');return
        if abs(msg.linear.y)>1e-6 and abs(msg.angular.z)>1e-6:
            self.disarm('UNSUPPORTED_LATERAL_AND_ROTATION');return
        if abs(msg.linear.x)>1e-6 and abs(msg.angular.z)>1e-6 and abs(msg.linear.x/msg.angular.z)<.5:
            self.disarm('TURN_RADIUS_BELOW_0_5_M');return
        self.command=msg;self.command_received=time.monotonic()

    def on_goals(self,msg):
        if not self.armed:return
        eligible=[s for s in msg.status_list if s.status==GoalStatus.STATUS_EXECUTING
                  and s.goal_info.stamp.sec*1000000000+s.goal_info.stamp.nanosec>=self.arm_stamp]
        key=bytes(eligible[-1].goal_info.goal_id.uuid) if eligible else None
        if key!=self.goal_key:
            self.goal_key=key;self.command=None;self.command_received=0

    def on_state(self,future):
        try:
            self.controller_active=future.result().current_state.id==3
            self.controller_stamp=time.monotonic()
        except Exception:
            self.controller_active=False

    def on_parameters(self,future):
        try:
            values=future.result().values
            self.plugin=values[0].string_value if values[0].type==ParameterType.PARAMETER_STRING else ''
            self.sim_time=values[1].type!=ParameterType.PARAMETER_BOOL or values[1].bool_value
            self.parameters_stamp=time.monotonic()
        except Exception:
            self.plugin=''

    def poll(self):
        now=time.monotonic()
        publishers=self.get_publishers_info_by_topic('/cmd_vel')
        own=lambda p:p.node_name==self.get_name() and p.node_namespace==self.get_namespace()
        self.foreign_publishers=[p.node_namespace.rstrip('/')+'/'+p.node_name
                                 for p in publishers if not own(p)]
        self.last_graph=now
        if len(publishers)==1 and own(publishers[0]):
            if not self.graph_ready_since:self.graph_ready_since=now
        else:self.graph_ready_since=0
        inputs=self.get_publishers_info_by_topic('/cmd_vel_nav')
        allowed=('controller_server','behavior_server')
        counts={name:sum(p.node_name==name and p.node_namespace=='/' for p in inputs) for name in allowed}
        if any(p.node_namespace!='/' or p.node_name not in allowed or p.topic_type!='geometry_msgs/msg/Twist' for p in inputs):
            self.input_source_issue='UNEXPECTED_NAVIGATION_PUBLISHER'
        elif any(count>1 for count in counts.values()):
            self.input_source_issue='DUPLICATE_NAVIGATION_PUBLISHER'
        else:self.input_source_issue=None if counts['controller_server']==1 else 'NAVIGATION_PUBLISHERS_NOT_DISCOVERED'
        if self.state_client.service_is_ready() and (self.state_future is None or self.state_future.done()):
            self.state_future=self.state_client.call_async(GetState.Request())
            self.state_future.add_done_callback(self.on_state)
        if self.params_client.service_is_ready() and (self.params_future is None or self.params_future.done()):
            request=GetParameters.Request(names=['FollowPath.plugin','use_sim_time'])
            self.params_future=self.params_client.call_async(request)
            self.params_future.add_done_callback(self.on_parameters)

    def blocked_reason(self,arming=False):
        now=time.monotonic()
        if self.stopping:return 'STOPPING'
        if self.foreign_publishers:return 'OTHER_CMD_VEL_PUBLISHERS: '+', '.join(self.foreign_publishers)
        if not self.graph_ready_since or now-self.graph_ready_since<2 or now-self.last_graph>.6:
            return 'WAITING_FOR_EXCLUSIVE_CMD_VEL_DISCOVERY'
        if self.input_source_issue:return self.input_source_issue
        if not self.controller_active or now-self.controller_stamp>1.5:return 'CONTROLLER_NOT_ACTIVE'
        if not self.plugin or now-self.parameters_stamp>1.5 or self.sim_time:return 'CONTROLLER_CONFIGURATION_UNAVAILABLE'
        if not self.odom or now-self.odom_received>.5:return 'ODOMETRY_STALE'
        if self.odom.header.frame_id!='odom' or self.odom.child_frame_id!='ranger_base_link':return 'ODOMETRY_FRAMES_INVALID'
        v=self.odom.twist.twist
        if not all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z)):return 'ODOMETRY_INVALID'
        if arming and (math.hypot(v.linear.x,v.linear.y)>.03 or abs(v.angular.z)>.05):return 'BASE_NOT_STATIONARY'
        if not self.system or now-self.system_received>.5:return 'SYSTEM_STATE_STALE'
        if self.system.vehicle_state!=SystemState.VEHICLE_STATE_NORMAL or self.system.error_code:
            return 'BASE_FAULT_OR_ESTOP'
        if self.system.control_mode!=SystemState.CONTROL_MODE_CAN:return 'BASE_NOT_IN_CAN_CONTROL'
        if self.plugin in ('ranger_nav_plugins::AdaptiveController','ranger_nav_plugins::AckermannController'):
            names={'fl_steering_wheel','fr_steering_wheel','rl_steering_wheel','rr_steering_wheel'}
            msg=self.joints
            if not msg or now-self.joints_received>.5 or not names.issubset(msg.name):return 'MEASURED_STEERING_MISSING'
            if len(msg.position)!=len(msg.name) or not all(math.isfinite(msg.position[msg.name.index(n)]) for n in names):
                return 'MEASURED_STEERING_INVALID'
        elif self.plugin!='ranger_nav_plugins::ProgressDWBController':return 'UNSUPPORTED_CONTROLLER_PLUGIN'
        return None

    def enable(self,request,response):
        if not request.data:
            self.disarm('USER_DISABLED');response.success=True;response.message='DISARMED';return response
        if self.armed:
            response.success=False;response.message='Already armed; disable before starting another timed test';return response
        reason=self.blocked_reason(arming=True)
        if reason:
            response.success=False;response.message=reason;self.publish_status(reason);return response
        self.armed=True;self.arm_until=time.monotonic()+self.arm_duration
        self.arm_stamp=self.get_clock().now().nanoseconds
        self.goal_key=None;self.command=None;self.command_received=0
        response.success=True;response.message='ARMED: send a NEW Nav2 Goal; automatic disarm after '+str(self.arm_duration)+' seconds'
        self.publish_status('ARMED_WAITING_FOR_NEW_GOAL');return response

    def disarm(self,reason):
        was_armed=self.armed;self.armed=False;self.command=None;self.command_received=0;self.goal_key=None
        # Do not publish on startup/disarmed alongside other applications.
        if was_armed:self.output.publish(Twist())
        self.publish_status(reason)

    def publish_status(self,reason):
        now=time.monotonic()
        if reason==self.last_reason and now-self.last_status<1:return
        data=dict(armed=self.armed,reason=reason,controller=self.plugin,
                  foreign_publishers=self.foreign_publishers,max_linear=self.max_linear,max_angular=self.max_angular)
        self.status_pub.publish(String(data=json.dumps(data)))
        if reason!=self.last_reason:self.get_logger().info(json.dumps(data))
        self.last_reason=reason;self.last_status=now

    def tick(self):
        reason=self.blocked_reason()
        if not self.armed:
            self.publish_status(reason or 'READY_DISARMED');return
        if reason:self.disarm(reason);return
        now=time.monotonic()
        if now>=self.arm_until:self.disarm('ARM_TIME_EXPIRED');return
        if self.command_received and now-self.command_received>self.command_timeout:
            self.disarm('NAVIGATION_COMMAND_TIMEOUT');return
        output=Twist()
        if self.goal_key is not None and self.command is not None:
            v=self.command
            scale=min(1.,self.max_linear/max(math.hypot(v.linear.x,v.linear.y),1e-12),
                      self.max_angular/max(abs(v.angular.z),1e-12))
            output.linear.x=v.linear.x*scale;output.linear.y=v.linear.y*scale;output.angular.z=v.angular.z*scale
        self.output.publish(output)
        self.publish_status('DRIVING' if self.command is not None else 'ARMED_WAITING_FOR_NEW_GOAL')


def main():
    if len(sys.argv)>1 and sys.argv[1] in ('arm','disarm'):
        rclpy.init()
        node=rclpy.create_node('drive_guard_command_'+uuid.uuid4().hex[:8])
        try:
            client=node.create_client(SetBool,'/autonomy/drive_enable')
            if not client.wait_for_service(timeout_sec=8):
                print('Drive guard service unavailable');return 1
            future=client.call_async(SetBool.Request(data=sys.argv[1]=='arm'))
            rclpy.spin_until_future_complete(node,future,timeout_sec=8)
            if not future.done():print('Drive guard request timed out; inspect drive.sh logs');return 1
            response=future.result()
            print(json.dumps(dict(success=response.success,message=response.message)))
            return 0 if response.success else 1
        finally:
            node.destroy_node();rclpy.shutdown()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopping=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:stopping.set())
    signal.signal(signal.SIGTERM,lambda *_:stopping.set())
    node=DriveGuard()
    try:
        while not stopping.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.stopping=True;node.output_timer.cancel()
        was_armed=node.armed
        node.disarm('PROCESS_STOPPING')
        if was_armed:
            for _ in range(5):
                node.output.publish(Twist());rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
