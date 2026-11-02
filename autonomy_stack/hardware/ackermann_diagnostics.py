#!/usr/bin/env python3
"""Passively compare Ranger ROS actuator feedback with standard AgileX V2 CAN feedback."""
import argparse
import json
import math
import os
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import ActuatorStateArray, MotionState, SystemState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def observe(values,key,value):
    if not math.isfinite(value):
        values.setdefault('invalid_fields',[])
        if key not in values['invalid_fields']:values['invalid_fields'].append(key)
        return
    item=values.setdefault(key,dict(first=value,last=value,min=value,max=value))
    item.update(last=value,min=min(item['min'],value),max=max(item['max'],value))


def decode_feedback(can_id,data):
    """SDK V2 wire order is big endian, independent of host byte order."""
    if can_id==0x271 and len(data)==8:
        return {'angles_rad':{str(i+5):v*.001 for i,v in enumerate(struct.unpack('>hhhh',data))}}
    if can_id==0x291 and len(data)>=2:
        return dict(motion_mode=data[0],mode_changing=data[1])
    if can_id==0x221 and len(data)==8:
        x,w,y,angle=struct.unpack('>hhhh',data)
        return dict(linear_x=x*.001,angular_z=w*.001,linear_y=y*.001,steering_angle_rad=angle*.001)
    if 0x251<=can_id<=0x258 and len(data)==8:
        rpm,current,pulses=struct.unpack('>hhi',data)
        return dict(motor_number=can_id-0x250,rpm=rpm,current_a=current*.1,pulse_count=pulses)
    return None


def expected_mode(x,y,w):
    if not all(math.isfinite(v) for v in (x,y,w)):return dict(mode='invalid')
    if y!=0:return dict(mode='parallel',mode_code=1)
    if x==0 and w==0:return dict(mode='stationary_hold')
    if w==0:return dict(mode='dual_ackermann',mode_code=0,radius_m=None)
    radius=abs(x/w)
    return dict(mode='spinning' if radius<.47644 else 'dual_ackermann',
                mode_code=2 if radius<.47644 else 0,radius_m=radius)


class AckermannProbe(Node):
    def __init__(self):
        super().__init__('ranger_hero_ackermann_diagnostics',enable_rosout=False)
        self.started=time.monotonic();self.topics={};self.last_received={}
        definitions=[(ActuatorStateArray,'/actuator_state'),(ActuatorStateArray,'/ranger/actuator_state'),
                     (MotionState,'/motion_state'),(SystemState,'/system_state'),(Odometry,'/odom'),
                     (JointState,os.environ.get('STEERING_FEEDBACK_TOPIC','/joint_states')),
                     (Twist,'/cmd_vel'),(Twist,'/cmd_vel_nav')]
        self.ack_subscriptions=[self.create_subscription(typ,topic,
            lambda msg,topic=topic:self.receive(topic,msg),qos_profile_sensor_data) for typ,topic in definitions]

    def receive(self,topic,msg):
        entry=self.topics.setdefault(topic,dict(samples=0))
        entry['samples']+=1;self.last_received[topic]=time.monotonic()
        if isinstance(msg,ActuatorStateArray):
            states=entry.setdefault('by_id',{})
            ids=[]
            for state in msg.states:
                ids.append(state.id)
                item=states.setdefault(str(state.id),{})
                for name,value in [('angle_rad',state.motor.motor_angles),('pulse_count',state.motor.pulse_count),
                                   ('rpm',state.motor.rpm),('current_a',state.motor.current)]:
                    observe(item,name,value)
            entry['last_ids']=ids;entry['duplicate_ids']=len(ids)!=len(set(ids))
            entry['last_stamp_sec']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        elif isinstance(msg,MotionState):
            entry.setdefault('motion_modes',{})[str(msg.motion_mode)]=entry.get('motion_modes',{}).get(str(msg.motion_mode),0)+1
        elif isinstance(msg,SystemState):
            entry['last']=dict(vehicle_state=msg.vehicle_state,control_mode=msg.control_mode,
                               error_code=msg.error_code,motion_mode=msg.motion_mode)
        elif isinstance(msg,JointState):
            entry['last_names']=list(msg.name)
            for name,value in zip(msg.name,msg.position):observe(entry.setdefault('positions_rad',{}),name,value)
        else:
            velocity=msg.twist.twist if isinstance(msg,Odometry) else msg
            for name,value in [('linear_x',velocity.linear.x),('linear_y',velocity.linear.y),('angular_z',velocity.angular.z)]:
                observe(entry.setdefault('velocity',{}),name,value)
            if isinstance(msg,Twist):
                mode=expected_mode(velocity.linear.x,velocity.linear.y,velocity.angular.z)
                entry['last_expected_driver_mode']=mode
                counts=entry.setdefault('expected_driver_mode_counts',{})
                counts[mode['mode']]=counts.get(mode['mode'],0)+1

    def report(self):
        now=time.monotonic()
        for topic,entry in self.topics.items():
            entry['last_received_age_s']=round(now-self.last_received[topic],4)
            entry['received_hz_approx']=round(entry['samples']/(now-self.started),2)
        publishers={topic:[dict(node=p.node_namespace.rstrip('/')+'/'+p.node_name,type=p.topic_type)
                           for p in self.get_publishers_info_by_topic(topic)] for topic in
                    ('/actuator_state','/ranger/actuator_state','/cmd_vel','/cmd_vel_nav','/odom')}
        return dict(topics=self.topics,publishers=publishers)


class CanProbe:
    def __init__(self,interface):
        self.started=time.monotonic();self.frames={};self.error=None;self.receiver=None;self.interface=interface
        try:
            self.receiver=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            # CAN_ERR_FLAG must NOT be in the mask: it selects error-only reception.
            mask=0x700|socket.CAN_EFF_FLAG|socket.CAN_RTR_FLAG
            self.receiver.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,struct.pack('=II',0x200,mask))
            self.receiver.bind((interface,));self.receiver.setblocking(False)
        except (OSError,AttributeError) as exc:
            self.error=f'{type(exc).__name__}: {exc}'
            self.close()

    def drain(self):
        if self.receiver is None:return
        for _ in range(256):
            try:frame=self.receiver.recv(72)
            except BlockingIOError:return
            except OSError as exc:
                self.error=f'{type(exc).__name__}: {exc}';self.close();return
            self.accept(frame)

    def accept(self,frame):
        if len(frame)!=16:return
        can_id,length,data=struct.unpack('=IB3x8s',frame)
        if can_id&~0x7ff or not 0x200<=can_id<=0x2ff or length>8:return
        data=data[:length]
        item=self.frames.setdefault(f'0x{can_id:03x}',dict(samples=0,examples_hex=[]))
        item['samples']+=1;item['last_received_s']=round(time.monotonic()-self.started,4)
        item['last_hex']=data.hex()
        if data.hex() not in item['examples_hex'] and len(item['examples_hex'])<6:item['examples_hex'].append(data.hex())
        decoded=decode_feedback(can_id,data)
        if decoded is not None:
            item['last_decoded']=decoded
            for key,value in decoded.items():
                if isinstance(value,dict):
                    for name,angle in value.items():observe(item.setdefault('ranges',{}),key+'.'+name,angle)
                else:observe(item.setdefault('ranges',{}),key,value)

    def report(self):
        return dict(interface=self.interface,error=self.error,frames=self.frames,
                    angle_frame_0x271_seen='0x271' in self.frames,
                    mode_frame_0x291_seen='0x291' in self.frames)

    def close(self):
        if self.receiver is not None:self.receiver.close();self.receiver=None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=20)
    parser.add_argument('--can-interface',default='can0')
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 1<=args.seconds<=120:parser.error('--seconds must be between 1 and 120')
    rclpy.init();node=AckermannProbe();can=CanProbe(args.can_interface)
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01);can.drain()
        report=dict(schema=1,commands_published=0,ros_domain=os.environ.get('ROS_DOMAIN_ID'),
                    capture_seconds=args.seconds,
                    configured_driver_geometry_m=dict(wheelbase=.494,track=.364,min_turn_radius=.47644),
                    ros=node.report(),can=can.report(),
                    notes=['Passive probe: no velocity, mode, joint or CAN commands are sent.',
                           'An unchanged measured angle is valid; a repeated ROS header alone does not prove new CAN feedback.',
                           'CAN 0x271 uses signed big-endian int16 values in 0.001 rad for the configured Mini V3 SDK.',
                           'CAN motor channels 5..8 are not a verified physical wheel/sign calibration.',
                           'Encoder pulse counts are reported without converting them into angles.',
                           'Expected driver modes are inferred from Twist using the observed V3 configuration.',
                           'The driver tests linear.y != 0 exactly; a tiny nonzero lateral command selects parallel mode.',
                           'No varying angles in a stationary capture does not prove that angle feedback is missing.'])
        print(json.dumps(report,indent=2,allow_nan=False))
    finally:
        can.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
