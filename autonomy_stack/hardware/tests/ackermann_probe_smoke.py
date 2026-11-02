"""Exercise the passive probe with real SocketCAN on vcan and synthetic ROS topics."""
import importlib.util
import math
from pathlib import Path
import socket
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from ranger_msgs.msg import ActuatorState, ActuatorStateArray, MotionState


assert {name for _,name in socket.if_nameindex()} <= {'lo'}, 'Requires an empty test network namespace'


def attribute(kind,payload):
    size=4+len(payload)
    return struct.pack('=HH',size,kind)+payload+b'\0'*(-size%4)


payload=struct.pack('=BBHiII',0,0,0,0,1,0xffffffff)+attribute(3,b'vcan_ack\0')
payload+=attribute(18|0x8000,attribute(1,b'vcan\0'))
with socket.socket(socket.AF_NETLINK,socket.SOCK_RAW,socket.NETLINK_ROUTE) as link:
    link.bind((0,0));link.settimeout(2)
    link.sendto(struct.pack('=IHHII',16+len(payload),16,0x605,1,0)+payload,(0,0))
    ack=link.recv(4096)
    assert struct.unpack_from('=i',ack,16)[0]==0,'Virtual CAN creation failed'

spec=importlib.util.spec_from_file_location('ack_probe',Path(__file__).resolve().parents[1]/'ackermann_diagnostics.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
assert module.expected_mode(.05,0,.025)['mode_code']==0
assert module.expected_mode(0,.05,0)['mode_code']==1
assert module.expected_mode(0,0,.1)['mode_code']==2
assert module.expected_mode(.05,1e-18,.025)['mode_code']==1,'Match the actual driver exact-y comparison'
assert module.expected_mode(0,0,0)['mode']=='stationary_hold'

rclpy.init();probe=module.AckermannProbe();fixture=rclpy.create_node('ack_probe_fixture')
publisher=fixture.create_publisher(ActuatorStateArray,'/ranger/actuator_state',10)
mode_pub=fixture.create_publisher(MotionState,'/motion_state',10)
cmd_pub=fixture.create_publisher(Twist,'/cmd_vel',10)
can=module.CanProbe('vcan_ack')
sender=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW);sender.bind(('vcan_ack',))
try:
    deadline=time.monotonic()+1.5;iteration=0
    while time.monotonic()<deadline:
        offset=iteration%2
        raw=[100+offset*10,-200+offset*10,300+offset*10,-400+offset*10]
        data=struct.pack('>hhhh',*raw)
        sender.send(struct.pack('=IB3x8s',0x271,8,data))
        sender.send(struct.pack('=IB3x8s',0x291,8,b'\0\0'+b'\0'*6))
        sender.send(struct.pack('=IB3x8s',0x255,8,struct.pack('>hhi',-123,24,-682634)))
        sender.send(struct.pack('=IB3x8s',0x111,8,b'\0'*8))
        states=[ActuatorState(id=i) for i in range(8)]
        for i,value in enumerate(raw):states[i+4].motor.motor_angles=value*.001
        publisher.publish(ActuatorStateArray(states=states))
        mode_pub.publish(MotionState(motion_mode=0))
        cmd=Twist();cmd.linear.x=.05;cmd.angular.z=.025;cmd_pub.publish(cmd)
        rclpy.spin_once(probe,timeout_sec=.01);can.drain();iteration+=1
    report=probe.report();frames=can.report()['frames']
    assert can.error is None,can.error
    assert set(frames)=={'0x271','0x291','0x255'},frames
    assert frames['0x255']['last_decoded']['pulse_count']==-682634
    assert frames['0x255']['last_decoded']['rpm']==-123
    assert math.isclose(frames['0x271']['ranges']['angles_rad.5']['min'],.1)
    assert math.isclose(frames['0x271']['ranges']['angles_rad.5']['max'],.11)
    assert report['topics']['/ranger/actuator_state']['samples']>1,report
    assert report['topics']['/cmd_vel']['last_expected_driver_mode']['mode_code']==0,report
    assert not any(p.topic_name in ('/cmd_vel','/cmd_vel_nav') for p in probe.publishers)
    print('PASS: passive ROS/CAN probe, signed big-endian angle/encoder decoding, mode selection and command-frame exclusion')
finally:
    sender.close();can.close();probe.destroy_node();fixture.destroy_node();rclpy.shutdown()
