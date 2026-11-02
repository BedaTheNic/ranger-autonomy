#!/usr/bin/python3
"""Travel, speed, accumulated yaw and a bounded RViz path from filtered odometry."""
import math
from collections import deque
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


class OdometryMetrics(Node):
    def __init__(self):
        super().__init__('odometry_metrics')
        self.previous = None
        self.distance = self.rotation = 0.0
        self.last_path_stamp = -math.inf
        self.poses = deque(maxlen=3000)
        self.publishers_by_name = {n: self.create_publisher(Float64, 'odometry/'+n, 10)
                                   for n in ('distance', 'speed', 'yaw', 'rotation')}
        self.path_pub = self.create_publisher(Path, 'odometry/path', 1)
        self.create_subscription(Odometry, 'odometry/filtered', self.update, 20)

    def update(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        if not all(math.isfinite(x) for x in (stamp, yaw, speed)):
            return
        if self.previous is not None:
            old_stamp, old_speed, old_yaw = self.previous
            dt = stamp-old_stamp
            if dt == 0:
                return
            if dt < 0:
                self.distance = self.rotation = 0.0
                self.poses.clear()
                self.last_path_stamp = -math.inf
            elif dt <= .2:
                # Avoid counting sub-mm/s filter jitter as travelled distance.
                self.distance += .5*((speed if speed >= .001 else 0.0)+
                                      (old_speed if old_speed >= .001 else 0.0))*dt
                self.rotation += math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))
        self.previous = (stamp, speed, yaw)
        for name, value in dict(distance=self.distance, speed=speed, yaw=yaw,
                                rotation=self.rotation).items():
            self.publishers_by_name[name].publish(Float64(data=value))
        if stamp-self.last_path_stamp >= .2:
            pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
            self.poses.append(pose)
            self.path_pub.publish(Path(header=msg.header, poses=list(self.poses)))
            self.last_path_stamp = stamp


def main():
    rclpy.init()
    node = OdometryMetrics()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
