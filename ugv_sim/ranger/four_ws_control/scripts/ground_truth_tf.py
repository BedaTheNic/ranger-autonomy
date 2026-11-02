#!/usr/bin/python3
"""Optional visualization TF for bare simulation. Disable before adding EKF."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GroundTruthTF(Node):
    def __init__(self):
        super().__init__('ground_truth_tf')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'ground_truth/odom', self.publish_tf,
                                 qos_profile_sensor_data)

    def publish_tf(self, msg):
        if msg.header.frame_id != 'world' or msg.child_frame_id != 'base_link':
            return
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
