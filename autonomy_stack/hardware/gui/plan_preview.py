#!/usr/bin/env python3
"""RViz clicks request ComputePathToPose only; no drive commands or navigation goals."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class PlanPreview(Node):
    def __init__(self):
        super().__init__('autonomy_plan_preview')
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub=self.create_publisher(Path,'/autonomy/preview_path',latched)
        self.status_pub=self.create_publisher(String,'/autonomy/preview_status',latched)
        self.client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.subscription=self.create_subscription(PoseStamped,'/autonomy/preview_goal',self.goal,10)
        self.busy=False
        self.goal_handle=None
        self.cancel_pending=False
        self.deadline=0
        self.timer=self.create_timer(.2,self.check_timeout)
        self.status('READY',message='2D Goal Pose requests a path only; no motion commands.')

    def status(self,state,**values):
        msg=String(data=json.dumps(dict(state=state,**values)))
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def goal(self,pose):
        if self.busy:
            self.status('BUSY',message='Wait for the current planning request to finish.')
            return
        p,q=pose.pose.position,pose.pose.orientation
        if pose.header.frame_id!='map' or not all(math.isfinite(v) for v in
                (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:
            self.status('INVALID_GOAL',message='Goal must be a finite map pose with a unit quaternion.')
            return
        if not self.client.server_is_ready():
            self.status('PLANNER_UNAVAILABLE',message='Wait for planner_server to become active, then retry.')
            return
        # Drop an old path immediately so a failed goal never shows an old route as its result.
        empty=Path();empty.header.frame_id='map';self.path_pub.publish(empty)
        request=ComputePathToPose.Goal()
        request.goal=pose
        request.goal.header.stamp=self.get_clock().now().to_msg()
        request.planner_id='GridBased';request.use_start=False
        self.busy=True;self.cancel_pending=False;self.deadline=time.monotonic()+20
        self.status('PLANNING',x=p.x,y=p.y)
        self.client.send_goal_async(request).add_done_callback(self.accepted)

    def accepted(self,future):
        try:
            self.goal_handle=future.result()
            if not self.goal_handle.accepted:
                self.busy=False;self.goal_handle=None;self.status('REJECTED');return
            if self.cancel_pending:
                self.goal_handle.cancel_goal_async()
            self.goal_handle.get_result_async().add_done_callback(self.finished)
        except Exception as exc:
            self.busy=False;self.goal_handle=None;self.status('ERROR',message=str(exc))

    def finished(self,future):
        self.busy=False;self.goal_handle=None
        try:
            result=future.result()
            if self.cancel_pending:
                self.status('TIMEOUT',message='Planning exceeded 20 seconds; result discarded.')
                return
            data=result.result
            if result.status==GoalStatus.STATUS_SUCCEEDED and data.error_code==0 and data.path.poses:
                self.path_pub.publish(data.path)
                self.status('SUCCEEDED',poses=len(data.path.poses),error_code=int(data.error_code))
            else:
                self.status('FAILED',action_status=int(result.status),error_code=int(data.error_code),
                            message=data.error_msg)
        except Exception as exc:
            self.status('ERROR',message=str(exc))

    def check_timeout(self):
        if self.busy and not self.cancel_pending and time.monotonic()>self.deadline:
            self.cancel_pending=True
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.status('CANCELING',message='Planning exceeded 20 seconds; cancel requested.')


def main():
    rclpy.init()
    node=PlanPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
