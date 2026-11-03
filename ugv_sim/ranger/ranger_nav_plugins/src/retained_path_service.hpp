// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "route_quality.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace ranger_nav_plugins
{
class RetainedPathService
{
public:
  RetainedPathService(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map,double padding)
  : map_(std::move(map)), padding_(padding), logger_(node->get_logger())
  {
    service_=node->create_service<nav2_msgs::srv::IsPathValid>("is_retained_path_valid",
      [this](std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> req,
             std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> res) {
        res->is_valid=false;
        geometry_msgs::msg::PoseStamped pose;
        if(req->path.header.frame_id!=map_->getGlobalFrameID() || !map_->getRobotPose(pose)) {return;}
        auto * grid=map_->getCostmap();
        std::unique_lock<Grid::mutex_t> lock(*grid->getMutex());
        if(req->path!=previous_) {previous_=req->path; cursor_=0;}
        auto footprint=map_->getRobotFootprint();
        nav2_costmap_2d::padFootprint(footprint,padding_);
        Pose2 invalid;
        res->is_valid=retainedPathFree(points(req->path),planar(pose.pose),cursor_,*grid,footprint,&invalid);
        if(!res->is_valid && !req->path.poses.empty()) {
          const auto stamp=req->path.header.stamp;
          RCLCPP_INFO(logger_,"Retained path %d.%09u invalid at (%.3f, %.3f, %.3f), reserve %.3f m",
            stamp.sec,stamp.nanosec,invalid.x,invalid.y,invalid.a,padding_);
        }
      });
  }
private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  double padding_;
  rclcpp::Logger logger_;
  nav_msgs::msg::Path previous_;
  size_t cursor_{0};
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr service_;
};
}
