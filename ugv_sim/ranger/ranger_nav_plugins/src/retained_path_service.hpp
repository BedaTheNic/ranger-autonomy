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
        size_t blocked_index=0;
        res->is_valid=retainedPathFree(points(req->path),planar(pose.pose),cursor_,*grid,footprint,&invalid,&blocked_index);
        if(res->is_valid) {blocked_header_.reset();}
        if(!res->is_valid && !req->path.poses.empty()) {
          blocked_header_=req->path.header;
          res->invalid_pose_indices.push_back(static_cast<int32_t>(blocked_index));
          const auto stamp=req->path.header.stamp;
          BlockingCell cell;poseFree(invalid,*grid,footprint,&cell);
          blocked_cell_=cell;
          RCLCPP_INFO(logger_,"Retained path %d.%09u invalid at (%.3f, %.3f, %.3f), reserve %.3f m; index %zu; %s cell (%.3f, %.3f) cost %d",
            stamp.sec,stamp.nanosec,invalid.x,invalid.y,invalid.a,padding_,blocked_index,cell.part,cell.x,cell.y,cell.cost);
        }
      });
  }
  // Both the validation callback and planner access this state under the
  // same costmap mutex.
  bool replacesBlockedPath(const std_msgs::msg::Header & header) const {
    return blocked_header_ && *blocked_header_==header;
  }
  const BlockingCell & blockingCell() const {return blocked_cell_;}
private:
  BlockingCell blocked_cell_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  double padding_;
  rclcpp::Logger logger_;
  nav_msgs::msg::Path previous_;
  std::optional<std_msgs::msg::Header> blocked_header_;
  size_t cursor_{0};
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr service_;
};
}
