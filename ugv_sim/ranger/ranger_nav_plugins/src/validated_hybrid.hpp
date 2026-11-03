// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "retained_path_service.hpp"
#include "nav2_smac_planner/smoother.hpp"
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

namespace ranger_nav_plugins
{
// Load native Hybrid in pluginlib's separate library scope. Keep the raw
// search result and smooth a copy, once, without repeating the A* search.
class ValidatedHybrid : public nav2_core::GlobalPlanner
{
public:
  bool expose_validity{true};
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    map_=map;
#ifdef RANGER_NAV2_JAZZY
    native_=loader_.createSharedInstance("nav2_smac_planner::SmacPlannerHybrid");
#else
    native_=loader_.createSharedInstance("nav2_smac_planner/SmacPlannerHybrid");
#endif
    native_->configure(parent,name,tf,map);
    auto node=parent.lock();
    if(node->get_parameter(name+".smooth_path").as_bool()) {
      throw std::runtime_error("ValidatedHybrid requires native smooth_path=false; it smooths its own copy");
    }
    nav2_smac_planner::SmootherParams params;
    params.get(node,name);
    smoother_=std::make_unique<nav2_smac_planner::Smoother>(params);
    smoother_->initialize(node->get_parameter(name+".minimum_turning_radius").as_double());
    if(expose_validity) {validity_=std::make_unique<RetainedPathService>(node,map_,padding_);}
    max_time_=node->get_parameter(name+".max_planning_time").as_double();
  }
  void activate() override {native_->activate();}
  void deactivate() override {native_->deactivate();}
  void cleanup() override {validity_.reset(); native_->cleanup(); native_.reset(); smoother_.reset();}
  void setLimits(const MotionLimits & limits,double padding) {limits_=limits; padding_=padding;}
  std::string variant;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
#ifdef RANGER_NAV2_JAZZY
    , std::function<bool()> cancel
#endif
    ) override
  {
    variant="none";
    auto * grid=map_->getCostmap();
    std::unique_lock<Grid::mutex_t> lock(*grid->getMutex());
    const auto begun=std::chrono::steady_clock::now();
    auto raw=native_->createPlan(start,goal
#ifdef RANGER_NAV2_JAZZY
      ,cancel
#endif
    );
    auto footprint=map_->getRobotFootprint();
    nav2_costmap_2d::padFootprint(footprint,padding_);
    auto smoothed=raw;
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begun).count();
    if(raw.poses.size()<3 || max_time_<=elapsed || !smoother_->smooth(smoothed,grid,max_time_-elapsed)) {
      smoothed.poses.clear();
    }
    switch(selectHybrid(points(raw),points(smoothed),*grid,footprint,limits_,padding_>0)) {
      case HybridChoice::raw: variant="raw";return raw;
      case HybridChoice::smoothed: variant="smoothed";return smoothed;
      case HybridChoice::none: break;
    }
    auto reason=[&](const nav_msgs::msg::Path & trajectory) {
      auto p=points(trajectory);Pose2 invalid;
      if(p.empty()) {return std::string("empty");}
      if(!pathFree(p,*grid,footprint,&invalid)) {return std::string("footprint at ")+
        std::to_string(invalid.x)+","+std::to_string(invalid.y);}
      for(size_t i=0;i+1<p.size();++i) {
        auto e=edge(p,i,limits_);
        if(!e.ackermann) {return std::string("edge ")+std::to_string(i)+"/"+std::to_string(p.size())+
          " radius "+std::to_string(e.radius)+" ds "+std::to_string(e.length);}
      }
      return std::string("endpoint or gear sequence");
    };
    throw std::runtime_error("Hybrid raw: "+reason(raw)+"; smoothed: "+reason(smoothed));
  }
private:
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{"nav2_core","nav2_core::GlobalPlanner"};
  std::shared_ptr<nav2_core::GlobalPlanner> native_;
  std::unique_ptr<nav2_smac_planner::Smoother> smoother_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  std::unique_ptr<RetainedPathService> validity_;
  MotionLimits limits_;
  double padding_{0.0},max_time_{1};
};
}
