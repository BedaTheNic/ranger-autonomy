// SPDX-License-Identifier: Apache-2.0
#include <mutex>
#include <sstream>
#include "mode_policy.hpp"
#include "clearance_lattice.hpp"
#include "validated_hybrid.hpp"
#include "retained_path_service.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
class AdaptivePlanner : public nav2_core::GlobalPlanner
{
public:

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    node_=parent;map_=map;auto node=parent.lock();
    ack_=std::make_unique<ValidatedHybrid>();
    holo_=std::make_unique<ClearanceLattice>();
    ack_->expose_validity=false;
    static_cast<ClearanceLattice *>(holo_.get())->expose_validity=false;
    ack_->configure(parent,name+".Ackermann",tf,map);
    holo_->configure(parent,name+".Holonomic",tf,map);
    auto parameter=[&](const std::string & key,double & value) {
      if(!node->has_parameter(name+"."+key)) {node->declare_parameter(name+"."+key,value);}
      value=node->get_parameter(name+"."+key).as_double();
      if(!std::isfinite(value)||value<=0) {throw std::runtime_error("Invalid ETA limit "+key);}
    };
    parameter("minimum_turning_radius",limits_.radius);
    parameter("forward_speed",limits_.forward);parameter("reverse_speed",limits_.reverse);
    parameter("holonomic_speed",limits_.lateral);parameter("angular_speed",limits_.angular);
    parameter("steering_rate",limits_.steering_rate);
    parameter("planning_padding",planning_padding_);
    parameter("lattice_search_padding",static_cast<ClearanceLattice *>(holo_.get())->search_padding);
    parameter("goal_connection_distance",static_cast<ClearanceLattice *>(holo_.get())->goal_connection_distance);
    static_cast<ClearanceLattice *>(holo_.get())->planning_padding=planning_padding_;
    ack_->setLimits(limits_,planning_padding_);
    validity_=std::make_unique<RetainedPathService>(node,map_,planning_padding_);
    publisher_=node->create_publisher<std_msgs::msg::String>("navigation_plan_decision",rclcpp::QoS(1).transient_local());
  }
  void activate() override {ack_->activate();holo_->activate();publisher_->on_activate();}
  void deactivate() override {ack_->deactivate();holo_->deactivate();publisher_->on_deactivate();}
  void cleanup() override {validity_.reset();ack_->cleanup();holo_->cleanup();publisher_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
#ifdef RANGER_NAV2_JAZZY
    , std::function<bool()> cancel_checker
#endif
    ) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * grid=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*grid->getMutex());
    auto footprint=map_->getRobotFootprint();
    nav2_costmap_2d::padFootprint(footprint,planning_padding_);
    if(!poseFree(planar(goal.pose),*grid,footprint)) {
      throw std::runtime_error("Goal footprint is occupied or lacks tracking clearance; choose a goal farther from the obstacle");
    }
    nav_msgs::msg::Path a,h;std::string failure;
#ifdef RANGER_NAV2_JAZZY
    if(cancel_checker()) {throw nav2_core::PlannerCancelled("Planning canceled");}
    try {a=ack_->createPlan(start,goal,cancel_checker);} catch(const nav2_core::PlannerCancelled &) {throw;} catch(const std::exception & e) {failure=e.what();}
    try {h=holo_->createPlan(start,goal,cancel_checker);} catch(const nav2_core::PlannerCancelled &) {throw;} catch(const std::exception & e) {failure+="; "+std::string(e.what());}
#else
    try {a=ack_->createPlan(start,goal);} catch(const std::exception & e) {failure=e.what();}
    try {h=holo_->createPlan(start,goal);} catch(const std::exception & e) {failure+="; "+std::string(e.what());}
#endif
    // Native search uses discretized poses, and smoothing may change them.
    // Check exactly the same continuous footprint poses sent to IsPathValid.
    Pose2 invalid;
    auto location=[&]() {return " at "+std::to_string(invalid.x)+","+std::to_string(invalid.y)+","+std::to_string(invalid.a);};
    if(!a.poses.empty() && !pathFree(points(a),*grid,footprint,&invalid)) {
      a.poses.clear();failure+="; Ackermann candidate failed continuous footprint validation"+location();
    }
    if(!h.poses.empty() && !pathFree(points(h),*grid,footprint,&invalid)) {
      h.poses.clear();failure+="; holonomic candidate failed continuous footprint validation"+location();
    }
    unsigned aligned=0;
    if(h.poses.size()>1) {
      auto candidate=points(h);
      aligned=alignStraights(candidate,*grid,footprint,limits_);
      if(aligned) {
        // The lattice output is already dense. Only zero-distance pivots were
        // inserted; keep the tested samples exactly as used by the controller.
        auto aligned_path=densePath(candidate,h.header,std::numeric_limits<double>::infinity());
        if(pathFree(points(aligned_path),*grid,footprint)) {h=std::move(aligned_path);}
        else {aligned=0;}
      }
    }
    double ta=INFINITY,th=INFINITY;
    if(a.poses.size()>1) {
      auto p=points(a);
      ta=estimate(p,std::vector<bool>(p.size()-1,true),limits_,yaw(start.pose)).seconds;
    }
    if(h.poses.size()>1) {
      auto d=decide(points(h),limits_,yaw(start.pose));th=std::min(d.mixed.seconds,d.holonomic.seconds);
    }
    // Same-cell goals still need a path so the controller can finish orientation.
    if(a.poses.size()==1) {ta=std::abs(wrap(yaw(goal.pose)-yaw(start.pose)))/limits_.angular;}
    if(h.poses.size()==1) {th=std::abs(wrap(yaw(goal.pose)-yaw(start.pose)))/limits_.angular;}
    if(!std::isfinite(ta)&&!std::isfinite(th)) {throw std::runtime_error("No feasible candidate route: "+failure);}
    const double la=std::isfinite(ta)?routeLength(points(a),planar(start.pose),planar(goal.pose)):INFINITY;
    const double lh=std::isfinite(th)?routeLength(points(h),planar(start.pose),planar(goal.pose)):INFINITY;
    bool ack=preferShorterRoute(la,ta,lh,th);
    int ack_clear=-1, holo_clear=-1;
    if(std::isfinite(la) && std::isfinite(lh) && std::abs(la-lh)<=1e-6) {
      ack_clear=clearanceBands(points(a),*grid,footprint);
      holo_clear=clearanceBands(points(h),*grid,footprint);
      if(ack_clear!=holo_clear) {ack=ack_clear>holo_clear;}
    }
    std::ostringstream json;
    json<<"{\"selected\":\""<<(ack?"ackermann_route":"mixed_route")<<"\",\"ackermann_eta\":";
    if(std::isfinite(ta)) {json<<ta;} else {json<<"null";}
    json<<",\"mixed_eta\":";if(std::isfinite(th)) {json<<th;} else {json<<"null";}
    json<<",\"objective\":\"shortest_feasible\",\"ackermann_length\":";
    if(std::isfinite(la)) {json<<la;} else {json<<"null";}
    json<<",\"holonomic_length\":";
    if(std::isfinite(lh)) {json<<lh;} else {json<<"null";}
    json<<",\"aligned_legs\":"<<aligned<<",\"hybrid_variant\":\""<<ack_->variant
        <<"\",\"ack_clearance_bands\":"<<ack_clear<<",\"holo_clearance_bands\":"<<holo_clear<<"}";
    if(!failure.empty()) {RCLCPP_DEBUG(node_.lock()->get_logger(),"Candidate rejection: %s",failure.c_str());}
    std_msgs::msg::String message;message.data=json.str();publisher_->publish(message);
    RCLCPP_INFO(node_.lock()->get_logger(),"Route length decision: %s",message.data.c_str());
    return ack?a:h;
  }
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::unique_ptr<ValidatedHybrid> ack_;
  std::unique_ptr<RetainedPathService> validity_;
  std::unique_ptr<nav2_core::GlobalPlanner> holo_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  MotionLimits limits_;std::mutex mutex_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  double planning_padding_{.04};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::AdaptivePlanner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ClearanceLattice,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ValidatedHybrid,nav2_core::GlobalPlanner)
