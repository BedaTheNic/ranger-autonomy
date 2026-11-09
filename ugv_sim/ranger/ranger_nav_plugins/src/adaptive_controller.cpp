// SPDX-License-Identifier: Apache-2.0
#include <sstream>
#include "mode_policy.hpp"
#include "heading_maneuver.hpp"
#include "reversal_alignment.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ranger_nav_plugins
{
class AdaptiveController : public nav2_core::Controller
{
public:
  AdaptiveController():loader_("nav2_core","nav2_core::Controller") {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    node_=parent;name_=name;tf_=tf;map_=map;auto node=parent.lock();clock_=node->get_clock();
    auto parameter=[&](const std::string & key,double & value) {
      if(!node->has_parameter(name+"."+key)) {node->declare_parameter(name+"."+key,value);}
      value=node->get_parameter(name+"."+key).as_double();
      if(!std::isfinite(value)||value<=0) {throw std::runtime_error("Invalid hybrid limit "+key);}
    };
    parameter("minimum_turning_radius",limits_.radius);
    parameter("forward_speed",limits_.forward);parameter("reverse_speed",limits_.reverse);
    parameter("holonomic_speed",limits_.lateral);parameter("angular_speed",limits_.angular);
    parameter("steering_rate",limits_.steering_rate);
    parameter("minimum_mode_time",minimum_mode_time_);
    ack_=loader_.createSharedInstance("ranger_nav_plugins::AckermannController");
    holo_=loader_.createSharedInstance("ranger_nav_plugins::ProgressDWBController");
    ack_->configure(parent,name+".Ackermann",tf,map);
    holo_->configure(parent,name+".Holonomic",tf,map);
    mode_pub_=node->create_publisher<std_msgs::msg::String>("navigation_mode",rclcpp::QoS(1).transient_local());
    decision_pub_=node->create_publisher<std_msgs::msg::String>("navigation_mode_decision",10);
    marker_pub_=node->create_publisher<visualization_msgs::msg::MarkerArray>("navigation_modes",rclcpp::QoS(1).transient_local());
  }
  void activate() override
  {ack_->activate();holo_->activate();mode_pub_->on_activate();decision_pub_->on_activate();marker_pub_->on_activate();}
  void deactivate() override
  {reference_.poses.clear();ack_->deactivate();holo_->deactivate();mode_pub_->on_deactivate();decision_pub_->on_deactivate();marker_pub_->on_deactivate();}
  void cleanup() override {ack_->cleanup();holo_->cleanup();mode_pub_.reset();decision_pub_.reset();marker_pub_.reset();}
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if(path.poses.empty()) {throw std::runtime_error("Empty adaptive reference");}
    if(sameStampedPath(reference_,path)) {
      RCLCPP_INFO(node_.lock()->get_logger(),"Resuming retained path at index %zu of %zu",cursor_,path.poses.size());
      return;
    }
    std::optional<bool> entering_car;
    if(!reference_.poses.empty() && reference_.header.frame_id==path.header.frame_id) {
      const auto previous=planar(reference_.poses.back().pose),next=planar(path.poses.back().pose);
      if(distance(previous,next)<1e-5 && std::abs(wrap(previous.a-next.a))<1e-5) {
        if(published_mode_=="ACKERMANN") {entering_car=true;}
        else if(published_mode_.rfind("HOLONOMIC",0)==0) {entering_car=false;}
        else if(mode_!=0) {entering_car=(mode_==1);}
      }
    }
    reference_=path;terminal_alignment_=false;cursor_=0;handoff_=0;loaded_mode_=0;loaded_end_=0;
    geometry_=points(path);
    plan_decision_=decide(geometry_,limits_,geometry_.front().a,entering_car);
    strict_ackermann_=strictAckermannReference(geometry_,limits_);
    recovery_active_=false;recovery_heading_.reset();
    reversal_target_.reset();
    // Keep the current driving mode across a necessary global replan.
  }
  void setSpeedLimit(const double & limit,const bool & percentage) override
  {
    speed_scale_=speedLimitScale(limit,percentage,limits_.forward);
    ack_->setSpeedLimit(limit,percentage);holo_->setSpeedLimit(limit,percentage);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override
  {
    geometry_msgs::msg::TwistStamped stop;stop.header=pose.header;
    auto global_pose=pose;
    if(pose.header.frame_id!=reference_.header.frame_id) {
      auto transform=tf_->lookupTransform(reference_.header.frame_id,pose.header.frame_id,tf2::TimePointZero);
      tf2::doTransform(pose,global_pose,transform);
    }
    const auto & all=geometry_;const auto robot=planar(global_pose.pose);
    auto pivot=nextPivot(all,cursor_);
    while(pivot && distance(robot,all[*pivot])<.04 && std::abs(wrap(robot.a-all[*pivot].a))<.08) {
      cursor_=*pivot;pivot=nextPivot(all,cursor_);
    }
    // Never project onto an overlapping branch beyond an unexecuted pivot.
    auto searchable=all;
    if(pivot) {searchable.resize(*pivot+1);}
    auto projection=project(searchable,robot,cursor_,3.0,!strict_ackermann_);cursor_=projection.index;
    if(all.size()<2) {
      if(loaded_mode_!=2) {holo_->setPlan(reference_);loaded_mode_=2;}
      return holo_->computeVelocityCommands(pose,speed,checker);
    }
    if(recovery_active_) {
      auto command=recover(pose,speed);
      if(command) {return *command;}
    }
    const size_t begin=std::min(cursor_,all.size()-2);
    const auto & decision=plan_decision_;
    size_t active=std::max(begin,handoff_);
    bool wanted_ack=decision.modes[active];
    // Finish an aligned straight with the same controller. Latch only a
    // genuinely necessary terminal correction so noisy pose updates cannot
    // alternate the controllers near the goal.
    if(wanted_ack && finalHolonomicApproach(all,projection,robot,limits_)) {
      geometry_msgs::msg::Pose tolerance;
      geometry_msgs::msg::Twist speed_tolerance;
      if(checker && checker->getTolerances(tolerance,speed_tolerance)) {
        const double remaining=distance(robot,all.back());
        const double yaw_reachable=std::abs(yaw(tolerance))+remaining/limits_.radius;
        if(std::abs(wrap(robot.a-all.back().a))>yaw_reachable) {terminal_alignment_=true;}
      }
    }
    if(terminal_alignment_) {wanted_ack=false;}
    // Hand over before MPPI brakes for a mandatory pivot or crab section.
    size_t end=active;
    while(end<decision.modes.size() && decision.modes[end]==wanted_ack) {++end;}
    double distance_to_handoff=distance(robot,projection.pose);
    for(size_t i=begin; wanted_ack && end<decision.modes.size() && i<end && distance_to_handoff<.30; ++i) {
      distance_to_handoff+=distance(i==begin?projection.pose:all[i],all[i+1]);
    }
    if(wanted_ack && end<decision.modes.size() && distance_to_handoff<.30 && (!pivot || end<=*pivot)) {
      handoff_=end;active=end;wanted_ack=false;
    }
    const double now=clock_->now().seconds();
    if (reversal_target_) {
      auto alignment = alignReversal(pose, speed, checker, decision, now);
      if (alignment) {return *alignment;}
    }
    if(wanted_ack && mode_==2 && now-mode_stamp_<minimum_mode_time_) {wanted_ack=false;}
    if(wanted_ack && mode_==2 && projection.error>.06) {wanted_ack=false;}
    if(wanted_ack && now<alignment_blocked_until_) {wanted_ack=false;}
    auto current_edge=edge(all,active,limits_);feasible_=current_edge.ackermann;
    if(wanted_ack && mode_!=1 && std::abs(wrap(all[active].a-robot.a))>.10) {
      auto pivot=stop;
      pivot.twist.angular.z=std::clamp(1.3*wrap(all[active].a-robot.a),-.22,.22);
      pivot.twist.angular.z*=speed_scale_;
      // Translation must stop before changing to the pivot steering pattern.
      if(std::hypot(speed.linear.x,speed.linear.y)>.025) {
        publish("BRAKING_FOR_ALIGNMENT",decision,current_edge.radius,now);return stop;
      }
      auto * grid=map_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
      const auto local_robot=planar(pose.pose);
      const double local_heading=local_robot.a+wrap(all[active].a-robot.a);
      if(rotationFree(local_robot,local_heading,*grid,map_->getRobotFootprint())) {
        publish("HOLONOMIC_ALIGN",decision,current_edge.radius,now);return pivot;
      }
      alignment_blocked_until_=now+5;wanted_ack=false;
    }
    const int wanted=wanted_ack?1:2;
    // Straight, already aligned mode changes do not introduce a stop.
    const double heading_error=steeringChange(current_edge.heading,robot.a);
    const bool incompatible=wanted_ack?
      (std::abs(speed.linear.y)>.025 || std::abs(speed.angular.z)>.05):
      (heading_error>.15 || std::abs(speed.angular.z)>.05);
    if(mode_ && wanted!=mode_ && incompatible && std::hypot(speed.linear.x,speed.linear.y)>.035) {
      publish("BRAKING_FOR_MODE_CHANGE",decision,current_edge.radius,now);return stop;
    }
    if(wanted!=mode_) {
      mode_=wanted;mode_stamp_=now;++switches_;
      RCLCPP_INFO(node_.lock()->get_logger(),"Driving mode: %s",mode_==1?"ACKERMANN":"HOLONOMIC");
    }
    size_t terminal=pivot?*pivot:reference_.poses.size()-1;
    if(mode_==1) {
      terminal=begin;
      while(terminal+1<reference_.poses.size() && decision.modes[terminal]) {++terminal;}
    }
    if(mode_==2 && !decision.modes[active]) {
      size_t boundary=active+1;
      while(boundary<terminal && !decision.modes[boundary]) {++boundary;}
      // A small overlapping lookahead lets DWB hand over while moving.
      double preview=0;
      while(boundary<terminal && preview<.25 && decision.modes[boundary]) {
        preview+=distance(all[boundary],all[boundary+1]);++boundary;
      }
      terminal=std::min(terminal,boundary);
    }
    if(loaded_mode_!=mode_ || loaded_end_!=terminal) {
      nav_msgs::msg::Path segment;segment.header=reference_.header;
      segment.poses.assign(reference_.poses.begin()+begin,reference_.poses.begin()+terminal+1);
      if(segment.poses.size()==1 && terminal+1<reference_.poses.size()) {
        segment.poses.push_back(reference_.poses[terminal+1]);
      }
      (mode_==1?ack_:holo_)->setPlan(segment);
      loaded_mode_=mode_;loaded_end_=terminal;
    }
    geometry_msgs::msg::TwistStamped command;
    try {command=(mode_==1?ack_:holo_)->computeVelocityCommands(pose,speed,checker);}
    catch (const ReversalAlignment & reversal) {
      const auto target = planar(reversal.target.pose);
      if (distance(planar(pose.pose), target) > .15) {
        throw AckermannTrackingError("Missed reversal outside the local rejoin tolerance");
      }
      nav_msgs::msg::Path approach;
      approach.header = pose.header;
      approach.poses = {pose, reversal.target};
      holo_->setPlan(approach);
      reversal_target_ = target;
      braking_for_reversal_ = true;
      auto alignment = alignReversal(pose, speed, checker, decision, now);
      return alignment.value_or(stop);
    }
    catch(const AckermannTrackingError &) {
      if(!finalHolonomicApproach(all,projection,robot,limits_)) {throw;}
      terminal_alignment_=true;
      return stop;
    }
    catch(const std::runtime_error & error) {
      if(mode_!=1 || std::string(error.what())!="Optimizer fail to compute path") {throw;}
      // Only hand a monotonic part to the holonomic controller: never jump over
      // a forward/reverse cusp by following an overlapping XY branch.
      size_t finish=begin+1;double length=distance(all[begin],all[finish]);
      const int direction=edge(all,begin,limits_).direction;
      while(finish<terminal && length<.8 && edge(all,finish,limits_).direction==direction) {
        length+=distance(all[finish],all[finish+1]);++finish;
      }
      recovery_path_.header=reference_.header;
      recovery_path_.poses.assign(reference_.poses.begin()+begin,reference_.poses.begin()+finish+1);
      recovery_active_=true;recovery_braking_=true;recovery_until_=now+4;
      recovery_heading_.reset();mode_=2;mode_stamp_=now;loaded_mode_=0;
      RCLCPP_WARN(node_.lock()->get_logger(),"Ackermann samples blocked: checking a local holonomic orientation/rejoin maneuver");
      auto recovery=recover(pose,speed);
      return recovery?*recovery:stop;
    }
    const auto & v=command.twist;
    if(!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z) ||
      (std::abs(v.linear.y)>1e-6 && std::abs(v.angular.z)>1e-6) ||
      (mode_==1 && (std::abs(v.linear.y)>1e-6 || std::abs(v.angular.z)*limits_.radius>std::abs(v.linear.x)+1e-5))) {
      throw std::runtime_error("Adaptive command violated the selected kinematics");
    }
    publish(mode_==1?"ACKERMANN":
      (std::hypot(v.linear.x,v.linear.y)<1e-6 && std::abs(v.angular.z)>1e-6?"HOLONOMIC_ROTATE":"HOLONOMIC"),
      decision,current_edge.radius,now);
    return command;
  }
private:
  std::optional<geometry_msgs::msg::TwistStamped> alignReversal(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker, const Decision & decision, double now)
  {
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    const auto robot = planar(pose.pose);
    const double speed_xy = std::hypot(speed.linear.x, speed.linear.y);
    if (braking_for_reversal_ && (speed_xy > .025 || std::abs(speed.angular.z) > .04)) {
      publish("BRAKING_FOR_ALIGNMENT", decision, 0.0, now);
      return command;
    }
    braking_for_reversal_ = false;
    if (distance(robot, *reversal_target_) < .03 &&
      std::abs(wrap(reversal_target_->a - robot.a)) < .06)
    {
      // Finish the pivot and stop before loading the outgoing gear.
      if (speed_xy > .025 || std::abs(speed.angular.z) > .04) {return command;}
      reversal_target_.reset();
      return std::nullopt;
    }
    // Reuse the footprint-checked terminal approach and committed rotation.
    // The Ackermann path handler retains its unconsumed outgoing leg.
    command = holo_->computeVelocityCommands(pose, speed, checker);
    feasible_ = false;
    publish(std::abs(command.twist.angular.z) > 1e-6 ? "HOLONOMIC_ROTATE" : "HOLONOMIC_RECOVERY",
      decision, 0.0, now);
    return command;
  }
  std::optional<geometry_msgs::msg::TwistStamped> recover(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed)
  {
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    const double now=clock_->now().seconds();
    if(recovery_braking_) {
      if(std::hypot(speed.linear.x,speed.linear.y)>.025) {return command;}
      recovery_braking_=false;
    }
    const auto p=points(transformPath(recovery_path_,pose.header.frame_id,*tf_));
    const auto robot=planar(pose.pose);auto projection=project(p,robot);
    auto * grid=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    auto turn=recovery_heading_.compute(robot,speed,p,projection,*grid,map_->getRobotFootprint(),now,speed_scale_,true);
    if(turn) {command.twist=*turn;}
    else {
      if(now>recovery_until_ || distance(robot,p.back())<.08) {recovery_active_=false;return std::nullopt;}
      auto target=lookahead(p,projection,.3);
      command.twist=pursuit(robot,target,false,1,limits_.radius,
        std::min(.08*speed_scale_,std::hypot(speed.linear.x,speed.linear.y)+.03));
      if(!collisionFree(robot,command.twist,*grid,map_->getRobotFootprint())) {
        recovery_active_=false;throw std::runtime_error("No collision-free holonomic rejoin maneuver");
      }
    }
    const auto & decision=plan_decision_;
    publish(turn?"HOLONOMIC_ROTATE":"HOLONOMIC_RECOVERY",decision,INFINITY,now);
    return command;
  }
  void publish(const std::string & mode,const Decision & d,double radius,double now)
  {
    if(mode==published_mode_ && now-last_publish_<.5) {return;}
    published_mode_=mode;last_publish_=now;
    std_msgs::msg::String message;message.data=mode;mode_pub_->publish(message);
    std::ostringstream json;json<<"{\"mode\":\""<<mode<<"\",\"required_radius\":";
    if(std::isfinite(radius)) {json<<radius;} else {json<<"null";}
    json<<",\"ackermann_feasible\":"<<(feasible_?"true":"false")<<",\"minimum_radius\":"<<limits_.radius<<",\"mixed_eta\":";
    if(std::isfinite(d.mixed.seconds)) {json<<d.mixed.seconds;} else {json<<"null";}
    json<<",\"holonomic_eta\":";
    if(std::isfinite(d.holonomic.seconds)) {json<<d.holonomic.seconds;} else {json<<"null";}
    if(cursor_<d.modes.size()) {
      const size_t end=segmentEnd(geometry_,d.modes,cursor_);
      json<<",\"scheduled_mode\":\""<<(d.modes[cursor_]?"ackermann":"holonomic")
          <<"\",\"segment_end_index\":"<<end<<",\"segment_goal\":["<<geometry_[end].x
          <<","<<geometry_[end].y<<","<<geometry_[end].a<<"]";
    }
    json<<",\"transitions\":"<<switches_<<"}";
    message.data=json.str();decision_pub_->publish(message);
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker text;
    text.header.frame_id=map_->getBaseFrameID();text.header.stamp=clock_->now();
    text.ns="mode";text.id=0;text.type=text.TEXT_VIEW_FACING;text.action=text.ADD;
    text.pose.orientation.w=1;text.pose.position.z=.9;text.scale.z=.22;
    text.color.a=1;text.color.r=mode=="ACKERMANN"?.15:1.;text.color.g=.8;text.color.b=.3;text.text=mode;
    markers.markers.push_back(text);
    visualization_msgs::msg::Marker line;
    line.header=reference_.header;line.header.stamp=clock_->now();line.ns="mode_path";line.id=1;
    line.type=line.LINE_LIST;line.action=line.ADD;line.pose.orientation.w=1;line.scale.x=.035;
    const auto & p=geometry_;const auto & modes=d.modes;
    for(size_t i=cursor_;i+1<p.size();++i) {
      std_msgs::msg::ColorRGBA color;color.a=.8;color.r=modes[i]?.1:1.;color.g=modes[i]?.85:.45;color.b=.45;
      for(size_t j=i;j<i+2;++j) {
        geometry_msgs::msg::Point point;point.x=p[j].x;point.y=p[j].y;point.z=.06;
        line.points.push_back(point);line.colors.push_back(color);
      }
    }
    markers.markers.push_back(line);marker_pub_->publish(markers);
  }
  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr ack_,holo_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_,decision_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string name_,published_mode_;
  nav_msgs::msg::Path reference_;
  std::vector<Pose2> geometry_;
  Decision plan_decision_;
  bool terminal_alignment_{false};
  MotionLimits limits_;
  size_t cursor_{0},handoff_{0},loaded_end_{0};
  int mode_{0},loaded_mode_{0};unsigned switches_{0};bool feasible_{false},strict_ackermann_{false};
  double mode_stamp_{-100},minimum_mode_time_{2},alignment_blocked_until_{-100},last_publish_{-100};
  double speed_scale_{1.0};
  HeadingManeuver recovery_heading_;
  std::optional<Pose2> reversal_target_;
  bool braking_for_reversal_{false};
  nav_msgs::msg::Path recovery_path_;
  bool recovery_active_{false},recovery_braking_{false};double recovery_until_{0};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::AdaptiveController,nav2_core::Controller)
