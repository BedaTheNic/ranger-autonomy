// SPDX-License-Identifier: Apache-2.0
#include <exception>
#include "dwb_core/dwb_local_planner.hpp"
#include "dwb_core/exceptions.hpp"
#include "dwb_core/illegal_trajectory_tracker.hpp"
#ifdef RANGER_NAV2_JAZZY
#include "nav2_core/controller_exceptions.hpp"
#else
#include "nav2_core/exceptions.hpp"
#endif
#include "heading_maneuver.hpp"
#include "holonomic_velocity.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
#ifdef RANGER_NAV2_JAZZY
using TrackingException = nav2_core::ControllerException;
#else
using TrackingException = nav2_core::PlannerException;
#endif
class ProgressDWBController : public dwb_core::DWBLocalPlanner
{
public:
  void setSpeedLimit(const double & limit,const bool & percentage) override
  {
    const double maximum=node_.lock()->get_parameter(dwb_plugin_name_+".max_speed_xy").as_double();
    speed_scale_=speedLimitScale(limit,percentage,maximum);
    dwb_core::DWBLocalPlanner::setSpeedLimit(limit,percentage);
  }
  void setPlan(const nav_msgs::msg::Path & path) override
  {
    if(path.poses.empty()) {throw TrackingException("Empty holonomic reference");}
    reference_=path;cursor_=0;furthest_=0;assist_.reset();heading_.reset();
    const auto pivot=nextPivot(points(path),0);
    terminal_pivot_=bool(pivot);active_end_=pivot?*pivot:path.poses.size()-1;
    loadSegment();
    cumulative_.assign(path.poses.size(),0);
    for(size_t i=1;i<path.poses.size();++i) {
      cumulative_[i]=cumulative_[i-1]+distance(planar(path.poses[i-1].pose),planar(path.poses[i].pose));
    }
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override
  {
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    auto path=points(transformPath(reference_,pose.header.frame_id,*tf_));
    if(path.empty()) {return dwb_core::DWBLocalPlanner::computeVelocityCommands(pose,speed,checker);}
    const auto robot=planar(pose.pose);
    auto pivot=nextPivot(path,cursor_);
    while(pivot && distance(robot,path[*pivot])<.04 && std::abs(wrap(robot.a-path[*pivot].a))<.08) {
      cursor_=*pivot;pivot=nextPivot(path,cursor_);
    }
    const size_t end=pivot?*pivot:path.size()-1;
    if(end!=active_end_) {active_end_=end;loadSegment();heading_.reset();assist_.reset();}
    terminal_pivot_=bool(pivot);path.resize(end+1);
    // Use the same checked translation-then-rotation approach at the actual
    // goal. Keep both XY and yaw inside the global goal checker's tolerances.
    if(distance(robot,path.back())<.18) {
      auto * map=costmap_ros_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      // Enter a pivot at 2.5 cm, but let the committed rotation absorb
      // small localization changes up to the existing 4 cm pivot tolerance.
      // Otherwise each noisy threshold crossing realigns all four wheels
      // for translation, then for rotation again, preventing useful yaw.
      const double approach_tolerance=heading_.active()?.04:.025;
      if(distance(robot,path.back())>approach_tolerance) {
        heading_.reset();
        command.twist=pursuit(robot,path.back(),false,1,.8,
          std::min(.08*speed_scale_,std::hypot(speed.linear.x,speed.linear.y)+.03));
        if(!collisionFree(robot,command.twist,*map,costmap_ros_->getRobotFootprint())) {
          throw TrackingException("Terminal approach is blocked");
        }
      } else {
        const Projection endpoint{path.size()-1,0,distance(robot,path.back()),path.back()};
        auto turn=heading_.compute(robot,speed,path,endpoint,*map,
          costmap_ros_->getRobotFootprint(),clock_->now().seconds(),speed_scale_,false,true);
        if(turn) {command.twist=*turn;}
        else if(std::abs(wrap(path.back().a-robot.a))>.08) {
          throw TrackingException("Terminal rotation is blocked");
        }
      }
      remember(command.twist);assist_.reset();return command;
    }
    auto projection=project(path,robot,cursor_,3.0,true);cursor_=projection.index;
    double progress=cumulative_[cursor_];
    if(cursor_+1<path.size()) {progress+=projection.fraction*distance(path[cursor_],path[cursor_+1]);}
    furthest_=std::max(furthest_,progress);
    const double corner=cornerDistance(path,projection);
    auto target=lookahead(path,projection,std::min(.35,corner));
    const double tx=target.x-robot.x,ty=target.y-robot.y;
    target_x_=std::cos(robot.a)*tx+std::sin(robot.a)*ty;
    target_y_=-std::sin(robot.a)*tx+std::cos(robot.a)*ty;
    auto * map=costmap_ros_->getCostmap();
    const double now=clock_->now().seconds();
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      auto turn=heading_.compute(robot,speed,path,projection,*map,costmap_ros_->getRobotFootprint(),now,speed_scale_,false);
      if(turn) {command.twist=*turn;remember(command.twist);assist_.reset();
        RCLCPP_INFO_THROTTLE(logger_,*clock_,2000,"Holonomic orientation maneuver: swept rotation to %.3f rad",heading_.target());
        return command;}
      // A lateral right-angle turn must reach its vertex before changing
      // direction; a long carrot would cut into the obstacle at the corner.
      if(std::isfinite(corner) && distance(robot,lookahead(path,projection,corner))>.05) {
        auto approach=pursuit(robot,target,false,1,.8,
          std::min(.12*speed_scale_,std::hypot(speed.linear.x,speed.linear.y)+.05));
        if(collisionFree(robot,approach,*map,costmap_ros_->getRobotFootprint())) {
          command.twist=approach;remember(approach);return command;
        }
      }
    }
    std::exception_ptr nominal_error;
    try {command=dwb_core::DWBLocalPlanner::computeVelocityCommands(pose,speed,checker);}
    catch(const TrackingException &) {nominal_error=std::current_exception();}
    const double dx=target.x-robot.x,dy=target.y-robot.y,d=std::hypot(dx,dy);
    const double vx=std::cos(robot.a)*command.twist.linear.x-std::sin(robot.a)*command.twist.linear.y;
    const double vy=std::sin(robot.a)*command.twist.linear.x+std::cos(robot.a)*command.twist.linear.y;
    double advancing=d>1e-6?(vx*dx+vy*dy)/d:0;
    if(advancing<.75*std::hypot(vx,vy)) {advancing=0;}
    const bool help=assist_.needed({furthest_,0,0},clock_->now().seconds(),advancing);
    if(!help && !nominal_error) {return command;}
    const double maximum=node_.lock()->get_parameter(dwb_plugin_name_+".max_speed_xy").as_double();
    auto candidate=pursuit(robot,target,false,1,.8,
      std::min({.12,maximum*speed_scale_,std::hypot(speed.linear.x,speed.linear.y)+.05}));
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    auto turn=heading_.compute(robot,speed,path,projection,*map,costmap_ros_->getRobotFootprint(),now,speed_scale_,true);
    if(turn) {command.twist=*turn;remember(command.twist);assist_.reset();return command;}
    if(collisionFree(robot,candidate,*map,costmap_ros_->getRobotFootprint())) {
      command.twist=candidate;
      // The next DWB iteration must remember the command actually executed,
      // not its rejected oscillating candidate.
      remember(candidate);
      RCLCPP_INFO_THROTTLE(logger_,*clock_,3000,
        "Progress assist: collision-checked holonomic tracking on retained path");
      return command;
    }
    if(nominal_error) {std::rethrow_exception(nominal_error);}
    return command;
  }
protected:
  dwb_msgs::msg::TrajectoryScore coreScoringAlgorithm(
    const geometry_msgs::msg::Pose2D & pose,const nav_2d_msgs::msg::Twist2D velocity,
    std::shared_ptr<dwb_msgs::msg::LocalPlanEvaluation> & results) override
  {
    dwb_msgs::msg::TrajectoryScore best;best.total=-1;
    std::exception_ptr failure;
    try {best=dwb_core::DWBLocalPlanner::coreScoringAlgorithm(pose,velocity,results);}
    catch(const dwb_core::NoLegalTrajectoriesException &) {failure=std::current_exception();}
    auto node=node_.lock();
    auto value=[&](const std::string & key) {return node->get_parameter(dwb_plugin_name_+"."+key).as_double();};
    const double dt=value("sim_period");
    TranslationWindow window{
      std::max(value("min_vel_x")*speed_scale_,velocity.x+value("decel_lim_x")*dt),
      std::min(value("max_vel_x")*speed_scale_,velocity.x+value("acc_lim_x")*dt),
      std::max(value("min_vel_y")*speed_scale_,velocity.y+value("decel_lim_y")*dt),
      std::min(value("max_vel_y")*speed_scale_,velocity.y+value("acc_lim_y")*dt),
      velocity.theta+value("decel_lim_theta")*dt,velocity.theta+value("acc_lim_theta")*dt,
      value("max_speed_xy")*speed_scale_};
    auto aligned=alignedTranslation(target_x_,target_y_,.7*std::hypot(target_x_,target_y_),window);
    if(aligned) {
      nav_2d_msgs::msg::Twist2D v;v.x=aligned->linear.x;v.y=aligned->linear.y;
      auto trajectory=traj_generator_->generateTrajectory(pose,velocity,v);
      try {
        // This is an extra DWB candidate, checked by ALL ordinary critics,
        // including footprint collision and mode constraints before selection.
        auto score=scoreTrajectory(trajectory,best.total);
        if(results) {results->twists.push_back(score);}
        if(best.total<0 || score.total<best.total) {
          best=score;
          if(results) {results->best_index=results->twists.size()-1;}
        }
      } catch(const dwb_core::IllegalTrajectoryException &) {}
    }
    if(best.total<0 && failure) {std::rethrow_exception(failure);}
    return best;
  }
private:
  void loadSegment()
  {
    auto segment=reference_;
    segment.poses.assign(reference_.poses.begin()+cursor_,reference_.poses.begin()+active_end_+1);
    dwb_core::DWBLocalPlanner::setPlan(segment);
  }
  void remember(const geometry_msgs::msg::Twist & command)
  {
    nav_2d_msgs::msg::Twist2D actual;
    actual.x=command.linear.x;actual.y=command.linear.y;actual.theta=command.angular.z;
    for(auto & critic:critics_) {
      if(critic->getName()=="Oscillation") {critic->reset();}
      critic->debrief(actual);
    }
  }
  HeadingManeuver heading_;
  nav_msgs::msg::Path reference_;
  size_t cursor_{0},active_end_{0};double furthest_{0};
  std::vector<double> cumulative_;
  ProgressAssist assist_;
  double speed_scale_{1.0};
  double target_x_{0},target_y_{0};
  bool terminal_pivot_{false};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ProgressDWBController,nav2_core::Controller)
