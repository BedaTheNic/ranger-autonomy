// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <optional>
#include "navigation_geometry.hpp"
namespace ranger_nav_plugins
{
// A committed, fully swept rotation: never translate while changing the chassis
// heading. Evaluate the complete turn, including intermediate corner clearance.
class HeadingManeuver
{
public:
  void reset() {active_=false;cooldown_=0;}
  bool active() const {return active_;}
  double target() const {return heading_;}
  std::optional<geometry_msgs::msg::Twist> compute(
    const Pose2 & robot,const geometry_msgs::msg::Twist & speed,
    const std::vector<Pose2> & path,const Projection & projection,
    nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint,
    double now,double speed_scale,bool difficult,bool final_alignment=false)
  {
    if(active_ && (std::abs(wrap(heading_-robot.a))<.06 || now>deadline_)) {
      active_=false;cooldown_=now+2;return geometry_msgs::msg::Twist{};
    }
    // Mandatory path pivots are not optional heading changes. A previous
    // alignment's cooldown must not masquerade as a blocked rotation.
    if(!active_ && (now>=cooldown_ || final_alignment)) {
      // Look beyond the bend so alignment can finish while there is still
      // room to turn, before entering a passage too narrow for a pivot.
      const auto target=final_alignment?path.back():lookahead(path,projection,2.0,true);
      const double desired=target.a;
      const double current=freeDistance(path,projection,robot.a,map,footprint);
      // Respect the planner's body orientation, not only its XY polyline.
      if(std::abs(wrap(desired-robot.a))>(final_alignment?.06:.30) && rotationFree(robot,desired,map,footprint) &&
        (final_alignment || freeDistance(path,projection,desired,map,footprint)+.05>=current)) {
        begin(desired,robot.a,now,speed_scale);
      } else if(difficult || (!final_alignment && current<.6)) {
        // A still-moving controller must be able to turn before it enters
        // a passage where its current heading will block the footprint.
        const double tangent=std::atan2(target.y-robot.y,target.x-robot.x);
        double best=current+.20,chosen=robot.a;
        for(double candidate:{desired,tangent,wrap(tangent+M_PI),wrap(robot.a+M_PI_2),wrap(robot.a-M_PI_2)}) {
          const double turn=std::abs(wrap(candidate-robot.a));
          if(turn<.25 || !rotationFree(robot,candidate,map,footprint)) {continue;}
          const double free=freeDistance(path,projection,candidate,map,footprint);
          const double score=free-.02*turn;
          if(free>=.3 && score>best) {best=score;chosen=candidate;}
        }
        if(std::abs(wrap(chosen-robot.a))>.25) {begin(chosen,robot.a,now,speed_scale);}
      }
    }
    if(!active_) {return std::nullopt;}
    geometry_msgs::msg::Twist command;
    // Stop safely if a new obstacle enters the swept rotation area.
    if(!rotationFree(robot,heading_,map,footprint)) {
      active_=false;cooldown_=now+2;return command;
    }
    if(std::hypot(speed.linear.x,speed.linear.y)>.025) {return command;}
    const double bound=.25*speed_scale;
    const double target=std::clamp(1.3*wrap(heading_-robot.a),-bound,bound);
    command.angular.z=std::clamp(target,speed.angular.z-.05,speed.angular.z+.05);
    command.angular.z=std::clamp(command.angular.z,-bound,bound);
    return command;
  }
private:
  static double freeDistance(const std::vector<Pose2> & path,const Projection & p,double heading,
    nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint)
  {
    double result=0;
    for(int i=1;i<=16;++i) {
      const double length=.05*i;
      auto q=lookahead(path,p,length,true);q.a=heading;
      if(!poseFree(q,map,footprint)) {break;}
      result=length;
    }
    return result;
  }
  void begin(double heading,double current,double now,double scale)
  {heading_=heading;active_=true;deadline_=now+std::abs(wrap(heading-current))/std::max(.01,.25*scale)+6;}
  bool active_{false};double heading_{0},deadline_{0},cooldown_{0};
};
}
