// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "navigation_geometry.hpp"

namespace ranger_nav_plugins
{
using Twist = geometry_msgs::msg::Twist;
inline double linearSpeed(const Twist & v) {return std::hypot(v.linear.x,v.linear.y);}
inline Twist scaled(Twist v,double scale)
{
  v.linear.x*=scale; v.linear.y*=scale; v.angular.z*=scale; return v;
}
inline bool finitePlanar(const Twist & v)
{return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z);}

inline bool sweptArc(Pose2 & pose,const Twist & v,double duration,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint)
{
  const int steps=std::max({1,int(std::ceil(duration*linearSpeed(v)/.02)),
    int(std::ceil(duration*std::abs(v.angular.z)/.02))});
  const auto start=pose;
  for(int i=0;i<=steps;++i) {
    pose=integrate(start,v,duration*i/steps);
    if(!poseFree(pose,map,footprint)) {return false;}
  }
  return true;
}
inline double brakeTime(const Twist & v,double decel,double angular_decel)
{return std::max(linearSpeed(v)/decel,std::abs(v.angular.z)/angular_decel);}

// Sensor/pose delay is travelled at the observed twist. At the command twist
// allow one control interval, then decelerate both components to zero along
// the arc. A linear velocity ramp integrates to half its duration at full speed.
inline double stoppingScale(Pose2 pose,const Twist & measured,const Twist & wanted,
  double delay,double interval,double decel,double angular_decel,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint)
{
  if(!sweptArc(pose,measured,delay,map,footprint)) {return 0;}
  auto braking=pose;
  if(!sweptArc(braking,measured,.5*brakeTime(measured,decel,angular_decel),map,footprint)) {return 0;}
  for(double scale : {1.,.75,.5,.25}) {
    auto candidate=scaled(wanted,scale);
    auto end=pose;
    if(sweptArc(end,candidate,interval+.5*brakeTime(candidate,decel,angular_decel),map,footprint)) {
      return scale;
    }
  }
  return 0;
}
}
