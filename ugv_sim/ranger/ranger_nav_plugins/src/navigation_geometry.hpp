// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace ranger_nav_plugins
{
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
inline double yaw(const geometry_msgs::msg::Pose & p)
{
  const auto & q=p.orientation;
  return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
}
struct Pose2 {double x{0}, y{0}, a{0};};
inline Pose2 planar(const geometry_msgs::msg::Pose & p)
{return {p.position.x,p.position.y,yaw(p)};}
inline double distance(const Pose2 & a,const Pose2 & b)
{return std::hypot(a.x-b.x,a.y-b.y);}
inline double speedLimitScale(double limit,bool percentage,double maximum)
{
  if(!std::isfinite(limit)||limit<0||!std::isfinite(maximum)||maximum<=0) {
    throw std::runtime_error("Invalid navigation speed limit");
  }
  // Nav2's NO_SPEED_LIMIT sentinel is zero, not a stop command.
  return limit==0?1.0:std::clamp(percentage?limit/100.0:limit/maximum,0.0,1.0);
}
inline nav_msgs::msg::Path transformPath(const nav_msgs::msg::Path & path,
  const std::string & frame, tf2_ros::Buffer & tf)
{
  if (path.header.frame_id==frame) {return path;}
  auto transform=tf.lookupTransform(frame,path.header.frame_id,tf2::TimePointZero);
  auto result=path; result.header.frame_id=frame;
  for (size_t i=0;i<path.poses.size();++i) {
    tf2::doTransform(path.poses[i],result.poses[i],transform);
  }
  return result;
}
inline std::vector<Pose2> points(const nav_msgs::msg::Path & path)
{
  std::vector<Pose2> result;
  for (const auto & p:path.poses) {result.push_back(planar(p.pose));}
  return result;
}
struct Projection {size_t index{0}; double fraction{0}, error{0}; Pose2 pose;};
inline std::optional<size_t> nextPivot(const std::vector<Pose2> & path,size_t begin)
{
  for(size_t i=begin;i+1<path.size();++i) {
    if(distance(path[i],path[i+1])<1e-5 && std::abs(wrap(path[i+1].a-path[i].a))>1e-4) {
      size_t end=i+1;
      while(end+1<path.size() && distance(path[i],path[end+1])<1e-5) {++end;}
      return end;
    }
  }
  return std::nullopt;
}
inline bool sharpCorner(const std::vector<Pose2> & path,size_t i)
{
  if(i==0 || i+1>=path.size() || distance(path[i-1],path[i])<1e-5 ||
    distance(path[i],path[i+1])<1e-5) {return false;}
  const double turn=std::abs(wrap(std::atan2(path[i+1].y-path[i].y,path[i+1].x-path[i].x)-
    std::atan2(path[i].y-path[i-1].y,path[i].x-path[i-1].x)));
  return turn>.7 && turn<M_PI-.15;
}
inline Projection project(const std::vector<Pose2> & path,const Pose2 & robot,
  size_t begin=0, double search_length=3.0,bool preserve_corners=false)
{
  if(path.empty()) {throw std::runtime_error("Empty reference path");}
  begin=std::min(begin,path.size()-1);
  Projection best{begin,0,distance(path[begin],robot),path[begin]};
  double walked=0;
  for(size_t i=begin;i+1<path.size() && walked<=search_length;++i) {
    const auto & a=path[i]; const auto & b=path[i+1];
    const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    const double t=l2>1e-12?std::clamp(((robot.x-a.x)*dx+(robot.y-a.y)*dy)/l2,0.0,1.0):0;
    Pose2 p{a.x+t*dx,a.y+t*dy,a.a+t*wrap(b.a-a.a)};
    const double error=distance(p,robot);
    if(error<best.error-1e-7) {best={i,t,error,p};}
    if(preserve_corners && sharpCorner(path,i+1)) {
      if(distance(robot,b)>.05) {break;}
      // Arrival consumes the corner even if the next segment has an equal
      // projection error. Otherwise a zero-length carrot can hold us here.
      best={i+1,0,distance(robot,b),b};
    }
    walked+=std::sqrt(l2);
  }
  return best;
}
inline double cornerDistance(const std::vector<Pose2> & path,const Projection & p,double limit=.6)
{
  double length=0;auto previous=p.pose;
  for(size_t i=p.index+1;i<path.size();++i) {
    length+=distance(previous,path[i]);
    if(length>limit) {break;}
    if(sharpCorner(path,i)) {return length;}
    previous=path[i];
  }
  return INFINITY;
}
inline Pose2 lookahead(const std::vector<Pose2> & path,const Projection & start,double length,
  bool stop_at_pivot=false)
{
  Pose2 a=start.pose;
  for(size_t i=start.index+1;i<path.size();++i) {
    const auto & b=path[i]; double ds=distance(a,b);
    // A zero-distance turn encodes where there is room to rotate. Do not
    // anticipate its outgoing heading while still approaching that point.
    if(stop_at_pivot && ds<1e-5 && std::abs(wrap(b.a-a.a))>1e-4) {return a;}
    if(ds>1e-8 && ds>=length) {
      double t=length/ds; return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.a+t*wrap(b.a-a.a)};
    }
    length-=ds; a=b;
  }
  return path.back();
}
inline geometry_msgs::msg::Twist pursuit(const Pose2 & robot,const Pose2 & target,
  bool ackermann,int direction,double radius,double speed)
{
  geometry_msgs::msg::Twist v;
  double dx=target.x-robot.x,dy=target.y-robot.y;
  double bx=std::cos(robot.a)*dx+std::sin(robot.a)*dy;
  double by=-std::sin(robot.a)*dx+std::cos(robot.a)*dy;
  double d=std::hypot(bx,by);
  if(d<.008) {return v;}
  const double magnitude=std::min(speed,.7*d);
  if(ackermann) {
    v.linear.x=direction*magnitude;
    v.angular.z=v.linear.x*std::clamp(2*by/(d*d),-1/radius,1/radius);
  } else {
    v.linear.x=magnitude*bx/d; v.linear.y=magnitude*by/d;
  }
  return v;
}
inline Pose2 integrate(const Pose2 & p,const geometry_msgs::msg::Twist & v,double dt)
{
  const double w=v.angular.z,da=w*dt;
  double bx=v.linear.x*dt,by=v.linear.y*dt;
  if(std::abs(w)>1e-8) {
    bx=(v.linear.x*std::sin(da)+v.linear.y*(std::cos(da)-1))/w;
    by=(v.linear.x*(1-std::cos(da))+v.linear.y*std::sin(da))/w;
  }
  return {p.x+std::cos(p.a)*bx-std::sin(p.a)*by,
    p.y+std::sin(p.a)*bx+std::cos(p.a)*by,wrap(p.a+da)};
}
inline bool poseFree(const Pose2 & p,nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  unsigned int x,y;
  if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.a) ||
    !map.worldToMap(p.x,p.y,x,y) || map.getCost(x,y)>=253) {return false;}
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&map);
  const double cost=checker.footprintCostAtPose(p.x,p.y,p.a,footprint);
  return cost>=0 && cost<254;
}
inline bool rotationFree(const Pose2 & p,double heading,nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  const double angle=wrap(heading-p.a);
  const int count=std::max(1,int(std::ceil(std::abs(angle)/.025)));
  for(int i=0;i<=count;++i) {
    if(!poseFree({p.x,p.y,p.a+angle*i/count},map,footprint)) {return false;}
  }
  return true;
}
inline bool pathFree(const std::vector<Pose2> & path,nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint,Pose2 * invalid=nullptr)
{
  if(path.empty()) {return false;}
  if(!poseFree(path.front(),map,footprint)) {if(invalid) {*invalid=path.front();} return false;}
  for(size_t i=1;i<path.size();++i) {
    const auto & a=path[i-1];const auto & b=path[i];const double da=wrap(b.a-a.a);
    const int count=std::max({1,int(std::ceil(distance(a,b)/.025)),int(std::ceil(std::abs(da)/.025))});
    for(int j=1;j<=count;++j) {
      const double t=double(j)/count;
      Pose2 p{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.a+t*da};
      if(!poseFree(p,map,footprint)) {if(invalid) {*invalid=p;} return false;}
    }
  }
  return true;
}
// Caller holds the costmap mutex. Check footprint edges and the inflated
// centre cost, including unknown cells.
inline bool collisionFree(const Pose2 & start,const geometry_msgs::msg::Twist & v,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint,
  double horizon=1.2)
{
  if(!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.angular.z)) {return false;}
  for(double t=0;t<=horizon+.025;t+=.05) {
    if(!poseFree(integrate(start,v,t),map,footprint)) {return false;}
  }
  return true;
}

class ProgressAssist
{
public:
  void reset() {initialized_=false; active_=false;}
  bool active() const {return active_;}
  bool needed(const Pose2 & pose,double now,double nominal_speed)
  {
    if(!initialized_ || now<stamp_) {
      initialized_=true; anchor_=pose; stamp_=now; active_=false;
    }
    if(distance(anchor_,pose)+.2*std::abs(wrap(anchor_.a-pose.a))>.025) {
      anchor_=pose; stamp_=now;
    }
    if(active_ && now-active_since_>1.0 && nominal_speed>.045) {active_=false;}
    if(now-stamp_>1.2 && !active_) {active_=true;active_since_=now;}
    return active_;
  }
private:
  Pose2 anchor_;
  double stamp_{0},active_since_{0};
  bool initialized_{false},active_{false};
};
}
