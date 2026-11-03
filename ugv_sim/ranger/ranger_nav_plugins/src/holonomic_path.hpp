// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include "navigation_geometry.hpp"

namespace ranger_nav_plugins
{
inline double pathLength(const std::vector<Pose2> & path)
{
  double length=0;
  for(size_t i=1;i<path.size();++i) {length+=distance(path[i-1],path[i]);}
  return length;
}
// Compare candidates from the same requested endpoints. A grid-snapped route
// must not appear shorter merely because it omits its first/last centimetres.
// These gaps are geometric distance estimates; the controller still executes
// the original kinematically checked route and its terminal approach.
inline double routeLength(const std::vector<Pose2> & path,const Pose2 & start,const Pose2 & goal)
{
  if(path.empty()) {return INFINITY;}
  return distance(start,path.front())+pathLength(path)+distance(path.back(),goal);
}
// Translation and pivot are separate feasible swerve modes. A free straight
// connection attains the Euclidean lower bound without changing body heading
// while moving. Try the requested heading only if turning at the goal is blocked.
inline std::vector<Pose2> directHolonomic(const Pose2 & start,const Pose2 & goal,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint)
{
  for(const double heading:{start.a,goal.a}) {
    std::vector<Pose2> path{start,{start.x,start.y,heading},{goal.x,goal.y,heading},goal};
    if(pathFree(path,map,footprint)) {
      path.erase(std::unique(path.begin(),path.end(),[](const Pose2 & a,const Pose2 & b) {
        return distance(a,b)<1e-8 && std::abs(wrap(a.a-b.a))<1e-8;
      }),path.end());
      return path;
    }
  }
  return {};
}
inline nav_msgs::msg::Path densePath(const std::vector<Pose2> & points,
  const std_msgs::msg::Header & header,double spacing=.04)
{
  nav_msgs::msg::Path path;path.header=header;
  auto append=[&](const Pose2 & p) {
    geometry_msgs::msg::PoseStamped pose;pose.header=header;
    pose.pose.position.x=p.x;pose.pose.position.y=p.y;
    pose.pose.orientation.z=std::sin(p.a/2);pose.pose.orientation.w=std::cos(p.a/2);
    path.poses.push_back(pose);
  };
  if(points.empty()) {return path;}
  append(points.front());
  for(size_t i=1;i<points.size();++i) {
    const auto & a=points[i-1];const auto & b=points[i];
    // Preserve a pivot as a zero-distance pair, understood by both controllers.
    const int count=std::max(1,int(std::ceil(distance(a,b)/spacing)));
    for(int j=1;j<=count;++j) {
      double t=double(j)/count;append({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.a+t*wrap(b.a-a.a)});
    }
  }
  return path;
}
inline nav_msgs::msg::Path checkedDensePath(const std::vector<Pose2> & route,
  const std_msgs::msg::Header & header, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  auto result=densePath(route,header);
  if(!pathFree(points(result),map,footprint)) {result.poses.clear();}
  return result;
}
// Shorten an omni route, including optional lattice pivots. A pivot can move
// or disappear only when a complete translate/pivot replacement is proven free.
// Required rotations in narrow passages survive because that replacement fails.
// Never apply this operation to a route constrained to Ackermann motion.
inline std::vector<Pose2> shortcutHolonomic(const std::vector<Pose2> & path,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint,
  const std::function<bool()> & out_of_time=[] {return false;})
{
  if(path.empty()) {return {};}
  std::vector<Pose2> result{path.front()};
  for(size_t i=0;i+1<path.size();) {
    if(out_of_time()) {result.insert(result.end(),path.begin()+i+1,path.end());break;}
    size_t next=i+1;
    std::vector<Pose2> replacement;
    for(size_t j=path.size()-1;j>i+1;--j) {
      if(out_of_time()) {break;}
      replacement=directHolonomic(path[i],path[j],map,footprint);
      if(!replacement.empty()) {
        if(!checkedDensePath(replacement,std_msgs::msg::Header{},map,footprint).poses.empty()) {
          next=j;break;
        }
        replacement.clear();
      }
    }
    if(replacement.empty()) {result.push_back(path[next]);}
    else {result.insert(result.end(),replacement.begin()+1,replacement.end());}
    i=next;
  }
  return result;
}
inline bool preferShorterRoute(double a_length,double a_eta,double b_length,double b_eta)
{
  // ETA only breaks numerical ties; it cannot buy a longer route.
  if(std::abs(a_length-b_length)>1e-6) {return a_length<b_length;}
  return a_eta<=b_eta;
}
}
