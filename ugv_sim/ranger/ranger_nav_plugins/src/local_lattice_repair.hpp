// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "holonomic_path.hpp"

namespace ranger_nav_plugins
{
// Smac's bin checks can accept a short edge whose interpolated rectangle
// clips a cell. Repair only that neighbourhood, keeping both attachment poses.
inline bool repairLatticeEdge(std::vector<Pose2> & path, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, const std::function<bool()> & expired)
{
  size_t bad=0;
  while(bad+1<path.size() && pathFree({path[bad],path[bad+1]},map,footprint)) {++bad;}
  if(bad+1==path.size()) {return true;}
  struct Alternative {double length;Pose2 pose;};
  for(double reach : {.25,.50,.85}) {
    if(expired()) {return false;}
    size_t first=bad,last=bad+1;
    double d=0;
    while(first>0 && (d<reach || !poseFree(path[first],map,footprint))) {
      d+=distance(path[first],path[first-1]);--first;
    }
    d=0;
    while(last+1<path.size() && (d<reach || !poseFree(path[last],map,footprint))) {
      d+=distance(path[last],path[last+1]);++last;
    }
    const Pose2 a=path[first],b=path[last];
    if(!poseFree(a,map,footprint) || !poseFree(b,map,footprint)) {continue;}
    const double tangent=std::atan2(b.y-a.y,b.x-a.x);
    std::vector<Alternative> options;
    for(double along : {.25,.5,.75}) {
      for(int shift=-8;shift<=8;++shift) {
        const double offset=shift*map.getResolution();
        Pose2 anchor{a.x+along*(b.x-a.x)-offset*std::sin(tangent),
          a.y+along*(b.y-a.y)+offset*std::cos(tangent),0};
        for(double heading : {a.a,b.a,tangent,path[bad].a}) {
          anchor.a=heading;
          options.push_back({distance(a,anchor)+distance(anchor,b),anchor});
        }
      }
    }
    std::stable_sort(options.begin(),options.end(),[](const Alternative & x,const Alternative & y) {
      return x.length<y.length;
    });
    for(const auto & opt:options) {
      if(expired()) {return false;}
      auto incoming=directHolonomic(a,opt.pose,map,footprint);
      if(incoming.empty()) {continue;}
      auto outgoing=directHolonomic(opt.pose,b,map,footprint);
      if(outgoing.empty()) {continue;}
      incoming.insert(incoming.end(),outgoing.begin()+1,outgoing.end());
      const auto replacement=checkedDensePath(incoming,std_msgs::msg::Header{},map,footprint);
      if(replacement.poses.empty()) {continue;}
      auto segment=points(replacement);
      path.erase(path.begin()+first,path.begin()+last+1);
      path.insert(path.begin()+first,segment.begin(),segment.end());
      return true;
    }
  }
  return false;
}
}
