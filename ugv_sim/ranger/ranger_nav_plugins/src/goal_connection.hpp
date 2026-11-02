// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "navigation_geometry.hpp"

namespace ranger_nav_plugins
{
struct GoalConnection
{
  unsigned int x,y,heading;
  std::vector<Pose2> poses;
};

// Reach a nearby lattice pose, pivot there, then translate at the exact
// requested heading. Never substitute a nearby point for the user's goal.
inline std::optional<GoalConnection> goalConnection(const Pose2 & goal,
  nav2_costmap_2d::Costmap2D & map,const nav2_costmap_2d::Footprint & footprint,
  const nav2_costmap_2d::Footprint & anchor_footprint,
  const std::vector<double> & headings,double maximum_distance)
{
  unsigned int gx,gy;
  if(!std::isfinite(maximum_distance)||maximum_distance<=0 || headings.empty() ||
    !poseFree(goal,map,footprint) || !map.worldToMap(goal.x,goal.y,gx,gy)) {return std::nullopt;}
  const int cells=std::ceil(maximum_distance/map.getResolution());
  double best=INFINITY;std::optional<GoalConnection> result;
  for(int dy=-cells;dy<=cells;++dy) {
    for(int dx=-cells;dx<=cells;++dx) {
      const int x=int(gx)+dx,y=int(gy)+dy;
      if(x<0||y<0||x>=int(map.getSizeInCellsX())||y>=int(map.getSizeInCellsY())) {continue;}
      Pose2 anchor;map.mapToWorld(x,y,anchor.x,anchor.y);
      const double length=distance(anchor,goal);
      if(length>maximum_distance) {continue;}
      for(unsigned int bin=0;bin<headings.size();++bin) {
        anchor.a=headings[bin];const double angle=std::abs(wrap(anchor.a-goal.a));
        // Keep the connector local in both position and orientation.
        if(angle>M_PI_4) {continue;}
        const double score=length+.15*angle;
        if(score>=best || !poseFree(anchor,map,anchor_footprint)) {continue;}
        std::vector<Pose2> connection{anchor,{anchor.x,anchor.y,goal.a},goal};
        if(!pathFree(connection,map,footprint)) {continue;}
        best=score;result=GoalConnection{unsigned(x),unsigned(y),bin,std::move(connection)};
      }
    }
  }
  return result;
}
}
