// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "holonomic_path.hpp"
#include "mode_policy.hpp"

namespace ranger_nav_plugins
{
using Footprint = nav2_costmap_2d::Footprint;
using Grid = nav2_costmap_2d::Costmap2D;

// Clearance in additional half-cell padding bands. Only compare equal-distance
// alternatives, and cap open-space clearance so ETA breaks open-space ties.
inline int clearanceBands(const std::vector<Pose2> & path, Grid & grid, const Footprint & base)
{
  int low = 0, high = 12;
  while (low < high) {
    const int middle = (low + high + 1) / 2;
    auto expanded = base;
    nav2_costmap_2d::padFootprint(expanded, middle * grid.getResolution() / 2);
    if (pathFree(path, grid, expanded)) {low = middle;} else {high = middle - 1;}
  }
  return low;
}

enum class HybridChoice {none, raw, smoothed};

inline bool validCarRoute(const std::vector<Pose2> & path, Grid & grid,const Footprint & footprint,
  const MotionLimits & limits)
{
  if(!pathFree(path,grid,footprint)) {return false;}
  for(size_t i=0;i+1<path.size();++i) {
    if(!edge(path,i,limits).ackermann) {return false;}
  }
  return true;
}

inline std::vector<int> gears(const std::vector<Pose2> & path,const MotionLimits & limits)
{
  std::vector<int> result;
  for(size_t i=0;i+1<path.size();++i) {
    const auto e=edge(path,i,limits);
    if(e.length>1e-5 && (result.empty() || result.back()!=e.direction)) {result.push_back(e.direction);}
  }
  return result;
}

inline HybridChoice selectHybrid(const std::vector<Pose2> & raw, const std::vector<Pose2> & smooth,
  Grid & grid,const Footprint & footprint,const MotionLimits & limits, bool compare_clearance=true)
{
  const bool raw_ok=validCarRoute(raw,grid,footprint,limits);
  const bool smooth_ok=!raw.empty() && !smooth.empty() &&
    distance(raw.front(),smooth.front())<1e-5 && distance(raw.back(),smooth.back())<1e-5 &&
    std::abs(wrap(raw.front().a-smooth.front().a))<1e-4 &&
    std::abs(wrap(raw.back().a-smooth.back().a))<1e-4 &&
    gears(raw,limits)==gears(smooth,limits) && validCarRoute(smooth,grid,footprint,limits);
  if(!smooth_ok) {return raw_ok ? HybridChoice::raw : HybridChoice::none;}
  if(!raw_ok) {return HybridChoice::smoothed;}
  const double difference=pathLength(smooth)-pathLength(raw);
  if(std::abs(difference)>1e-6) {return difference<0 ? HybridChoice::smoothed : HybridChoice::raw;}
  return compare_clearance && clearanceBands(raw,grid,footprint)>clearanceBands(smooth,grid,footprint) ?
    HybridChoice::raw : HybridChoice::smoothed;
}

inline bool retainedPathFree(const std::vector<Pose2> & path, const Pose2 & robot,
  size_t & cursor, Grid & grid, const Footprint & footprint, Pose2 * invalid=nullptr,size_t * invalid_index=nullptr)
{
  if(path.empty()) {return false;}
  // Bounded forward search keeps an overlapping return branch from skipping
  // an unexecuted reversal. Test the projected remainder, including pivots.
  auto p=project(path,robot,cursor,1.0);
  cursor=p.index;
  std::vector<Pose2> remaining{p.pose};
  remaining.insert(remaining.end(),path.begin()+p.index+1,path.end());
  size_t relative=0;
  const bool clear=pathFree(remaining,grid,footprint,invalid,&relative);
  if(!clear && invalid_index) {*invalid_index=p.index+relative;}
  return clear;
}
}
