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
  Grid & grid,const Footprint & footprint,const MotionLimits & limits)
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
  return clearanceBands(raw,grid,footprint)>clearanceBands(smooth,grid,footprint) ?
    HybridChoice::raw : HybridChoice::smoothed;
}

inline double routeEta(const std::vector<Pose2> & path, const MotionLimits & limits)
{
  auto choice = decide(path, limits, path.front().a);
  return std::min(choice.holonomic.seconds, choice.mixed.seconds);
}

// For a straight crab leg compare an explicitly aligned forward/reverse leg
// with the same XY trace. Retain endpoint orientations and all original pivots.
inline unsigned alignStraights(std::vector<Pose2> & path, Grid & grid,
  const Footprint & footprint, const MotionLimits & limits)
{
  unsigned accepted = 0;
  for (size_t first = 0; first + 1 < path.size();) {
    const auto origin = path[first];
    const auto next = path[first+1];
    if (distance(origin,next)<1e-6 || std::abs(wrap(next.a-origin.a))>1e-6) {
      ++first; continue;
    }
    const double bearing = std::atan2(next.y-origin.y,next.x-origin.x);
    size_t end = first+1;
    while (end+1 < path.size()) {
      const auto & a=path[end]; const auto & b=path[end+1];
      if (distance(a,b)<1e-6 || std::abs(wrap(b.a-origin.a))>1e-6 ||
        std::abs(wrap(std::atan2(b.y-a.y,b.x-a.x)-bearing))>1e-5) {break;}
      ++end;
    }
    const double heading = std::cos(bearing-origin.a)>=0 ? bearing : wrap(bearing+M_PI);
    if (distance(origin,path[end])>=.65 && std::abs(wrap(heading-origin.a))>.025) {
      auto aligned=path;
      for(size_t i=first;i<=end;++i) {aligned[i].a=heading;}
      aligned.insert(aligned.begin()+end+1,path[end]);
      aligned.insert(aligned.begin()+first,origin);
      const std::vector<Pose2> leg(aligned.begin()+first,aligned.begin()+end+3);
      if (pathFree(leg,grid,footprint)) {
        const std::vector<Pose2> original(path.begin()+first,path.begin()+end+1);
        const int before=clearanceBands(original,grid,footprint);
        const int after=clearanceBands(leg,grid,footprint);
        // Same local ETA tie window as the mode scheduler. Avoid gratuitous
        // pivots on a diagonal which the base can execute faster as crab.
        if (after>=before && routeEta(aligned,limits)<=routeEta(path,limits)+.25) {
          path=std::move(aligned); end+=2; ++accepted;
        }
      }
    }
    first=end;
  }
  return accepted;
}

inline bool retainedPathFree(const std::vector<Pose2> & path, const Pose2 & robot,
  size_t & cursor, Grid & grid, const Footprint & footprint, Pose2 * invalid=nullptr)
{
  if(path.empty()) {return false;}
  // Bounded forward search keeps an overlapping return branch from skipping
  // an unexecuted reversal. Test the projected remainder, including pivots.
  auto p=project(path,robot,cursor,1.0);
  cursor=p.index;
  std::vector<Pose2> remaining{p.pose};
  remaining.insert(remaining.end(),path.begin()+p.index+1,path.end());
  return pathFree(remaining,grid,footprint,invalid);
}
}
