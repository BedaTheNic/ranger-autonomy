// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "navigation_geometry.hpp"

namespace ranger_nav_plugins
{
struct MotionLimits
{
  double radius{.8}, forward{.2}, reverse{.15}, lateral{.2}, angular{.25};
  double steering_rate{1.0}, acceleration{.3}, deceleration{.4}, wheelbase{.46};
};
struct Edge
{
  double length{0}, curvature{0}, radius{INFINITY}, heading{0};
  int direction{1};
  bool ackermann{false};
};
inline Edge edge(const std::vector<Pose2> & path,size_t i,const MotionLimits & limits)
{
  Edge e; const auto & a=path[i]; const auto & b=path[i+1];
  const double dx=b.x-a.x,dy=b.y-a.y;
  e.length=std::hypot(dx,dy);e.heading=std::atan2(dy,dx);
  const double da=wrap(b.a-a.a);
  if(e.length<1e-5) {e.ackermann=std::abs(da)<1e-4;return e;}
  e.direction=std::cos(e.heading-a.a)>=0?1:-1;
  const double slip=std::abs(wrap(e.heading-a.a-da/2-(e.direction<0?M_PI:0)));
  e.curvature=da/(e.length*e.direction);
  double curvature=std::abs(e.curvature);
  // Include geometric curvature, not only quaternion changes. A sharp corner
  // with constant chassis heading must not be mistaken for a straight car path.
  if(i+2<path.size()) {
    const double next=distance(b,path[i+2]);
    if(next>1e-5) {
      const double turn=std::abs(wrap(std::atan2(path[i+2].y-b.y,path[i+2].x-b.x)-e.heading));
      if(turn<M_PI/2) {curvature=std::max(curvature,turn/((e.length+next)/2));}
      else if(turn<M_PI-.15) {curvature=INFINITY;}
    }
  }
  e.radius=curvature>1e-6?1/curvature:INFINITY;
  // A fixed-heading crab translation can have infinite radius yet still
  // require lateral velocity. Allow rounding, not sustained sideways slip.
  // Hybrid-A* quantizes chassis headings. A short constant-bin piece
  // next to a translating turn is part of the arc, not a long crab leg.
  auto adjacent_turn=[&](int step) {
    size_t j=step>0?i+1:i;
    for(double span=0;span<.25;) {
      if((step>0 && j+1>=path.size()) || (step<0 && j==0)) {break;}
      size_t k=step>0?j+1:j-1;
      double ds=distance(path[k],path[j]);
      if(ds<1e-5) {break;}  // an explicit pivot separates two legs
      if(std::abs(wrap(path[k].a-path[j].a))>1e-6) {return true;}
      span+=ds;j=k;
    }
    return false;
  };
  const bool quantized_arc=e.length<.15 && (adjacent_turn(-1) || adjacent_turn(1));
  const double slip_tolerance = std::abs(da) < 1e-6 ? (quantized_arc ? .07 : .025) : .16;
  e.ackermann=slip<slip_tolerance && e.radius>=limits.radius-.02;
  return e;
}
// Finish a short, monotonic terminal section holonomically so yaw alignment
// does not drive an Ackermann arc away from an already reached goal position.
inline bool finalHolonomicApproach(const std::vector<Pose2> & path,
  const Projection & p,const Pose2 & robot,const MotionLimits & limits)
{
  if(path.empty() || distance(robot,path.back())>=.18 || nextPivot(path,p.index)) {return false;}
  double length=0;int direction=0;
  for(size_t i=p.index;i+1<path.size();++i) {
    const auto e=edge(path,i,limits);
    length+=i==p.index?distance(p.pose,path[i+1]):e.length;
    if(length>.30) {return false;}
    if(e.length<1e-5) {continue;}
    if(direction && direction!=e.direction) {return false;}
    direction=e.direction;
  }
  return true;
}
inline double steeringChange(double a,double b)
{return std::abs(.5*wrap(2*(a-b)));}
struct Estimate {double seconds{INFINITY}, length{0}; unsigned switches{0}, reversals{0};};
inline Estimate estimate(const std::vector<Pose2> & path,const std::vector<bool> & modes,
  const MotionLimits & l,double initial_yaw)
{
  Estimate out;out.seconds=0;
  double body=initial_yaw,steering=0;bool previous=false,started=false;int direction=0;
  for(size_t i=0;i+1<path.size();++i) {
    auto e=edge(path,i,l);
    const bool ack=modes[i];
    if(ack && !e.ackermann) {out.seconds=INFINITY;return out;}
    if(i && modes[i-1]!=ack) {++out.switches;}
    if(e.length<1e-5) {
      double turn=std::abs(wrap(path[i+1].a-body));
      if(turn>1e-4) {out.seconds+=turn/l.angular;body=path[i+1].a;}
      continue;
    }
    double speed=ack?(e.direction>0?l.forward:l.reverse):l.lateral;
    double desired;
    if(ack) {
      if(!started || !previous) {out.seconds+=std::abs(wrap(path[i].a-body))/l.angular;}
      if(direction!=0 && previous && direction!=e.direction) {
        out.seconds+=speed/l.deceleration+speed/l.acceleration;++out.reversals;
      }
      speed=std::min(speed,l.angular/std::max(std::abs(e.curvature),1e-6));
      desired=std::atan(l.wheelbase/2*e.curvature);body=path[i+1].a;
    } else {
      // Holonomic execution now honours planned body headings with swept
      // in-place alignment before a difficult translation.
      out.seconds+=std::abs(wrap(path[i].a-body))/l.angular;
      body=path[i].a;desired=wrap(e.heading-body);
    }
    // Wheels can align while stationary or while driving within their rate.
    // Translation and steering time are conservatively summed for crab changes.
    out.seconds+=e.length/speed;
    out.seconds+=steeringChange(desired,steering)/l.steering_rate;
    out.length+=e.length;steering=desired;direction=e.direction;
    previous=ack;started=true;
  }
  if(!path.empty()) {out.seconds+=std::abs(wrap(path.back().a-body))/l.angular;}
  if(out.length>0) {out.seconds+=.5*l.forward/l.acceleration+.5*l.forward/l.deceleration;}
  return out;
}
struct Decision
{
  std::vector<bool> modes;
  Estimate mixed,holonomic;
};
// Consecutive samples share a segment up to a mode change, pivot, corner
// or reversal. The result is the pose index of the segment goal.
inline size_t segmentEnd(const std::vector<Pose2> & path,const std::vector<bool> & modes,size_t first)
{
  const bool pivot=distance(path[first],path[first+1])<1e-5;
  size_t end=first+1;
  while(end<modes.size() && modes[end]==modes[first]) {
    if(pivot!=(distance(path[end],path[end+1])<1e-5)) {break;}
    if(!pivot) {
      const auto & a=path[end-1];const auto & b=path[end];const auto & c=path[end+1];
      const double turn=wrap(std::atan2(c.y-b.y,c.x-b.x)-std::atan2(b.y-a.y,b.x-a.x));
      if(std::abs(turn)>.7) {break;}
    }
    ++end;
  }
  return end;
}
// A car reference with changing body heading or a direction cusp must be
// executed with its pose/direction sequence. Crab-following only its XY trace
// loses that sequence at overlapping forward/reverse branches.
inline bool strictAckermannReference(const std::vector<Pose2> & path,const MotionLimits & l)
{
  if(path.size()<2) {return false;}
  bool oriented=false;int direction=0;
  for(size_t i=0;i+1<path.size();++i) {
    const auto e=edge(path,i,l);
    if(!e.ackermann) {return false;}
    if(e.length<1e-5) {continue;}
    oriented|=std::abs(wrap(path[i+1].a-path.front().a))>.08 ||
      (direction!=0 && direction!=e.direction);
    direction=e.direction;
  }
  return oriented;
}
// A non-car edge requires holonomic mode. Between these required sections,
// a straight can stay holonomic at either end without paying a handoff.
// Preserve car curves and gear sequences that crab tracking cannot execute.
inline Decision decide(const std::vector<Pose2> & path,const MotionLimits & l,double initial_yaw,
  std::optional<bool> entering_car=std::nullopt)
{
  Decision d;
  for(size_t i=0;i+1<path.size();++i) {d.modes.push_back(edge(path,i,l).ackermann);}
  const bool all_car=std::all_of(d.modes.begin(),d.modes.end(),[](bool car){return car;});
  bool requires_car=false;
  for(size_t first=0;first<d.modes.size();) {
    if(!d.modes[first]) {++first;continue;}
    size_t end=first+1;
    while(end<d.modes.size() && d.modes[end]) {++end;}
    const std::vector<Pose2> run(path.begin()+first,path.begin()+end+1);
    if(strictAckermannReference(run,l)) {
      requires_car=true;
    } else if(!all_car) {
      std::fill(d.modes.begin()+first,d.modes.begin()+end,false);
    }
    first=end;
  }
  d.mixed=estimate(path,d.modes,l,initial_yaw);
  if(!requires_car) {
    std::vector<bool> omni(d.modes.size(),false);
    d.holonomic=estimate(path,omni,l,initial_yaw);
    const bool use_holonomic=all_car && entering_car.has_value() ?
      !*entering_car : d.holonomic.seconds<d.mixed.seconds-1e-6;
    if(use_holonomic) {
      d.modes=std::move(omni);d.mixed=d.holonomic;
    }
  }
  return d;
}
}
