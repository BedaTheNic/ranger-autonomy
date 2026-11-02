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
  e.ackermann=slip<.16 && e.radius>=limits.radius-.02;
  return e;
}
inline std::vector<bool> schedule(const std::vector<Pose2> & path,const MotionLimits & limits,
  bool continuing_ackermann=false)
{
  std::vector<bool> modes;
  for(size_t i=0;i+1<path.size();++i) {modes.push_back(edge(path,i,limits).ackermann);}
  for(size_t i=0;i<modes.size();) {
    size_t end=i;double length=0;
    while(end<modes.size() && modes[end]==modes[i]) {length+=distance(path[end],path[end+1]);++end;}
    if(modes[i] && length<.65 && (i!=0 || end!=modes.size()) &&
      !(continuing_ackermann && i==0)) {
      for(size_t j=i;j<end;++j) {modes[j]=false;}
    }
    i=end;
  }
  return modes;
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
    if(started && previous!=ack) {++out.switches;}
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
  bool use_mixed{false};
};
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
inline Decision decide(const std::vector<Pose2> & path,const MotionLimits & l,double initial_yaw,
  bool continuing_ackermann=false,bool force_ackermann=false)
{
  Decision d;d.modes=schedule(path,l,continuing_ackermann);
  if(force_ackermann || strictAckermannReference(path,l)) {
    std::fill(d.modes.begin(),d.modes.end(),true);
    d.mixed=estimate(path,d.modes,l,initial_yaw);d.use_mixed=true;
    return d;
  }
  d.mixed=estimate(path,d.modes,l,initial_yaw);
  d.holonomic=estimate(path,std::vector<bool>(d.modes.size(),false),l,initial_yaw);
  // Equal ETA favours a feasible Ackermann segment; no fictitious faster speed.
  d.use_mixed=d.mixed.seconds<=d.holonomic.seconds+.25;
  if(!d.use_mixed) {std::fill(d.modes.begin(),d.modes.end(),false);}
  return d;
}
}
