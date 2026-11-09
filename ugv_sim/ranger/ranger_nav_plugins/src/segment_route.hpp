// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "route_quality.hpp"

namespace ranger_nav_plugins
{
struct TravelPreferences
{
  double corridor_width{1.6};   // wall to wall, metres
  double centering_weight{2.0};
  double front_weight{1.0};
  double obstacle_weight{1.5};
};

inline double bearing(const Pose2 & a, const Pose2 & b)
{return std::atan2(b.y-a.y,b.x-a.x);}

// Soft cost from the actual polygon edge to nearby occupied cell centres.
// Sample along translations and rotations. A narrow but feasible path is
// still eligible when the map offers no room for a wider detour.
inline double edgeExposure(const std::vector<Pose2> & path,Grid & grid,const Footprint & footprint,
  double * minimum_gap=nullptr)
{
  constexpr double preferred=.20;
  if(minimum_gap) {*minimum_gap=preferred;}
  double radius=0.;
  for(const auto & v:footprint) {radius=std::max(radius,std::hypot(v.x,v.y));}
  if(footprint.size()<3 || path.empty()) {return 0.;}
  const double resolution=grid.getResolution(),cell_radius=resolution/std::sqrt(2.);
  const int window=std::ceil((radius+preferred+cell_radius)/resolution);
  double peak=0.,integral=0.;
  for(size_t i=1;i<path.size();++i) {
    const auto & a=path[i-1];const auto & b=path[i];
    const double yaw_delta=wrap(b.a-a.a);
    const double motion=distance(a,b)+radius*std::abs(yaw_delta);
    const int count=std::max(1,static_cast<int>(std::ceil(motion/.06)));
    for(int sample=1;sample<=count;++sample) {
      const double t=(sample-.5)/count;
      const double x=a.x+t*(b.x-a.x),y=a.y+t*(b.y-a.y),yaw=a.a+t*yaw_delta;
      unsigned mx,my;
      if(!grid.worldToMap(x,y,mx,my)) {continue;}
      const double c=std::cos(yaw),s=std::sin(yaw);
      double nearest2=std::pow(preferred+cell_radius,2);
      for(int row=std::max(0,int(my)-window);row<=std::min(int(grid.getSizeInCellsY())-1,int(my)+window);++row) {
        const auto * costs=grid.getCharMap()+size_t(row)*grid.getSizeInCellsX();
        for(int col=std::max(0,int(mx)-window);col<=std::min(int(grid.getSizeInCellsX())-1,int(mx)+window);++col) {
          if(costs[col]<254) {continue;}
          double wx,wy;grid.mapToWorld(col,row,wx,wy);
          const double dx=wx-x,dy=wy-y;
          const double bx=c*dx+s*dy,by=-s*dx+c*dy;
          for(size_t k=0,j=footprint.size()-1;k<footprint.size();j=k++) {
            const auto & v=footprint[j];const auto & w=footprint[k];
            const double ex=w.x-v.x,ey=w.y-v.y,len2=ex*ex+ey*ey;
            if(len2<=1e-12) {continue;}
            const double along=std::clamp(((bx-v.x)*ex+(by-v.y)*ey)/len2,0.,1.);
            const double px=bx-v.x-along*ex,py=by-v.y-along*ey;
            nearest2=std::min(nearest2,px*px+py*py);
          }
        }
      }
      const double gap=std::clamp(std::sqrt(nearest2)-cell_radius,0.,preferred);
      if(minimum_gap) {*minimum_gap=std::min(*minimum_gap,gap);}
      const double loss=1.-gap/preferred;
      peak=std::max(peak,loss*loss);
      integral+=motion/count*loss*loss;
    }
  }
  // A short brush with an obstacle must also influence a long route.
  return 2.*peak+4.*integral;
}

// Signed lateral distance to the midpoint between opposite boundaries. An
// isolated wall or a junction opening is not by itself evidence of a corridor.
inline double corridorOffset(const Pose2 & p, double heading, Grid & grid, double width)
{
  const double nx=-std::sin(heading), ny=std::cos(heading);
  const double step=grid.getResolution()/2;
  auto boundary=[&](double sign) {
    for(double d=step; d<=width; d+=step) {
      unsigned x,y;
      if(!grid.worldToMap(p.x+sign*nx*d,p.y+sign*ny*d,x,y) || grid.getCost(x,y)>=254) {
        return d-step/2;
      }
    }
    return std::numeric_limits<double>::infinity();
  };
  const double left=boundary(1),right=boundary(-1);
  return left+right<=width ? (left-right)/2 : 0.;
}

struct TravelCost
{
  double length{0}, heading_loss{0}, center_loss{0}, rear_distance{0}, inflation_loss{0};
  double score(const TravelPreferences & pref) const
  {return length+pref.front_weight*heading_loss+pref.centering_weight*center_loss;}
};

inline TravelCost travelCost(const std::vector<Pose2> & path, Grid & grid,const TravelPreferences & pref)
{
  TravelCost cost;
  for(size_t i=1;i<path.size();++i) {
    const auto & a=path[i-1];const auto & b=path[i];
    const double ds=distance(a,b);
    if(ds<1e-6) {continue;}
    const double heading=bearing(a,b);
    const Pose2 mid{(a.x+b.x)/2,(a.y+b.y)/2,a.a+.5*wrap(b.a-a.a)};
    unsigned int mx,my;
    if(grid.worldToMap(mid.x,mid.y,mx,my)) {
      const double inflation=grid.getCost(mx,my)/253.;
      cost.inflation_loss+=ds*inflation*inflation;
    }
    cost.length+=ds;
    const double forward_projection=std::cos(heading-mid.a);
    cost.heading_loss+=ds*(1-forward_projection);
    if(forward_projection<-.025) {cost.rear_distance+=ds;}
    cost.center_loss+=ds*2*std::abs(corridorOffset(mid,heading,grid,pref.corridor_width))/pref.corridor_width;
  }
  return cost;
}

inline double tangent(const std::vector<Pose2> & p,size_t i)
{
  if(i==0) {return bearing(p[0],p[1]);}
  if(i+1==p.size()) {return bearing(p[i-1],p[i]);}
  const double incoming=bearing(p[i-1],p[i]);
  return incoming+.5*wrap(bearing(p[i],p[i+1])-incoming);
}

// Estimate the corridor centre across successive cross-sections. Average
// over half a metre to reject cell noise and taper into fixed segment ends.
inline bool centerLeg(std::vector<Pose2> & leg,Grid & grid,const TravelPreferences & pref,
  bool center_ends=false)
{
  if(leg.size()<2) {return false;}
  std::vector<double> along(leg.size()),offset(leg.size()),angle(leg.size());
  for(size_t i=0;i<leg.size();++i) {
    if(i) {along[i]=along[i-1]+distance(leg[i-1],leg[i]);}
    angle[i]=tangent(leg,i);
    offset[i]=corridorOffset(leg[i],angle[i],grid,pref.corridor_width);
  }
  bool changed=false;
  for(size_t i=0;i<leg.size();++i) {
    double sum=0,weights=0;
    const auto first=std::lower_bound(along.begin(),along.end(),along[i]-.5);
    for(size_t j=first-along.begin();j<leg.size() && along[j]<along[i]+.5;++j) {
      const double weight=(.5-std::abs(along[j]-along[i]))*
        (along[std::min(j+1,leg.size()-1)]-along[j?j-1:0]);
      sum+=weight*offset[j];weights+=weight;
    }
    const double t=center_ends ? 1. : std::min({1.,along[i],along.back()-along[i]});
    const double shift=weights>0 ? sum/weights*t*t*(3-2*t) : 0.;
    leg[i].x-=std::sin(angle[i])*shift;
    leg[i].y+=std::cos(angle[i])*shift;
    changed|=std::abs(shift)>grid.getResolution()/2;
  }
  return changed;
}

inline void appendPose(std::vector<Pose2> & path,const Pose2 & p)
{
  if(path.empty() || distance(path.back(),p)>1e-8 || std::abs(wrap(path.back().a-p.a))>1e-6) {
    path.push_back(p);
  }
}

// Try travelling several bends with one body heading. Only the endpoint
// rotations survive. The caller validates the entire footprint sweep.
inline std::vector<Pose2> constantHeadingRoute(const std::vector<Pose2> & path,double heading)
{
  std::vector<Pose2> result{path.front()};
  for(const auto & p:path) {appendPose(result,{p.x,p.y,heading});}
  appendPose(result,path.back());
  return result;
}

// Small grid or localization yaw differences can introduce a duplicate XY
// goal with a sub-degree pivot. Preserve both requested endpoint poses and
// recheck the swept path after merging these numerical turns.
inline std::vector<Pose2> mergeSmallPivots(const std::vector<Pose2> & path)
{
  if(path.size()<3) {return path;}
  constexpr double small_turn=.015;
  std::vector<Pose2> result{path.front()};
  for(size_t i=1;i+1<path.size();++i) {
    const auto & p=path[i];
    if((distance(p,result.back())<1e-6 && std::abs(wrap(p.a-result.back().a))<small_turn) ||
      (distance(p,path.back())<1e-6 && i+2==path.size() &&
       std::abs(wrap(p.a-path.back().a))<small_turn)) {continue;}
    result.push_back(p);
  }
  result.push_back(path.back());
  return result;
}

// Minimize straight translations through ordered points on the searched
// route at one body heading. A shortcut exists only if its full sweep fits.
// The bound limits collision-checking work while the costmap is locked.
inline std::vector<Pose2> minimumLegRoute(const std::vector<Pose2> & path,double body,
  Grid & grid,const Footprint & footprint)
{
  if(path.size()<3 || !rotationFree(path.front(),body,grid,footprint) ||
    !rotationFree(path.back(),body,grid,footprint)) {return {};}
  const double spacing=std::max(.12,pathLength(path)/18.);
  std::vector<Pose2> anchors{path.front()};
  for(size_t i=1;i+1<path.size();++i) {
    if(distance(anchors.back(),path[i])>=spacing ||
      (sharpCorner(path,i) && distance(anchors.back(),path[i])>.06)) {
      anchors.push_back(path[i]);
    }
  }
  anchors.push_back(path.back());
  if(anchors.size()>28) {
    std::vector<Pose2> subset;
    for(size_t i=0;i<28;++i) {subset.push_back(anchors[i*(anchors.size()-1)/27]);}
    anchors=std::move(subset);
  }
  for(auto & p:anchors) {p.a=body;}
  const size_t n=anchors.size();
  std::vector<size_t> legs(n,n),parent(n);
  std::vector<double> length(n,INFINITY);
  legs[0]=0;length[0]=0;
  for(size_t end=1;end<n;++end) {
    for(size_t start=0;start<end;++start) {
      const double candidate=length[start]+distance(anchors[start],anchors[end]);
      if(legs[start]+1>legs[end] ||
        (legs[start]+1==legs[end] && candidate>=length[end])) {continue;}
      if(!pathFree({anchors[start],anchors[end]},grid,footprint)) {continue;}
      legs[end]=legs[start]+1;length[end]=candidate;parent[end]=start;
    }
  }
  if(legs.back()==n) {return {};}
  std::vector<Pose2> vertices;
  for(size_t i=n-1;i!=0;i=parent[i]) {vertices.push_back(anchors[i]);}
  vertices.push_back(anchors[0]);
  std::reverse(vertices.begin(),vertices.end());
  vertices.front().a=path.front().a;vertices.back().a=path.back().a;
  return constantHeadingRoute(vertices,body);
}

struct RouteRank
{
  unsigned switches;
  size_t segments;
  double seconds, rear_distance;
  double gap{0.};
};

// Minimize controller changes, then avoid sustained rearward travel. Within
// the resulting time budget, prefer fewer maneuvers.
inline size_t selectSchedule(const std::vector<RouteRank> & routes,double slack)
{
  // Prefer tracking room where the environment offers it. These are relative
  // rankings; even a zero-clearance candidate is eligible in a tight passage.
  double best_gap=0.;
  for(const auto & r:routes) {best_gap=std::max(best_gap,r.gap);}
  const double target=std::min(.16,best_gap)-.025;
  unsigned switches=std::numeric_limits<unsigned>::max();
  for(const auto & r:routes) {if(r.gap>=target) {switches=std::min(switches,r.switches);}}
  double least_rear=INFINITY;
  for(const auto & r:routes) {
    if(r.gap>=target && r.switches==switches) {least_rear=std::min(least_rear,r.rear_distance);}
  }
  const double rear_budget=least_rear+.20;
  double fastest=INFINITY;
  for(const auto & r:routes) {
    if(r.gap>=target && r.rear_distance<=rear_budget && r.switches==switches) {fastest=std::min(fastest,r.seconds);}
  }
  size_t best=routes.size();
  for(size_t i=0;i<routes.size();++i) {
    const auto & r=routes[i];
    if(r.gap<target || r.rear_distance>rear_budget || r.switches!=switches || r.seconds>fastest*(1+slack)+1e-6) {continue;}
    if(best==routes.size() || r.segments<routes[best].segments ||
      (r.segments==routes[best].segments && r.seconds<routes[best].seconds)) {best=i;}
  }
  return best;
}

// A blind exit or tight goal may rule out a turn at an endpoint. Within
// each connected forward-clear span, turn at the first and last poses with
// room for the full sweep. Retain the reference heading on the approaches.
inline std::vector<Pose2> faceInClearSpans(const std::vector<Pose2> & body,
  const std::vector<Pose2> & forward,Grid & grid,const Footprint & footprint)
{
  std::vector<Pose2> result;
  for(size_t i=0;i<body.size();) {
    size_t end=i;
    while(end+1<body.size() && pathFree({forward[end],forward[end+1]},grid,footprint)) {++end;}
    auto can_turn=[&](size_t k) {return rotationFree(body[k],forward[k].a,grid,footprint);};
    size_t entry=i,exit=end;
    while(entry<exit && !can_turn(entry)) {++entry;}
    while(exit>entry && !can_turn(exit)) {--exit;}
    if(entry<exit) {
      for(size_t j=i;j<=entry;++j) {appendPose(result,body[j]);}
      for(size_t j=entry;j<=exit;++j) {appendPose(result,forward[j]);}
      for(size_t j=exit;j<=end;++j) {appendPose(result,body[j]);}
    } else {
      for(size_t j=i;j<=end;++j) {appendPose(result,body[j]);}
    }
    i=end+1;
  }
  return result;
}

// Keep the original path as a feasible fallback. A leg ends at a planned
// pivot, cusp or corner. Trying both body headings also handles a slot where
// translation fits but the complete sweep of the proposed pivot does not.
inline std::vector<Pose2> shapeTravelLegs(const std::vector<Pose2> & route, Grid & grid,
  const Footprint & footprint,const TravelPreferences & pref)
{
  if(route.size()<2) {return route;}
  auto dense=points(densePath(route,std_msgs::msg::Header{},.065));
  std::vector<Pose2> result;
  appendPose(result,dense.front());
  for(size_t first=0;first+1<dense.size();) {
    size_t end=first+1;
    if(distance(dense[first],dense[end])<1e-5) {
      appendPose(result,dense[end]);first=end;continue;
    }
    while(end+1<dense.size() && distance(dense[end],dense[end+1])>=1e-5 &&
      std::abs(wrap(bearing(dense[end],dense[end+1])-bearing(dense[end-1],dense[end])))<.22) {
      ++end;
    }
    std::vector<Pose2> leg(dense.begin()+first,dense.begin()+end+1);
    auto best=leg;
    double best_cost=travelCost(leg,grid,pref).score(pref);
    auto accept=[&](const std::vector<Pose2> & candidate) {
      std::vector<Pose2> trial{dense[first]};
      // A sideways connection to a turning location has constant body yaw.
      appendPose(trial,{candidate.front().x,candidate.front().y,dense[first].a});
      for(const auto & p:candidate) {appendPose(trial,p);}
      appendPose(trial,{candidate.back().x,candidate.back().y,dense[end].a});
      appendPose(trial,dense[end]);
      const double cost=travelCost(trial,grid,pref).score(pref);
      if(cost+1e-5>=best_cost) {return true;}
      if(!pathFree(trial,grid,footprint)) {return false;}
      best=std::move(trial);best_cost=cost;
      return true;
    };
    auto face_forward=[&](const std::vector<Pose2> & body) {
      if(pref.front_weight==0) {return;}
      auto forward=body;
      for(size_t i=0;i<forward.size();++i) {forward[i].a=tangent(body,i);}
      if(!accept(forward)) {accept(faceInClearSpans(body,forward,grid,footprint));}
    };
    if(pathLength(leg)>=.12 || (first==0 && end+1==dense.size())) {
      face_forward(leg);
      if(pref.centering_weight>0) {
        for(bool center_ends:{false,true}) {
          auto centered=leg;
          if(centerLeg(centered,grid,pref,center_ends)) {
            face_forward(centered);
            accept(centered);
          }
        }
      }
    }
    for(const auto & p:best) {appendPose(result,p);}
    first=end;
  }
  // Two adjacent forward-facing legs often share an obsolete intermediate
  // heading. Remove it only if the direct rotation is swept-footprint safe.
  std::vector<Pose2> joined;
  for(size_t i=0;i<result.size();) {
    size_t j=i;
    while(j+1<result.size() && distance(result[i],result[j+1])<1e-7) {++j;}
    appendPose(joined,result[i]);
    if(j>i && rotationFree(result[i],result[j].a,grid,footprint)) {appendPose(joined,result[j]);}
    else {for(size_t k=i+1;k<=j;++k) {appendPose(joined,result[k]);}}
    i=j+1;
  }
  return joined;
}
}
