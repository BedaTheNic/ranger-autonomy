// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <queue>
#include <memory>
#include <opencv2/imgproc.hpp>
#include "segment_route.hpp"

namespace ranger_nav_plugins
{
// The first scan of an object sees only its near face. Project that observed
// component a short distance behind the returns for candidate ranking only.
// Never write inferred cells into the master, retained-path or local costmap.
inline std::unique_ptr<Grid> occlusionRiskGrid(Grid & measured,const BlockingCell & hit,
  const Pose2 & viewpoint,double depth)
{
  unsigned cx,cy;
  if(depth<=0. || hit.cost!=254 || !measured.worldToMap(hit.x,hit.y,cx,cy) ||
    measured.getCost(cx,cy)!=254) {return nullptr;}
  auto risk=std::make_unique<Grid>(measured);
  const double cell=measured.getResolution();
  const int radius=std::ceil(2.*depth/cell);
  const int xmin=std::max(0,int(cx)-radius),ymin=std::max(0,int(cy)-radius);
  const int xmax=std::min(int(measured.getSizeInCellsX())-1,int(cx)+radius);
  const int ymax=std::min(int(measured.getSizeInCellsY())-1,int(cy)+radius);
  const int width=xmax-xmin+1;
  std::vector<bool> visited(size_t(width)*(ymax-ymin+1));
  std::queue<std::pair<int,int>> queue;
  std::vector<std::pair<int,int>> surface;
  int left=cx,right=cx,bottom=cy,top=cy;
  queue.emplace(cx,cy);visited[(cy-ymin)*width+cx-xmin]=true;
  while(!queue.empty()) {
    const auto [x,y]=queue.front();queue.pop();
    surface.emplace_back(x,y);
    left=std::min(left,x);right=std::max(right,x);bottom=std::min(bottom,y);top=std::max(top,y);
    for(int ny=std::max(ymin,y-1);ny<=std::min(ymax,y+1);++ny) {
      for(int nx=std::max(xmin,x-1);nx<=std::min(xmax,x+1);++nx) {
        const int i=(ny-ymin)*width+nx-xmin;
        if(!visited[i] && measured.getCost(nx,ny)==254) {
          visited[i]=true;queue.emplace(nx,ny);
        }
      }
    }
  }
  // Two visible faces already delimit part of the object's depth. Avoid
  // extrapolating its full depth a second time beyond observed side returns.
  const int depth_cells=std::ceil(depth/cell);
  const int missing_x=std::max(0,depth_cells-(right-left+1));
  const int missing_y=std::max(0,depth_cells-(top-bottom+1));
  if(hit.x>viewpoint.x) {right+=missing_x;} else {left-=missing_x;}
  if(hit.y>viewpoint.y) {top+=missing_y;} else {bottom-=missing_y;}
  for(const auto & [x,y]:surface) {
    double wx,wy;measured.mapToWorld(x,y,wx,wy);
    const double dx=wx-viewpoint.x,dy=wy-viewpoint.y,len=std::hypot(dx,dy);
    if(len<=cell) {continue;}
    for(double d=.5*cell;d<=depth+.001;d+=.5*cell) {
      unsigned mx,my;
      if(!measured.worldToMap(wx+d*dx/len,wy+d*dy/len,mx,my)) {break;}
      mx=std::clamp(int(mx),left,right);my=std::clamp(int(my),bottom,top);
      if(risk->getCost(mx,my)<254) {risk->setCost(mx,my,254);}
    }
  }
  return risk;
}

// Search a soft clearance gradient even when Smac already found a shorter path.
// The final candidate still goes through the ordinary continuous polygon check.
inline std::vector<Pose2> clearanceRoute(const Pose2 & begin, const Pose2 & finish,
  Grid & grid, const Footprint & footprint, double heading,const Grid * risk=nullptr)
{
  unsigned sx, sy, gx, gy;
  if (!grid.worldToMap(begin.x,begin.y,sx,sy) || !grid.worldToMap(finish.x,finish.y,gx,gy)) {return {};}
  if (footprint.size()<3 || !rotationFree(begin,heading,grid,footprint) ||
    !rotationFree(finish,heading,grid,footprint)) {return {};}
  const double cell=grid.getResolution();
  // Limit the optional search to a local patch. The native planners still
  // explore the complete map for routes that need a larger detour.
  const int margin=std::ceil(2./cell);
  const int ox=std::max(0,int(std::min(sx,gx))-margin), oy=std::max(0,int(std::min(sy,gy))-margin);
  const int w=std::min(int(grid.getSizeInCellsX()),int(std::max(sx,gx))+margin+1)-ox;
  const int h=std::min(int(grid.getSizeInCellsY()),int(std::max(sy,gy))+margin+1)-oy;
  // The search is an optional candidate and runs under the master grid lock.
  if (size_t(w)*h>160000) {return {};}
  sx-=ox;gx-=ox;sy-=oy;gy-=oy;
  const int start=sy*w+sx, goal=gy*w+gx;
  if (start==goal) {return {};}
  double radius=0.;
  for (const auto & p:footprint) {radius=std::max(radius,std::hypot(p.x,p.y));}
  const int half=std::ceil(radius/cell)+1;
  cv::Mat kernel=cv::Mat::zeros(2*half+1,2*half+1,CV_8U);
  std::vector<cv::Point> vertices;
  const double c=std::cos(heading),s=std::sin(heading);
  for (const auto & p:footprint) {
    vertices.emplace_back(std::lround(16*(half+(c*p.x-s*p.y)/cell)),
      std::lround(16*(half+(s*p.x+c*p.y)/cell)));
  }
  cv::fillPoly(kernel,std::vector<std::vector<cv::Point>>{vertices},cv::Scalar(1),cv::LINE_8,4);
  cv::Mat global(grid.getSizeInCellsY(),grid.getSizeInCellsX(),CV_8U,grid.getCharMap());
  const cv::Mat costs=global(cv::Rect(ox,oy,w,h));
  cv::Mat free;
  cv::erode(costs<254,free,kernel,cv::Point(half,half),1,cv::BORDER_CONSTANT,cv::Scalar(0));
  free.setTo(0,costs>=253);
  if (!free.ptr()[start] || !free.ptr()[goal]) {return {};}
  cv::Mat field;
  cv::distanceTransform(free,field,cv::DIST_L2,cv::DIST_MASK_5);
  cv::Mat risk_field;
  if(risk) {
    // A possible hidden surface only changes the search cost. Even where
    // its estimated clearance is zero, the measured free space stays open.
    cv::Mat image(h,w,CV_8U,cv::Scalar(0));
    for(int y=0;y<h;++y) {
      const auto * src=risk->getCharMap()+size_t(y+oy)*risk->getSizeInCellsX()+ox;
      std::copy_n(src,w,image.ptr<unsigned char>(y));
    }
    cv::Mat risk_free;
    cv::erode(image<254,risk_free,kernel,cv::Point(half,half),1,cv::BORDER_CONSTANT,cv::Scalar(0));
    cv::distanceTransform(risk_free,risk_field,cv::DIST_L2,cv::DIST_MASK_5);
  }
  const float * cells=field.ptr<float>();
  const float * predicted=risk_field.empty()?nullptr:risk_field.ptr<float>();
  constexpr double comfortable=.20;
  auto clearance=[&](int index) {
    const double gap=std::min(comfortable,double(cells[index])*cell-.5*cell);
    if(!predicted) {return gap;}
    return .5*(gap+std::min(gap,double(predicted[index])*cell-.5*cell));
  };
  const double diagonal=std::sqrt(2.)*cell;
  auto heuristic=[&](int index) {
    return cell*std::hypot(double(index%w)-gx,double(index/w)-gy);
  };
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  std::vector<double> cost(size_t(w)*h,INFINITY);
  std::vector<int> parent(cost.size(),-1);
  cost[start]=0.;open.emplace(heuristic(start),start);
  while (!open.empty()) {
    const auto [priority,i]=open.top();open.pop();
    if (priority>cost[i]+heuristic(i)+1.e-8) {continue;}
    if (i==goal) {break;}
    const int x=i%w,y=i/w;
    for (int dy=-1;dy<=1;++dy) {
      for (int dx=-1;dx<=1;++dx) {
        if ((!dx && !dy) || x+dx<0 || x+dx>=w || y+dy<0 || y+dy>=h) {continue;}
        const int j=i+dy*w+dx;
        if (!free.ptr()[j] || (dx && dy && (!free.ptr()[i+dx] || !free.ptr()[i+dy*w]))) {continue;}
        const double deficit=std::max(0.,comfortable-std::min(clearance(i),clearance(j)))/comfortable;
        const double proposed=cost[i]+(dx && dy?diagonal:cell)*(1.+14.*deficit*deficit);
        if (proposed>=cost[j]) {continue;}
        cost[j]=proposed;parent[j]=i;open.emplace(proposed+heuristic(j),j);
      }
    }
  }
  if (parent[goal]<0) {return {};}
  std::vector<Pose2> raw;
  for (int i=goal;i!=-1;i=parent[i]) {
    Pose2 p{};grid.mapToWorld(ox+i%w,oy+i/w,p.x,p.y);raw.push_back(p);
  }
  std::reverse(raw.begin(),raw.end());raw.front()=begin;raw.back()=finish;
  // A shortcut may straighten the raster steps only while preserving the
  // available room through bends. Compare the chord with corresponding
  // samples of the searched route; a tight endpoint cannot mask a tight bend.
  auto at=[&](double x,double y) {
    unsigned mx,my;
    if (!grid.worldToMap(x,y,mx,my) || int(mx)<ox || int(my)<oy || int(mx)>=ox+w || int(my)>=oy+h) {return 0.;}
    return clearance((my-oy)*w+mx-ox);
  };
  std::vector<Pose2> result{begin};
  const size_t horizon=std::max<size_t>(2,std::ceil(8./cell));
  for (size_t first=0;first+1<raw.size();) {
    size_t last=std::min(raw.size()-1,first+horizon);
    for (;last>first+1;--last) {
      bool fits=true;
      const int samples=std::max(2,int((last-first)*2));
      for (int k=1;k<samples;++k) {
        const double t=double(k)/samples;
        const auto & ref=raw[first+std::lround(t*(last-first))];
        const double x=raw[first].x+t*(raw[last].x-raw[first].x);
        const double y=raw[first].y+t*(raw[last].y-raw[first].y);
        if (at(x,y)+.50*cell<at(ref.x,ref.y)) {fits=false;break;}
      }
      if (fits) {break;}
    }
    result.push_back(raw[last]);first=last;
  }
  return result;
}

// Refine a centerline between two facing walls at sub-cell positions. The
// samples have uniform arc spacing, so the averaging window has a physical
// length through both straight sections and tight bends.
inline std::vector<Pose2> centeredCurve(const std::vector<Pose2> & line, double body,
  Grid & grid, const Footprint & footprint, bool follow_tangent)
{
  if (line.size()<2) {return {};}
  const double length=pathLength(line);
  // Cap work in the optional refinement while the global costmap is locked.
  if (length<1e-6 || length>12.) {return {};}
  const int count=std::max(2,int(std::ceil(length/.08)));
  std::vector<Pose2> samples;
  size_t segment=1;
  double along=0.;
  for(int k=0;k<=count;++k) {
    const double at=length*k/count;
    while(segment+1<line.size() && along+distance(line[segment-1],line[segment])<at) {
      along+=distance(line[segment-1],line[segment]);++segment;
    }
    const auto & a=line[segment-1];const auto & b=line[segment];
    const double t=std::clamp((at-along)/std::max(distance(a,b),1e-9),0.,1.);
    samples.push_back({a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),body});
  }
  // Smooth raster corners and redistribute lateral space where the physical
  // polygon approaches a wall. Requested endpoints remain anchored.
  for(int pass=0;pass<3;++pass) {
    auto next=samples;
    for(int i=1;i<count;++i) {
      const auto & a=samples[std::max(0,i-3)];const auto & b=samples[std::min(count,i+3)];
      const double angle=bearing(a,b);
      Pose2 center{.25*(samples[i-1].x+2*samples[i].x+samples[i+1].x),
        .25*(samples[i-1].y+2*samples[i].y+samples[i+1].y),follow_tangent?angle:body};
      Pose2 best=center;
      double best_score=-INFINITY;
      for(int shift=-5;shift<=5;++shift) {
        const double offset=shift*grid.getResolution()/4;
        Pose2 trial{center.x-offset*std::sin(angle),center.y+offset*std::cos(angle),center.a};
        if(!poseFree(trial,grid,footprint)) {continue;}
        double gap=0.;edgeExposure({trial,trial},grid,footprint,&gap);
        const double score=gap-.18*std::abs(offset);
        if(score>best_score) {best_score=score;best=trial;}
      }
      next[i]=best;
    }
    samples.swap(next);
  }
  // Blend residual grid noise before computing headings from the final path.
  const auto refined=samples;
  for(int i=1;i<count;++i) {
    auto & p=samples[i];
    p.x=.25*(refined[i-1].x+2*refined[i].x+refined[i+1].x);
    p.y=.25*(refined[i-1].y+2*refined[i].y+refined[i+1].y);
  }
  if(follow_tangent) {
    for(int i=0;i<=count;++i) {samples[i].a=bearing(samples[std::max(0,i-2)],samples[std::min(count,i+2)]);}
  }
  std::vector<Pose2> result{line.front()};
  for(const auto & p:samples) {appendPose(result,p);}
  appendPose(result,line.back());
  if(!pathFree(result,grid,footprint)) {return {};}
  return result;
}

// Try one in-place turn between the two main travel directions of a bend.
// The same controller can follow both legs holonomically, while the chassis
// still faces into each leg. Select the location with the most tracking room.
inline std::vector<Pose2> twoHeadingCurve(const std::vector<Pose2> & curve,
  Grid & grid,const Footprint & footprint)
{
  if(curve.size()<6) {return {};}
  std::vector<Pose2> best;
  double best_gap=0.,best_exposure=INFINITY;
  double travelled=0.;
  for(size_t i=2;i+2<curve.size();++i) {
    travelled+=distance(curve[i-1],curve[i]);
    if(travelled<.24) {continue;}
    travelled=0.;
    if(distance(curve.front(),curve[i])<.4 || distance(curve[i],curve.back())<.4) {continue;}
    const double entry=bearing(curve.front(),curve[i]),exit=bearing(curve[i],curve.back());
    if(std::abs(wrap(exit-entry))<.25) {continue;}
    std::vector<Pose2> route{curve.front()};
    for(size_t j=0;j<curve.size();++j) {
      if(j==i) {appendPose(route,{curve[j].x,curve[j].y,entry});}
      appendPose(route,{curve[j].x,curve[j].y,j<i?entry:exit});
    }
    appendPose(route,curve.back());
    if(!pathFree(route,grid,footprint)) {continue;}
    double gap=0.;
    const double exposure=edgeExposure(route,grid,footprint,&gap);
    if(gap>best_gap+.005 || (gap>=best_gap && exposure<best_exposure)) {
      best=std::move(route);best_gap=gap;best_exposure=exposure;
    }
  }
  return best;
}
}
