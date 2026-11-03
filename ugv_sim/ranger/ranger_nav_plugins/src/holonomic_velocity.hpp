// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "navigation_geometry.hpp"

namespace ranger_nav_plugins
{
struct TranslationWindow
{
  double min_x,max_x,min_y,max_y,min_w,max_w,max_speed;
};
// Intersect the desired direction ray with the actual DWA acceleration window.
// Independent clipping of vx/vy would change the direction and recreate stairs.
inline std::optional<geometry_msgs::msg::Twist> alignedTranslation(
  double dx,double dy,double desired_speed,const TranslationWindow & window)
{
  const double length=std::hypot(dx,dy);
  if(length<1e-6 || window.min_w>0 || window.max_w<0) {return std::nullopt;}
  dx/=length;dy/=length;
  double low=0,high=std::min(desired_speed,window.max_speed);
  auto intersect=[&](double direction,double minimum,double maximum) {
    if(std::abs(direction)<1e-9) {return minimum<=0 && maximum>=0;}
    double a=minimum/direction,b=maximum/direction;
    if(a>b) {std::swap(a,b);}
    low=std::max(low,a);high=std::min(high,b);return low<=high;
  };
  if(!intersect(dx,window.min_x,window.max_x) || !intersect(dy,window.min_y,window.max_y) ||
    !std::isfinite(high) || high<.001) {return std::nullopt;}
  geometry_msgs::msg::Twist v;v.linear.x=dx*high;v.linear.y=dy*high;return v;
}
}
