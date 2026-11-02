// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "navigation_geometry.hpp"
#include "goal_connection.hpp"

namespace ranger_nav_plugins
{
// Keep the native Smac lattice search, but give its discretized collision
// checks a small margin over the footprint used to validate the output.
// The shared costmap/footprint is never modified.
class ClearanceLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  double search_padding{.025};
  double planning_padding{.04};
  double goal_connection_distance{.25};
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
#ifdef RANGER_NAV2_JAZZY
    , std::function<bool()> cancel_checker
#endif
    ) override
  {
    using namespace nav2_smac_planner;
    std::lock_guard<std::mutex> configuration_lock(_mutex);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*_costmap->getMutex());
    const auto started=std::chrono::steady_clock::now();
    unsigned int sx,sy,gx,gy;
    if(!_costmap->worldToMap(start.pose.position.x,start.pose.position.y,sx,sy) ||
      !_costmap->worldToMap(goal.pose.position.x,goal.pose.position.y,gx,gy)) {
      throw std::runtime_error("Lattice start or goal outside costmap");
    }
    const auto sb=NodeLattice::motion_table.getClosestAngularBin(yaw(start.pose));
    auto footprint=_costmap_ros->getRobotFootprint();
    nav2_costmap_2d::padFootprint(footprint,planning_padding);
    auto expanded=footprint;nav2_costmap_2d::padFootprint(expanded,search_padding);
    auto cell_pose=[&](unsigned int x,unsigned int y,unsigned int bin) {
      auto p=getWorldCoords(x,y,_costmap);p.orientation=getWorldOrientation(NodeLattice::motion_table.getAngleFromBin(bin));
      return planar(p);
    };
    std::vector<double> headings;
    for(unsigned int bin=0;bin<_metadata.number_of_headings;++bin) {
      headings.push_back(NodeLattice::motion_table.getAngleFromBin(bin));
    }
    bool use_expanded=poseFree(cell_pose(sx,sy,sb),*_costmap,expanded);
    auto connection=goalConnection(planar(goal.pose),*_costmap,footprint,
      use_expanded?expanded:footprint,headings,goal_connection_distance);
    if(!connection && use_expanded) {
      use_expanded=false;
      connection=goalConnection(planar(goal.pose),*_costmap,footprint,footprint,headings,goal_connection_distance);
    }
    if(!connection) {throw std::runtime_error("No collision-free lattice connection to the exact goal");}
    gx=connection->x;gy=connection->y;const auto gb=connection->heading;
    if(use_expanded) {footprint=expanded;}
    // Use the circumscribed radius of this expanded footprint, not the
    // smaller shared footprint. Away from obstacles retain the centre-cell
    // cost gradient: always taking the maximum edge cost flattens narrow
    // corridors to 253 and makes wall-hugging routes as cheap as the centre.
    double radius=0;
    for(const auto & point:footprint) {radius=std::max(radius,std::hypot(point.x,point.y));}
    radius+=std::sqrt(2.)*_costmap->getResolution();
    double threshold=1.;
    for(const auto & layer:*_costmap_ros->getLayeredCostmap()->getPlugins()) {
      auto inflation=std::dynamic_pointer_cast<nav2_costmap_2d::InflationLayer>(layer);
      double coverage=0;
      if(inflation && inflation->isEnabled() &&
        _costmap_ros->get_parameter(layer->getName()+".inflation_radius",coverage) && coverage>=radius) {
        threshold=std::max(1.,double(inflation->computeCost(radius/_costmap->getResolution())));
      }
    }
    _collision_checker.setFootprint(footprint,false,threshold);
    _a_star->setCollisionChecker(&_collision_checker);
    _a_star->setStart(sx,sy,sb);_a_star->setGoal(gx,gy,gb);
    nav_msgs::msg::Path result;result.header.frame_id=_global_frame;result.header.stamp=_clock->now();
    geometry_msgs::msg::PoseStamped pose;pose.header=result.header;
    NodeLattice::CoordinateVector coordinates;int iterations=0;
    if(sx==gx && sy==gy && sb==gb) {
      pose.pose=start.pose;result.poses.push_back(pose);
    } else if(!_a_star->createPath(coordinates,iterations,_tolerance/_costmap->getResolution()
#ifdef RANGER_NAV2_JAZZY
      , cancel_checker
#endif
      )) {
      throw std::runtime_error("No lattice route with collision discretization margin");
    }
    for(auto it=coordinates.rbegin();it!=coordinates.rend();++it) {
      pose.pose=getWorldCoords(it->x,it->y,_costmap);
      pose.pose.orientation=getWorldOrientation(it->theta);
      if(!result.poses.empty()) {
        const auto a=planar(result.poses.back().pose),b=planar(pose.pose);
        if(distance(a,b)<1e-4 && std::abs(wrap(a.a-b.a))<1e-4) {continue;}
      }
      result.poses.push_back(pose);
    }
    if(_smoother && iterations>1) {
      const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
      _smoother->smooth(result,_costmap,std::max(0.,_max_planning_time-elapsed));
    }
    for(const auto & p:connection->poses) {
      if(!result.poses.empty()) {
        const auto previous=planar(result.poses.back().pose);
        if(distance(previous,p)<1e-6 && std::abs(wrap(previous.a-p.a))<1e-6) {continue;}
      }
      pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation=getWorldOrientation(p.a);result.poses.push_back(pose);
    }
    result.poses.back().pose=goal.pose;
    return result;
  }
};
}
