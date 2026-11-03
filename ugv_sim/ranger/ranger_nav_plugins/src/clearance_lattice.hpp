// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "navigation_geometry.hpp"
#include "goal_connection.hpp"
#include "retained_path_service.hpp"
#include "local_lattice_repair.hpp"

namespace ranger_nav_plugins
{
// Keep the native Smac lattice search, but give its discretized collision
// checks a small margin over the footprint used to validate the output.
// The shared costmap/footprint is never modified.
class ClearanceLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  bool expose_validity{true};
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    SmacPlannerLattice::configure(parent,name,tf,map);
    if(expose_validity) {validity_=std::make_unique<RetainedPathService>(parent.lock(),map,planning_padding);}
  }
  void cleanup() override {validity_.reset(); SmacPlannerLattice::cleanup();}
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
    const auto initial_bin=NodeLattice::motion_table.getClosestAngularBin(yaw(start.pose));
    auto footprint=_costmap_ros->getRobotFootprint();
    nav2_costmap_2d::padFootprint(footprint,planning_padding);
    std_msgs::msg::Header header;header.frame_id=_global_frame;header.stamp=_clock->now();
    auto direct=directHolonomic(planar(start.pose),planar(goal.pose),*_costmap,footprint);
    if(!direct.empty()) {
      auto result=checkedDensePath(direct,header,*_costmap,footprint);
      if(!result.poses.empty()) {return result;}
    }
    const auto executing_footprint=footprint;
    auto expanded=footprint;nav2_costmap_2d::padFootprint(expanded,search_padding);
    auto cell_pose=[&](unsigned int x,unsigned int y,unsigned int bin) {
      auto p=getWorldCoords(x,y,_costmap);p.orientation=getWorldOrientation(NodeLattice::motion_table.getAngleFromBin(bin));
      return planar(p);
    };
    std::vector<double> headings;
    for(unsigned int bin=0;bin<_metadata.number_of_headings;++bin) {
      headings.push_back(NodeLattice::motion_table.getAngleFromBin(bin));
    }
    auto search=[&](bool use_expanded) {
      const auto & search_footprint=use_expanded ? expanded : executing_footprint;
      unsigned int start_x=sx,start_y=sy,start_bin=initial_bin;
      auto entry=directHolonomic(planar(start.pose),cell_pose(sx,sy,start_bin),*_costmap,executing_footprint);
      if(entry.empty() || !poseFree(cell_pose(sx,sy,start_bin),*_costmap,search_footprint)) {
        auto alternate=goalConnection(planar(start.pose),*_costmap,executing_footprint,
          search_footprint,headings,goal_connection_distance);
        if(!alternate) {throw std::runtime_error("No swept connection from start to lattice");}
        start_x=alternate->x;start_y=alternate->y;start_bin=alternate->heading;
        entry.assign(alternate->poses.rbegin(),alternate->poses.rend());
      }
      auto connection=goalConnection(planar(goal.pose),*_costmap,executing_footprint,
        search_footprint,headings,goal_connection_distance);
      if(!connection) {throw std::runtime_error("No swept connection from lattice to goal");}
      gx=connection->x;gy=connection->y;const auto gb=connection->heading;
      // Use the circumscribed radius of this expanded footprint, not the
      // smaller shared footprint. Away from obstacles retain the centre-cell
      // cost gradient: always taking the maximum edge cost flattens narrow
      // corridors to 253 and makes wall-hugging routes as cheap as the centre.
      double radius=0;
      for(const auto & point:search_footprint) {radius=std::max(radius,std::hypot(point.x,point.y));}
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
      _collision_checker.setFootprint(search_footprint,false,threshold);
      _a_star->setCollisionChecker(&_collision_checker);
      _a_star->setStart(start_x,start_y,start_bin);_a_star->setGoal(gx,gy,gb);
      nav_msgs::msg::Path result;result.header.frame_id=_global_frame;result.header.stamp=_clock->now();
      geometry_msgs::msg::PoseStamped pose;pose.header=result.header;
      NodeLattice::CoordinateVector coordinates;int iterations=0;
      if(start_x==gx && start_y==gy && start_bin==gb) {
        pose.pose=getWorldCoords(start_x,start_y,_costmap);pose.pose.orientation=getWorldOrientation(headings[start_bin]);result.poses.push_back(pose);
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
      auto route=points(result);
      auto join=directHolonomic(entry.back(),route.front(),*_costmap,executing_footprint);
      if(join.empty()) {throw std::runtime_error("Unsafe first lattice edge");}
      entry.insert(entry.end(),join.begin()+1,join.end());
      entry.insert(entry.end(),route.begin()+1,route.end());
      // Shortcuts are optional. Keep an extra cell where possible so a new
      // LiDAR observation does not immediately consume the tracking reserve.
      auto shortcut_footprint=search_footprint;
      nav2_costmap_2d::padFootprint(shortcut_footprint,_costmap->getResolution());
      route=shortcutHolonomic(entry,*_costmap,shortcut_footprint,[&]() {
#ifdef RANGER_NAV2_JAZZY
        if(cancel_checker()) {throw nav2_core::PlannerCancelled("Planning canceled");}
#endif
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>
          _max_planning_time;
      });
      auto shortened=checkedDensePath(route,result.header,*_costmap,executing_footprint);
      if(!shortened.poses.empty()) {return shortened;}
      auto original=checkedDensePath(entry,result.header,*_costmap,executing_footprint);
      if(!original.poses.empty()) {return original;}
      auto repair=points(densePath(entry,result.header));
      auto expired=[&]() {
#ifdef RANGER_NAV2_JAZZY
        if(cancel_checker()) {throw nav2_core::PlannerCancelled("Planning canceled");}
#endif
        return std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count()>_max_planning_time;
      };
      for(unsigned attempt=0;attempt<4 && !expired();++attempt) {
        if(!repairLatticeEdge(repair,*_costmap,executing_footprint,expired)) {break;}
        if(pathFree(repair,*_costmap,executing_footprint)) {
          return densePath(repair,result.header,std::numeric_limits<double>::infinity());
        }
      }
      Pose2 invalid;
      pathFree(points(densePath(entry,result.header)),*_costmap,executing_footprint,&invalid);
      throw std::runtime_error("Lattice edge outside reserve at "+std::to_string(invalid.x)+","+
        std::to_string(invalid.y)+","+std::to_string(invalid.a));
    };
    try {
      return search(true);
#ifdef RANGER_NAV2_JAZZY
    } catch(const nav2_core::PlannerCancelled &) {throw;
#endif
    } catch(const std::runtime_error &) {
      // Retry from a clean search graph, within the same hard execution reserve.
      return search(false);
    }
  }
private:
  std::unique_ptr<RetainedPathService> validity_;
};
}
