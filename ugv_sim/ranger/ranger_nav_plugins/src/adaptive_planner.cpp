// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <mutex>
#include <sstream>
#include "clearance_route.hpp"
#include "clearance_lattice.hpp"
#include "validated_hybrid.hpp"
#include "retained_path_service.hpp"
#include "nav2_smac_planner/smac_planner_lattice.hpp"
#include "std_msgs/msg/string.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
class AdaptivePlanner : public nav2_core::GlobalPlanner
{
public:

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    node_=parent;map_=map;auto node=parent.lock();
    ack_=std::make_unique<ValidatedHybrid>();
    holo_=std::make_unique<ClearanceLattice>();
    ack_->expose_validity=false;
    static_cast<ClearanceLattice *>(holo_.get())->expose_validity=false;
    ack_->configure(parent,name+".Ackermann",tf,map);
    holo_->configure(parent,name+".Holonomic",tf,map);
    auto parameter=[&](const std::string & key,double & value, bool allow_zero=false) {
      if(!node->has_parameter(name+"."+key)) {node->declare_parameter(name+"."+key,value);}
      value=node->get_parameter(name+"."+key).as_double();
      if(!std::isfinite(value) || (allow_zero ? value<0 : value<=0)) {
        throw std::runtime_error("Invalid planner parameter "+key);
      }
    };
    parameter("minimum_turning_radius",limits_.radius);
    parameter("forward_speed",limits_.forward);parameter("reverse_speed",limits_.reverse);
    parameter("holonomic_speed",limits_.lateral);parameter("angular_speed",limits_.angular);
    parameter("steering_rate",limits_.steering_rate);
    parameter("obstacle_weight",preferences_.obstacle_weight,true);
    parameter("segment_time_slack",time_slack_,true);
    parameter("corridor_width",preferences_.corridor_width);
    parameter("centering_weight",preferences_.centering_weight,true);
    parameter("front_facing_weight",preferences_.front_weight,true);
    parameter("planning_padding",planning_padding_,true);
    parameter("unobserved_obstacle_depth",unobserved_depth_,true);
    parameter("lattice_search_padding",static_cast<ClearanceLattice *>(holo_.get())->search_padding,true);
    parameter("goal_connection_distance",static_cast<ClearanceLattice *>(holo_.get())->goal_connection_distance);
    static_cast<ClearanceLattice *>(holo_.get())->planning_padding=planning_padding_;
    ack_->setLimits(limits_,planning_padding_);
    validity_=std::make_unique<RetainedPathService>(node,map_,planning_padding_);
    mode_sub_=node->create_subscription<std_msgs::msg::String>("navigation_mode",rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if(msg->data=="ACKERMANN") {active_car_.store(1);}
        else if(msg->data.rfind("HOLONOMIC",0)==0) {active_car_.store(0);}
      });
    publisher_=node->create_publisher<std_msgs::msg::String>("navigation_plan_decision",rclcpp::QoS(1).transient_local());
  }
  void activate() override {ack_->activate();holo_->activate();publisher_->on_activate();}
  void deactivate() override {ack_->deactivate();holo_->deactivate();publisher_->on_deactivate();}
  void cleanup() override {mode_sub_.reset();active_car_.store(-1);last_goal_.reset();validity_.reset();ack_->cleanup();holo_->cleanup();publisher_.reset();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
#ifdef RANGER_NAV2_JAZZY
    , std::function<bool()> cancel_checker
#endif
    ) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * grid=map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*grid->getMutex());
    const bool replacing=last_goal_ && last_goal_->pose==goal.pose &&
      last_goal_->header.frame_id==goal.header.frame_id && validity_->replacesBlockedPath(last_plan_header_);
    const int entering_mode=replacing?active_car_.load():-1;
    auto risk=replacing?occlusionRiskGrid(*grid,validity_->blockingCell(),planar(start.pose),unobserved_depth_):nullptr;
    auto & ranking_grid=risk?*risk:*grid;
    auto footprint=map_->getRobotFootprint();
    nav2_costmap_2d::padFootprint(footprint,planning_padding_);
    if(!poseFree(planar(goal.pose),*grid,footprint)) {
      throw std::runtime_error("Goal footprint is occupied or lacks tracking clearance; choose a goal farther from the obstacle");
    }
    nav_msgs::msg::Path a,h;std::string failure;
#ifdef RANGER_NAV2_JAZZY
    if(cancel_checker()) {throw nav2_core::PlannerCancelled("Planning canceled");}
    try {a=ack_->createPlan(start,goal,cancel_checker);} catch(const nav2_core::PlannerCancelled &) {throw;} catch(const std::exception & e) {failure=e.what();}
    try {h=holo_->createPlan(start,goal,cancel_checker);} catch(const nav2_core::PlannerCancelled &) {throw;} catch(const std::exception & e) {failure+="; "+std::string(e.what());}
#else
    try {a=ack_->createPlan(start,goal);} catch(const std::exception & e) {failure=e.what();}
    try {h=holo_->createPlan(start,goal);} catch(const std::exception & e) {failure+="; "+std::string(e.what());}
#endif
    // Native search uses discretized poses, and smoothing may change them.
    // Check exactly the same continuous footprint poses sent to IsPathValid.
    Pose2 invalid;
    auto location=[&]() {return " at "+std::to_string(invalid.x)+","+std::to_string(invalid.y)+","+std::to_string(invalid.a);};
    if(!a.poses.empty() && !pathFree(points(a),*grid,footprint,&invalid)) {
      a.poses.clear();failure+="; Ackermann candidate failed continuous footprint validation"+location();
    }
    if(!h.poses.empty() && !pathFree(points(h),*grid,footprint,&invalid)) {
      h.poses.clear();failure+="; holonomic candidate failed continuous footprint validation"+location();
    }
    struct Candidate {
      nav_msgs::msg::Path path;
      Decision decision;
      TravelCost cost;
      std::string source;
      double score{INFINITY},length{INFINITY};
    };
    std::vector<Candidate> candidates;
    std::vector<RouteRank> ranks;
    auto consider=[&](const std::vector<Pose2> & input,const std_msgs::msg::Header & header,
      const std::string & label) {
      auto p=mergeSmallPivots(input);
      if(!pathFree(p,*grid,footprint)) {
        if(p.size()==input.size() || !pathFree(input,*grid,footprint)) {return;}
        p=input;
      }
      const auto cost=travelCost(p,*grid,preferences_);
      const double length=routeLength(p,planar(start.pose),planar(goal.pose));
      std::optional<bool> entering_car;
      if(entering_mode>=0) {entering_car=bool(entering_mode);}
      auto policy=decide(p,limits_,yaw(start.pose),entering_car);
      if(!std::isfinite(policy.mixed.seconds)) {return;}
      double gap=0.;
      double exposure=edgeExposure(p,*grid,footprint,&gap);
      if(risk) {
        double estimated_gap=0.;
        exposure=.5*(exposure+edgeExposure(p,ranking_grid,footprint,&estimated_gap));
        gap=.5*(gap+estimated_gap);
      }
      const double penalty=(length-cost.length+preferences_.front_weight*cost.heading_loss+
        preferences_.centering_weight*cost.center_loss+preferences_.obstacle_weight*
        (cost.inflation_loss+exposure))/
        limits_.forward;
      const double score=policy.mixed.seconds+penalty;
      size_t segments=0;
      for(size_t i=0;i<policy.modes.size();i=segmentEnd(p,policy.modes,i)) {++segments;}
      const bool handoff=entering_mode>=0 && !policy.modes.empty() && policy.modes.front()!=bool(entering_mode);
      ranks.push_back({policy.mixed.switches+unsigned(handoff),segments,score,cost.rear_distance,gap});
      candidates.push_back({densePath(p,header,INFINITY),std::move(policy),cost,label,score,length});
    };
    auto assess=[&](const nav_msgs::msg::Path & original,const std::string & source) {
      if(original.poses.empty()) {return;}
      const auto raw=points(original);
      consider(raw,original.header,source);
      // Keep the pose/gear sequence on a car path with overlapping branches.
      if(source=="hybrid" && gears(raw,limits_).size()>1) {return;}
      if(source=="lattice" && directHolonomic(raw.front(),raw.back(),*grid,footprint).empty()) {
        // State lattice poses include orientations that can fit a bottleneck.
        // Try to carry one of those headings through multiple corners.
        std::vector<double> tried;
        for(double heading:{raw.front().a,raw.back().a,raw[raw.size()/2].a,
          bearing(raw.front(),raw.back()),wrap(bearing(raw.front(),raw.back())+M_PI_4),
          wrap(bearing(raw.front(),raw.back())-M_PI_4)}) {
          if(std::any_of(tried.begin(),tried.end(),[&](double h){return std::abs(wrap(heading-h))<.03;})) {continue;}
          tried.push_back(heading);
          auto simple=minimumLegRoute(raw,heading,*grid,footprint);
          if(!simple.empty()) {consider(points(densePath(simple,original.header)),original.header,"minimum_legs");}
        }
      }
      const auto shaped=shapeTravelLegs(raw,*grid,footprint,preferences_);
      consider(shaped,original.header,source+"_segments");
      // Center across adjoining primitives as one passage. Taper both ends
      // so the shortcut keeps the requested start and goal.
      auto centered=raw;
      const bool shifted=centerLeg(centered,*grid,preferences_);
      std::vector<const std::vector<Pose2> *> geometries{&raw,&shaped};
      if(shifted) {geometries.push_back(&centered);consider(centered,original.header,source+"_centered");}
      for(const auto * geometry:geometries) {
        consider(constantHeadingRoute(*geometry,raw.front().a),original.header,source+"_constant_start");
        if(std::abs(wrap(raw.front().a-raw.back().a))>.025) {
          consider(constantHeadingRoute(*geometry,raw.back().a),original.header,source+"_constant_goal");
        }
      }
    };
    assess(a,"hybrid");assess(h,"lattice");
    const auto begin=planar(start.pose),finish=planar(goal.pose);
    const double direct_length=distance(begin,finish);
    if(direct_length>.10) {
      auto header=start.header;header.stamp=node_.lock()->now();
      const double direction=bearing(begin,finish);
      const double heading=direction+std::clamp(wrap(begin.a-direction),-M_PI_4,M_PI_4);
      auto polyline=[&](const std::vector<Pose2> & vertices,double body,const std::string & label) {
        const auto route=constantHeadingRoute(vertices,body);
        if(!pathFree(route,*grid,footprint)) {return false;}
        consider(points(densePath(route,header)),header,label);
        return true;
      };
      const bool direct=polyline({begin,finish},heading,"direct");
      // For a blocked line of sight, explicitly try two straight translations
      // with one bend on either side. Native searches still handle longer mazes.
      if(direct_length<=8. && (!direct ||
        edgeExposure(constantHeadingRoute({begin,finish},heading),*grid,footprint)>.2)) {
        // A swerve base can orient itself in open space before starting a
        // continuous Ackermann arc, avoiding a sequence of reversing cusps.
        if(!risk && direct_length>2.*limits_.radius && !h.poses.empty()) {
          const auto lattice=points(h);
          const auto entry=lookahead(lattice,{0,0.,0.,lattice.front()},limits_.radius);
          const double approach=bearing(begin,entry);
          if(distance(begin,entry)>.15 && std::abs(wrap(approach-begin.a))>.25 &&
              rotationFree(begin,approach,*grid,footprint)) {
            auto aligned=start;
            aligned.pose.orientation=tf2::toMsg(tf2::Quaternion(0.,0.,std::sin(.5*approach),std::cos(.5*approach)));
            try {
#ifdef RANGER_NAV2_JAZZY
              auto arc=ack_->createPlan(aligned,goal,cancel_checker);
#else
              auto arc=ack_->createPlan(aligned,goal);
#endif
              if(!arc.poses.empty()) {
                std::vector<Pose2> route{begin,planar(aligned.pose)};
                for(const auto & p:points(arc)) {appendPose(route,p);}
                consider(route,header,"aligned_hybrid");
              }
#ifdef RANGER_NAV2_JAZZY
            } catch(const nav2_core::PlannerCancelled &) {throw;
#endif
            } catch(const std::exception & e) {failure+="; aligned Ackermann: "+std::string(e.what());}
          }
        }
        const bool refine_bend=!risk && direct_length>2.*limits_.radius;
        std::vector<double> headings{heading,begin.a,finish.a,direction,wrap(direction+M_PI_4),
          wrap(direction-M_PI_4),wrap(direction+M_PI_2),wrap(direction-M_PI_2)};
        if(refine_bend) {
          headings.push_back(wrap(finish.a+M_PI/12));
          headings.push_back(wrap(finish.a-M_PI/12));
        }
        std::vector<double> tried;
        for(double body:headings) {
          if(std::any_of(tried.begin(),tried.end(),[&](double h) {return std::abs(wrap(h-body))<.03;})) {continue;}
          tried.push_back(body);
          const auto centerline=clearanceRoute(begin,finish,*grid,footprint,body,risk.get());
          if(centerline.empty()) {continue;}
          polyline(centerline,body,"clearance_route");
          // A long bend can admit substantially more room with the chassis
          // aligned along the corridor. Also try keeping one heading through it.
          if(refine_bend) {
            for(bool forward:{false,true}) {
              const auto refined=centeredCurve(centerline,body,*grid,footprint,forward);
              if(refined.empty()) {continue;}
              consider(refined,header,forward?"forward_clearance_curve":"fixed_clearance_curve");
              if(!forward) {
                const auto pivot=twoHeadingCurve(refined,*grid,footprint);
                if(!pivot.empty()) {consider(pivot,header,"single_turn_curve");}
              }
            }
          }
        }
        const double nx=-std::sin(direction),ny=std::cos(direction);
        auto offsetPoint=[&](double fraction,double offset) {
          return Pose2{begin.x+fraction*(finish.x-begin.x)+offset*nx,
            begin.y+fraction*(finish.y-begin.y)+offset*ny,heading};
        };
        for(double fraction:{.35,.50,.65}) {
          for(double offset:{-.95,-.75,-.55,.55,.75,.95}) {
            const Pose2 via=offsetPoint(fraction,offset);
            polyline({begin,via,finish},begin.a,"one_bend");
            if(std::abs(wrap(heading-begin.a))>.025) {
              polyline({begin,via,finish},heading,"one_bend_aligned");
            }
            // Face between both legs so the second translation can keep
            // moving forward around a wide obstacle.
            const double first=bearing(begin,via),second=bearing(via,finish);
            polyline({begin,via,finish},first+.5*wrap(second-first),"one_bend_forward");
          }
        }
        // A single bend often puts the robot's outer wheel beside the unseen
        // side of an obstacle. Also try a short straight alongside it before
        // converging on the goal. The same clearance score chooses the route.
        for(double offset:{-.80,-.65,-.50,.50,.65,.80}) {
          for(double fraction:{.25,.38}) {
            const std::vector<Pose2> bypass{begin,offsetPoint(fraction,offset),
              offsetPoint(1.-fraction,offset),finish};
            for(double body:{begin.a,direction,heading}) {polyline(bypass,body,"two_bends");}
          }
        }
      }
    }
    if(candidates.empty()) {throw std::runtime_error("No feasible candidate route: "+failure);}
    const size_t selected=selectSchedule(ranks,time_slack_);
    const auto & best=candidates.at(selected);
    const auto values=points(best.path);
    std::ostringstream json;
    auto number=[&](double value) {if(std::isfinite(value)) {json<<value;} else {json<<"null";}};
    const auto & modes=best.decision.modes;
    const bool uses_car=std::find(modes.begin(),modes.end(),true)!=modes.end();
    const bool uses_holonomic=std::find(modes.begin(),modes.end(),false)!=modes.end();
    const char * route_type=uses_car ? (uses_holonomic?"mixed_route":"ackermann_route") : "holonomic_route";
    json<<"{\"selected\":\""<<route_type<<"\",\"source\":\""<<best.source
        <<"\",\"objective\":\"fewest_switches_time_and_clearance\",\"selected_eta_s\":"<<best.decision.mixed.seconds
        <<",\"ackermann_eta\":";
    number(a.poses.empty()?INFINITY:estimate(points(a),std::vector<bool>(a.poses.size()-1,true),
      limits_,yaw(start.pose)).seconds);
    json<<",\"mixed_eta\":";number(best.decision.mixed.seconds);
    json<<",\"ackermann_length\":";number(routeLength(points(a),planar(start.pose),planar(goal.pose)));
    json<<",\"holonomic_length\":";number(routeLength(points(h),planar(start.pose),planar(goal.pose)));
    json<<",\"selected_length_m\":"<<best.length;
    json<<",\"heading_loss_m\":"<<best.cost.heading_loss<<",\"center_loss_m\":"<<best.cost.center_loss
        <<",\"entry_mode\":\""<<(entering_mode<0?"unspecified":(entering_mode?"ackermann":"holonomic"))
        <<"\",\"mode_changes\":"<<ranks[selected].switches<<",\"maneuvers\":"<<ranks[selected].segments
        <<",\"occlusion_depth_m\":"<<(risk?unobserved_depth_:0.)<<",\"minimum_gap_m\":"<<ranks[selected].gap<<",\"rear_distance_m\":"<<best.cost.rear_distance<<",\"score_s\":"<<best.score<<",\"segments\":[";
    for(size_t first=0;first<modes.size();) {
      const bool pivot=distance(values[first],values[first+1])<1e-5;
      const size_t end=segmentEnd(values,modes,first);
      if(first) {json<<",";}
      json<<"{\"mode\":\""<<(modes[first]?"ackermann":"holonomic")
          <<"\",\"pivot\":"<<(pivot?"true":"false")<<",\"end_index\":"<<end
          <<",\"goal\":["<<values[end].x<<","<<values[end].y<<","<<values[end].a<<"]}";
      first=end;
    }
    json<<"]}";
    std_msgs::msg::String message;message.data=json.str();publisher_->publish(message);
    RCLCPP_INFO(node_.lock()->get_logger(),"Segment route decision: %s",message.data.c_str());
    last_goal_=goal;last_plan_header_=best.path.header;
    return best.path;
  }
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::unique_ptr<ValidatedHybrid> ack_;
  std::unique_ptr<RetainedPathService> validity_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  std::atomic<int> active_car_{-1};
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;
  std_msgs::msg::Header last_plan_header_;
  std::unique_ptr<nav2_core::GlobalPlanner> holo_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  TravelPreferences preferences_;
  MotionLimits limits_;std::mutex mutex_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  double planning_padding_{0.0},time_slack_{.10},unobserved_depth_{.20};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::AdaptivePlanner,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ClearanceLattice,nav2_core::GlobalPlanner)
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ValidatedHybrid,nav2_core::GlobalPlanner)
