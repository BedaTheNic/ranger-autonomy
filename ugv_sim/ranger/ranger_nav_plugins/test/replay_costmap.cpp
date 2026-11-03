// SPDX-License-Identifier: Apache-2.0
// Run the installed planner against an archived, unmodified global costmap.
#include <fstream>
#include <iostream>
#include "nav2_core/global_planner.hpp"
#include "pluginlib/class_loader.hpp"

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  int status=1;
  try {
    if(argc<3) {throw std::runtime_error("Expected costmap fixture and lattice file");}
    std::ifstream input(argv[1],std::ios::binary);
    std::string line;std::getline(input,line);std::istringstream data(line);
    unsigned nx,ny;double ox,oy,res,sx,sy,sa,gx,gy,ga;
    if(!(data>>nx>>ny>>ox>>oy>>res>>sx>>sy>>sa>>gx>>gy>>ga)) {throw std::runtime_error("Invalid fixture header");}
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_server");
    node->declare_parameter("GridBased.Holonomic.lattice_filepath",std::string(argv[2]));
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("snapshot_costmap");
    map->set_parameter(rclcpp::Parameter("global_frame","map"));
    map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{"inflation_layer"}));
    map->declare_parameter("inflation_layer.plugin","nav2_costmap_2d::InflationLayer");
    map->set_parameter(rclcpp::Parameter("footprint_padding",.02));
    map->configure();
    std::vector<geometry_msgs::msg::Point> fp;nav2_costmap_2d::makeFootprintFromString(
      "[[0.32,0.30],[0.32,-0.30],[-0.32,-0.30],[-0.32,0.30]]",fp);
    map->setRobotFootprint(fp);
    map->getLayeredCostmap()->resizeMap(nx,ny,res,ox,oy);
    if(!input.read(reinterpret_cast<char *>(map->getCostmap()->getCharMap()),size_t(nx)*ny)) {
      throw std::runtime_error("Truncated costmap");
    }
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto planner=loader.createSharedInstance("ranger_nav_plugins::AdaptivePlanner");
    planner->configure(node,"GridBased",tf,map);planner->activate();
    auto makePose=[](double x,double y,double a) {
      geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";p.pose.position.x=x;p.pose.position.y=y;
      p.pose.orientation.w=std::cos(a/2);p.pose.orientation.z=std::sin(a/2);return p;
    };
    try {
      auto path=planner->createPlan(makePose(sx,sy,sa),makePose(gx,gy,ga)
#ifdef RANGER_NAV2_JAZZY
        ,[]{return false;}
#endif
      );
      double length=0,turn=0;int pivots=0;
      for(size_t i=1;i<path.poses.size();++i) {
        const auto & a=path.poses[i-1].pose;const auto & b=path.poses[i].pose;
        const double ds=std::hypot(b.position.x-a.position.x,b.position.y-a.position.y);
        length+=ds;
        if(ds<1e-6) {
          const double da=std::abs(std::remainder(2*std::atan2(b.orientation.z,b.orientation.w)-
            2*std::atan2(a.orientation.z,a.orientation.w),2*M_PI));
          turn+=da;if(da>.001) {++pivots;}
        }
      }
      std::cout<<"Validated poses: "<<path.poses.size()<<", length: "<<length
        <<", planned pivots: "<<pivots<<", pivot radians: "<<turn<<std::endl;
      if(!path.poses.empty()) {status=0;}
    } catch(const std::exception & e) {std::cerr<<e.what()<<std::endl;}
    planner->deactivate();planner->cleanup();map->cleanup();node.reset();planner.reset();
  } catch(const std::exception & e) {std::cerr<<e.what()<<std::endl;}
  rclcpp::shutdown();return status;
}
