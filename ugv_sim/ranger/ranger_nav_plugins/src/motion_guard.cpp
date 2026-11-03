// SPDX-License-Identifier: Apache-2.0
#include "stopping_distance.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ranger_nav_plugins
{
class MotionGuard : public rclcpp::Node
{
public:
  MotionGuard() : Node("navigation_motion_guard")
  {
    const auto polygon=declare_parameter<std::string>("footprint","");
    if(!nav2_costmap_2d::makeFootprintFromString(polygon,footprint_) || footprint_.size()<3) {
      throw std::runtime_error("A polygonal local footprint is required");
    }
    nav2_costmap_2d::padFootprint(footprint_,declare_parameter("footprint_padding",0.0));
    adaptive_=declare_parameter("adaptive",true);
    period_=1./declare_parameter("control_frequency",10.);
    decel_=declare_parameter("linear_deceleration",.4);
    angular_decel_=declare_parameter("angular_deceleration",.5);
    if(!std::isfinite(period_) || period_<=0 || !(decel_>0) || !(angular_decel_>0)) {
      throw std::runtime_error("Invalid stopping model");
    }
    pub_=create_publisher<Twist>("cmd_vel",10);
    scan_=create_subscription<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {
        if(!m->ranges.empty()) {scan_stamp_=rclcpp::Time(m->header.stamp).seconds();}
      });
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry/filtered",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_stamp_=rclcpp::Time(m->header.stamp).seconds();
        pose_=planar(m->pose.pose); speed_=m->twist.twist; odom_frame_=m->header.frame_id;
      });
    grid_sub_=create_subscription<nav2_msgs::msg::Costmap>("local_costmap/costmap_raw",1,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr m) {
        const auto & meta=m->metadata;
        if(meta.size_x==0 || meta.size_y==0 || meta.resolution<=0 ||
           m->data.size()!=size_t(meta.size_x)*meta.size_y) {grid_.reset();return;}
        grid_=std::make_unique<nav2_costmap_2d::Costmap2D>(meta.size_x,meta.size_y,
          meta.resolution,meta.origin.position.x,meta.origin.position.y);
        std::copy(m->data.begin(),m->data.end(),grid_->getCharMap());
        grid_stamp_=rclcpp::Time(m->header.stamp).seconds(); grid_frame_=m->header.frame_id;
      });
    input_=create_subscription<Twist>("cmd_vel_candidate",1,[this](Twist::ConstSharedPtr v) {
      command(*v);
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      if(moving_ && (std::chrono::steady_clock::now()-received_>std::chrono::milliseconds(300) ||
        !fresh(now().seconds()))) {send(Twist{},"watchdog");}
    });
  }
private:
  bool fresh(double stamp) const {
    auto ok=[stamp](double t,double max_age) {return t>0 && stamp>=t-.02 && stamp-t<=max_age;};
    return grid_ && odom_frame_==grid_frame_ && ok(scan_stamp_,.5) &&
      ok(grid_stamp_,.65) && ok(odom_stamp_,.35) && finitePlanar(speed_);
  }
  void send(const Twist & v,const char * reason) {
    moving_=linearSpeed(v)>1e-8 || std::abs(v.angular.z)>1e-8;
    if(reason!=reason_) {
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,"Command gate: %s",reason);
      reason_=reason;
    }
    pub_->publish(v);
  }
  void command(const Twist & requested) {
    received_=std::chrono::steady_clock::now();
    const double stamp=now().seconds();
    if(!finitePlanar(requested) || (linearSpeed(requested)==0 && requested.angular.z==0)) {
      send(Twist{},"stop");return;
    }
    if(!fresh(stamp)) {send(Twist{},"stale scan, grid or odometry");return;}
    if(std::abs(requested.linear.y)>1e-6 && std::abs(requested.angular.z)>1e-6) {
      send(Twist{},"incompatible motion");return;
    }
    auto candidate=requested;
    // Ramp speed on this command's ray. Never blend an old lateral component
    // into a pivot or an Ackermann arc. Braking and cancellation take priority.
    if(adaptive_) {
      const double dt=std::clamp(stamp-last_command_,0.,period_);
      if(linearSpeed(candidate)>0) {
        candidate=scaled(candidate,std::min(1.,(linearSpeed(speed_)+.3*dt)/linearSpeed(candidate)));
      } else if(candidate.angular.z!=0) {
        candidate=scaled(candidate,std::min(1.,(std::abs(speed_.angular.z)+.5*dt)/std::abs(candidate.angular.z)));
      }
    }
    last_command_=stamp;
    const double age=std::max({0.,stamp-scan_stamp_,stamp-grid_stamp_,stamp-odom_stamp_});
    const double scale=stoppingScale(pose_,speed_,candidate,age,period_,decel_,angular_decel_,*grid_,footprint_);
    send(scaled(candidate,scale),scale==1 ? "clear" : (scale==0 ? "blocked" : "braking distance"));
  }
  nav2_costmap_2d::Footprint footprint_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> grid_;
  Pose2 pose_; Twist speed_;
  std::string odom_frame_,grid_frame_,reason_;
  bool adaptive_{false},moving_{false};
  double period_,decel_,angular_decel_,scan_stamp_{0},grid_stamp_{0},odom_stamp_{0},last_command_{-1};
  std::chrono::steady_clock::time_point received_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Twist>::SharedPtr input_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ranger_nav_plugins::MotionGuard>());
  rclcpp::shutdown();
}
