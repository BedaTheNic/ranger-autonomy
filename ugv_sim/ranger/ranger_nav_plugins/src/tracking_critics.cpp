// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
// Preserve the holonomic reference position without requiring the chassis to
// face its direction of travel. Score the whole prediction, not just its end.
class PathTrackingCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D & path) override
  {
    path_ = path.poses;
    return !path_.empty();
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (path_.empty() || trajectory.poses.empty()) {
      throw dwb_core::IllegalTrajectoryException(name_, "Empty tracking reference or trajectory");
    }
    double sum = 0.0;
    for (const auto & p : trajectory.poses) {
      double nearest = std::hypot(p.x - path_.front().x, p.y - path_.front().y);
      double heading=path_.front().theta;
      for (size_t i = 1; i < path_.size(); ++i) {
        const auto & a = path_[i - 1]; const auto & b = path_[i];
        const double dx = b.x - a.x, dy = b.y - a.y;
        const double l2 = dx * dx + dy * dy;
        const double t = l2 > 1e-12 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2, 0.0, 1.0) : 0.0;
        const double error=std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
        if(error<nearest) {
          nearest=error;heading=a.theta+t*std::atan2(std::sin(b.theta-a.theta),std::cos(b.theta-a.theta));
        }
      }
      sum += nearest + .15*std::abs(std::atan2(std::sin(p.theta-heading),std::cos(p.theta-heading)));
    }
    return sum / trajectory.poses.size();
  }
private:
  std::vector<geometry_msgs::msg::Pose2D> path_;
};

// Penalize unnecessary changes of wheel steering commands. This is a soft
// preference: collision checking and mode constraints still reject bad motions.
// On a new path the first command can freely choose a steering direction.
class SteeringContinuityCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    const auto prefix = dwb_plugin_name_ + "." + name_ + ".";
    if (!node->has_parameter(prefix + "wheelbase")) {node->declare_parameter(prefix + "wheelbase", 0.46);}
    if (!node->has_parameter(prefix + "track")) {node->declare_parameter(prefix + "track", 0.412);}
    wheelbase_ = node->get_parameter(prefix + "wheelbase").as_double();
    track_ = node->get_parameter(prefix + "track").as_double();
    if (!std::isfinite(wheelbase_) || !std::isfinite(track_) || wheelbase_ <= 0 || track_ <= 0) {
      throw std::runtime_error("Steering continuity requires positive finite geometry");
    }
  }
  void reset() override {valid_ = false;}
  void debrief(const nav_2d_msgs::msg::Twist2D & velocity) override
  {
    if (moving(velocity)) {previous_ = steering(velocity); valid_ = true;}
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (!valid_ || !moving(trajectory.velocity)) {return 0.0;}
    const auto angles = steering(trajectory.velocity);
    double change = 0.0;
    for (size_t i = 0; i < 4; ++i) {change = std::max(change, std::abs(angles[i]-previous_[i]));}
    return change;
  }
private:
  bool moving(const nav_2d_msgs::msg::Twist2D & v) const
  {return std::hypot(v.x, v.y) + std::abs(v.theta) > 1e-5;}
  std::array<double, 4> steering(const nav_2d_msgs::msg::Twist2D & v) const
  {
    std::array<double, 4> result{};
    for (size_t i = 0; i < 4; ++i) {
      const double x = i < 2 ? wheelbase_/2 : -wheelbase_/2;
      const double y = i % 2 == 0 ? track_/2 : -track_/2;
      double a = std::atan2(v.y+v.theta*x, v.x-v.theta*y);
      if (a > M_PI_2) {a -= M_PI;}
      if (a < -M_PI_2) {a += M_PI;}
      if (valid_ && std::abs(std::abs(a)-M_PI_2) < 1e-9 &&
        std::abs(-a-previous_[i]) < std::abs(a-previous_[i])) {a = -a;}
      result[i] = a;
    }
    return result;
  }
  bool valid_{false};
  double wheelbase_{0.46}, track_{0.412};
  std::array<double, 4> previous_{};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::PathTrackingCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::SteeringContinuityCritic, dwb_core::TrajectoryCritic)
