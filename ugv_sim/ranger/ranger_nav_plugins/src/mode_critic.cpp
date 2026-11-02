// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include "dwb_core/trajectory_critic.hpp"
#include "dwb_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
// Constrain candidates before collision scoring and selection; do not modify
// a selected trajectory into a different, unchecked motion after planning.
class ModeConstraintCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    const auto key = dwb_plugin_name_ + "." + name_ + ".minimum_turning_radius";
    if (!node->has_parameter(key)) {node->declare_parameter(key, 0.5);}
    radius_ = node->get_parameter(key).as_double();
    if (!std::isfinite(radius_) || radius_ <= 0.0) {
      throw std::runtime_error("minimum_turning_radius must be positive");
    }
  }
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    const auto & v = trajectory.velocity;
    if (std::abs(v.y) > 1e-6 && std::abs(v.theta) > 1e-6) {
      throw dwb_core::IllegalTrajectoryException(name_, "Parallel mode cannot rotate");
    }
    if (std::abs(v.x) > 1e-6 && std::abs(v.theta) * radius_ > std::abs(v.x) + 1e-6) {
      throw dwb_core::IllegalTrajectoryException(name_, "Steering radius too small");
    }
    return 0.0;
  }
private:
  double radius_{0.5};
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::ModeConstraintCritic, dwb_core::TrajectoryCritic)
