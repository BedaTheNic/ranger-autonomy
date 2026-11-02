// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include "dwb_critics/obstacle_footprint.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ranger_nav_plugins
{
// Humble's default sum_scores=false retains only the FINAL pose's soft cost.
// Check every pose with the upstream footprint collision checker, and retain
// the worst clearance cost. Unlike summation, this does not penalize a path
// merely for containing more samples (or reward a stationary trajectory).
class MaxObstacleFootprintCritic : public dwb_critics::ObstacleFootprintCritic
{
public:
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    double worst = 0.0;
    for (const auto & pose : trajectory.poses) {
      worst = std::max(worst, scorePose(pose));
    }
    return worst;
  }
};
}
PLUGINLIB_EXPORT_CLASS(ranger_nav_plugins::MaxObstacleFootprintCritic, dwb_core::TrajectoryCritic)
