// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "dwb_core/exceptions.hpp"
#include "../src/clearance_critic.cpp"

class FixtureCritic : public ranger_nav_plugins::MaxObstacleFootprintCritic
{
public:
  explicit FixtureCritic(nav2_costmap_2d::Costmap2D & map)
  {
    costmap_ = &map;
    for (auto xy : {std::pair<double, double>{.1, .1}, {.1, -.1},
      {-.1, -.1}, {-.1, .1}})
    {
      geometry_msgs::msg::Point p; p.x = xy.first; p.y = xy.second;
      footprint_spec_.push_back(p);
    }
  }
};

TEST(ClearanceCritic, CloseCornerCannotBeHiddenByFreeEndpoint)
{
  nav2_costmap_2d::Costmap2D map(50, 40, .1, 0., 0., 0);
  FixtureCritic critic(map);
  dwb_msgs::msg::Trajectory2D near;
  for (double x : {1., 2., 3.}) {
    geometry_msgs::msg::Pose2D p; p.x = x; p.y = 1.; near.poses.push_back(p);
  }
  // Inflated obstacle neighbourhood touched only by the intermediate pose.
  for (unsigned int x = 18; x <= 22; ++x) {
    for (unsigned int y = 8; y <= 12; ++y) {map.setCost(x, y, 200);}
  }
  EXPECT_DOUBLE_EQ(critic.scorePose(near.poses.back()), 0.);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(near), 200.);
  auto clear = near;
  for (auto & p : clear.poses) {p.y = 2.;}
  EXPECT_LT(critic.scoreTrajectory(clear), critic.scoreTrajectory(near));
  // Sampling density must not change the maximum score.
  near.poses.insert(near.poses.begin() + 1, near.poses[1]);
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(near), 200.);
  for (unsigned int x = 18; x <= 22; ++x) {
    for (unsigned int y = 8; y <= 12; ++y) {map.setCost(x, y, 254);}
  }
  EXPECT_THROW(critic.scoreTrajectory(near), dwb_core::IllegalTrajectoryException);
}
