// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/tracking_critics.cpp"

TEST(PathTracking, ScoresCornerCuttingEvenWhenBothEndpointsAreOnPath)
{
  ranger_nav_plugins::PathTrackingCritic critic;
  nav_2d_msgs::msg::Path2D path;
  geometry_msgs::msg::Pose2D p;
  p.x = 0; p.y = 0; path.poses.push_back(p);
  p.x = 1; path.poses.push_back(p);
  p.y = 1; path.poses.push_back(p);
  geometry_msgs::msg::Pose2D zero_pose;
  nav_2d_msgs::msg::Twist2D zero_velocity;
  ASSERT_TRUE(critic.prepare(zero_pose, zero_velocity, zero_pose, path));
  dwb_msgs::msg::Trajectory2D following; following.poses = path.poses;
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(following), 0.0);
  auto shortcut = following; shortcut.poses[1].x = shortcut.poses[1].y = 0.5;
  EXPECT_NEAR(critic.scoreTrajectory(shortcut), 1.0 / 6.0, 1e-9);
  auto dense = shortcut;
  for (auto q : shortcut.poses) {dense.poses.push_back(q);}
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(shortcut), critic.scoreTrajectory(dense));
  for (auto & q : following.poses) {q.theta = M_PI_2;}
  EXPECT_GT(critic.scoreTrajectory(following), 0.0);
  // A lateral reference may explicitly retain a sideways chassis heading.
  for (auto & q : path.poses) {q.theta = M_PI_2;}
  ASSERT_TRUE(critic.prepare(zero_pose, zero_velocity, zero_pose, path));
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(following), 0.0);
}
TEST(SteeringContinuity, InitialAlignmentAndReverseLateralTravelRemainPossible)
{
  ranger_nav_plugins::SteeringContinuityCritic critic;
  dwb_msgs::msg::Trajectory2D t; t.velocity.y = 0.1;
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(t), 0.0);
  nav_2d_msgs::msg::Twist2D v; v.x = 0.1; critic.debrief(v);
  EXPECT_NEAR(critic.scoreTrajectory(t), M_PI_2, 1e-9);
  critic.debrief(t.velocity);
  t.velocity.y = -0.1;
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(t), 0.0);
  nav_2d_msgs::msg::Twist2D stopped;
  critic.debrief(stopped);
  t.velocity.x = 0.1; t.velocity.y = 0.0;
  EXPECT_NEAR(critic.scoreTrajectory(t), M_PI_2, 1e-9);
  critic.reset();
  EXPECT_DOUBLE_EQ(critic.scoreTrajectory(t), 0.0);
}
