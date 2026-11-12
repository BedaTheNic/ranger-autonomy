// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/footprint_escape.hpp"

namespace ranger_nav_plugins
{
class FootprintEscapeTest : public ::testing::Test
{
protected:
  FootprintEscapeTest()
  {
    for (const auto & vertex : std::vector<std::pair<double, double>>{
        {-.36, -.20}, {.36, -.20}, {.36, .20}, {-.36, .20}})
    {
      geometry_msgs::msg::Point point;
      point.x = vertex.first;
      point.y = vertex.second;
      footprint.push_back(point);
    }
  }

  void mark(double x, double y, unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
  {
    unsigned col, row;
    ASSERT_TRUE(map.worldToMap(x, y, col, row));
    map.setCost(col, row, cost);
  }

  nav2_costmap_2d::Costmap2D map{151, 151, .02, -1.51, -1.51, 0};
  nav2_costmap_2d::Footprint footprint;
  Pose2 start;
  FootprintBlocked blocked_turn{"Terminal rotation is blocked", {0, 0, M_PI_2}, true};
};

TEST_F(FootprintEscapeTest, FindsOffsetForTurnAndChecksReturnAtGoalHeading)
{
  mark(.24, .24);
  ASSERT_TRUE(poseFree(start, map, footprint));
  ASSERT_FALSE(rotationFree(start, M_PI_2, map, footprint));
  ASSERT_TRUE(poseFree(blocked_turn.target, map, footprint));
  auto offset = findFootprintEscape(start, blocked_turn, .24, map, footprint);
  ASSERT_TRUE(offset.has_value());
  EXPECT_GT(distance(*offset, start), .025);
  EXPECT_LE(distance(*offset, start), .24);
  EXPECT_TRUE(escapeTranslationFree(start, *offset, map, footprint));
  EXPECT_TRUE(escapeRotationFree(*offset, M_PI_2, map, footprint));
  EXPECT_TRUE(escapeRestoresTracking(*offset, blocked_turn, map, footprint));
}

TEST(FootprintEscapeRotationTest, RejectsIntermediateCellSquareOverlap)
{
  nav2_costmap_2d::Costmap2D map(41, 41, .05, -1.025, -1.025, 0);
  nav2_costmap_2d::Footprint footprint;
  for (const auto & vertex : std::vector<std::pair<double, double>>{
      {-.32, -.30}, {.32, -.30}, {.32, .30}, {-.32, .30}})
  {
    geometry_msgs::msg::Point point;
    point.x = vertex.first;
    point.y = vertex.second;
    footprint.push_back(point);
  }
  unsigned col, row;
  ASSERT_TRUE(map.worldToMap(-.40, -.25, col, row));
  map.setCost(col, row, nav2_costmap_2d::LETHAL_OBSTACLE);
  const Pose2 start{0, 0, 1.2}, finish{0, 0, 1.3};
  ASSERT_TRUE(escapeTranslationFree(start, start, map, footprint));
  ASSERT_TRUE(escapeTranslationFree(finish, finish, map, footprint));
  EXPECT_FALSE(escapeRotationFree(start, finish.a, map, footprint));
}

TEST_F(FootprintEscapeTest, RejectsOccupiedStartAndUnknownSweptCells)
{
  mark(.30, .10);
  EXPECT_FALSE(findFootprintEscape(start, blocked_turn, .24, map, footprint).has_value());
  mark(.30, .10, nav2_costmap_2d::FREE_SPACE);
  mark(.40, .19, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(escapeTranslationFree(start, {.12, 0, 0}, map, footprint));
  EXPECT_TRUE(escapeTranslationFree(start, {-.12, 0, 0}, map, footprint));
}

TEST_F(FootprintEscapeTest, RefusesTurnWhenFinalGoalIsOccupied)
{
  mark(.24, .24);
  mark(.01, .30);
  EXPECT_FALSE(poseFree(blocked_turn.target, map, footprint));
  EXPECT_FALSE(findFootprintEscape(start, blocked_turn, .24, map, footprint).has_value());
}

TEST_F(FootprintEscapeTest, RestoresAnApproachBlockedByAnOffCentreObstacle)
{
  mark(.42, .20);
  const FootprintBlockage blocked_approach{{.90, 0, 0}, false};
  ASSERT_FALSE(escapeRestoresTracking(start, blocked_approach, map, footprint));
  auto offset = findFootprintEscape(start, blocked_approach, .24, map, footprint);
  ASSERT_TRUE(offset.has_value());
  EXPECT_LT(offset->y, 0);
  EXPECT_TRUE(escapeTranslationFree(start, *offset, map, footprint));
  EXPECT_TRUE(escapeRestoresTracking(*offset, blocked_approach, map, footprint));
}

TEST_F(FootprintEscapeTest, RespectsRemainingBudgetAndRejectsUnneededEscape)
{
  EXPECT_FALSE(findFootprintEscape(start, blocked_turn, .24, map, footprint).has_value());
  mark(.24, .24);
  EXPECT_FALSE(findFootprintEscape(start, blocked_turn, .05, map, footprint).has_value());
}
}  // namespace ranger_nav_plugins
