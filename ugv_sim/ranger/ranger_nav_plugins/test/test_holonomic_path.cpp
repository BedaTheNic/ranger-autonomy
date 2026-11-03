// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/holonomic_path.hpp"
#include "../src/holonomic_velocity.hpp"
using namespace ranger_nav_plugins;
static nav2_costmap_2d::Footprint footprint()
{
  nav2_costmap_2d::Footprint f;
  for(auto xy:std::vector<std::pair<double,double>>{{.35,.32},{.35,-.32},{-.35,-.32},{-.35,.32}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;f.push_back(p);
  }
  return f;
}
TEST(HolonomicPath, ArbitraryDiagonalAttainsEuclideanLowerBoundWithoutRotatingBody)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  auto path=directHolonomic({0,0,.23},{2,1,.23},map,footprint());
  ASSERT_EQ(path.size(),2u);EXPECT_NEAR(pathLength(path),std::sqrt(5.),1e-9);
  auto dense=points(densePath(path,std_msgs::msg::Header{}));
  for(auto p:dense) {EXPECT_NEAR(p.y,.5*p.x,1e-9);EXPECT_NEAR(p.a,.23,1e-9);}
  EXPECT_LT(distance(dense[0],dense[1]),.041);
}
TEST(HolonomicPath, GoalYawIsAnExplicitPivotNotRotationDuringCrabMotion)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  auto path=directHolonomic({0,0,0},{2,1,M_PI_2},map,footprint());
  ASSERT_EQ(path.size(),3u);EXPECT_EQ(path[1].a,0);EXPECT_EQ(path.back().a,M_PI_2);
  EXPECT_DOUBLE_EQ(distance(path[1],path[2]),0);EXPECT_TRUE(nextPivot(points(densePath(path,std_msgs::msg::Header{})),0));
}
TEST(HolonomicPath, NoCornerCutThroughObstacleOrUnknown)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  for(unsigned char cost:{254,255}) {
    map.resetMap(0,0,200,200);
    unsigned int x,y;map.worldToMap(1,1,x,y);map.setCost(x,y,cost);
    EXPECT_TRUE(directHolonomic({0,0,0},{2,2,0},map,footprint()).empty());
    std::vector<Pose2> around{{0,0,0},{0,2,0},{2,2,0}};
    auto shortened=shortcutHolonomic(around,map,footprint());
    EXPECT_EQ(shortened.size(),3u);EXPECT_TRUE(pathFree(shortened,map,footprint()));
  }
}
TEST(HolonomicPath, WholeFootprintRejectsClearCentreLineBesideWall)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  unsigned int x,y;map.worldToMap(1,.25,x,y);map.setCost(x,y,254);
  EXPECT_TRUE(directHolonomic({0,0,0},{2,0,0},map,footprint()).empty());
}
TEST(HolonomicPath, RemovesOptionalPivotsOnlyWithAFullCollisionCheckedReplacement)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  std::vector<Pose2> stairs{{0,0,0},{.4,0,0},{.4,.4,0},{.8,.4,0},{.8,.8,0},
    {.8,.8,M_PI_2},{1.2,.8,M_PI_2},{1.2,1.2,M_PI_2}};
  auto result=shortcutHolonomic(stairs,map,footprint());
  ASSERT_EQ(result.size(),3u);EXPECT_TRUE(nextPivot(result,0));
  EXPECT_DOUBLE_EQ(distance(result[1],result[2]),0);
  EXPECT_LT(pathLength(result),pathLength(stairs));EXPECT_TRUE(pathFree(result,map,footprint()));
}
TEST(HolonomicPath, KeepsRequiredPivotBetweenNarrowPerpendicularPassages)
{
  nav2_costmap_2d::Costmap2D map(350,350,.01,-1,-1,254);
  for(unsigned int x=0;x<350;++x) {
    for(unsigned int y=0;y<350;++y) {
      double wx,wy;map.mapToWorld(x,y,wx,wy);
      if((wx<1.6 && std::abs(wy)<.34) ||
        (wx>.9 && wx<2.1 && std::abs(wy)<.6) ||
        (wx>1.16 && wx<1.84 && wy>0)) {map.setCost(x,y,0);}
    }
  }
  std::vector<Pose2> path{{0,0,0},{1.5,0,0},{1.5,0,M_PI_2},{1.5,2,M_PI_2}};
  ASSERT_TRUE(pathFree(path,map,footprint()));
  EXPECT_TRUE(directHolonomic(path.front(),path.back(),map,footprint()).empty());
  auto result=shortcutHolonomic(path,map,footprint());
  EXPECT_TRUE(nextPivot(result,0));EXPECT_TRUE(pathFree(result,map,footprint()));
  EXPECT_LE(pathLength(result),pathLength(path));
}
TEST(HolonomicPath, ShortcutBudgetExhaustionKeepsTheOriginalFeasibleRoute)
{
  nav2_costmap_2d::Costmap2D map(200,200,.025,-1,-1,0);
  std::vector<Pose2> p{{0,0,0},{0,1,0},{1,1,0}};
  auto r=shortcutHolonomic(p,map,footprint(),[]{return true;});
  EXPECT_EQ(r.size(),p.size());EXPECT_DOUBLE_EQ(pathLength(r),pathLength(p));
}
TEST(HolonomicPath, ShorterRouteWinsEvenWithSlowerEstimatedArrival)
{
  EXPECT_TRUE(preferShorterRoute(2,50,3,20));
  EXPECT_FALSE(preferShorterRoute(3,10,2,50));
  EXPECT_TRUE(preferShorterRoute(2,10,2,20));
  EXPECT_FALSE(preferShorterRoute(INFINITY,INFINITY,2,20));
}
TEST(HolonomicPath, SnappedEndpointsCannotMakeADetourLookShorterThanTheDirectRoute)
{
  const Pose2 start{0,0,0},goal{1,0,0};
  std::vector<Pose2> snapped{{.025,.01,0},{.975,.01,0}};
  ASSERT_LT(pathLength(snapped),distance(start,goal));
  EXPECT_GT(routeLength(snapped,start,goal),distance(start,goal));
  EXPECT_FALSE(preferShorterRoute(routeLength(snapped,start,goal),1,1,10));
  EXPECT_DOUBLE_EQ(routeLength({start,goal},start,goal),1);
  EXPECT_TRUE(std::isinf(routeLength({},start,goal)));
}
TEST(HolonomicVelocity, PreservesDiagonalRatioUnderAccelerationAndSpeedLimits)
{
  TranslationWindow w{-.05,.05,-.05,.05,-.06,.05,.2};
  auto v=alignedTranslation(2,1,.2,w);ASSERT_TRUE(v);
  EXPECT_NEAR(v->linear.x,.05,1e-9);EXPECT_NEAR(v->linear.y,.025,1e-9);
  w={-.2,.2,-.2,.2,-.06,.05,.1};
  v=alignedTranslation(-2,-1,.2,w);ASSERT_TRUE(v);
  EXPECT_NEAR(std::hypot(v->linear.x,v->linear.y),.1,1e-9);
  EXPECT_NEAR(v->linear.x/v->linear.y,2,1e-9);EXPECT_DOUBLE_EQ(v->angular.z,0);
}
TEST(HolonomicVelocity, DoesNotInventInstantDirectionOrRotationChanges)
{
  EXPECT_FALSE(alignedTranslation(0,1,.2,{.1,.2,-.05,.05,-.06,.05,.2}));
  EXPECT_FALSE(alignedTranslation(1,1,.2,{-.05,.05,-.05,.05,.1,.2,.2}));
  EXPECT_FALSE(alignedTranslation(1,1,.2,{-.05,.05,-.05,.05,-.06,.05,0}));
}

#include "../src/route_quality.hpp"
#include "../src/stopping_distance.hpp"

TEST(RouteQuality, RetainedPathUsesReserveAndIgnoresPassedObstacle)
{
  Grid grid(400,120,.01,-1.,-.6,0);
  auto base=footprint(); auto reserve=base; nav2_costmap_2d::padFootprint(reserve,.04);
  std::vector<Pose2> path{{0,0,0},{.5,0,0},{1,0,0},{1.5,0,0},{2,0,0}};
  unsigned mx,my; ASSERT_TRUE(grid.worldToMap(1.6,.35,mx,my)); grid.setCost(mx,my,254);
  size_t cursor=1;
  EXPECT_TRUE(retainedPathFree(path,{.7,0,0},cursor,grid,base));
  EXPECT_FALSE(retainedPathFree(path,{.7,0,0},cursor,grid,reserve));
  grid.setCost(mx,my,0); grid.worldToMap(0,0,mx,my); grid.setCost(mx,my,254);
  EXPECT_TRUE(retainedPathFree(path,{.8,0,0},cursor,grid,reserve));
}

TEST(RouteQuality, NearStraightCanAlignWithoutChangingEndpointsOrLength)
{
  Grid grid(600,300,.02,-1.,-2.,0);
  const Pose2 a{0,0,0}, b{8,.208,0};
  auto path=points(densePath({a,b},std_msgs::msg::Header{}));
  const double length=pathLength(path);
  EXPECT_EQ(alignStraights(path,grid,footprint(),{}),1u);
  EXPECT_NEAR(pathLength(path),length,1e-8);
  EXPECT_DOUBLE_EQ(path.front().a,a.a);EXPECT_DOUBLE_EQ(path.back().a,b.a);
  EXPECT_TRUE(pathFree(path,grid,footprint()));
  auto diagonal=points(densePath({a,{2,1,0}},std_msgs::msg::Header{}));
  EXPECT_EQ(alignStraights(diagonal,grid,footprint(),{}),0u);
}

TEST(StoppingDistance, SlowsBeforeObstacleAndAccountsForSensorDelay)
{
  Grid grid(400,120,.01,-1,-.6,0);
  unsigned x,y;grid.worldToMap(.435,0,x,y);
  for(unsigned row=0;row<grid.getSizeInCellsY();++row) {grid.setCost(x,row,254);}
  Twist v;v.linear.x=.2;
  EXPECT_DOUBLE_EQ(stoppingScale({0,0,0},v,v,0,.1,.4,.5,grid,footprint()),1.);
  const auto limited=stoppingScale({0,0,0},v,v,.12,.1,.4,.5,grid,footprint());
  EXPECT_LT(limited,1.);EXPECT_GT(limited,0.);
  EXPECT_DOUBLE_EQ(stoppingScale({0,0,0},v,v,.3,.1,.4,.5,grid,footprint()),0.);
}

TEST(RouteQuality, InvalidSmoothingKeepsRawRouteAndItsReversal)
{
  Grid grid(300,200,.02,-2,-2,0);
  const std::vector<Pose2> raw{{0,0,0},{1,0,0},{.5,0,0}};
  EXPECT_EQ(selectHybrid(raw,{{0,0,0},{.5,0,0}},grid,footprint(),{}),HybridChoice::raw);
  const std::vector<Pose2> bend{{0,0,0},{.4,.15,1.2},{1,0,0},{.5,0,0}};
  EXPECT_EQ(selectHybrid(raw,bend,grid,footprint(),{}),HybridChoice::raw);
  unsigned x,y;grid.worldToMap(.75,.2,x,y);grid.setCost(x,y,254);
  EXPECT_EQ(selectHybrid(raw,bend,grid,footprint(),{}),HybridChoice::none);
}
