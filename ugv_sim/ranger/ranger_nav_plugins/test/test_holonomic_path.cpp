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

#include "../src/segment_route.hpp"
TEST(HolonomicPath, CloseButFeasibleWheelSweepIsCostlierThanOpenDetour)
{
  Grid grid(160,160,.025,-1.,-1.,0);
  unsigned x,y;grid.worldToMap(.95,.37,x,y);grid.setCost(x,y,254);
  const std::vector<Pose2> near{{0.,0.,0.},{2.,0.,0.}};
  const std::vector<Pose2> detour{{0.,0.,0.},{1.,-.3,0.},{2.,0.,0.}};
  ASSERT_TRUE(pathFree(near,grid,footprint()));
  ASSERT_TRUE(pathFree(detour,grid,footprint()));
  EXPECT_GT(edgeExposure(near,grid,footprint()),edgeExposure(detour,grid,footprint()));
}

TEST(TravelSegments, GoalBehindUsesForwardTravelAndPreservesBothYaws)
{
  Grid grid(300,150,.025,-2,-1.5,0);
  const std::vector<Pose2> raw{{0,0,M_PI},{2,0,M_PI}};
  const auto shaped=shapeTravelLegs(raw,grid,footprint(),{});
  EXPECT_TRUE(pathFree(shaped,grid,footprint()));
  EXPECT_LT(travelCost(shaped,grid,{}).heading_loss,1e-6);
  EXPECT_NEAR(wrap(shaped.front().a-M_PI),0,1e-6);
  EXPECT_NEAR(wrap(shaped.back().a-M_PI),0,1e-6);
  EXPECT_TRUE(nextPivot(shaped,0));
  EXPECT_NEAR(pathLength(shaped),2,1e-6);
}

TEST(TravelSegments, BalancesOppositeWallsAndKeepsRequestedOffCenterEndpoints)
{
  Grid grid(300,200,.025,-1,-2.5,0);
  for(unsigned x=0;x<300;++x) {grid.setCost(x,73,254);grid.setCost(x,126,254);}
  const std::vector<Pose2> raw{{0,.22,0},{4,.22,0}};
  auto shaped=shapeTravelLegs(raw,grid,footprint(),{});
  EXPECT_TRUE(pathFree(shaped,grid,footprint()));
  EXPECT_NEAR(shaped.front().y,.22,1e-8);EXPECT_NEAR(shaped.back().y,.22,1e-8);
  EXPECT_LT(std::abs(project(shaped,{2,0,0}).pose.y),.03);
  EXPECT_LT(travelCost(shaped,grid,{}).center_loss,travelCost(raw,grid,{}).center_loss);
  for(unsigned x=0;x<300;++x) {grid.setCost(x,126,0);}
  EXPECT_DOUBLE_EQ(corridorOffset({2,.22,0},0,grid,1.6),0.);
}

TEST(TravelSegments, KeepsBackwardOrCrabTravelWhenTheSweptTurnCannotFit)
{
  Grid grid(300,200,.025,-1,-2.5,254);
  for(unsigned x=0;x<300;++x) {
    for(unsigned y=85;y<115;++y) {grid.setCost(x,y,0);}
  }
  for(double yaw:{M_PI,M_PI_2}) {
    const std::vector<Pose2> raw{{0,0,yaw},{3,0,yaw}};
    auto shaped=shapeTravelLegs(raw,grid,footprint(),{});
    EXPECT_TRUE(pathFree(shaped,grid,footprint()));
    EXPECT_FALSE(nextPivot(shaped,0));
    EXPECT_NEAR(travelCost(shaped,grid,{}).heading_loss,travelCost(raw,grid,{}).heading_loss,1e-6);
  }
  // The same narrow endpoints with a wider bay between them permit two turns.
  for(unsigned x=70;x<134;++x) {
    for(unsigned y=64;y<136;++y) {grid.setCost(x,y,0);}
  }
  const std::vector<Pose2> raw{{0,0,M_PI},{3,0,M_PI}};
  auto shaped=shapeTravelLegs(raw,grid,footprint(),{});
  EXPECT_TRUE(pathFree(shaped,grid,footprint()));EXPECT_TRUE(nextPivot(shaped,0));
  EXPECT_LT(travelCost(shaped,grid,{}).heading_loss,travelCost(raw,grid,{}).heading_loss-1.);
}

TEST(TravelSegments, IntermediateHeadingOfLongRotationIsPreserved)
{
  std::vector<Pose2> turn{{0,0,0},{0,0,2.},{0,0,-2.3},{1,0,-2.3}};
  EXPECT_EQ(nextPivot(turn,0),1u);
  EXPECT_EQ(nextPivot(turn,1),2u);
}

TEST(TravelSegments, ChoosesFewestHandoffsThenSimplestComparableTime)
{
  std::vector<RouteRank> candidates{{2,5,8.,0.},{0,4,10.,0.},{0,2,10.8,0.},
    {0,1,18.,0.},{0,1,5.,2.}};
  EXPECT_EQ(selectSchedule(candidates,.10),2u);
  EXPECT_EQ(selectSchedule(candidates,0.),1u);
}

TEST(TravelSegments, TinyTerminalPivotKeepsExactGoalWithoutAnotherManeuver)
{
  std::vector<Pose2> path{{0,0,.002},{1,1,.002},{1,1,0.}};
  const auto simpler=mergeSmallPivots(path);
  ASSERT_EQ(simpler.size(),2u);
  EXPECT_DOUBLE_EQ(simpler.front().a,.002);EXPECT_DOUBLE_EQ(simpler.back().a,0.);
  path.back().a=M_PI_2;
  EXPECT_EQ(mergeSmallPivots(path).size(),3u);
}

TEST(TravelSegments, LineOfSightGraphDropsUnnecessaryStopsAroundObstacle)
{
  Grid grid(250,200,.025,-2,-2,0);
  unsigned x,y;grid.worldToMap(0.,0.,x,y);grid.setCost(x,y,254);
  std::vector<Pose2> corners{{-1.,0,0},{-.9,.3,0},{-.65,.65,0},{0,.95,0},{.7,.75,0},{1.,0,0}};
  auto original=points(densePath(corners,std_msgs::msg::Header{}));
  auto simple=minimumLegRoute(original,0.,grid,footprint());
  ASSERT_GE(simple.size(),3u);EXPECT_LT(simple.size(),corners.size());
  EXPECT_TRUE(pathFree(simple,grid,footprint()));
}

#include "../src/clearance_route.hpp"

TEST(TravelSegments, ClearanceSearchFindsAnOffCentreDetourForAnOrientedFootprint)
{
  Grid grid(110,90,.05,-2.75,-2.25,0);
  unsigned mx,my;
  grid.worldToMap(0.,0.,mx,my);
  // Only the near face is initially visible to the moving LiDAR.
  for(int row=-2;row<=2;++row) {grid.setCost(mx,my+row,254);}
  const Pose2 start{-1.2,0.,0.},goal{1.2,0.,0.};
  auto line=clearanceRoute(start,goal,grid,footprint(),0.);
  ASSERT_GE(line.size(),3u);
  auto route=constantHeadingRoute(line,0.);
  ASSERT_TRUE(pathFree(route,grid,footprint()));
  // The next scan reveals the hidden side of the same object.
  for(int row=-2;row<=2;++row) {
    for(int col=1;col<=3;++col) {grid.setCost(mx+col,my+row,254);}
  }
  EXPECT_TRUE(pathFree(route,grid,footprint()));
}

TEST(TravelSegments, ClearancePreferenceAllowsTightOnlyPassageAndStopsRewardingOpenSpace)
{
  EXPECT_EQ(selectSchedule({{0,1,7.,0.,.01},{0,3,10.,0.,.12}},.1),1u);
  EXPECT_EQ(selectSchedule({{0,1,7.,0.,.00}},.1),0u);
  EXPECT_EQ(selectSchedule({{0,1,7.,0.,.15},{1,3,10.,0.,.20}},.1),0u);
}

TEST(TravelSegments, UnseenSurfaceChangesOnlyDetourRanking)
{
  Grid original(100,100,.05,-2.5,-2.5,0);
  unsigned x,y;original.worldToMap(.5,0.,x,y);original.setCost(x,y,254);
  double wx,wy;original.mapToWorld(x,y,wx,wy);
  BlockingCell hit{wx,wy,254,"edge"};
  auto risk=occlusionRiskGrid(original,hit,{-1.,wy,0.},.20);
  ASSERT_TRUE(risk);
  EXPECT_EQ(original.getCost(x+2,y),0);
  EXPECT_EQ(risk->getCost(x+2,y),254);
  EXPECT_EQ(risk->getCost(x-2,y),0);
  EXPECT_EQ(risk->getCost(x+7,y),0);
  hit.cost=255;
  EXPECT_FALSE(occlusionRiskGrid(original,hit,{-1.,wy,0.},.20));
}

TEST(TravelSegments, TwoVisibleFacesBoundTheEstimatedShadow)
{
  Grid grid(100,100,.05,-2.5,-2.5,0);
  unsigned x,y;grid.worldToMap(.5,.5,x,y);
  for(unsigned k=0;k<5;++k) {grid.setCost(x+k,y,254);grid.setCost(x,y+k,254);}
  double wx,wy;grid.mapToWorld(x,y,wx,wy);
  auto shadow=occlusionRiskGrid(grid,{wx,wy,254,"edge"},{-1.,-1.,0.},.20);
  ASSERT_TRUE(shadow);
  EXPECT_EQ(shadow->getCost(x+2,y+2),254);
  EXPECT_EQ(shadow->getCost(x+5,y+2),0);
  EXPECT_EQ(shadow->getCost(x+2,y+5),0);
  EXPECT_EQ(grid.getCost(x+2,y+2),0);
}

TEST(TravelSegments, SubcellRefinementGivesTheFootprintRoomBesideAnEdge)
{
  Grid grid(120,100,.05,-3.,-2.5,0);
  unsigned mx,my;grid.worldToMap(0.,-.38,mx,my);
  for(unsigned x=mx-4;x<=mx+4;++x) {grid.setCost(x,my,254);}
  const std::vector<Pose2> reference{{-1.5,0.,.3},{0.,0.,0.},{1.5,0.,-.4}};
  double before=0.,after=0.;
  edgeExposure(constantHeadingRoute(reference,0.),grid,footprint(),&before);
  const auto refined=centeredCurve(reference,0.,grid,footprint(),true);
  ASSERT_FALSE(refined.empty());
  EXPECT_TRUE(pathFree(refined,grid,footprint()));
  EXPECT_NEAR(distance(refined.front(),reference.front()),0.,1e-9);
  EXPECT_NEAR(distance(refined.back(),reference.back()),0.,1e-9);
  EXPECT_EQ(refined.front().a,reference.front().a);
  EXPECT_EQ(refined.back().a,reference.back().a);
  edgeExposure(refined,grid,footprint(),&after);
  EXPECT_GT(after,before+.03);
  for(unsigned y=0;y<grid.getSizeInCellsY();++y) {grid.setCost(mx,y,254);}
  EXPECT_TRUE(centeredCurve(reference,0.,grid,footprint(),true).empty());
}

TEST(TravelSegments, WideBendAllowsOneIntermediateTurnUnderOneController)
{
  Grid grid(150,150,.05,-3.,-3.,0);
  const std::vector<Pose2> corners{{0.,0.,0.},{1.5,0.,0.},{1.5,1.5,M_PI_2}};
  const auto input=points(densePath(corners,std_msgs::msg::Header{},.08));
  const auto route=twoHeadingCurve(input,grid,footprint());
  ASSERT_FALSE(route.empty());
  EXPECT_TRUE(pathFree(route,grid,footprint()));
  EXPECT_EQ(route.front().a,0.);
  EXPECT_NEAR(route.back().a,M_PI_2,1.e-9);
  EXPECT_EQ(decide(route,{},0.).mixed.switches,0u);
  unsigned middle_pivots=0;
  for(size_t i=0;i+1<route.size();++i) {
    if(distance(route[i],route[i+1])<1e-6 && distance(route[i],route.front())>.01 &&
        distance(route[i],route.back())>.01) {++middle_pivots;}
  }
  EXPECT_EQ(middle_pivots,1u);
}

TEST(HolonomicPath, ResumeReusesOnlyTheSameStampedGeometry)
{
  std_msgs::msg::Header header;header.frame_id="map";header.stamp.sec=4;
  const auto plan=densePath({{0.,0.,0.},{0.,0.,M_PI_2},{0.,1.,M_PI_2}},header);
  auto resumed=plan;
  EXPECT_TRUE(sameStampedPath(plan,resumed));
  resumed.poses.back().pose.position.y+=.01;
  EXPECT_FALSE(sameStampedPath(plan,resumed));
  resumed=plan;++resumed.header.stamp.sec;
  EXPECT_FALSE(sameStampedPath(plan,resumed));
  resumed.header.stamp.sec=0;
  EXPECT_FALSE(sameStampedPath(resumed,resumed));
}
