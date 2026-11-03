// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/navigation_geometry.hpp"
#include "../src/heading_maneuver.hpp"
#include "../src/goal_connection.hpp"
using namespace ranger_nav_plugins;
TEST(Geometry, AssistanceHonoursSpeedLimitUnitsAndReset)
{
  EXPECT_DOUBLE_EQ(speedLimitScale(0,false,.2),1);
  EXPECT_DOUBLE_EQ(speedLimitScale(.05,false,.2),.25);
  EXPECT_DOUBLE_EQ(speedLimitScale(25,true,.2),.25);
  EXPECT_DOUBLE_EQ(speedLimitScale(150,true,.2),1);
  EXPECT_THROW(speedLimitScale(-1,false,.2),std::runtime_error);
}
TEST(Geometry, ReversePursuitPreservesAckermannCurvature)
{
  auto v=pursuit({0,0,0},{-.08,.01,-.1},true,-1,.8,.08);
  EXPECT_LT(v.linear.x,0);EXPECT_LT(v.angular.z,0);
  EXPECT_DOUBLE_EQ(v.linear.y,0);
  EXPECT_LE(std::abs(v.angular.z)*.8,std::abs(v.linear.x)+1e-9);
}
TEST(Geometry, LateralCandidateDoesNotRotate)
{
  auto v=pursuit({0,0,M_PI_2},{1,0,0},false,1,.8,.12);
  EXPECT_NEAR(v.linear.x,0,1e-9);EXPECT_LT(v.linear.y,0);
  EXPECT_DOUBLE_EQ(v.angular.z,0);
}
TEST(Geometry, AckermannProgressAssistCannotDriveAwayFromAMissedCusp)
{
  for (const int direction : {-1, 1}) {
    const auto command = pursuit({0, 0, 0}, {-direction * .06, .02, 0},
      true, direction, .8, .08);
    EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}
TEST(Geometry, CornerProjectionCannotSkipTheVertexToAnAckermannExit)
{
  std::vector<Pose2> p{{0,0,0},{0,1,0},{1,1,0}};
  auto held=project(p,{.15,.9,0},0,3,true);
  EXPECT_EQ(held.index,0u);EXPECT_NEAR(cornerDistance(p,held),.1,1e-9);
  auto arrived=project(p,{.03,.99,0},0,3,true);
  EXPECT_EQ(arrived.index,1u);
  EXPECT_TRUE(std::isinf(cornerDistance(p,arrived)));
  auto incoming=project(p,{0,.97,0},0,3,true);
  EXPECT_EQ(incoming.index,1u);
  EXPECT_GT(lookahead(p,incoming,.35).x,.3);
}
TEST(Geometry, MandatoryPivotSeparatesOverlappingTranslationBranches)
{
  std::vector<Pose2> path{{1,0,0},{0,0,0},{0,0,-M_PI_4},{0,0,-M_PI_2},{1,0,-M_PI_2}};
  auto pivot=nextPivot(path,0);ASSERT_TRUE(pivot);EXPECT_EQ(*pivot,3u);
  EXPECT_FALSE(nextPivot(path,*pivot));
  auto prefix=path;prefix.resize(*pivot+1);
  EXPECT_EQ(project(prefix,{.8,0,0},0,3,true).index,0u);
  EXPECT_EQ(prefix.back().a,-M_PI_2);
}
TEST(Geometry, NewObstacleCancelsAnActiveRotation)
{
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  nav2_costmap_2d::Footprint f;
  for(auto xy:std::vector<std::pair<double,double>>{{.34,.32},{.34,-.32},{-.34,-.32},{-.34,.32}}) {
    geometry_msgs::msg::Point q;q.x=xy.first;q.y=xy.second;f.push_back(q);
  }
  std::vector<Pose2> path{{0,0,0},{1,0,M_PI_2}};
  geometry_msgs::msg::Twist speed;
  HeadingManeuver h;auto projection=project(path,{0,0,0});
  ASSERT_TRUE(h.compute({0,0,0},speed,path,projection,map,f,0,1,false));
  EXPECT_TRUE(h.active());
  for(unsigned int x=0;x<100;++x) {map.setCost(x,58,254);}
  auto stopped=h.compute({0,0,0},speed,path,projection,map,f,.1,1,false);
  ASSERT_TRUE(stopped);EXPECT_DOUBLE_EQ(stopped->angular.z,0);EXPECT_FALSE(h.active());
}
TEST(Geometry, FootprintSweepRejectsCollisionAndUnknown)
{
  nav2_costmap_2d::Costmap2D map(100,100,.05,-2.5,-2.5,0);
  nav2_costmap_2d::Footprint footprint;
  for(const auto & p:std::vector<std::pair<double,double>>{{.32,.3},{.32,-.3},{-.32,-.3},{-.32,.3}}) {
    geometry_msgs::msg::Point q;q.x=p.first;q.y=p.second;footprint.push_back(q);
  }
  geometry_msgs::msg::Twist v;v.linear.x=.2;
  EXPECT_TRUE(collisionFree({0,0,0},v,map,footprint));
  unsigned int x,y;map.worldToMap(.5,0,x,y);map.setCost(x,y,254);
  EXPECT_FALSE(collisionFree({0,0,0},v,map,footprint));
  map.setCost(x,y,255);EXPECT_FALSE(collisionFree({0,0,0},v,map,footprint));
  map.setCost(x,y,0);map.worldToMap(0,0,x,y);map.setCost(x,y,254);
  EXPECT_FALSE(collisionFree({0,0,0},v,map,footprint));
}
TEST(Geometry, AssistanceStartsBeforeGlobalProgressTimeoutAndKeepsCommitment)
{
  ProgressAssist a; EXPECT_FALSE(a.needed({0,0,0},0,0));
  EXPECT_FALSE(a.needed({0,0,0},1,0));
  EXPECT_TRUE(a.needed({0,0,0},1.3,0));
  EXPECT_TRUE(a.needed({.03,0,0},1.7,.1));
  EXPECT_FALSE(a.needed({.08,0,0},2.5,.1));
  a.reset();EXPECT_FALSE(a.needed({0,0,0},3,0));
}

static nav2_costmap_2d::Footprint rectangle()
{
  nav2_costmap_2d::Footprint f;
  for(auto xy:std::vector<std::pair<double,double>>{{.34,.32},{.34,-.32},{-.34,-.32},{-.34,.32}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;f.push_back(p);
  }
  return f;
}
TEST(Geometry, WholeRotationRejectsIntermediateCornerCollision)
{
  nav2_costmap_2d::Costmap2D m(200,200,.02,-2,-2,0);auto f=rectangle();
  for(unsigned int x=0;x<200;++x) {m.setCost(x,120,254);}
  EXPECT_TRUE(poseFree({0,0,0},m,f));EXPECT_TRUE(poseFree({0,0,M_PI_2},m,f));
  EXPECT_FALSE(rotationFree({0,0,0},M_PI_2,m,f));
  EXPECT_FALSE(pathFree({{0,0,0},{0,0,M_PI_2}},m,f));
}
TEST(Geometry, ContinuousPathCheckFindsObstacleBetweenWaypoints)
{
  nav2_costmap_2d::Costmap2D m(200,200,.02,-2,-2,0);auto f=rectangle();
  unsigned int x,y;m.worldToMap(.7,0,x,y);m.setCost(x,y,254);
  EXPECT_TRUE(poseFree({0,0,0},m,f));EXPECT_TRUE(poseFree({1.4,0,0},m,f));
  EXPECT_FALSE(pathFree({{0,0,0},{1.4,0,0}},m,f));
}
TEST(Geometry, HolonomicRotationBrakesAndCommitsToReferenceHeading)
{
  nav2_costmap_2d::Costmap2D m(200,200,.02,-2,-2,0);auto f=rectangle();
  HeadingManeuver h;std::vector<Pose2> path{{0,0,0},{0,.4,M_PI_2},{0,.8,M_PI_2}};
  auto p=project(path,{0,0,0});geometry_msgs::msg::Twist speed;speed.linear.y=.1;
  auto brake=h.compute({0,0,0},speed,path,p,m,f,0,1,false);
  ASSERT_TRUE(brake);EXPECT_DOUBLE_EQ(brake->angular.z,0);EXPECT_DOUBLE_EQ(brake->linear.y,0);
  speed.linear.y=0;auto turn=h.compute({0,0,0},speed,path,p,m,f,.1,1,false);
  ASSERT_TRUE(turn);EXPECT_GT(turn->angular.z,0);EXPECT_DOUBLE_EQ(turn->linear.x,0);
  EXPECT_DOUBLE_EQ(turn->linear.y,0);EXPECT_TRUE(h.active());
}
TEST(Geometry, DifficultCrabMotionCanRotateBeforeNarrowPassage)
{
  nav2_costmap_2d::Costmap2D m(250,200,.02,-2,-2,0);auto f=rectangle();
  for(unsigned int x=125;x<250;++x) {
    for(unsigned int y=0;y<200;++y) {if(y<82||y>=118) {m.setCost(x,y,254);}}
  }
  HeadingManeuver h;std::vector<Pose2> path{{0,0,M_PI_4},{1,0,M_PI_4}};
  auto p=project(path,{0,0,M_PI_4});geometry_msgs::msg::Twist speed;
  auto turn=h.compute({0,0,M_PI_4},speed,path,p,m,f,0,1,true);
  ASSERT_TRUE(turn);EXPECT_LT(turn->angular.z,0);EXPECT_NEAR(h.target(),0,.01);
  // After completing the useful turn, do not rotate back into the heading
  // that blocked the narrow passage, even after the cooldown expires.
  auto finished=h.compute({0,0,0},speed,path,p,m,f,5,1,false);
  ASSERT_TRUE(finished);EXPECT_DOUBLE_EQ(finished->angular.z,0);
  EXPECT_FALSE(h.compute({0,0,0},speed,path,p,m,f,8,1,false));
}

TEST(Geometry, ApproachingNarrowPassageRotatesBeforeControllerStalls)
{
  nav2_costmap_2d::Costmap2D m(250,200,.02,-2,-2,0);auto f=rectangle();
  for(unsigned int x=125;x<250;++x) {
    for(unsigned int y=0;y<200;++y) {if(y<82||y>=118) {m.setCost(x,y,254);}}
  }
  HeadingManeuver h;std::vector<Pose2> path{{0,0,M_PI_4},{1,0,M_PI_4}};
  auto p=project(path,{0,0,M_PI_4});geometry_msgs::msg::Twist speed;speed.linear.x=.1;
  auto braking=h.compute({0,0,M_PI_4},speed,path,p,m,f,0,1,false);
  ASSERT_TRUE(braking);EXPECT_TRUE(h.active());EXPECT_NEAR(h.target(),0,.01);
  EXPECT_DOUBLE_EQ(braking->linear.x,0);EXPECT_DOUBLE_EQ(braking->angular.z,0);
  speed.linear.x=0;
  auto turn=h.compute({0,0,M_PI_4},speed,path,p,m,f,.1,1,false);
  ASSERT_TRUE(turn);EXPECT_LT(turn->angular.z,0);
}

TEST(Geometry, OpenCrabRouteDoesNotAddUnnecessaryRotation)
{
  nav2_costmap_2d::Costmap2D m(250,200,.02,-2,-2,0);auto f=rectangle();
  HeadingManeuver h;std::vector<Pose2> path{{0,0,M_PI_4},{1,0,M_PI_4}};
  geometry_msgs::msg::Twist speed;
  EXPECT_FALSE(h.compute({0,0,M_PI_4},speed,path,project(path,{0,0,M_PI_4}),m,f,0,1,false));
}

TEST(Geometry, MandatoryPivotContinuesDuringOptionalHeadingCooldown)
{
  nav2_costmap_2d::Costmap2D m(250,200,.02,-2,-2,0);auto f=rectangle();
  HeadingManeuver h;geometry_msgs::msg::Twist speed;
  std::vector<Pose2> first{{0,0,0},{1,0,M_PI_4}};
  ASSERT_TRUE(h.compute({0,0,0},speed,first,project(first,{0,0,0}),m,f,0,1,false));
  ASSERT_TRUE(h.compute({0,0,M_PI_4},speed,first,project(first,{0,0,M_PI_4}),m,f,4,1,false));
  EXPECT_FALSE(h.active());
  std::vector<Pose2> pivot{{0,0,M_PI_4},{0,0,M_PI_2}};
  auto next=h.compute({0,0,M_PI_4},speed,pivot,project(pivot,{0,0,M_PI_4}),m,f,4.1,1,false,true);
  ASSERT_TRUE(next);EXPECT_GT(next->angular.z,0);EXPECT_TRUE(h.active());
}

TEST(Geometry, FinalAlignmentUsesEndpointBeyondProjectionSearchWindow)
{
  nav2_costmap_2d::Costmap2D m(400,100,.05,-2,-2.5,0);auto f=rectangle();
  std::vector<Pose2> path;
  for(int i=0;i<=100;++i) {path.push_back({i*.1,0,0});}
  path.push_back({10,0,-M_PI_2});
  const Pose2 robot{10,0,0};const auto stale_projection=project(path,robot);
  ASSERT_LT(stale_projection.pose.x,4.);
  HeadingManeuver h;geometry_msgs::msg::Twist speed;
  auto turn=h.compute(robot,speed,path,stale_projection,m,f,0,1,false,true);
  ASSERT_TRUE(turn);EXPECT_LT(turn->angular.z,0);EXPECT_NEAR(h.target(),-M_PI_2,1e-9);
}

TEST(Geometry, PlannedPivotHeadingIsNotAppliedBeforeItsPosition)
{
  nav2_costmap_2d::Costmap2D m(250,200,.02,-2,-2,0);auto f=rectangle();
  std::vector<Pose2> path{{0,0,0},{1,0,0},{1,0,M_PI_2},{1,1,M_PI_2}};
  const auto p=project(path,{0,0,0});
  EXPECT_DOUBLE_EQ(lookahead(path,p,2,true).a,0);
  HeadingManeuver h;geometry_msgs::msg::Twist speed;
  EXPECT_FALSE(h.compute({0,0,0},speed,path,p,m,f,0,1,false));
  path.resize(3);
  auto turn=h.compute({1,0,0},speed,path,project(path,{1,0,0}),m,f,1,1,false,true);
  ASSERT_TRUE(turn);EXPECT_GT(turn->angular.z,0);EXPECT_NEAR(h.target(),M_PI_2,1e-9);
}

TEST(Geometry, GoalConnectorKeepsExactPoseWhenSnappedCornerIsOccupied)
{
  nav2_costmap_2d::Costmap2D m(360,240,.05,-2,-5,0);auto f=rectangle();
  nav2_costmap_2d::padFootprint(f,.04);auto expanded=f;nav2_costmap_2d::padFootprint(expanded,.025);
  for(unsigned int x=0;x<360;++x) {
    for(unsigned int y=0;y<240;++y) {
      double wx,wy;m.mapToWorld(x,y,wx,wy);
      if(wx>=13.6 && wx<14.2 && wy>=4.9 && wy<5.5) {m.setCost(x,y,254);}
    }
  }
  const Pose2 goal{14.349,5.975,-1.03037};
  ASSERT_TRUE(poseFree(goal,m,f));
  EXPECT_FALSE(poseFree({14.325,5.975,-std::atan2(2.,1.)},m,f));
  auto c=goalConnection(goal,m,f,expanded,{-M_PI_2,-std::atan2(2.,1.),-M_PI_4},.25);
  ASSERT_TRUE(c);ASSERT_EQ(c->poses.size(),3u);
  EXPECT_DOUBLE_EQ(c->poses.back().x,goal.x);EXPECT_DOUBLE_EQ(c->poses.back().y,goal.y);
  EXPECT_DOUBLE_EQ(c->poses.back().a,goal.a);EXPECT_TRUE(pathFree(c->poses,m,f));
  EXPECT_DOUBLE_EQ(distance(c->poses[0],c->poses[1]),0);
  EXPECT_DOUBLE_EQ(c->poses[1].a,c->poses[2].a);
  unsigned int x,y;m.worldToMap(goal.x,goal.y,x,y);m.setCost(x,y,254);
  EXPECT_FALSE(goalConnection(goal,m,f,expanded,{-M_PI_2,-std::atan2(2.,1.),-M_PI_4},.25));
}

TEST(Geometry, GoalConnectorChecksIntermediateRotationNotJustEndpoints)
{
  nav2_costmap_2d::Costmap2D m(200,200,.02,-2,-2,0);auto f=rectangle();
  for(unsigned int x=0;x<200;++x) {m.setCost(x,123,254);}
  const Pose2 goal{.01,.01,1.2},anchor{.01,.01,.45};
  ASSERT_TRUE(poseFree(goal,m,f));ASSERT_TRUE(poseFree(anchor,m,f));
  EXPECT_FALSE(goalConnection(goal,m,f,f,{anchor.a},.005));
}
