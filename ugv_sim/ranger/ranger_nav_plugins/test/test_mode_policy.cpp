// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "../src/mode_policy.hpp"
using namespace ranger_nav_plugins;
TEST(ModePolicy, RadiusAloneCannotAuthorizeLateralDriving)
{
  MotionLimits l;
  std::vector<Pose2> side{{0,0,0},{0,1,0},{0,2,0}};
  auto e=edge(side,0,l);
  EXPECT_TRUE(std::isinf(e.radius));EXPECT_FALSE(e.ackermann);
  auto d=decide(side,l,0);EXPECT_FALSE(d.modes[0]);
  EXPECT_FALSE(std::isfinite(estimate(side,{true,true},l,0).seconds));
}
TEST(ModePolicy, StraightForwardAndReverseAreFeasible)
{
  MotionLimits l;
  for(double direction:{1.,-1.}) {
    std::vector<Pose2> path{{0,0,0},{direction,0,0},{2*direction,0,0}};
    auto d=decide(path,l,0);
    EXPECT_TRUE(edge(path,0,l).ackermann);
    EXPECT_EQ(d.modes[0],direction>0);EXPECT_EQ(d.modes[1],direction>0);
    EXPECT_GT(d.mixed.seconds,2/l.forward);
  }
}
TEST(ModePolicy, TurningRadiusAndInPlaceRotationAreChecked)
{
  MotionLimits l;
  for(double r:{.5,1.1}) {
    std::vector<Pose2> path;
    for(int i=0;i<30;++i) {double a=i*.04;path.push_back({r*std::sin(a),r*(1-std::cos(a)),a});}
    EXPECT_EQ(edge(path,5,l).ackermann,r>.8);
  }
  std::vector<Pose2> pivot{{0,0,0},{0,0,M_PI_2}};
  EXPECT_FALSE(edge(pivot,0,l).ackermann);
  EXPECT_FALSE(std::isfinite(estimate(pivot,{true},l,0).seconds));
}
TEST(ModePolicy, ETAIncludesSteeringRotationAndReversals)
{
  MotionLimits l;
  std::vector<Pose2> straight{{0,0,0},{2,0,0}};
  auto forward=estimate(straight,{true},l,0);
  auto misaligned=estimate(straight,{true},l,M_PI_2);
  EXPECT_GT(misaligned.seconds,forward.seconds+5);
  std::vector<Pose2> reverse{{0,0,0},{1,0,0},{0,0,0}};
  auto e=estimate(reverse,{true,true},l,0);
  EXPECT_EQ(e.reversals,1u);EXPECT_GT(e.seconds,forward.seconds+2);
}
TEST(ModePolicy, ShortAckermannIslandsDoNotCauseChatter)
{
  MotionLimits l;
  std::vector<Pose2> p{{0,0,0},{0,1,0},{.1,1,0},{.2,1,0},{.2,2,0}};
  auto modes=schedule(p,l);
  EXPECT_TRUE(std::none_of(modes.begin(),modes.end(),[](bool v){return v;}));
}
TEST(ModePolicy, HolonomicETAChargesIntermediateBodyRotation)
{
  MotionLimits l;
  std::vector<Pose2> straight{{0,0,0},{1,0,0},{2,0,0}};
  auto rotating=straight;rotating[1].a=M_PI_2;rotating[2].a=M_PI_2;
  const auto base=estimate(straight,{false,false},l,0);
  const auto turn=estimate(rotating,{false,false},l,0);
  EXPECT_GT(turn.seconds,base.seconds+5);
}
TEST(ModePolicy, ShrinkingAnActiveSegmentDoesNotTriggerEarlyHandoff)
{
  MotionLimits l;
  std::vector<Pose2> p{{0,0,0},{.2,0,0},{.4,0,0},{.4,1,0}};
  EXPECT_FALSE(schedule(p,l)[0]);
  EXPECT_TRUE(schedule(p,l,true)[0]);
  EXPECT_FALSE(schedule(p,l,true).back());
}
TEST(ModePolicy, CarReferenceKeepsDirectionSequenceAtACusp)
{
  MotionLimits l;
  std::vector<Pose2> p{{0,0,0},{1,0,0},{.5,0,0}};
  EXPECT_TRUE(strictAckermannReference(p,l));
  auto d=decide(p,l,0);
  EXPECT_TRUE(d.modes[0]);EXPECT_TRUE(d.modes[1]);
  EXPECT_FALSE(std::isfinite(d.holonomic.seconds));
}
TEST(ModePolicy, TerminalCarSectionCanFinishByHolonomicAlignment)
{
  std::vector<Pose2> path{{0,0,0},{1,0,0}};
  EXPECT_TRUE(finalHolonomicApproach(path,project(path,{.9,0,.3}),{.9,0,.3},{}));
  EXPECT_FALSE(finalHolonomicApproach(path,project(path,{.6,0,0}),{.6,0,0},{}));
}
TEST(ModePolicy, NearEndpointCannotSkipUntravelledLoop)
{
  std::vector<Pose2> path{{0,0,0},{2,0,0},{2,2,M_PI_2},{0,.1,M_PI}};
  const Pose2 robot{0,0,0};const Projection p{0,0,0,robot};
  EXPECT_FALSE(finalHolonomicApproach(path,p,robot,{}));
}
TEST(ModePolicy, TerminalHandoffDoesNotSkipPivotOrReversal)
{
  const Pose2 robot{0,0,0};const Projection p{0,0,0,robot};
  EXPECT_FALSE(finalHolonomicApproach({robot,{.1,0,0},{.1,0,M_PI_2}},p,robot,{}));
  EXPECT_FALSE(finalHolonomicApproach({robot,{.1,0,0},{.05,0,0}},p,robot,{}));
}
