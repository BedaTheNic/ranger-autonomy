// Test the deployed XML and native Nav2 control nodes with deterministic action
// doubles. ROS path validation and actual driving are tested in Gazebo separately.
#include <chrono>
#include <fstream>
#include <sstream>
#include <thread>
#include <gtest/gtest.h>
#ifdef RANGER_NAV2_JAZZY
#include "behaviortree_cpp/bt_factory.h"
#else
#include "behaviortree_cpp_v3/bt_factory.h"
#endif
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "../src/recovery_hold.hpp"

using Goals = std::vector<geometry_msgs::msg::PoseStamped>;
using Status = BT::NodeStatus;
struct Scenario
{
  int plans = 0;
  int checks = 0;
  int follows = 0;
  int clears = 0;
  int planner_failures = 0;
  int planner_running_ticks = 0;
  bool valid = true;
  bool validity_unavailable = false;
  bool controller_failure = false;
  bool prune = false;
};

class MockAction : public BT::ActionNodeBase
{
public:
  MockAction(const std::string & name, const BT::NodeConfiguration & config,
    const std::string & kind, Scenario & scenario)
  : BT::ActionNodeBase(name, config), kind_(kind), s_(scenario) {}
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<bool>("valid"), BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<Goals>("goals"), BT::InputPort<Goals>("input_goals"),
      BT::OutputPort<Goals>("output_goals"), BT::InputPort<double>("radius"),
      BT::InputPort<std::string>("planner_id"), BT::InputPort<int>("server_timeout"),
      BT::InputPort<std::string>("controller_id"),
      BT::InputPort<std::string>("goal_checker_id"),
      BT::InputPort<std::string>("service_name"), BT::InputPort<int>("wait_duration")};
  }
  void halt() override
  {
#ifdef RANGER_NAV2_JAZZY
    resetStatus();
#else
    setStatus(Status::IDLE);
#endif
  }
  Status tick() override
  {
    if (kind_ == "ComputePathToPose" || kind_ == "ComputePathThroughPoses") {
      if (status() != Status::RUNNING) {++s_.plans;}
      if (s_.planner_running_ticks > 0) {--s_.planner_running_ticks; return Status::RUNNING;}
      if (s_.planner_failures > 0) {
        --s_.planner_failures;
        setOutput("path", nav_msgs::msg::Path{});
        return Status::FAILURE;
      }
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.header.stamp.sec = s_.plans;
      path.poses.resize(2);
      setOutput("path", path);
      s_.valid = true;
    } else if (kind_ == "MockReadPathValidity") {
      ++s_.checks;
      if(s_.validity_unavailable) {s_.validity_unavailable=false;return Status::FAILURE;}
      setOutput("valid",s_.valid);
      return Status::SUCCESS;
    } else if (kind_ == "FollowPath") {
      ++s_.follows;
      if (s_.controller_failure) {s_.controller_failure = false; return Status::FAILURE;}
      return Status::RUNNING;
    } else if (kind_ == "GoalUpdated") {
      return Status::FAILURE;
    } else if (kind_ == "ClearEntireCostmap") {
      ++s_.clears;
    } else if (kind_ == "RemovePassedGoals") {
      auto goals = getInput<Goals>("input_goals").value();
      if (s_.prune && goals.size() > 1) {goals.erase(goals.begin()); s_.prune = false;}
      setOutput("output_goals", goals);
    }
    return Status::SUCCESS;
  }
private:
  std::string kind_;
  Scenario & s_;
};

class PathPolicy : public testing::TestWithParam<std::string>
{
protected:
  Scenario s;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  BT::Tree tree;
  void SetUp() override
  {
    factory.registerFromPlugin(PATH_CACHE_LIBRARY);
    for (const auto & lib : {"pipeline_sequence", "recovery_node", "rate_controller"}) {
      factory.registerFromPlugin(std::string(NAV2_BT_LIBRARY_DIR) + "/libnav2_" + lib + "_bt_node.so");
    }
    for (const std::string kind : {"ComputePathToPose", "ComputePathThroughPoses",
      "MockReadPathValidity", "FollowPath", "GoalUpdated", "ClearEntireCostmap", "Wait", "RemovePassedGoals"})
    {
      factory.registerBuilder<MockAction>(kind, [this, kind](const std::string & name,
        const BT::NodeConfiguration & config) {
          return std::make_unique<MockAction>(name, config, kind, s);
        });
    }
    std::ifstream file(std::string(PATH_CACHE_XML_DIR) + "/" + GetParam() + ".xml");
    ASSERT_TRUE(file.good());
    std::stringstream xml; xml << file.rdbuf();
    auto text = xml.str();
    text.replace(text.find("<ReadPathValidity "),17,"<MockReadPathValidity ");
#ifdef RANGER_NAV2_JAZZY
    text.replace(text.find("<root "), 6, "<root BTCPP_format=\"4\" ");
#endif
    // Keep native rate control but shorten the interval for unit tests.
    text.replace(text.find("hz=\"1.0\""), 8, "hz=\"1000\"");
    geometry_msgs::msg::PoseStamped goal; goal.pose.position.x = 3.;
    bb->set("goal", goal);
    bb->set("goals", Goals{geometry_msgs::msg::PoseStamped{}, goal});
    tree = factory.createTreeFromText(text, bb);
  }
  void ticks(int count = 5)
  {
    for (int i = 0; i < count; ++i) {
#ifdef RANGER_NAV2_JAZZY
      EXPECT_EQ(tree.tickOnce(), Status::RUNNING);
#else
      EXPECT_EQ(tree.tickRoot(), Status::RUNNING);
#endif
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
};

TEST_P(PathPolicy, RetainsPathWhileControllerKeepsRunning)
{
  ticks(20);
  EXPECT_EQ(s.plans, 1);
  EXPECT_GT(s.checks, 10);
  EXPECT_EQ(s.follows, 20);
}
TEST_P(PathPolicy, ChangedGoalAndBlockedPathEachRequestOneNewPlan)
{
  ticks();
  auto goal = bb->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x += 1.;
  bb->set("goal", goal);
  ticks(); EXPECT_EQ(s.plans, 2);
  s.valid = false;
  ticks(); EXPECT_EQ(s.plans, 3);
}
TEST_P(PathPolicy, OneBlockedCheckHaltsFollowingAndResumesWithoutAReplacement)
{
  ticks();
  const auto before=s.follows;
  s.valid=false;
  ticks(1);
  EXPECT_EQ(s.plans,1);
  EXPECT_EQ(s.follows,before);
  s.valid=true;
  ticks(2);
  EXPECT_GT(s.follows,before);
  EXPECT_EQ(s.plans,1);
}

TEST_P(PathPolicy, ControllerFailureRetriesTheSameCollisionFreePlan)
{
  ticks();
  s.controller_failure = true;
  ticks();
  EXPECT_EQ(s.plans, 1);
  EXPECT_EQ(s.clears, 0);
}

TEST_P(PathPolicy, ActiveLocalEscapeFinishesBeforeBlockedPathIsReplaced)
{
  ticks();
  auto path = bb->get<nav_msgs::msg::Path>("path");
  path.poses.front().header.stamp.sec = 10;
  ranger_nav_plugins::RecoveryHold hold;
  hold.update(path, ranger_nav_plugins::RecoveryHold::Clock::now());
  bb->set("ranger_escape_heartbeat", hold);
  bb->set("ranger_escape_ros_now", 10.0);
  s.valid = false;
  const auto following = s.follows;
  ticks(10);
  EXPECT_EQ(s.plans, 1);
  EXPECT_EQ(s.follows, following + 10);
  // Completion permits the usual two-check obstruction policy again.
  hold.active = false;
  bb->set("ranger_escape_heartbeat", hold);
  ticks();
  EXPECT_EQ(s.plans, 2);
}

TEST_P(PathPolicy, StaleOrDifferentPathEscapeDoesNotSuppressReplanning)
{
  ticks();
  auto path = bb->get<nav_msgs::msg::Path>("path");
  path.poses.front().header.stamp.sec = 9;
  ranger_nav_plugins::RecoveryHold hold;
  hold.update(path, ranger_nav_plugins::RecoveryHold::Clock::now());
  bb->set("ranger_escape_heartbeat", hold);
  bb->set("ranger_escape_ros_now", 10.0);
  s.valid = false;
  ticks();
  EXPECT_EQ(s.plans, 2);
  path.poses.front().header.stamp.sec = 10;
  hold.update(path, ranger_nav_plugins::RecoveryHold::Clock::now());
  bb->set("ranger_escape_heartbeat", hold);
  s.valid = false;
  ticks();
  EXPECT_EQ(s.plans, 3);
}

TEST_P(PathPolicy, NewGoalPreemptsEvenDuringFreshLocalEscape)
{
  ticks();
  auto path = bb->get<nav_msgs::msg::Path>("path");
  path.poses.front().header.stamp.sec = 10;
  ranger_nav_plugins::RecoveryHold hold;
  hold.update(path, ranger_nav_plugins::RecoveryHold::Clock::now());
  bb->set("ranger_escape_heartbeat", hold);
  bb->set("ranger_escape_ros_now", 10.0);
  auto goal = bb->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.x += 1.0;
  bb->set("goal", goal);
  ticks();
  EXPECT_EQ(s.plans, 2);
}

TEST(RecoveryHold, BothSourceAndReceiptExpiryAndTotalDurationAreBounded)
{
  using ranger_nav_plugins::RecoveryHold;
  nav_msgs::msg::Path status;
  status.header.frame_id = "map";
  status.header.stamp.sec = 1;
  status.poses.resize(1);
  status.poses.front().header.stamp.sec = 10;
  const auto now = RecoveryHold::Clock::now();
  RecoveryHold hold;
  hold.update(status, now);
  EXPECT_TRUE(hold.permits(status.header, 10.0, now));
  EXPECT_FALSE(hold.permits(status.header, 10.5, now));
  EXPECT_FALSE(hold.permits(status.header, 9.0, now));
  EXPECT_FALSE(hold.permits(status.header, 10.0, now + std::chrono::seconds(1)));
  // A stream of repeated heartbeats must not extend the hard lease.
  hold.update(status, now + std::chrono::seconds(27));
  EXPECT_FALSE(hold.permits(status.header, 10.0, now + std::chrono::seconds(27)));
  status.poses.clear();
  hold.update(status, now);
  EXPECT_FALSE(hold.permits(status.header, 10.0, now));
}
TEST_P(PathPolicy, GoalChangedDuringPlanningIsNotCachedAsTheOldPlansGoal)
{
  s.planner_running_ticks = 3;
  ticks(1);
  auto goal = bb->get<geometry_msgs::msg::PoseStamped>("goal");
  goal.pose.position.y += 2.;
  bb->set("goal", goal);
  bb->set("goals", Goals{goal});
  ticks(10);
  EXPECT_EQ(s.plans, 2);
}
TEST_P(PathPolicy, NewRequestToSameGoalDoesNotReusePreviousPath)
{
  ticks();
  tree.haltTree();
  ticks(); EXPECT_EQ(s.plans, 2);
}
TEST_P(PathPolicy, FailedInitialPlanRetriesWithoutClearingObservations)
{
  s.planner_failures = 1;
  ticks(); EXPECT_EQ(s.plans, 2); EXPECT_EQ(s.clears, 0);
}
TEST_P(PathPolicy, UnavailableValidityRetriesWithoutReplacingThePath)
{
  ticks();s.validity_unavailable=true;
  ticks(10);EXPECT_EQ(s.plans,1);EXPECT_EQ(s.clears,0);
}
TEST_P(PathPolicy, PruningPassedWaypointsDoesNotReplanButNewWaypointsDo)
{
  if (GetParam() != "navigate_through_poses") {GTEST_SKIP();}
  ticks();
  s.prune = true;
  ticks(); EXPECT_EQ(s.plans, 1); EXPECT_EQ(bb->get<Goals>("goals").size(), 1u);
  auto goals = bb->get<Goals>("goals"); goals.back().pose.position.y += 2.;
  bb->set("goals", goals);
  ticks(); EXPECT_EQ(s.plans, 2);
}
INSTANTIATE_TEST_SUITE_P(NavigationTrees, PathPolicy,
  testing::Values("navigate_to_pose", "navigate_through_poses"));
