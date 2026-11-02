#include <vector>

#ifdef RANGER_NAV2_JAZZY
#include "behaviortree_cpp/bt_factory.h"
#else
#include "behaviortree_cpp_v3/bt_factory.h"
#endif
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ranger_nav_plugins
{
struct NavigationGoalSnapshot
{
  geometry_msgs::msg::PoseStamped goal;
  std::vector<geometry_msgs::msg::PoseStamped> goals;
};

NavigationGoalSnapshot currentGoal(const BT::Blackboard::Ptr & blackboard)
{
  NavigationGoalSnapshot result;
  (void)blackboard->get("goal", result.goal);
  (void)blackboard->get("goals", result.goals);
  return result;
}

// Record the goal at planning dispatch, before the asynchronous action. A new
// goal arriving during that action must not be mistaken for the planned goal.
// Also record after pruning passed waypoints on an otherwise valid cached path.
class RememberPathGoal : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    config().blackboard->set("ranger_path_goal_snapshot", currentGoal(config().blackboard));
    return BT::NodeStatus::SUCCESS;
  }
};

class PathGoalUnchanged : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    NavigationGoalSnapshot saved;
    if (!config().blackboard->get("ranger_path_goal_snapshot", saved)) {
      return BT::NodeStatus::FAILURE;
    }
    const auto current = currentGoal(config().blackboard);
    return current.goal == saved.goal && current.goals == saved.goals ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// A collision-free path may still be untrackable. Invalidate it after FollowPath
// fails, and at the start of each navigation request (including the same goal).
class ClearNavigationPath : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<nav_msgs::msg::Path>("path")};
  }
  BT::NodeStatus tick() override
  {
    setOutput("path", nav_msgs::msg::Path{});
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace ranger_nav_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ranger_nav_plugins::RememberPathGoal>("RememberPathGoal");
  factory.registerNodeType<ranger_nav_plugins::PathGoalUnchanged>("PathGoalUnchanged");
  factory.registerNodeType<ranger_nav_plugins::ClearNavigationPath>("ClearNavigationPath");
}
