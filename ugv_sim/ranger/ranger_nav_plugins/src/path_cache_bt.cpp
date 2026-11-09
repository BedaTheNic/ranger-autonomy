#include <vector>
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

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
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if(getInput("path",path) && path.poses.empty()) {return BT::NodeStatus::FAILURE;}
    NavigationGoalSnapshot saved;
    if (!config().blackboard->get("ranger_path_goal_snapshot", saved)) {
      return BT::NodeStatus::FAILURE;
    }
    const auto current = currentGoal(config().blackboard);
    return current.goal == saved.goal && current.goals == saved.goals ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

// A service timeout or missing transform interrupts following and retries
// validation. Only a confirmed collision authorizes replacing a retained path.
class ReadPathValidity : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::IsPathValid>
{
public:
  ReadPathValidity(const std::string & name,const BT::NodeConfiguration & config)
  : BtServiceNode(name,config,"is_retained_path_valid") {}
  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<nav_msgs::msg::Path>("path"),BT::OutputPort<bool>("valid")});
  }
  void on_tick() override {getInput("path",request_->path);}
  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response) override {
    if(!response->is_valid && response->invalid_pose_indices.empty()) {return BT::NodeStatus::FAILURE;}
    setOutput("valid",response->is_valid);
    return BT::NodeStatus::SUCCESS;
  }
};

// Legacy condition for trees that explicitly request a single raw check.
class CheckedPathClear : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<bool>("valid")};}
  BT::NodeStatus tick() override {
    return getInput<bool>("valid").value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

struct ObstructionCheck
{
  std_msgs::msg::Header path{};
  bool holding{false},confirmed{false};
};

// A single scan can move a rasterized wall endpoint by one cell. Suspend the
// follower immediately, then let the next rate-limited check decide whether
// to resume this path or replace it. This also preserves the obstacle layer.
class KeepCheckedPath : public BT::ConditionNode
{
public:
  using BT::ConditionNode::ConditionNode;
  static BT::PortsList providedPorts()
  {return {BT::InputPort<bool>("valid"), BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override
  {
    const auto bb=config().blackboard;
    ObstructionCheck check;
    bb->get("ranger_obstruction",check);
    if (getInput<bool>("valid").value()) {
      if (check.holding) {
        RCLCPP_INFO(rclcpp::get_logger("retained_path_policy"),"Obstruction cleared; resuming retained path");
        bb->set("ranger_obstruction",ObstructionCheck{});
      }
      return BT::NodeStatus::SUCCESS;
    }
    const auto header=getInput<nav_msgs::msg::Path>("path").value().header;
    const auto stamp=header.stamp;
    if (!check.holding || check.path!=header) {
      bb->set("ranger_obstruction",ObstructionCheck{header,true,false});
      RCLCPP_INFO(rclcpp::get_logger("retained_path_policy"),"Holding retained path %d.%09u on first obstruction",
        stamp.sec,stamp.nanosec);
      return BT::NodeStatus::SUCCESS;
    }
    if (!check.confirmed) {
      RCLCPP_INFO(rclcpp::get_logger("retained_path_policy"),"Replacing retained path %d.%09u after two blocked checks",
        stamp.sec,stamp.nanosec);
      check.confirmed=true;bb->set("ranger_obstruction",check);
    }
    return BT::NodeStatus::FAILURE;
  }
};

class FollowCheckedPath : public BT::DecoratorNode
{
public:
  using BT::DecoratorNode::DecoratorNode;
  static BT::PortsList providedPorts() {return {BT::InputPort<nav_msgs::msg::Path>("path")};}
  BT::NodeStatus tick() override
  {
    const auto bb=config().blackboard;
    ObstructionCheck check;
    bb->get("ranger_obstruction",check);
    if (check.holding) {
      if (getInput<nav_msgs::msg::Path>("path").value().header==check.path) {
        resetChild();
        return BT::NodeStatus::RUNNING;
      }
      bb->set("ranger_obstruction",ObstructionCheck{});
    }
    return child_node_->executeTick();
  }
};

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
    config().blackboard->set("ranger_obstruction",ObstructionCheck{});
    setOutput("path", nav_msgs::msg::Path{});
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace ranger_nav_plugins

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ranger_nav_plugins::RememberPathGoal>("RememberPathGoal");
  factory.registerNodeType<ranger_nav_plugins::PathGoalUnchanged>("PathGoalUnchanged");
  factory.registerNodeType<ranger_nav_plugins::ReadPathValidity>("ReadPathValidity");
  factory.registerNodeType<ranger_nav_plugins::CheckedPathClear>("CheckedPathClear");
  factory.registerNodeType<ranger_nav_plugins::KeepCheckedPath>("KeepCheckedPath");
  factory.registerNodeType<ranger_nav_plugins::FollowCheckedPath>("FollowCheckedPath");
  factory.registerNodeType<ranger_nav_plugins::ClearNavigationPath>("ClearNavigationPath");
}
