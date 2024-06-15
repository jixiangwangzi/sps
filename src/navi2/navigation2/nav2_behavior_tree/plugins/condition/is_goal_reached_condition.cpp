// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

#include "nav2_util/robot_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/node_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/is_goal_reached_condition.hpp"

namespace nav2_behavior_tree
{

IsGoalReachedCondition::IsGoalReachedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_goal_reached_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  callback_group_executor_thread = std::thread([this]() {callback_group_executor_.spin();});

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  goal_reached_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "goal_reached",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsGoalReachedCondition::goalReachedReceived, this, std::placeholders::_1),
    sub_option);

  RCLCPP_INFO(node_->get_logger(), "Initialized an IsGoalReachedCondition BT node");

}

IsGoalReachedCondition::~IsGoalReachedCondition()
{
  RCLCPP_DEBUG(node_->get_logger(), "Shutting down IsGoalReachedCondition BT node");
  callback_group_executor_.cancel();
  callback_group_executor_thread.join();
}

void IsGoalReachedCondition::goalReachedReceived(
  const typename std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Got odometry");

  is_goal_reached_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "is_goal_reached_ : %d.", is_goal_reached_);
}

BT::NodeStatus IsGoalReachedCondition::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);
  
  if (goal_ != current_goal) {
    goal_ = current_goal;
    is_goal_reached_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "get new goal!");
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    is_goal_reached_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "get new goals!");
  }

  if (is_goal_reached_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGoalReachedCondition>("IsGoalReached");
}
