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

#include "nav2_behavior_tree/plugins/condition/goal_approach_condition.hpp"

namespace nav2_behavior_tree
{

GoalApproachCondition::GoalApproachCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  getInput("goal_approach_dist", goal_approach_dist_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
}

GoalApproachCondition::~GoalApproachCondition()
{
  cleanup();
}

BT::NodeStatus GoalApproachCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    RCLCPP_WARN(node_->get_logger(),
      "get new goal, goal pose:(%f, %f)!", goal_.pose.position.x, goal_.pose.position.y);
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    goal_ = goals_.back();
    RCLCPP_WARN(node_->get_logger(),
      "get new goals, goal pose:(%f, %f)!", goal_.pose.position.x, goal_.pose.position.y);
  }

  if (isGoalApproach()) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void GoalApproachCondition::initialize()
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  nav2_util::declare_parameter_if_not_declared(
    node_, "transform_tolerance",
    rclcpp::ParameterValue(0.1));
  node_->get_parameter_or<double>("transform_tolerance", transform_tolerance_, 0.1);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  initialized_ = true;
  RCLCPP_INFO(node_->get_logger(), "transform_tolerance : %fs.", transform_tolerance_);
}

bool GoalApproachCondition::isGoalApproach()
{
  geometry_msgs::msg::PoseStamped current_pose;

  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }

  double dx = goal_.pose.position.x - current_pose.pose.position.x;
  double dy = goal_.pose.position.y - current_pose.pose.position.y;

  return (dx * dx + dy * dy) <= (goal_approach_dist_ * goal_approach_dist_);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::GoalApproachCondition>("GoalApproach");
}
