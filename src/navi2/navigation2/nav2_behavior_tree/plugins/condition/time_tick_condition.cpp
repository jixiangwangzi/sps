// Copyright (c) 2022 Joshua Wallace
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

#include "behaviortree_cpp_v3/condition_node.h"

#include "nav2_behavior_tree/plugins/condition/time_tick_condition.hpp"

namespace nav2_behavior_tree
{

TimeTickCondition::TimeTickCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  period_(1.0),
  first_time_(true)
{
  getInput("seconds", period_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus TimeTickCondition::tick()
{
  if (first_time_) {
    first_time_ = false;
    start_ = node_->now();
    RCLCPP_WARN(node_->get_logger(),
      "first time !!! NodeStatus::SUCCESS;");     
    return BT::NodeStatus::SUCCESS;
  }

  // Reset timer if the path has been updated
  // if (prev_path_ != path) {
  //   prev_path_ = path;
  //   start_ = node_->now();
  // }

  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();

  if (seconds < period_) {
    // RCLCPP_INFO(node_->get_logger(),
    //   "seconds : %fs, NodeStatus::RUNNING;", seconds);    
    return BT::NodeStatus::RUNNING;
  }

  first_time_ = true;
  start_ = node_->now();  // Reset the timer
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeTickCondition>("TimeTick");
}
