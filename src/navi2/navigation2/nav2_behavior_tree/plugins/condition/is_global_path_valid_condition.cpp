// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/condition/is_global_path_valid_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

IsGlobalPathValidCondition::IsGlobalPathValidCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_global_path_valid");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  //publish 
  path_safety_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "global_path_safety_state", custom_qos);

  publish_cycle_ = rclcpp::Duration::from_seconds(1);  
}

void IsGlobalPathValidCondition::pubPathSafetyState(const bool & state)
{
  auto current_time = node_->now();
  if ((last_publish_ + publish_cycle_ < current_time) ||
    (current_time < last_publish_)) {
    if (path_safety_state_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Bool tmp_msg;
      tmp_msg.data = state;
      path_safety_state_pub_->publish(std::move(tmp_msg));
    }
    last_publish_ = current_time;
  }

}

BT::NodeStatus IsGlobalPathValidCondition::tick()
{
  bool path_safety = true;
  nav_msgs::msg::Path path;
  getInput("path", path);
  double check_dis;
  getInput("check_dis", check_dis);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  request->check_distance = check_dis;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
      pubPathSafetyState(path_safety);
      return BT::NodeStatus::SUCCESS;
    }
  }
  path_safety = false;
  pubPathSafetyState(path_safety);
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGlobalPathValidCondition>("IsGlobalPathValid");
}
