// Copyright (c) 2018 Intel Corporation
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
#include <chrono>

#include "nav2_behavior_tree/plugins/condition/task_pause_condition.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

TaskPauseCondition::TaskPauseCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  is_pause_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // Costmap Filter enabling service
  enable_service_ = node_->create_service<std_srvs::srv::SetBool>(
    "/pause_nav_task",
    std::bind(
      &TaskPauseCondition::enableCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_services_default, callback_group_);

  RCLCPP_INFO(node_->get_logger(), "Initialized an TaskPauseCondition BT node");
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  //publish 
  task_pause_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "nav_task_pause_state", custom_qos);

  publish_cycle_ = rclcpp::Duration::from_seconds(1);    
}

TaskPauseCondition::~TaskPauseCondition()
{
}

void TaskPauseCondition::enableCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_ERROR(node_->get_logger(),
      "Pause task server in!!");
  is_pause_ = request->data;
  response->success = true; 
  if (is_pause_) {
    response->message = "Pause";
  } else {
    response->message = "Unpause";
  }
  RCLCPP_ERROR(node_->get_logger(),
    "Pause task server out!!is_pause_ : %d.", is_pause_); 
 
}

void TaskPauseCondition::pubTaksPauseState(const bool & state)
{
  auto current_time = node_->now();
  if ((last_publish_ + publish_cycle_ < current_time) ||
    (current_time < last_publish_)) {
    if (task_pause_state_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Bool tmp_msg;
      tmp_msg.data = state;
      task_pause_state_pub_->publish(std::move(tmp_msg));
    }
    last_publish_ = current_time;
  }

}

BT::NodeStatus TaskPauseCondition::tick()
{
  bool pause_state = false;
  callback_group_executor_.spin_some();
  
  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals; 
    is_pause_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "get new goal, reset pause flag!!");    
  }

  if (is_pause_) {
    pause_state = true;
    pubTaksPauseState(pause_state);
    // RCLCPP_WARN(node_->get_logger(),
    //   "Pause task!!");
    setOutput("is_pause_task", is_pause_);    
    return BT::NodeStatus::SUCCESS;  // Successfully detected a pause condition
  }
  pause_state = false;
  pubTaksPauseState(pause_state);  
  setOutput("is_pause_task", is_pause_);
  return BT::NodeStatus::FAILURE;  // Failed to detected a stuck condition
}



}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TaskPauseCondition>("TaskPauseCondition");
}
