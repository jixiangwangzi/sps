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

#include "nav2_behavior_tree/plugins/decorator/task_pause_controller.hpp"

using namespace std::chrono_literals; // NOLINT

namespace nav2_behavior_tree
{

TaskPauseController::TaskPauseController(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(condition_name, conf),
  is_pause_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  // Costmap Filter enabling service
  // enable_service_ = node_->create_service<std_srvs::srv::SetBool>(
  //   "/pause_controller",
  //   std::bind(
  //     &TaskPauseController::enableCallback, this,
  //     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  RCLCPP_INFO(node_->get_logger(), "Initialized an TaskPauseController BT node");
}

TaskPauseController::~TaskPauseController()
{
}

void TaskPauseController::enableCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  is_pause_ = request->data;
  response->success = true;
  if (is_pause_) {
    response->message = "Pause";
  } else {
    response->message = "Unpause";
  }
  RCLCPP_WARN(node_->get_logger(),
    "Pause nav task!!"); 
}

BT::NodeStatus TaskPauseController::tick()
{
  getInput("is_pause_task", is_pause_);

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
    // RCLCPP_WARN(node_->get_logger(),
    //   "path_length : %fm, dist_robot_to_goal_ : %fm.", path_length, dist_robot_to_goal_);
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        // RCLCPP_INFO(node_->get_logger(),
        //   "max wait time : %fs, child_state::RUNNING;", wait_time_);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
          "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        // RCLCPP_INFO(node_->get_logger(),
        //   "max wait time : %fs, child_state::SUCCESS;", wait_time_);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
          "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::FAILURE:
        RCLCPP_WARN(node_->get_logger(),
          "BT::NodeStatus::FAILURE;");
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
        //   "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::FAILURE;      
      default:
        RCLCPP_WARN(node_->get_logger(),
          "BT::NodeStatus::FAILURE;");
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
        //   "BT::NodeStatus::RUNNING;");                
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;  // Failed to detected a stuck condition
}



}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TaskPauseController>("TaskPauseController");
}
