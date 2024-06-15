// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/arbitrary_wait_action.hpp"

namespace nav2_behavior_tree
{

ArbitraryWait::ArbitraryWait(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  is_pause_task_(true),
  first_time_(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
  last_tick_time_ = node_->now();
  getInput("wait_time", max_wait_time_);
  getInput("max_interval_tick_time", max_interval_tick_time_);
  RCLCPP_INFO(node_->get_logger(),
    "max_wait_time_ : %fs, max_interval_tick_time :%fs.",
    max_wait_time_, max_interval_tick_time_);  
}

inline BT::NodeStatus ArbitraryWait::tick()
{

  // getInput("is_pause_task", is_pause_task_);
  // RCLCPP_ERROR(node_->get_logger(),
  //   "is_pause_task_ : %d.", is_pause_task_);
  // switch (status()) {
  //   case BT::NodeStatus::IDLE:
  //     RCLCPP_WARN(node_->get_logger(),
  //       "BT::NodeStatus::IDLE;");
  //     break;    
  //   case BT::NodeStatus::RUNNING:
  //     // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  //     RCLCPP_WARN(node_->get_logger(),
  //       "BT::NodeStatus::RUNNING;");
  //     break;
  //   case BT::NodeStatus::SUCCESS:
  //     RCLCPP_WARN(node_->get_logger(),
  //       "BT::NodeStatus::SUCCESS;");
  //     break;
  //   case BT::NodeStatus::FAILURE:
  //     RCLCPP_WARN(node_->get_logger(),
  //       "BT::NodeStatus::FAILURE;");
  //     break;
  //   default:
  //     RCLCPP_WARN(node_->get_logger(),
  //       "Default!!!!!;");
  //     break;
  // }

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    first_time_ = true;
    RCLCPP_WARN(node_->get_logger(),
      "get new task, reset time!");
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    first_time_ = true; 
    RCLCPP_WARN(node_->get_logger(),
      "get new task, reset time!");
  }


  auto interval_tick_time = (node_->now() - last_tick_time_).seconds();
  // RCLCPP_WARN(node_->get_logger(),
  //   "interval_tick_time : %fs.", interval_tick_time);

  //Todo: continue to timer when unpause  
  // if (interval_tick_time > 2. && !is_pause_task_) {
  if (max_interval_tick_time_ > 0. && interval_tick_time > max_interval_tick_time_ ) { 
    first_time_ = true;
    RCLCPP_WARN(node_->get_logger(),
      "Rset first_time_!!!!");
  }  

  if (first_time_) {
    start_ = node_->now();
    first_time_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "Rset time!!");
  }
  setStatus(BT::NodeStatus::RUNNING);
  
  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  wait_time_ = elapsed.seconds();
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,  
  // RCLCPP_WARN(node_->get_logger(),
    "waited seconds : %fs, max wait time : %fs.", wait_time_, max_wait_time_);  
  if (wait_time_ < max_wait_time_) {
    last_tick_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }
  first_time_ = true;
  start_ = node_->now();
  // RCLCPP_WARN(node_->get_logger(),
  //   "Rset first_time_!!");
  last_tick_time_ = node_->now();
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ArbitraryWait>("ArbitraryWait");
}
