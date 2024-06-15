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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_behavior_tree/plugins/condition/is_goal_safety_condition.hpp"

namespace nav2_behavior_tree
{

IsGoalSafetyCondition::IsGoalSafetyCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  check_dist_threshold_(3.5),
  period_(1.0),
  wait_time_(10.0),
  last_safety_flag_(true),
  first_check_goal_(false),
  consequent_flag_(false),
  transform_tolerance_(0.1),
  global_frame_("map"),
  robot_base_frame_("base_link")
{

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPoseSafety>("is_pose_safety");
  start_ = node_->now();
  check_start_ = node_->now();

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  getInput("check_dist_threshold", check_dist_threshold_);
  getInput("seconds", period_);
  getInput("wait_time", wait_time_);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  safety_flags_.set_capacity(3);
}
 
bool IsGoalSafetyCondition::isPoseSafety(const geometry_msgs::msg::PoseStamped & pose)
{
  // Determine how long its been since we've started this iteration
  auto check_rate = node_->now() - check_start_;

  // Now, get that in seconds
  auto check_rate_seconds = check_rate.seconds();
  // RCLCPP_INFO(node_->get_logger(),
  //   "wait_time_ : %fs, check_rate_seconds : %fs, first_check_goal_ : %d.", wait_time_, check_rate_seconds, first_check_goal_);
  if (check_rate_seconds < period_ || !first_check_goal_) {  
    first_check_goal_ = true;  
    // RCLCPP_WARN(node_->get_logger(),
    //   "Return !! max wait time : %fs, check_rate_seconds : %fs", wait_time_, check_rate_seconds);
    return last_safety_flag_;
  }
  check_start_ = node_->now();
  // RCLCPP_WARN(node_->get_logger(),
  //   "check pose:(%f, %f)!", pose.pose.position.x, pose.pose.position.y);
  auto request = std::make_shared<nav2_msgs::srv::IsPoseSafety::Request>();

  request->pose = pose;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_safety) {
      // RCLCPP_WARN(node_->get_logger(),
      //   "goal safety!!!!!!");
      // RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, 
      //   "goal safety!!!!!!");  
      safety_flags_.push_back(true);  
      return true;
    }
  }
  // RCLCPP_WARN(node_->get_logger(),
  //   "goal unsafety!!!!!!!!!!!!!!!!!");
  // RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, 
  //   "goal unsafety!!!!!!!!!!!!!!!!!");  
  safety_flags_.push_back(false);     
  return false;
}

bool IsGoalSafetyCondition::canRestTimer(
  const boost::circular_buffer<bool>& safety_flags)
{
  if (safety_flags.size() != 3) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
      "safety_flags size : %zu.", safety_flags.size());
    return false;
  }

  for (auto const & flag : safety_flags) {
    if (flag == false) {
      return false;
    }
  }
  return true;
}

BT::NodeStatus IsGoalSafetyCondition::tick()
{
  nav_msgs::msg::Path path;
  double path_length = 0.;

  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    check_pose_ = goal_;
    start_ = node_->now();
    first_check_goal_ = false;
    consequent_flag_ = false;
    safety_flags_.clear();
    RCLCPP_WARN(node_->get_logger(),
      "get new goal, goal pose:(%f, %f)!", check_pose_.pose.position.x, check_pose_.pose.position.y);
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    check_pose_ = goals_.back();
    start_ = node_->now(); 
    first_check_goal_ = false;  
    consequent_flag_ = false;
    safety_flags_.clear();
    RCLCPP_WARN(node_->get_logger(),
      "get new goals, goal pose:(%f, %f)!", check_pose_.pose.position.x, check_pose_.pose.position.y);
  }
  // Determine distance between robot and goal
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::SUCCESS;
  }

  // Get euclidean distance
  auto dist_robot_to_goal_ = nav2_util::geometry_utils::euclidean_distance(
    check_pose_.pose, current_pose.pose);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
    "dist_robot_to_goal_ : %fm.", dist_robot_to_goal_);

  // Prevent the phenomenon of early judgment when the path is a loop.
  getInput("path", path);
  path_length =
    nav2_util::geometry_utils::calculate_path_length(path, 0); 
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
    "path_length : %fm.", path_length);

  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();
  // RCLCPP_INFO(node_->get_logger(),
  //   "waited time : %fs.", seconds);  
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
    "waited seconds : %fs, max wait time : %fs.", seconds, wait_time_);

  last_safety_flag_ =  isPoseSafety(check_pose_); 

  if (!last_safety_flag_ && (path_length < check_dist_threshold_) &&
    (dist_robot_to_goal_ < check_dist_threshold_) && seconds < wait_time_) { 
    return BT::NodeStatus::FAILURE;     
  }
  
  // first_time_ = false;
  if (last_safety_flag_ !=  isPoseSafety(check_pose_)) {
    if (wait_time_ < 4 || canRestTimer(safety_flags_)) {
      RCLCPP_WARN(node_->get_logger(),
        "1111.Reset start time!!!");
      start_ = node_->now();
    }
  } else if (last_safety_flag_) {
    if (wait_time_ < 4 || canRestTimer(safety_flags_)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
        "222.Reset start time!!!");
      start_ = node_->now();
    }  
  } else if (path_length > check_dist_threshold_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
      "333.Reset start time!!!");
    start_ = node_->now();
  }
  // RCLCPP_INFO(node_->get_logger(),
  //   "max wait time : %fs, BT::NodeStatus::SUCCESS;", wait_time_);
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
  //   "BT::NodeStatus::SUCCESS;");
  return BT::NodeStatus::SUCCESS;

}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsGoalSafetyCondition>("IsGoalSafetyCondition");
}
