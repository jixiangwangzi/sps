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
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_behavior_tree/plugins/decorator/is_goal_safety_controller.hpp"

namespace nav2_behavior_tree
{

IsGoalSafetyController::IsGoalSafetyController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  check_dist_threshold_(3.5),
  period_(1.0),
  wait_time_(10.0),
  controller_interval_(0.1),
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
  last_unsafe_wait_time_ = node_->now();

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
 
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  getInput("check_dist_threshold", check_dist_threshold_);
  getInput("seconds", period_);
  getInput("wait_time", wait_time_);
  double controller_frequency;
  getInput("controller_frequency", controller_frequency);

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  //Todo Obtain the corresponding parameters through the node parameter service.
 
  // std::string controller_name = "controller_server";
  // node_->get_parameter(controller_name + "." + "controller_frequency", controller_frequency);
  controller_interval_ = 1. / controller_frequency;
  RCLCPP_INFO(
    node_->get_logger(),
    "controller_interval_ : %.2fs", controller_interval_);

  safety_flags_.set_capacity(3);

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  //publish 
  goal_unsafety_wait_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "goal_unsafety_wait_state", custom_qos);

  publish_cycle_ = rclcpp::Duration::from_seconds(1);  
}
 

bool IsGoalSafetyController::isPoseSafety(const geometry_msgs::msg::PoseStamped & pose)
{
  double check_period = period_;
  if (last_safety_flag_) {
    check_period = controller_interval_;
  }
  // Determine how long its been since we've started this iteration
  auto check_rate = node_->now() - check_start_;

  // Now, get that in seconds
  auto check_rate_seconds = check_rate.seconds();
  // RCLCPP_INFO(node_->get_logger(),
  //   "wait_time_ : %fs, check_rate_seconds : %fs, first_check_goal_ : %d.", wait_time_, check_rate_seconds, first_check_goal_);
  if (check_rate_seconds < check_period || !first_check_goal_) {  
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

bool IsGoalSafetyController::canRestTimer(
  const boost::circular_buffer<bool>& safety_flags)
{
  //Add a delayed exit mechanism to prevent misjudgment caused by moving obstacles.
  auto elapsed = node_->now() - last_unsafe_wait_time_;

  auto seconds = elapsed.seconds();
  if (seconds < 1.0) {
    RCLCPP_WARN(node_->get_logger(),
      "last goal unsafe just pass : %fs.", seconds);
    return false;
  }

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

void IsGoalSafetyController::pubGSafetyWaitState(const bool & state)
{
  auto current_time = node_->now();
  if ((last_publish_ + publish_cycle_ < current_time) ||
    (current_time < last_publish_)) {
    if (goal_unsafety_wait_state_pub_->get_subscription_count() > 0) {
      std_msgs::msg::Bool tmp_msg;
      tmp_msg.data = state;
      goal_unsafety_wait_state_pub_->publish(std::move(tmp_msg));
    }
    last_publish_ = current_time;
  }

}

BT::NodeStatus IsGoalSafetyController::tick()
{
  bool goal_unsafety_wait_state = false;
  // for debug
  // const BT::NodeStatus debug_state = status();

  // switch (debug_state) {
  //   case BT::NodeStatus::IDLE:
  //     RCLCPP_INFO(node_->get_logger(),
  //       "max wait time : %fs, BT::NodeStatus::IDLE!!!", wait_time_);
  //     break;  
  //   case BT::NodeStatus::RUNNING:
  //     RCLCPP_INFO(node_->get_logger(),
  //       "max wait time : %fs, BT::NodeStatus::RUNNING!!!", wait_time_);
  //     break;  
  //   case BT::NodeStatus::SUCCESS:
  //     RCLCPP_INFO(node_->get_logger(),
  //       "max wait time : %fs, BT::NodeStatus::SUCCESS!!!", wait_time_);
  //     break;  
  //   case BT::NodeStatus::FAILURE:
  //     RCLCPP_INFO(node_->get_logger(),
  //       "max wait time : %fs, BT::NodeStatus::FAILURE!!!", wait_time_);
  //     break;  
  //   default:
  //     RCLCPP_INFO(node_->get_logger(),
  //       "max wait time : %fs, BT::NodeStatus::FAILURE!!!", wait_time_);
  //     break;
  // }

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
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
    "dist_robot_to_goal_ : %fm.", dist_robot_to_goal_);

  // Prevent the phenomenon of early judgment when the path is a loop.
  getInput("path", path);
  path_length =
    nav2_util::geometry_utils::calculate_path_length(path, 0); 
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
    "path_length : %fm.", path_length);

  // Determine how long its been since we've started this iteration
  auto elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();
  // RCLCPP_WARN(node_->get_logger(),
  //   "waited seconds : %fs, max wait time : %fs.", seconds, wait_time_);  
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
    "goal unsafety waited seconds : %fs, max wait time : %fs.", seconds, wait_time_);

  last_safety_flag_ =  isPoseSafety(check_pose_); 
  if (!last_safety_flag_ && (path_length < check_dist_threshold_) &&
    (dist_robot_to_goal_ < check_dist_threshold_) && seconds < wait_time_) {      
    // RCLCPP_WARN(node_->get_logger(),
    //   "path_length : %fm, dist_robot_to_goal_ : %fm.", path_length, dist_robot_to_goal_);
    last_unsafe_wait_time_ = node_->now();
    goal_unsafety_wait_state = true;
    pubGSafetyWaitState(goal_unsafety_wait_state);
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        // RCLCPP_INFO(node_->get_logger(),
        //   "max wait time : %fs, child_state::RUNNING;", wait_time_);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
          "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        // RCLCPP_INFO(node_->get_logger(),
        //   "max wait time : %fs, child_state::SUCCESS;", wait_time_);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
          "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::FAILURE:
        RCLCPP_WARN(node_->get_logger(),
          "max wait time : %fs, child_state::FAILURE;", wait_time_);
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
        //   "BT::NodeStatus::RUNNING;");          
        return BT::NodeStatus::FAILURE;      
      default:
        RCLCPP_WARN(node_->get_logger(),
          "max wait time : %fs, child_state::FAILURE;", wait_time_);
        // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
        //   "BT::NodeStatus::RUNNING;");                
        return BT::NodeStatus::FAILURE;
    }
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
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
        "222.Reset start time!!!");
      start_ = node_->now();
    }  
  } else if (path_length > check_dist_threshold_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
      "333.Reset start time!!!");
    start_ = node_->now();
  }
  goal_unsafety_wait_state = false;
  pubGSafetyWaitState(goal_unsafety_wait_state);
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
  factory.registerNodeType<nav2_behavior_tree::IsGoalSafetyController>("IsGoalSafetyController");
}
