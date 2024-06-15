// Copyright (c) 2022 Neobotix GmbH
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
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/decorator/path_longer_on_approach.hpp"

namespace nav2_behavior_tree
{

PathLongerOnApproach::PathLongerOnApproach(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);
  getInput("detour_len", detour_len_);
  getInput("force_update_seconds", force_update_seconds_);
  getInput("wait_time", max_wait_time_);
  RCLCPP_INFO(node_->get_logger(),
    "detour_len : %fm, force_update_seconds :%fs, max_wait_time :%fs.",
    detour_len_, force_update_seconds_, max_wait_time_);
}

bool PathLongerOnApproach::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  bool path_updated = (new_path != old_path);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "path_updated : %s.", (path_updated ? "true" : "false"));

  return path_updated;
}

bool PathLongerOnApproach::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  double old_path_length =
    nav2_util::geometry_utils::calculate_path_length(old_path, 0);
  bool near_goal = old_path_length < prox_leng;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "near_goal : %s, old_path_length :%.4fm, prox_leng : %.4fm.", (near_goal ? "true" : "false"),
    old_path_length, prox_leng);
  return near_goal;
}

bool PathLongerOnApproach::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  double old_path_length =
    nav2_util::geometry_utils::calculate_path_length(old_path, 0);

  double new_path_length =
    nav2_util::geometry_utils::calculate_path_length(new_path, 0);

  bool new_path_longer = new_path_length > length_factor * old_path_length;

  // bool new_path_longer =
  //   nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "new_path_longer : %s, new_path_length :%.4fm, old_path_length :%.4fm.",
    (new_path_longer ? "true" : "false"), new_path_length, old_path_length);
  return new_path_longer;

  // return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
}

bool PathLongerOnApproach::isNewPathDetour(
  nav_msgs::msg::Path & new_path,
  double & length_factor)
{
  double new_path_length =
    nav2_util::geometry_utils::calculate_path_length(new_path, 0);

  bool new_path_detour = new_path_length > length_factor * detour_len_;

  // bool new_path_longer =
  //   nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "new_path_detour : %s, new_path_length :%.4fm.",
    (new_path_detour ? "true" : "false"), new_path_length);
  return new_path_detour;

  // return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
}

bool PathLongerOnApproach::isPathToGoal(
    nav_msgs::msg::Path & path,
    geometry_msgs::msg::PoseStamped & goal)
{

  // Get euclidean distance
  auto dist_to_goal = nav2_util::geometry_utils::euclidean_distance(
    path.poses.back(), goal);

  bool go_to_goal = dist_to_goal < 1.0;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "go_to_goal : %s, dist_to_goal :%.4fm.",
    (go_to_goal ? "true" : "false"), dist_to_goal);
  return go_to_goal;


}

inline BT::NodeStatus PathLongerOnApproach::tick()
{
  bool detour_wait_state = false;
  getInput("detour_wait_state", detour_wait_state);
  getInput("path", new_path_);

  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting point since we're starting a new iteration of
    // PathLongerOnApproach (moving from IDLE to RUNNING)
    first_time_ = true;
    RCLCPP_WARN(node_->get_logger(), 
      "first_time_ : %s.", (first_time_ ? "true" : "false"));
  }

  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    end_pose_ = goal_;
    start_ = node_->now();
    goal_update_time_ = node_->now();
    RCLCPP_WARN(node_->get_logger(),
      "get new goal!");
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    end_pose_ = goals_.back();
    start_ = node_->now();
    goal_update_time_ = node_->now();
    RCLCPP_WARN(node_->get_logger(),
      "get new goals!");
  }
  // Determine how long its been since goal update
  auto elapsed = node_->now() - goal_update_time_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();
  // double wait_time = 2.0;_
  if (seconds < force_update_seconds_) {
    old_path_ = new_path_;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
      "force update path!");
  }

  // Determine how long its been since we've started this iteration
  auto wait_elapsed = node_->now() - start_;

  // Now, get that in seconds
  auto wait_seconds = wait_elapsed.seconds();
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 2000,
    "detour waited seconds : %fs, max wait time : %fs.", wait_seconds, max_wait_time_);
    
  // RCLCPP_INFO(node_->get_logger(),
  //   "new path size :%lu, old path size : %.lu.", new_path_.poses.size(), old_path_.poses.size());
  
  // Check if the path is updated and valid, compare the old and the new path length,
  // given the goal proximity and check if the new path is longer
  // if (detour_wait_state || (isPathUpdated(new_path_, old_path_) &&
  //   // isRobotInGoalProximity(old_path_, prox_len_) &&
  //   isNewPathLonger(new_path_, old_path_, length_factor_) &&
  //   isNewPathDetour(new_path_, length_factor_) && wait_seconds < max_wait_time_) //&& !first_time_ //&& isPathToGoal(old_path_, end_pose_)
  //   )
  if (detour_wait_state)    
  {
    // new_heartbeat_ = false;
    // setOutput("new_heartbeat", new_heartbeat_); 
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
          "BT::NodeStatus::RUNNING;");
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::SUCCESS:
        // old_path_ = new_path_;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
          "child_state(SUCCESS)-->BT::NodeStatus::RUNNING;");
        return BT::NodeStatus::RUNNING;
      case BT::NodeStatus::FAILURE:
        old_path_ = new_path_;
        RCLCPP_INFO(node_->get_logger(),
          "BT::NodeStatus::FAILURE;");
        return BT::NodeStatus::FAILURE;
      default:
        old_path_ = new_path_;
        RCLCPP_INFO(node_->get_logger(), "BT::NodeStatus::FAILURE;");
        return BT::NodeStatus::FAILURE;
    }
  }
  old_path_ = new_path_;
  first_time_ = false;
  start_ = node_->now();
  // new_heartbeat_ = true;
  // setOutput("new_heartbeat", new_heartbeat_);  
  // RCLCPP_INFO(node_->get_logger(),
  //   "BT::NodeStatus::SUCCESS;");      
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PathLongerOnApproach>("PathLongerOnApproach");
}
