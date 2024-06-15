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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_msgs/srv/is_pose_safety.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child if the pose unsafety
 */
class IsGoalSafetyController : public BT::DecoratorNode
{
public:
  // typedef geometry_msgs::msg::PoseStamped Goal;
  // typedef std::vector<geometry_msgs::msg::PoseStamped> Goals;
  /**
   * @brief A constructor for nav2_behavior_tree::IsGoalSafetyController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalSafetyController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      // BT::InputPort<Goal>("input_goal", "Original goal"),
      // BT::InputPort<Goals>("input_goals", "Original goals"),
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "check_dist_threshold", 3.5,
        "Threshold for remaining global path length"),
      BT::InputPort<double>("seconds", 1.0, "The interval time to check the point"),
      BT::InputPort<double>("wait_time", 10.0, "Max wait time"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::InputPort<double>("controller_frequency", 10.0, "Max controller frequency"),     
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  bool isPoseSafety(const geometry_msgs::msg::PoseStamped & pose);
  bool canRestTimer(const boost::circular_buffer<bool>& safety_flags);
  void pubGSafetyWaitState(const bool & state);
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_unsafety_wait_state_pub_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  
  // bool first_time_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;

  rclcpp::Client<nav2_msgs::srv::IsPoseSafety>::SharedPtr client_;
  // The timeout value while waiting for a responce from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
  double check_dist_threshold_;
  geometry_msgs::msg::PoseStamped check_pose_;

  rclcpp::Time start_;
  rclcpp::Time check_start_;
  rclcpp::Time last_unsafe_wait_time_;
  double period_;
  double wait_time_;
  // double controller_frequency_;
  double controller_interval_;
  bool last_safety_flag_;
  bool first_check_goal_;
  bool consequent_flag_;
  boost::circular_buffer<bool> safety_flags_;

  double transform_tolerance_;
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__GOAL_UPDATED_CONTROLLER_HPP_
