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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/bool.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when a specified goal
 * is reached and FAILURE otherwise
 */
class IsGoalReachedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsGoalReachedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsGoalReachedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsGoalReachedCondition() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::IsGoalReachedCondition
   */
  ~IsGoalReachedCondition() override;

  /**
   * @brief Callback function for odom topic
   * @param msg Shared pointer to nav_msgs::msg::Odometry::SharedPtr message
   */
  void goalReachedReceived(const typename std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
    };
  }

protected:
  /**
   * @brief Cleanup function
   */
  void cleanup()
  {}

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;
  
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  bool is_goal_reached_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_GOAL_REACHED_CONDITION_HPP_
