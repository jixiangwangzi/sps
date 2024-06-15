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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TASK_PAUSE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TASK_PAUSE_CONDITION_HPP_

#include <string>
#include <atomic>
#include <deque>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that task pause and returns SUCCESS
 * if task not pause somewhere and FAILURE otherwise
 */
class TaskPauseController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::TaskPauseController
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TaskPauseController(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  TaskPauseController() = delete;

  /**
   * @brief A destructor for nav2_behavior_tree::TaskPauseController
   */
  ~TaskPauseController() override;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return
      {
        BT::InputPort<bool>(
          "is_pause_task", "True if pause nav task"),
      };
  }

protected:
  /**
   * @brief Navigator task enabling/disabling callback
   * @param request_header Service request header
   * @param request Service request
   * @param response Service response
   */
  void enableCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::thread callback_group_executor_thread;

  bool is_pause_;

  /**
   * @brief: A service to enable/disable costmap filter
   */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;

  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__TASK_PAUSE_CONDITION_HPP_
