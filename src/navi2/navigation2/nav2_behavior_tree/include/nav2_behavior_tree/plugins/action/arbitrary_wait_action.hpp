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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARBITRARY_WAIT_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARBITRARY_WAIT_ACTION_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to wait some time
 */
class ArbitraryWait : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::ArbitraryWait constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ArbitraryWait(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>(
        "is_pause_task", "True if pause nav task"),
      BT::InputPort<double>("wait_time", 10.0, "Max wait time"),
      BT::InputPort<double>("max_interval_tick_time", -1.0, "Max interval time for continuous counter"),
    };
  }

private:
  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Time start_;
  rclcpp::Time last_tick_time_;

  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  
  bool is_pause_task_ = false;
  bool first_time_ = false;
  double max_wait_time_ = 1.;
  double wait_time_ = 0.;
  double max_interval_tick_time_ = -1.;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ARBITRARY_WAIT_ACTION_HPP_
