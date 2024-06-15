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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_REALTIME_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_REALTIME_PATH_ACTION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/path.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "rclcpp/rclcpp.hpp"
namespace nav2_behavior_tree
{

struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @brief A BT::ActionNodeBase to shorten path by some distance
 */
class TruncateRealtimePath : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncateRealtimePath constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  TruncateRealtimePath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Original Path"),
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Truncate Realtime Path: A Path From Current Pose To Goal"),
      //BT::InputPort<double>("distance", 1.0, "distance"),
    };
  }
private:
  void pubTruncateRealtimePath(const nav_msgs::msg::Path& output_path);

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

  rclcpp::Node::SharedPtr node_;
  std::string robot_base_frame_, global_frame_;
  double transform_tolerance_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr truncate_realtime_path_pub_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__TRUNCATE_REALTIME_PATH_ACTION_HPP_
