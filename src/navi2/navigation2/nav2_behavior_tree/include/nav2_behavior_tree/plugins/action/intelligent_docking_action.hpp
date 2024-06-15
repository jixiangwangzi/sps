/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: ShiLiang Li
* Created Date: 2022-11-17
*/
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTELLIGENT_DOCKING_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTELLIGENT_DOCKING_ACTION_HPP_

#include <memory>
#include <string>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_util/geometry_utils.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "tf2_ros/buffer.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNodeBase to shorten path to some distance around robot
 */
class IntelligentDocking : public BT::ActionNodeBase
{
public:
  /**
   * @brief A nav2_behavior_tree::TruncatePathLocal constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IntelligentDocking(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "wait_time",
        "Wait time before reach goal"),
      BT::InputPort<double>(
        "wait_dist",
        "Wait dist before reach goal"),
			BT::InputPort<nav_msgs::msg::Path>(
				"input_path", "Inpute Path")
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

	rclcpp::Node::SharedPtr node_;
	nav_msgs::msg::Path global_path_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  double wait_time_;
  double wait_dist_;
  rclcpp::Time start_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__INTELLIGENT_DOCKING_ACTION_HPP_
