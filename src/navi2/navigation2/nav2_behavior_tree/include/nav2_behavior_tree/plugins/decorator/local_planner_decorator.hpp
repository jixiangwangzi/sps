/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: ShiLiang Li
* Created Date: 2022-10-30
*/

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__LOCAL_PLANNER_DECORATOR_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__LOCAL_PLANNER_DECORATOR_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class LocalPlannerDecorator : public BT::DecoratorNode
{
public:
  LocalPlannerDecorator(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("try_time", 10.0, "keep try local planner time")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  
  bool count_time_flg_ = false;
  double try_time_;
  rclcpp::Time start_;
  geometry_msgs::msg::PoseStamped current_goal_;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> current_goals_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__LOCAL_PLANNER_DECORATOR_HPP_


