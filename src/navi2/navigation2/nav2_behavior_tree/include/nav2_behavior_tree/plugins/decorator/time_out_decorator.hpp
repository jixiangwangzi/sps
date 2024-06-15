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
// #include "std_msgs/msg/bool.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class TimeOutDecorator : public BT::DecoratorNode
{
public:
  TimeOutDecorator(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("time_out", 10.0, "keep nomal running time")
    };
  }

private:
    void reset();

private:
  rclcpp::Node::SharedPtr node_;

  double time_out_;
  rclcpp::Time start_;
  bool is_start_;

  int navi_type_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__LOCAL_PLANNER_DECORATOR_HPP_


