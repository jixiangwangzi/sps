/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: zhao li
* Created Date: 2022-12-15
*/

#include "nav2_behavior_tree/plugins/decorator/time_out_decorator.hpp"

namespace nav2_behavior_tree
{

TimeOutDecorator::TimeOutDecorator(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
  getInput("time_out", time_out_);

  is_start_ = false;
  navi_type_ = 0;
}

void TimeOutDecorator::reset()
{
  start_ = node_->now();
  is_start_ = false;
}

BT::NodeStatus TimeOutDecorator::tick()
{
  RCLCPP_INFO_ONCE(node_->get_logger(), "timeout decorator is done");

  if(status() == BT::NodeStatus::IDLE)
  {
    reset();
  }

  setStatus(BT::NodeStatus::RUNNING);

  config().blackboard->get<int>("navi_type", navi_type_);

  if(!is_start_)
  {
    is_start_ = true;
    start_ = node_->now();
  }

  const BT::NodeStatus child_state = child_node_->executeTick();

  if(navi_type_ == 1)
  {
    auto elapsed = (node_->now() - start_).seconds();

    if (elapsed > time_out_)
    {
      RCLCPP_INFO(node_->get_logger(), "timeout decorator running time is over time!!!");
      reset();

      return BT::NodeStatus::FAILURE;
    }
  }else{
    reset();
  }

  if(child_state == BT::NodeStatus::FAILURE)
  {
    RCLCPP_INFO(node_->get_logger(), "timeout decorator is failure");
    reset();

    return BT::NodeStatus::FAILURE;
  }
  else if (child_state == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "timeout decorator is success");
    reset();

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TimeOutDecorator>("TimeOutDecorator");
}

