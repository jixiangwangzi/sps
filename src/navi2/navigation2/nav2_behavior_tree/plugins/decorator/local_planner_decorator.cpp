/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: ShiLiang Li
* Created Date: 2022-10-30
*/

#include "nav2_behavior_tree/plugins/decorator/local_planner_decorator.hpp"

namespace nav2_behavior_tree
{

LocalPlannerDecorator::LocalPlannerDecorator(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  start_ = node_->now();
  getInput("try_time", try_time_);
}

BT::NodeStatus LocalPlannerDecorator::tick()
{ 
  setStatus(BT::NodeStatus::RUNNING);

  RCLCPP_INFO_ONCE(node_->get_logger(), "local planner decorator is done");

  const BT::NodeStatus child_state = child_node_->executeTick();

    
  if (count_time_flg_ == false
      && child_state == BT::NodeStatus::FAILURE ) {
    count_time_flg_ = true;
    RCLCPP_INFO(node_->get_logger(), "begin cout timer to replan global plan");
    start_ = node_->now();
  }

  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals_);
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_);
  if (goal_ != current_goal_) {
    current_goal_ = goal_;
    start_ = node_->now();
    return BT::NodeStatus::SUCCESS;
  }else if(goals_ != current_goals_){

    current_goals_ = goals_;
    start_ = node_->now();
    return BT::NodeStatus::SUCCESS;
  }

  auto elapsed = (node_->now() - start_).seconds();
  if (elapsed > try_time_) {
    start_ = node_->now();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000, "Replan global plan is enabled !!!");
    return BT::NodeStatus::FAILURE;
  }

  if (child_state == BT::NodeStatus::FAILURE){
    RCLCPP_INFO(node_->get_logger(), "BTree: Local planner decorator failed!!");
    return BT::NodeStatus::RUNNING;
  }
  else if (child_state == BT::NodeStatus::SUCCESS){
    start_ = node_->now();  
    count_time_flg_ = false;
    return BT::NodeStatus::SUCCESS;
  }
  else {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::LocalPlannerDecorator>("LocalPlannerDecorator");
}

