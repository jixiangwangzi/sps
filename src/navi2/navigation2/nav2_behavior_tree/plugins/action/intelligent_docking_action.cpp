// Copyright (c) 2021 RoboTech Vision
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

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/intelligent_docking_action.hpp"

namespace nav2_behavior_tree
{

IntelligentDocking::IntelligentDocking(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
  getInput("wait_time", wait_time_);
  getInput("wait_dist", wait_dist_);
}

inline BT::NodeStatus IntelligentDocking::tick()
{
  getInput("input_path", global_path_);

  if (status() == BT::NodeStatus::IDLE) {
    start_ = node_->now();
  }

  setStatus(BT::NodeStatus::RUNNING);
  geometry_msgs::msg::PoseStamped pose;

  if (!nav2_util::getCurrentPose(
      pose, *tf_buffer_) ) {
    return BT::NodeStatus::FAILURE; 
  } 
  
  unsigned int closest_point_index = 0;
  float current_distance = std::numeric_limits<float>::max();
  float closest_distance = current_distance;
  geometry_msgs::msg::Point current_point = pose.pose.position;
  for (unsigned int i = 0; i < global_path_.poses.size(); ++i) {
    geometry_msgs::msg::Point path_point = global_path_.poses[i].pose.position;

    current_distance = nav2_util::geometry_utils::euclidean_distance(
      current_point,
      path_point);

    if (current_distance < closest_distance) {
      closest_point_index = i;
      closest_distance = current_distance;
    }
  }

  double length = 
    nav2_util::geometry_utils::calculate_path_length(global_path_, closest_point_index);
  
  
  if (length > wait_dist_) {
    return BT::NodeStatus::FAILURE;
  }
  double time_duration = (node_->now() - start_).seconds();
  if (time_duration > wait_time_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), 
      *(node_->get_clock()),
      1000, 
      "Intelligent docking is waiting");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING; 
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::IntelligentDocking>(
    "IntelligentDocking");
}
