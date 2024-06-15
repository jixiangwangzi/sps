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

#include <string>
#include <memory>
#include <limits>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/action/truncate_realtime_path_action.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace nav2_behavior_tree
{

TruncateRealtimePath::TruncateRealtimePath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_->get_parameter("transform_tolerance", transform_tolerance_);


  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  //publish 
  truncate_realtime_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    "truncate_realtime_path", custom_qos);

}

inline BT::NodeStatus TruncateRealtimePath::tick()
{
  //RCLCPP_INFO(node_->get_logger(),"TruncateRealtimePath tick");

  setStatus(BT::NodeStatus::RUNNING);

  nav_msgs::msg::Path input_path;
  getInput("input_path", input_path);
  nav_msgs::msg::Path output_path = input_path;
  output_path.poses.clear();
  

  if (input_path.poses.empty()) {
    setOutput("output_path", input_path);
    return BT::NodeStatus::FAILURE;
  }

  using namespace nav2_util::geometry_utils;  // NOLINT

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    return BT::NodeStatus::FAILURE;
  }

  try {
    // Find the closest pose to current pose on global path
    auto find_closest_pose_idx =
      [&current_pose, &input_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < input_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            current_pose, input_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };

    for(auto i = find_closest_pose_idx(); i < input_path.poses.size(); i++)
    {
      output_path.poses.push_back(input_path.poses[i]);
    }
    
  } catch (...) {
    // Ignore
  }

  setOutput("output_path", output_path);
  
  pubTruncateRealtimePath(output_path);
  return BT::NodeStatus::SUCCESS;
}

void TruncateRealtimePath::pubTruncateRealtimePath(const nav_msgs::msg::Path& output_path)
{
  if (truncate_realtime_path_pub_->get_subscription_count() < 1) {return;}
  truncate_realtime_path_pub_->publish(output_path);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::TruncateRealtimePath>("TruncateRealtimePath");
}
