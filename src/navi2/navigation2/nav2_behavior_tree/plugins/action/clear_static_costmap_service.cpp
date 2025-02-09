// Copyright (c) 2021 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/action/clear_static_costmap_service.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_behavior_tree
{

ClearStaticCostmapService::ClearStaticCostmapService(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(action_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::ClearStaticCostmap>("clear_static_costmap");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  getInput<double>("forward_dist", forward_dist_);
  getInput<double>("half_width", half_width_);
  getInput<bool>("mark", mark_);
}

BT::NodeStatus ClearStaticCostmapService::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::ClearStaticCostmap::Request>();
  request->mark = mark_;
  request->global_path = path;
  request->half_width = half_width_;
  request->forward_dist = forward_dist_;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->clear) {
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ClearStaticCostmapService>("ClearStaticCostmap");
}
