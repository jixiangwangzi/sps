// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2018 Intel Corporation
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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/action/local_path_plan_action.hpp"

namespace nav2_behavior_tree
{

LocalPathPlanAction::LocalPathPlanAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::LocalPathPlan>(xml_tag_name, action_name, conf)
{
}

void LocalPathPlanAction::on_tick()
{
  getInput("truncate_distance", goal_.truncate_distance);
  getInput("global_path", goal_.global_path);
  getInput("near_to_goal_length", goal_.near_to_goal_length);
  
}

BT::NodeStatus LocalPathPlanAction::on_success()
{
  setOutput("result_path", result_.result->result_path);
  setOutput("planning_time", rclcpp::Duration(result_.result->planning_time).seconds());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::LocalPathPlanAction>(
        name, "local_path_plan", config);
    };

  factory.registerBuilder<nav2_behavior_tree::LocalPathPlanAction>(
    "LocalPathPlan", builder);
}
