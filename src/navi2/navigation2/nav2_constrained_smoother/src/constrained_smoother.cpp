// Copyright (c) 2021 RoboTech Vision
// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
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

#include <algorithm>
#include <string>
#include <memory>
#include <utility>
#include <vector>

#include "nav2_constrained_smoother/constrained_smoother.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_constrained_smoother
{

void ConstrainedSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  costmap_sub_ = costmap_sub;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();

  smoother_ = std::make_unique<nav2_constrained_smoother::Smoother>();
  optimizer_params_.get(node.get(), name);
  smoother_params_.get(node.get(), name);
  smoother_->initialize(optimizer_params_);

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&ConstrainedSmoother::on_parameter_event_callback, this, std::placeholders::_1));
}

void ConstrainedSmoother::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up smoother: %s of type"
    " nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

void ConstrainedSmoother::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

void ConstrainedSmoother::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating smoother: %s of type "
    "nav2_constrained_smoother::ConstrainedSmoother",
    plugin_name_.c_str());
}

bool ConstrainedSmoother::smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time)
{
  if (path.poses.size() < 2) {
    return true;
  }

  // populate smoother input with (x, y, forward/reverse dir)
  std::vector<Eigen::Vector3d> path_world;
  path_world.reserve(path.poses.size());
  // smoother keeps record of start/end orientations so that it
  // can use them in the final path, preventing degradation of these (often important) values
  Eigen::Vector2d start_dir;
  Eigen::Vector2d end_dir;
  for (size_t i = 0; i < path.poses.size(); i++) {
    auto & pose = path.poses[i].pose;
    double angle = tf2::getYaw(pose.orientation);
    Eigen::Vector2d orientation(cos(angle), sin(angle));
    if (i == path.poses.size() - 1) {
      // Note: `reversing` indicates the direction of the segment after the point and
      // there is no segment after the last point. Most probably the value is irrelevant, but
      // copying it from the last but one point, just to make it defined...
      path_world.emplace_back(pose.position.x, pose.position.y, path_world.back()[2]);
      end_dir = orientation;
    } else {
      auto & pos_next = path.poses[i + 1].pose.position;
      Eigen::Vector2d mvmt(pos_next.x - pose.position.x, pos_next.y - pose.position.y);
      // robot is considered reversing when angle between its orientation and movement direction
      // is more than 90 degrees (i.e. dot product is less than 0)
      bool reversing = smoother_params_.reversing_enabled && orientation.dot(mvmt) < 0;
      // we transform boolean value of "reversing" into sign of movement direction (+1 or -1)
      // to simplify further computations
      path_world.emplace_back(pose.position.x, pose.position.y, reversing ? -1 : 1);
      if (i == 0) {
        start_dir = orientation;
      } else if (i == 1 && !smoother_params_.keep_start_orientation) {
        // overwrite start forward/reverse when orientation was set to be ignored
        // note: start_dir is overwritten inside Smoother::upsampleAndPopulate() method
        path_world[0][2] = path_world.back()[2];
      }
    }
  }

  smoother_params_.max_time = max_time.seconds();

  // Smooth plan
  auto costmap = costmap_sub_->getCostmap();
  if (!smoother_->smooth(path_world, start_dir, end_dir, costmap.get(), smoother_params_)) {
    RCLCPP_WARN(
      logger_,
      "%s: failed to smooth plan, Ceres could not find a usable solution to optimize.",
      plugin_name_.c_str());
    throw new nav2_core::PlannerException(
            "Failed to smooth plan, Ceres could not find a usable solution.");
  }

  // populate final path
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.poses.front().header;
  path.poses.clear();
  path.poses.reserve(path_world.size());
  for (auto & pw : path_world) {
    pose.pose.position.x = pw[0];
    pose.pose.position.y = pw[1];
    pose.pose.orientation.z = sin(pw[2] / 2);
    pose.pose.orientation.w = cos(pw[2] / 2);

    path.poses.push_back(pose);
  }

  return true;
}

void
ConstrainedSmoother::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".minimum_turning_radius") {
        smoother_params_.max_curvature = 1.0f / value.double_value;
      } else if (name == plugin_name_ + ".w_curve") {
        smoother_params_.curvature_weight = value.double_value;
      } else if (name == plugin_name_ + ".w_dist") {
        smoother_params_.distance_weight = value.double_value;
      } else if (name == plugin_name_ + ".w_smooth") {
        smoother_params_.smooth_weight = value.double_value;
      } else if (name == plugin_name_ + ".w_cost") {
        smoother_params_.costmap_weight = value.double_value;
      } else if (name == plugin_name_ + ".w_cost_cusp_multiplier") {
        smoother_params_.cusp_costmap_weight = value.double_value * smoother_params_.costmap_weight;
      } else if (name == plugin_name_ + ".cusp_zone_length") {
        smoother_params_.cusp_zone_length = value.double_value;
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".reversing_enabled") {
        smoother_params_.reversing_enabled = value.bool_value;
      } else if (name == plugin_name_ + ".keep_goal_orientation") {
        smoother_params_.keep_goal_orientation = value.bool_value;
      } else if (name == plugin_name_ + ".keep_start_orientation") {
        smoother_params_.keep_start_orientation = value.bool_value;
      }
    } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".path_downsampling_factor") {
        smoother_params_.path_downsampling_factor = value.integer_value;
      } else if (name == plugin_name_ + ".path_upsampling_factor") {
        smoother_params_.path_upsampling_factor = value.integer_value;
      }
    }
    
  }
}

}  // namespace nav2_constrained_smoother

// Register this smoother as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_constrained_smoother::ConstrainedSmoother,
  nav2_core::Smoother)
