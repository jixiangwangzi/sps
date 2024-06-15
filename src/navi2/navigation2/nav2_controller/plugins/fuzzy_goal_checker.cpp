/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <memory>
#include <string>
#include <limits>
#include <vector>
#include "nav2_controller/plugins/fuzzy_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

#include "nav2_costmap_2d/cost_values.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

FuzzyGoalChecker::FuzzyGoalChecker()
: xy_goal_tolerance_(0.25),
  yaw_goal_tolerance_(0.25),
  stateful_(true),
  check_xy_(true),
  check_fuzzy_(true),
  xy_goal_tolerance_sq_(0.0625),
  fuzzy_stop_max_tolerance_(0.5),
  unsafety_value_(127),
  costmap_model_(nullptr)
  // gk_loader_("nav2_core", "nav2_core::GoalChecker"),
  // primary_goal_checker_(nullptr)
{
}

void FuzzyGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  // std::string primary_goal_checker;
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".xy_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".stateful", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".fuzzy_stop_max_tolerance", rclcpp::ParameterValue(1.0));  
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".unsafety_value", rclcpp::ParameterValue(127));
  // nav2_util::declare_parameter_if_not_declared(
  //   node, plugin_name_ + ".primary_goal_checker", rclcpp::PARAMETER_STRING);        

  node->get_parameter(plugin_name + ".xy_goal_tolerance", xy_goal_tolerance_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".fuzzy_stop_max_tolerance", fuzzy_stop_max_tolerance_);
  node->get_parameter(plugin_name + ".unsafety_value", unsafety_value_);
  node->get_parameter(plugin_name + ".stateful", stateful_);

  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  fuzzy_stop_max_tolerance_sq_ = fuzzy_stop_max_tolerance_ * fuzzy_stop_max_tolerance_;
  // primary_goal_checker = node->get_parameter(plugin_name_ + ".primary_goal_checker").as_string();

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  footprint_spec_ = costmap_ros_->getRobotFootprint();

  costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
  std::string costmap_model_name("fuzzy_goal_checker/costmap_model");
  costmap_model_->initialize(node, costmap_model_name, plugin_name_, costmap_ros_);

  // try {
  //   primary_goal_checker_ = gk_loader_.createUniqueInstance(primary_goal_checker);
  //   RCLCPP_INFO(
  //     logger_, "Created internal goal checker for fuzzy goal checker: %s of type %s",
  //     plugin_name_.c_str(), primary_goal_checker.c_str());
  //   primary_goal_checker_->initialize(node, plugin_name_, costmap_ros_);  
  // } catch (const pluginlib::PluginlibException & ex) {
  //   RCLCPP_FATAL(
  //     logger_,
  //     "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
  //   return;
  // }


  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&FuzzyGoalChecker::dynamicParametersCallback, this, _1));
  RCLCPP_INFO(
    logger_, "xy_goal_tolerance_sq_ : %.2f, fuzzy_stop_max_tolerance: %.2f , unsafety_value : %.2d",
    xy_goal_tolerance_sq_, fuzzy_stop_max_tolerance_, unsafety_value_);

}

void FuzzyGoalChecker::reset()
{
  check_xy_ = true;
  check_fuzzy_ = true;
}

bool FuzzyGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  if (check_fuzzy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    double dist_sq =  dx * dx + dy * dy;
    // RCLCPP_WARN(logger_, "dist_sq : %f, fuzzy_stop_max_tolerance_sq_ : %f.", dist_sq, fuzzy_stop_max_tolerance_sq_); 
    if (dist_sq > fuzzy_stop_max_tolerance_sq_) {
      return false;
    }

    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_fuzzy_ = false;
    }

    if (!isPoseValid(goal_pose, footprint_spec_)) {
      RCLCPP_WARN(logger_, "Goal unsafety, Stop in place!");
      return true;
    }
  }

  if (check_xy_) {
    double dx = query_pose.position.x - goal_pose.position.x,
      dy = query_pose.position.y - goal_pose.position.y;
    if (dx * dx + dy * dy > xy_goal_tolerance_sq_) {
      return false;
    }
    // We are within the window
    // If we are stateful, change the state.
    if (stateful_) {
      check_xy_ = false;
    }
  }
  double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation),
    tf2::getYaw(goal_pose.orientation));
  return fabs(dyaw) < yaw_goal_tolerance_;
  // bool ret = primary_goal_checker_->isGoalReached(query_pose, goal_pose, velocity);
  // RCLCPP_INFO(logger_, "ret %d", ret); 
  // return ret;
}

bool FuzzyGoalChecker::isPoseValid(const geometry_msgs::msg::Pose & pose)
{

  unsigned int mx = 0;
  unsigned int my = 0;
  costmap_->worldToMap(pose.position.x, pose.position.y, mx, my);
  int cost = static_cast<int>(costmap_->getCost(mx, my));
  RCLCPP_INFO(logger_, "cost : %d, unsafety_value_ : %d", cost, unsafety_value_);  
  if (cost > unsafety_value_)
  {
    RCLCPP_WARN(logger_, "cost : %d, unsafety_value_ : %d", cost, unsafety_value_); 
    return false;
  }
  return true;
}

bool FuzzyGoalChecker::isPoseValid(const geometry_msgs::msg::Pose & pose,
  const std::vector<geometry_msgs::msg::Point>& footprint_spec) const
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf2::getYaw(pose.orientation);
  try {
    if ( costmap_model_->scorePose(pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec)) < 0 ) {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

bool FuzzyGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  double invalid_field = std::numeric_limits<double>::lowest();

  // populate the poses
  // bool rtn = primary_goal_checker_->getTolerances(pose_tolerance, vel_tolerance);

  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(yaw_goal_tolerance_);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  return true;
  // return true && rtn;
}

rcl_interfaces::msg::SetParametersResult
FuzzyGoalChecker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  for (auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".xy_goal_tolerance") {
        xy_goal_tolerance_ = parameter.as_double();
        xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
      } else if (name == plugin_name_ + ".yaw_goal_tolerance") {
        yaw_goal_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".fuzzy_stop_max_tolerance") {
        fuzzy_stop_max_tolerance_ = parameter.as_double();
        fuzzy_stop_max_tolerance_sq_ = fuzzy_stop_max_tolerance_ * fuzzy_stop_max_tolerance_;

      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".stateful") {
        stateful_ = parameter.as_bool();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == plugin_name_ + ".unsafety_value") {
        unsafety_value_ = parameter.as_int();
      }
    }
  }
  result.successful = true;
  return result;
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::FuzzyGoalChecker, nav2_core::GoalChecker)
