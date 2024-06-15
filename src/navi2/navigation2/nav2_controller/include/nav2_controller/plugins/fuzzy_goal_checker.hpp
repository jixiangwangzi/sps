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

#ifndef NAV2_CONTROLLER__PLUGINS__FUZZY_GOAL_CHECKER_HPP_
#define NAV2_CONTROLLER__PLUGINS__FUZZY_GOAL_CHECKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/goal_checker.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "dwb_critics/obstacle_footprint.hpp"

namespace nav2_controller
{

/**
 * @class FuzzyGoalChecker
 * @brief Goal Checker plugin that only checks the position difference
 *
 * This class can be stateful if the stateful parameter is set to true (which it is by default).
 * This means that the goal checker will not check if the xy position matches again once it is found to be true.
 */
class FuzzyGoalChecker : public nav2_core::GoalChecker
{
public:
  FuzzyGoalChecker();
  // ~FuzzyGoalChecker();
  // Standard GoalChecker Interface
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  /**
    * @brief Check pose safety
    * @param pose Shared pointer to nav2_msgs::msg::Pose
    */
  bool isPoseValid(const geometry_msgs::msg::Pose & pose);
  bool isPoseValid(const geometry_msgs::msg::Pose & pose, 
    const std::vector<geometry_msgs::msg::Point>& footprint_spec) const;

protected:
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  bool stateful_, check_xy_, check_fuzzy_;
  // Cached squared xy_goal_tolerance_
  double xy_goal_tolerance_sq_;
  double fuzzy_stop_max_tolerance_, fuzzy_stop_max_tolerance_sq_;
  int unsafety_value_;
  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  rclcpp::Logger logger_{rclcpp::get_logger("FuzzyGoalChecker")};

  // pluginlib::ClassLoader<nav2_core::GoalChecker> gk_loader_;
  // nav2_core::GoalChecker::Ptr primary_goal_checker_;
private:  
  std::vector<geometry_msgs::msg::Point> footprint_spec_;
  std::shared_ptr<dwb_critics::ObstacleFootprintCritic> costmap_model_;
};

}  // namespace nav2_controller

#endif  // NAV2_CONTROLLER__PLUGINS__FUZZY_GOAL_CHECKER_HPP_
