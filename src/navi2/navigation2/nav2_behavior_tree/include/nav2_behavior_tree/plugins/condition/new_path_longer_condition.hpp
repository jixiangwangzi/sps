// Copyright (c) 2022 Neobotix GmbH
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp_v3/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"

#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child everytime when the length of
 * the new path is smaller than the old one by the length given by the user.
 */
class NewPathLongerCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::NewPathLongerCondition
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  NewPathLongerCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Planned Path"),
      BT::InputPort<double>(
        "prox_len", 3.0,
        "Proximity length (m) for the path to be longer on approach"),
      BT::InputPort<double>(
        "length_factor", 2.0,
        "Length multiplication factor to check if the path is significantly longer"),
      BT::InputPort<double>(
        "detour_len", 4.0,
        "Proximity length (m) for the path to be longer on approach"),  
      BT::InputPort<double>(
        "force_update_seconds", 2.0,
        "Force to update path when goal update"),
      BT::InputPort<double>("wait_time", 10.0, "Max wait time"),
      BT::InputPort<double>("forward_sampling_distance", 1.0, "Forward sampling distance"),
      BT::InputPort<double>("oscillation_threshold", 1.3, "Path oscillation judgment threshold"),      
      BT::InputPort<std::string>("global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("robot_base_frame", std::string("base_link"), "Robot base frame"),
      BT::OutputPort<bool>("detour_wait_state", "True if detour wait state")
    };
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

private:
  /**
   * @brief Checks if the global path is updated
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @return whether the path is updated for the current goal
   */
  bool isPathUpdated(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path);

  /**
   * @brief Checks if the robot is in the goal proximity
   * @param old_path current path to the goal
   * @param prox_leng proximity length from the goal
   * @return whether the robot is in the goal proximity
   */
  bool isRobotInGoalProximity(
    nav_msgs::msg::Path & old_path,
    double & prox_leng);

  /**
   * @brief Checks if the new path is longer
   * @param new_path new path to the goal
   * @param old_path current path to the goal
   * @param length_factor multipler for path length check
   * @return whether the new path is longer
   */
  bool isNewPathLonger(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    double & length_factor);
 
  /**
   * @brief Checks if the new path is detour
   * @param new_path new path to the goal
   * @param length_factor multipler for path length check
   * @return whether the new path is detour
   */
  bool isNewPathDetour(
    nav_msgs::msg::Path & new_path,
    double & length_factor);

  bool isPathToGoal(
    nav_msgs::msg::Path & path,
    geometry_msgs::msg::PoseStamped & goal);

  bool isNewPathApproachGoal(
    nav_msgs::msg::Path & new_path,
    nav_msgs::msg::Path & old_path,
    geometry_msgs::msg::PoseStamped & goal);

  bool isPathOscillation(
    const nav_msgs::msg::Path & new_path,
    const nav_msgs::msg::Path & old_path);

  /**
   * @brief Finds the point on the path that is roughly the sampling
   * point distance away from the robot for use.
   * May throw exception if a point at least that far away cannot be found
   * @return pt location of the output point
   */
  geometry_msgs::msg::PoseStamped getSampledPathPt(
    const geometry_msgs::msg::PoseStamped & robot_pose,    
    const nav_msgs::msg::Path & path,
    const double & forward_sampling_distance);

  /**
   * @brief Uses TF to find the location of the sampled path point in base frame
   * @param pt location of the sampled path point
   * @return location of the pose in base frame
   */
  geometry_msgs::msg::Pose transformPoseToBaseFrame(
    const geometry_msgs::msg::PoseStamped & pt);

  void pubDetourState(const bool & state);

  /*
  get angle ACB, point C is the center point
  A(x1,y1)
  B(x2,y2)
  C(x3,y3)
  */
  double getAngle(const geometry_msgs::msg::Pose &p1,
  const geometry_msgs::msg::Pose &p2,
  const geometry_msgs::msg::Pose &p3)
  // double getAngle(double x1, double y1, double x2, double y2, double x3, double y3)
  {
    // double theta = atan2(x1 - x3, y1 - y3) - atan2(x2 - x3, y2 - y3);
    double theta = std::atan2(p1.position.x - p3.position.x, p1.position.y - p3.position.y) -
      atan2(p2.position.x - p3.position.x, p2.position.y - p3.position.y);
  	if (theta > M_PI)
	  	theta -= 2 * M_PI;
	  if (theta < -M_PI)
		  theta += 2 * M_PI;
 
  	theta = std::fabs(theta * 180.0 / M_PI);
	  return theta;
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detour_state_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mark_obstacle_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mark_obstacle_client_2_;
  
  // rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path new_path_;
  nav_msgs::msg::Path old_path_;
  double prox_len_ = std::numeric_limits<double>::max();
  double length_factor_ = std::numeric_limits<double>::max();
  double detour_len_ = 4.0;
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = true;
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  // bool goal_update_ = false;
  rclcpp::Time goal_update_time_;
  rclcpp::Time detour_start_time_;
  rclcpp::Time detour_duration_time_;
  bool detour_state_ = false;
  geometry_msgs::msg::PoseStamped end_pose_;
  double force_update_seconds_ = 2.0;
  rclcpp::Time trigger_start_;
  // double period_ = 0.;
  double wait_time_ = 0.;
  double forward_sampling_distance_ = 1.0;
  double transform_tolerance_ = 0.1;
  double oscillation_threshold_ = 1.3;
  
  //get robot pose
  std::string global_frame_;
  std::string robot_base_frame_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__PATH_LONGER_ON_APPROACH_HPP_
