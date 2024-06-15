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

#include <string>
#include <memory>
#include <vector>
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/new_path_longer_condition.hpp"

namespace nav2_behavior_tree
{

NewPathLongerCondition::NewPathLongerCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  transform_tolerance_(0.2),
  global_frame_("map"),
  robot_base_frame_("base_link")
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  RCLCPP_INFO(node_->get_logger(),
    "name : %s, namespace : %s transform_tolerance_: %f.",
     name.c_str(), node_->get_name(), transform_tolerance_);

  trigger_start_ = node_->now();
  detour_start_time_ = node_->now();
  getInput("prox_len", prox_len_);
  getInput("length_factor", length_factor_);
  getInput("detour_len", detour_len_);
  getInput("force_update_seconds", force_update_seconds_);
  getInput("wait_time", wait_time_);
  getInput("forward_sampling_distance", forward_sampling_distance_);
  getInput("oscillation_threshold", oscillation_threshold_);  
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  RCLCPP_INFO(node_->get_logger(),
    "detour_len : %.2fm, force_update_seconds :%.2fs, wait_time : %.2fs, forward_sampling_distance : %.2fm, oscillation_threshold : %.2frad.",
    detour_len_, force_update_seconds_, wait_time_, forward_sampling_distance_, oscillation_threshold_);

  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  //publish 
  detour_state_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "detour_wait_state", custom_qos);

  mark_obstacle_client_ = node_->create_client<std_srvs::srv::SetBool>("/mark_tmp_obstacle"); 
  mark_obstacle_client_2_ = node_->create_client<std_srvs::srv::SetBool>("/mark_sensors_obstacle");     
}

bool NewPathLongerCondition::isPathUpdated(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path)
{
  bool path_updated = (new_path != old_path);
  // bool path_updated = (new_path != old_path && old_path.poses.size() != 0 &&
  //        new_path.poses.size() != 0 &&
  //        old_path.poses.back() == new_path.poses.back());
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  RCLCPP_INFO(node_->get_logger(),
    "path_updated : %s.", (path_updated ? "true" : "false"));

  return path_updated;

}

bool NewPathLongerCondition::isRobotInGoalProximity(
  nav_msgs::msg::Path & old_path,
  double & prox_leng)
{
  double old_path_length =
    nav2_util::geometry_utils::calculate_path_length(old_path, 0);
  bool near_goal = old_path_length < prox_leng;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "near_goal : %s, old_path_length :%.4fm, prox_leng : %.4fm.", (near_goal ? "true" : "false"),
    old_path_length, prox_leng);
  return near_goal;
  // return nav2_util::geometry_utils::calculate_path_length(old_path, 0) < prox_leng;
}

bool NewPathLongerCondition::isNewPathLonger(
  nav_msgs::msg::Path & new_path,
  nav_msgs::msg::Path & old_path,
  double & length_factor)
{
  double old_path_length =
    nav2_util::geometry_utils::calculate_path_length(old_path, 0);

  double new_path_length =
    nav2_util::geometry_utils::calculate_path_length(new_path, 0);

  bool new_path_longer = new_path_length > length_factor * old_path_length;

  // bool new_path_longer =
  //   nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "new_path_longer : %s, new_path_length :%.4fm, old_path_length :%.4fm.",
    (new_path_longer ? "true" : "false"), new_path_length, old_path_length);
  if (new_path_longer) {
    RCLCPP_INFO(node_->get_logger(),
      "new_path_longer : %s, factor : %f, new_path_length :%.4fm, old_path_length :%.4fm.",
      (new_path_longer ? "true" : "false"), length_factor, new_path_length, old_path_length);    
  }
  return new_path_longer;

  // return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
}

bool NewPathLongerCondition::isNewPathDetour(
  nav_msgs::msg::Path & new_path,
  double & length_factor)
{
  double new_path_length =
    nav2_util::geometry_utils::calculate_path_length(new_path, 0);

  bool new_path_detour = new_path_length > length_factor * detour_len_;

  // bool new_path_longer =
  //   nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "new_path_detour : %s, new_path_length :%.4fm.",
    (new_path_detour ? "true" : "false"), new_path_length);
  return new_path_detour;

  // return nav2_util::geometry_utils::calculate_path_length(new_path, 0) >
  //        length_factor * nav2_util::geometry_utils::calculate_path_length(
  //   old_path, 0);
}

bool NewPathLongerCondition::isPathToGoal(
    nav_msgs::msg::Path & path,
    geometry_msgs::msg::PoseStamped & goal)
{
  // Get euclidean distance
  auto dist_to_goal = nav2_util::geometry_utils::euclidean_distance(
    path.poses.back(), goal);

  bool go_to_goal = dist_to_goal < 1.0;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  // RCLCPP_INFO(node_->get_logger(),
    "go_to_goal : %s, dist_to_goal :%.4fm.",
    (go_to_goal ? "true" : "false"), dist_to_goal);
  return go_to_goal;
}

bool NewPathLongerCondition::isPathOscillation(
  const nav_msgs::msg::Path & new_path,
  const nav_msgs::msg::Path & old_path)
{
  if (new_path.poses.size() < 10) {
    return false;
  }
  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_WARN(node_->get_logger(), "Current robot pose is not available.");
    return false;
  }
  //check twice
  int check_nut = 4;
  for (int i = 1; i < check_nut; i++) {
    double forward_sampling_distance = i * forward_sampling_distance_;
    double oscillation_threshold = oscillation_threshold_ / i;
    geometry_msgs::msg::Pose pose_1 =
      transformPoseToBaseFrame(getSampledPathPt(
        current_pose, new_path, forward_sampling_distance));
    double  angular_1 =
      std::atan2(pose_1.position.y, pose_1.position.x);

    geometry_msgs::msg::Pose pose_2 =
      transformPoseToBaseFrame(getSampledPathPt(
        current_pose, old_path, forward_sampling_distance));
    double  angular_2 =
      std::atan2(pose_2.position.y, pose_2.position.x);
  
    double det_angular = std::fabs(angular_1 - angular_2);
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
    RCLCPP_WARN(node_->get_logger(),
      "The %d new_angular : %.2f, old_angular : %.2f, det_angular : %.2f, oscillation_threshold : %f.",
      i, angular_1, angular_2, det_angular, oscillation_threshold);
    if (det_angular < oscillation_threshold) {
      return false;
    }

  }  
  return true;
}

geometry_msgs::msg::PoseStamped
NewPathLongerCondition::getSampledPathPt(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & path,
  const double & forward_sampling_distance)
{

  // if (path.poses.size() < 2) {
  //   RCLCPP_WARN(
  //     node_->get_logger(),
  //     "Path.", forward_sampling_distance_);
  // }
  geometry_msgs::msg::PoseStamped temp_pose = path.poses.front();
  geometry_msgs::msg::Pose start = temp_pose.pose;
  double dx, dy;

  size_t start_index = 1;
  if (true) {
    // Find the closest pose to current pose on global path
    nav_msgs::msg::Path current_path = path;
    auto find_closest_pose_idx =
      [&robot_pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
          double curr_dist = nav2_util::geometry_utils::euclidean_distance(
            robot_pose, current_path.poses[curr_idx]);
          if (curr_dist < curr_min_dist) {
            curr_min_dist = curr_dist;
            closest_pose_idx = curr_idx;
          }
        }
        return closest_pose_idx;
      };
    start_index = find_closest_pose_idx();  
    // RCLCPP_INFO_THROTTLE(
    //   node_->get_logger(), *(node_->get_clock()), 3000, 
    RCLCPP_INFO(
      node_->get_logger(),     
      "closest_pose_idx :%lu.", start_index);
  }

  // Find the first point at least sampling distance away
  for (unsigned int i = start_index; i != path.poses.size(); i++) {
    dx = path.poses[i].pose.position.x - start.position.x;
    dy = path.poses[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= forward_sampling_distance) {
      temp_pose = path.poses[i];
      temp_pose.header.frame_id = path.header.frame_id;
      temp_pose.header.stamp =  node_->now();  // Get current time transformation
      return temp_pose;
    }
  }

  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *(node_->get_clock()), 3000,
    "Unable to find a sampling point at least %0.2f from the robot.", forward_sampling_distance_);
  return temp_pose;  
}

geometry_msgs::msg::Pose
NewPathLongerCondition::transformPoseToBaseFrame(
  const geometry_msgs::msg::PoseStamped & pt)
{
  geometry_msgs::msg::PoseStamped pt_base;
  if (!nav2_util::transformPoseInTargetFrame(
    pt, pt_base, *tf_, "base_link", transform_tolerance_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Can`t transform poase to base_link!");
  }
  return pt_base.pose;
}

void NewPathLongerCondition::pubDetourState(const bool & state)
{
  if (detour_state_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Bool tmp_msg;
    tmp_msg.data = state;
    detour_state_pub_->publish(std::move(tmp_msg));
  }
}

inline BT::NodeStatus NewPathLongerCondition::tick()
{
  bool detour_wait_state = false;
  double penalty_cost = 0.8;
  getInput("path", new_path_);

  // if (status() == BT::NodeStatus::IDLE) {
  //   // Reset the starting point since we're starting a new iteration of
  //   // NewPathLongerCondition (moving from IDLE to RUNNING)
  //   first_time_ = true;
  //   RCLCPP_WARN(node_->get_logger(), 
  //     "first_time_ : %s.", (first_time_ ? "true" : "false"));
  // }

  setStatus(BT::NodeStatus::RUNNING);

  std::vector<geometry_msgs::msg::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals", current_goals);
  geometry_msgs::msg::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    end_pose_ = goal_;
    trigger_start_ = node_->now();
    goal_update_time_ = node_->now();
    detour_state_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "get new goal!");
  } else if (goals_ != current_goals) {
    goals_ = current_goals;
    end_pose_ = goals_.back();
    trigger_start_ = node_->now();
    goal_update_time_ = node_->now();
    detour_state_ = false;
    RCLCPP_WARN(node_->get_logger(),
      "get new goals!");
  }
  // Determine how long its been since goal update
  auto elapsed = node_->now() - goal_update_time_;

  // Now, get that in seconds
  auto seconds = elapsed.seconds();
  // double wait_time = 2.0;_
  if (seconds < force_update_seconds_) {
    old_path_ = new_path_;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
      "force update path!");
  }

  // Determine how long its been since we've started this iteration
  // RCLCPP_WARN(node_->get_logger(),
  //   "node_->now() : %s, trigger_start_ : %s.",
  //   std::to_string(node_->now().nanoseconds()).c_str(),
  //   std::to_string(trigger_start_.nanoseconds()).c_str()); 
  auto check_elapsed = node_->now() - trigger_start_;

  // Now, get that in seconds
  auto during_seconds = check_elapsed.seconds();
  // RCLCPP_WARN(node_->get_logger(),
    // "during seconds : %fs, max wait time : %fs.", during_seconds, wait_time_); 
  // RCLCPP_WARN(node_->get_logger(),  
  RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
    "wait during seconds : %fs, max wait time : %fs.", during_seconds, wait_time_);

  if (!detour_state_) {
    detour_start_time_ = node_->now();
  }  

  // Check if the path is updated and valid, compare the old and the new path length,
  // given the goal proximity and check if the new path is longer
  if (isPathUpdated(new_path_, old_path_) &&
    // isRobotInGoalProximity(old_path_, prox_len_) &&
    (isNewPathLonger(new_path_, old_path_, length_factor_) ||
    //  isPathOscillation(new_path_, old_path_)
     (isPathOscillation(new_path_, old_path_) && 
     isNewPathLonger(new_path_, old_path_, penalty_cost))
    ) &&
    isNewPathDetour(new_path_, length_factor_) && during_seconds < wait_time_
    )
  {
    detour_wait_state = true;
    pubDetourState(detour_wait_state);
    setOutput("detour_wait_state", detour_wait_state);
    detour_state_ = false;
    // detour_start_time_ = node_->now(); 
    return BT::NodeStatus::SUCCESS; 
  }
  // set detour state
  if (during_seconds >= wait_time_) {
    RCLCPP_WARN(node_->get_logger(),
      "set detour state!!!!");
    detour_state_ = true;
  }
  //When the long detour starts, the obstacle mark is triggered to prevent
  // turning back and walking during the long detour.
  if (detour_state_) {
    auto detour_duration_time = node_->now() - detour_start_time_;
    auto detour_duration_time_s = detour_duration_time.seconds();
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
      "detour_duration_time : %fs.", detour_duration_time_s);
    if (detour_duration_time_s < 0.5) {
      RCLCPP_WARN(node_->get_logger(),
        "Call mark obstacle.");
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = true;
      // auto res = std::make_shared<std_srvs::srv::SetBool::Response>();
      if (mark_obstacle_client_->service_is_ready()) {
          mark_obstacle_client_->async_send_request(req);
      }

      if (mark_obstacle_client_2_->service_is_ready()) {
          mark_obstacle_client_2_->async_send_request(req);
      }
    }
  }

  old_path_ = new_path_;
  trigger_start_ = node_->now();
  detour_wait_state = false;
  pubDetourState(detour_wait_state);
  setOutput("detour_wait_state", detour_wait_state);
  // first_time_ = false;
  // RCLCPP_INFO(node_->get_logger(),
  //   "BT::NodeStatus::FAILURE;");  
  // RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 3000,
  //   "BT::NodeStatus::FAILURE;");      
  return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::NewPathLongerCondition>("NewPathLongerCondition");
}
