// Copyright (c) 2022 CloudMinds
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

#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <exception>

#include "nav2_util/node_utils.hpp"
#include "nav2_waypoint_follower/plugins/wait_resume_at_waypoint.hpp"

namespace nav2_waypoint_follower
{
WaitResumeAtWaypoint::WaitResumeAtWaypoint()
: is_enabled_(true), wait_timeout_(180), is_pause_(false), curr_waypoint_index_(0), node_(nullptr), resume_follow_waypoints_service_(nullptr), wait_resume_task_publisher_(nullptr)
{
}

WaitResumeAtWaypoint::~WaitResumeAtWaypoint()
{
  wait_resume_task_publisher_->on_deactivate();
  resume_follow_waypoints_service_.reset();
  wait_resume_task_publisher_.reset();
}

void WaitResumeAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  logger_ = node_->get_logger();
  std::string node_name = node_->get_name();

  resume_follow_waypoints_service_ = node_->create_service<cm_msgs::srv::ResumeFollowWaypoints>(
    "/" + node_name + "/resume_follow_waypoints",
    std::bind(&WaitResumeAtWaypoint::ResumeFollowWaypointsCallBack, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
  );
  wait_resume_task_publisher_ = node_->create_publisher<cm_msgs::msg::WaitResumeFeedback>("/" + node_name + "/wait_resume_feedback", rclcpp::QoS(rclcpp::KeepAll()).transient_local().reliable());
  wait_resume_task_publisher_->on_activate();

  nav2_util::declare_parameter_if_not_declared(
    node_,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true)
  );
  nav2_util::declare_parameter_if_not_declared(
    node_,
    plugin_name + ".waypoint_type",
    rclcpp::ParameterValue(100)
  );
  nav2_util::declare_parameter_if_not_declared(
    node_,
    plugin_name + ".wait_timeout",
    rclcpp::ParameterValue(300)
  );

  node_->get_parameter(
    plugin_name + ".enabled",
    is_enabled_
  );
  node_->get_parameter(
    plugin_name + ".waypoint_type",
    type_
  );
  node_->get_parameter(
    plugin_name + ".wait_timeout",
    wait_timeout_
  );
}

bool WaitResumeAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint {%.2f, %.2f}, stop and wait for resume command util %i seconds",
    curr_waypoint_index, curr_pose.pose.position.x, curr_pose.pose.position.y,
    wait_timeout_
  );
  is_pause_ = true;
  curr_waypoint_index_ = curr_waypoint_index;
  curr_pose_ =  curr_pose;

  double start = node_->now().seconds();
  double now_stamp = node_->now().seconds();
  double last_feedback = 0;
  while (is_pause_ && (now_stamp - start < wait_timeout_)) {
    if (now_stamp - last_feedback > 2) {
      cm_msgs::msg::WaitResumeFeedback msg;
      msg.header.frame_id = "wait_resume_at_waypoint";
      msg.header.stamp = node_->now();
      msg.current_waypoint_index = curr_waypoint_index_;
      msg.current_pose = curr_pose_;
      msg.state = cm_msgs::msg::WaitResumeFeedback::WAITING;
      wait_resume_task_publisher_->publish(msg);
      RCLCPP_INFO(logger_, "Waiting for resume command from cloud...");
      last_feedback = now_stamp;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    now_stamp = node_->now().seconds();
  }
  cm_msgs::msg::WaitResumeFeedback msg;
  msg.header.frame_id = "wait_resume_at_waypoint";
  msg.header.stamp = node_->now();
  msg.current_waypoint_index = curr_waypoint_index_;
  msg.current_pose = curr_pose_;
  msg.state = cm_msgs::msg::WaitResumeFeedback::RESUMED;
  wait_resume_task_publisher_->publish(msg);
  return true;
}

void WaitResumeAtWaypoint::ResumeFollowWaypointsCallBack(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<cm_msgs::srv::ResumeFollowWaypoints::Request> /*request*/,
  const std::shared_ptr<cm_msgs::srv::ResumeFollowWaypoints::Response> response) {
  RCLCPP_INFO(logger_, "Received request to resume follow waypoints!");
  if (is_pause_) {
    cm_msgs::msg::WaitResumeFeedback msg;
    msg.header.frame_id = "wait_resume_at_waypoint";
    msg.header.stamp = node_->now();
    msg.current_waypoint_index = curr_waypoint_index_;
    msg.current_pose = curr_pose_;
    msg.state = cm_msgs::msg::WaitResumeFeedback::RESUMING;
    wait_resume_task_publisher_->publish(msg);
    is_pause_ = false;
    response->result = response->RESULT_SUCCESS;
    response->description = "Resume follow waypoints success!";
    RCLCPP_INFO(logger_, "Resume follow waypoints success!");
  }
  else {
    response->result = response->RESULT_NO_TASK_ACTIVATE;
    response->description = "Resume follow waypoints failed, no task activate!";
    RCLCPP_WARN(logger_, "Resume follow waypoints failed, no task activate!");
  }
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::WaitResumeAtWaypoint,
  nav2_core::WaypointTaskExecutor
)
