// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("nav2_planner", "", true, options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  default_ids_{"GridBased"},
  default_types_{"nav2_navfn_planner/NavfnPlanner"},
  costmap_(nullptr),
  costmap_model_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }

  std::string param_file;
  for (size_t i = 0; i < options.arguments().size(); ++i) {
    if (options.arguments()[i] == "--params-file") {
      param_file = options.arguments()[i+1];
      break;
    }
  }
  RCLCPP_INFO(get_logger(), "param_file ==> %s", param_file.c_str());
  std::vector<std::string> costmap_node_options_arguments;
  costmap_node_options_arguments.push_back("--ros-args");
  costmap_node_options_arguments.push_back("-r");
  costmap_node_options_arguments.push_back(std::string("__ns:=") + nav2_util::add_namespaces(std::string{get_namespace()}, "global_costmap"));
  costmap_node_options_arguments.push_back("--ros-args");
  costmap_node_options_arguments.push_back("-r");
  costmap_node_options_arguments.push_back("global_costmap:" + std::string("__node:=") + "global_costmap");
  if (!param_file.empty()) {
    costmap_node_options_arguments.push_back("--params-file");
    costmap_node_options_arguments.push_back(param_file);
  }
  // The costmap node is used in the implementation of the controller
  auto costmap_node_options = rclcpp::NodeOptions().arguments(costmap_node_options_arguments);
  for (const auto &i: costmap_node_options.arguments()) {
    RCLCPP_INFO(get_logger(), "global_costmap arguments: %s", i.c_str());
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", std::string{get_namespace()}, "global_costmap", costmap_node_options);

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
}

PlannerServer::~PlannerServer()
{
  planners_.clear();
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();
  clock_ = node->get_clock();
  
  footprint_spec_ = costmap_ros_->getRobotFootprint();

  costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
  std::string costmap_model_name("nav2_planner/costmap_model");
  costmap_model_->initialize(node, costmap_model_name, std::string{get_namespace()}, costmap_ros_);

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_pose_ = std::make_unique<ActionServerToPose>(
    rclcpp_node_,
    "compute_path_to_pose",
    std::bind(&PlannerServer::computePlan, this));

  action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
    rclcpp_node_,
    "compute_path_through_poses",
    std::bind(&PlannerServer::computePlanThroughPoses, this));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_pose_->activate();
  action_server_poses_->activate();
  costmap_ros_->on_activate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();

  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
    "is_global_path_valid",
    std::bind(
      &PlannerServer::isPathValid, this,
      std::placeholders::_1, std::placeholders::_2));

  is_pose_safety_service_ = node->create_service<nav2_msgs::srv::IsPoseSafety>(
    "is_pose_safety",
    std::bind(
      &PlannerServer::isPoseSafety, this,
      std::placeholders::_1, std::placeholders::_2));
      
  clear_static_costmap_service_ = node->create_service<nav2_msgs::srv::ClearStaticCostmap>(
    "clear_static_costmap",
    std::bind(
      &PlannerServer::clearStaticCostmap, this,
      std::placeholders::_1, std::placeholders::_2));

  make_plan_service_ = node->create_service<nav_msgs::srv::GetPlan>(
    "make_plan",
    std::bind(
      &PlannerServer::makePlanService, this,
      std::placeholders::_1, std::placeholders::_2));      

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_pose_->deactivate();
  action_server_poses_->deactivate();
  plan_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_pose_.reset();
  action_server_poses_.reset();
  plan_publisher_.reset();
  tf_.reset();
  costmap_ros_->on_cleanup(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();
  costmap_ = nullptr;

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void PlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
bool PlannerServer::getStartPose(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal,
  geometry_msgs::msg::PoseStamped & start)
{
  if (goal->use_start) {
    start = goal->start;
  } else if (!costmap_ros_->getRobotPose(start)) {
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::transformPosesToGlobalFrame(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  geometry_msgs::msg::PoseStamped & curr_start,
  geometry_msgs::msg::PoseStamped & curr_goal)
{
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
    !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the start or goal pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }

  return true;
}

template<typename T>
bool PlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path,
  const std::string & planner_id)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_id.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Found valid path of size %lu to (%.2f, %.2f)",
    path.poses.size(), goal.pose.position.x,
    goal.pose.position.y);

  return true;
}

void
PlannerServer::computePlanThroughPoses()
{
  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_poses_->get_current_goal();
  auto result = std::make_shared<ActionThroughPoses::Result>();
  nav_msgs::msg::Path concat_path;

  try {
    if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_poses_, goal);

    if (goal->goals.size() == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Compute path through poses requested a plan with no viapoint poses, returning.");
      action_server_poses_->terminate_current();
    }

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_poses_, goal, start)) {
      return;
    }

    // Get consecutive paths through these points
    std::vector<geometry_msgs::msg::PoseStamped>::iterator goal_iter;
    geometry_msgs::msg::PoseStamped curr_start, curr_goal;
    for (unsigned int i = 0; i != goal->goals.size(); i++) {
      // Get starting point
      if (i == 0) {
        curr_start = start;
      } else {
        curr_start = goal->goals[i - 1];
      }
      curr_goal = goal->goals[i];

      // Transform them into the global frame
      if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal)) {
        return;
      }

      // Get plan from start -> goal
      nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

      // check path for validity
      if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id)) {
        return;
      }

      // Concatenate paths together
      concat_path.poses.insert(
        concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
      concat_path.header = curr_path.header;
    }

    // Publish the plan for visualization purposes
    result->path = concat_path;
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_poses_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(),
      "%s plugin failed to plan through %li points with final goal (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
      goal->goals.back().pose.position.y, ex.what());
    action_server_poses_->terminate_current();
  }
  
  auto end_time = steady_clock_.now() - start_time;
  RCLCPP_WARN_THROTTLE(get_logger(), *(clock_), 1000, 
    "computePlanThroughPoses(): take %.4f s", end_time.seconds());
  // Calculate distance on the path for debug
  // double path_length =
  //   nav2_util::geometry_utils::calculate_path_length(result->path);

  // RCLCPP_INFO(get_logger(), "computePlanThroughPoses(): path_length : %f.", path_length);     
}

void
PlannerServer::computePlan()
{
  auto start_time = steady_clock_.now();
  RCLCPP_INFO(get_logger(), "start computePlan()");
  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_pose_->get_current_goal();
  auto result = std::make_shared<ActionToPose::Result>();

  try {
    if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_)) {
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_pose_, goal);

    // Use start pose if provided otherwise use current robot pose
    geometry_msgs::msg::PoseStamped start;
    if (!getStartPose(action_server_pose_, goal, start)) {
      return;
    }

    // Transform them into the global frame
    geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
    if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose)) {
      return;
    }

    result->path = getPlan(start, goal_pose, goal->planner_id);

    if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id)) {
      return;
    }

    // Publish the plan for visualization purposes
    publishPlan(result->path);

    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;

    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_pose_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
      goal->planner_id.c_str(), goal->goal.pose.position.x,
      goal->goal.pose.position.y, ex.what());
    action_server_pose_->terminate_current();
  }
      // Calculate distance on the path
    double path_len =
      nav2_util::geometry_utils::calculate_path_length(result->path);
  auto end_time = steady_clock_.now() - start_time;
  RCLCPP_WARN(
    get_logger(), "Get %.4fm path take  %.4f s", path_len, end_time.seconds()); 
}

nav_msgs::msg::Path
PlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::string & planner_id)
{
  RCLCPP_INFO(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end()) {
    return planners_[planner_id]->createPlan(start, goal);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Planner names are: %s", planner_id.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}

void
PlannerServer::publishPlan(const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}

void PlannerServer::isPathValid(
  const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
{
  // auto start_time = steady_clock_.now();
  response->is_valid = true;

  if (request->path.poses.empty()) {
    response->is_valid = false;
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  unsigned int closest_point_index = 0;
  if (costmap_ros_->getRobotPose(current_pose)) {
    float current_distance = std::numeric_limits<float>::max();
    float closest_distance = current_distance;
    geometry_msgs::msg::Point current_point = current_pose.pose.position;
    for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
      geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

      current_distance = nav2_util::geometry_utils::euclidean_distance(
        current_point,
        path_point);

      if (current_distance < closest_distance) {
        closest_point_index = i;
        closest_distance = current_distance;
      }
    }

    /**
     * The lethal check starts at the closest point to avoid points that have already been passed
     * and may have become occupied
     */
    unsigned int mx = 0;
    unsigned int my = 0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      costmap_->worldToMap(
        request->path.poses[i].pose.position.x,
        request->path.poses[i].pose.position.y, mx, my);
      unsigned int cost = costmap_->getCost(mx, my);

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
      }
    }
  }
  // auto end_time = steady_clock_.now() - start_time;
  // RCLCPP_WARN(
  //   get_logger(), "Check safety used %f s", end_time.seconds()); 
}

void PlannerServer::isPoseSafety(
  const std::shared_ptr<nav2_msgs::srv::IsPoseSafety::Request> request,
  std::shared_ptr<nav2_msgs::srv::IsPoseSafety::Response> response)
{
  geometry_msgs::msg::Pose tmp_pose = request->pose.pose;
  // RCLCPP_WARN(
  //   get_logger(), "tmep_pose(%f, %f).", tmp_pose.position.x, tmp_pose.position.y);  
 
  response->is_safety = true;

  if (!isPoseValid(tmp_pose, footprint_spec_)) {
    response->is_safety = false;
  }

  // unsigned int mx = 0;
  // unsigned int my = 0;

  // if(!costmap_->worldToMap(
  //     tmp_pose.pose.position.x,
  //     tmp_pose.pose.position.y, mx, my))
  // {
  //   RCLCPP_WARN(
  //     get_logger(), "Computing map coords failed");
  //   response->is_safety = false;
  //   return;
  // }
  
  // unsigned int cost = costmap_->getCost(mx, my);

  // if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
  //   cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  // {
  //   response->is_safety = false;
  // }

}

bool PlannerServer::isPoseValid(const geometry_msgs::msg::Pose & pose,
  const std::vector<geometry_msgs::msg::Point>& footprint_spec) const
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf2::getYaw(pose.orientation);
  try {
    if ( costmap_model_->scorePose(pose2d,
      dwb_critics::getOrientedFootprint(pose2d, footprint_spec)) < 0 ) {
      return false;
    }
  } catch (...) {
    return false;
  }
  return true;
}

void PlannerServer::clearStaticCostmap(
  const std::shared_ptr<nav2_msgs::srv::ClearStaticCostmap::Request> request,
  std::shared_ptr<nav2_msgs::srv::ClearStaticCostmap::Response> response)
{
  RCLCPP_INFO(get_logger(), "enter into static costmap mark>>>>");
  bool mark    = request->mark;
  forward_dist_  = request->forward_dist;
  half_width_ = request->half_width;
  global_path_ = request->global_path;
  if (mark) {
    remPoints();
    markPoints();
    response->clear = true;
  } else {
    resetPoints();
    response->clear = true;
  }
}

void PlannerServer::makePlanService(
  const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
  std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
  waitForCostmap();

  std::string global_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped start = request->start;
  geometry_msgs::msg::PoseStamped goal_pose = request->goal;

  if(start.header.frame_id != global_frame){
    RCLCPP_ERROR(
      get_logger(), "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      global_frame.c_str(), start.header.frame_id.c_str());
    return;
  }

  if(goal_pose.header.frame_id != global_frame){
    RCLCPP_ERROR(
      get_logger(),"The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
      global_frame.c_str(), goal_pose.header.frame_id.c_str());
    return;
  }
  std::string planner_id = "GridBased";
  response->plan = getPlan(start, goal_pose, planner_id);

  if (response->plan.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path from (%.2f, %.2f) to (%.2f, %.2f)", planner_id.c_str(),
      start.pose.position.x, start.pose.position.y,
      goal_pose.pose.position.x, goal_pose.pose.position.y);
    return;
  }
  // Publish the plan for visualization purposes
  publishPlan(response->plan);  
}

void PlannerServer::remPoints()
{
//  if (!origin_cost_.empty()) {
//    origin_cost_.clear();
//  }
  int index = 0;
  for (int i = global_path_.poses.size() - 1; i >= 0; i--) {
    double x = global_path_.poses[i].pose.position.x;
    double y = global_path_.poses[i].pose.position.y;
    geometry_msgs::msg::PoseStamped current_pose;
    if (!costmap_ros_->getRobotPose(current_pose)) {
      RCLCPP_ERROR(get_logger(), "clear static costmap failed!!");
    }
    double delt_x = x - current_pose.pose.position.x;
    double delt_y = y - current_pose.pose.position.y;
    double dist = hypot(delt_x, delt_y);
    if (dist < 1.3 && dist >1.1) {
      index = i;
    }
  }
  double r_theta = tf2::getYaw(global_path_.poses[index].pose.orientation);
  Eigen::Vector2d path_orient = Eigen::Vector2d(std::cos(r_theta), std::sin(r_theta));
  Eigen::Vector2d vertical_vect(-path_orient.y(), path_orient.x());

  double middle_x = global_path_.poses[index].pose.position.x;
  double middle_y = global_path_.poses[index].pose.position.y;
  int poi_x, poi_y;

  for (double i = -half_width_; i <=half_width_; i+=0.01) {
    double mid_x = middle_x + i * vertical_vect.x();
    double mid_y = middle_y + i * vertical_vect.y();
    costmap_ros_->getCostmap()->worldToMapEnforceBounds(mid_x, mid_y, poi_x, poi_y);
    int index = costmap_ros_->getCostmap()-> getIndex(poi_x, poi_y);
    unsigned char cost = costmap_ros_->getCostmap()->getCost(poi_x, poi_y);
        costmap_ros_->getCostmap()->setCost(poi_x, poi_y, 255);
    origin_cost_.insert(std::make_pair(index, cost));
  } 
  RCLCPP_INFO(get_logger(), "remember the static costmap index and cost!!");
}

void PlannerServer::markPoints()
{
  RCLCPP_INFO(get_logger(),"begin mark static map...");
  std::vector<std::shared_ptr<nav2_costmap_2d::Layer> >* plugins 
                = costmap_ros_->getLayeredCostmap()->getPlugins();
  for (auto pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) 
  { 
    std::shared_ptr<nav2_costmap_2d::Layer>& plugin = *pluginp;
    if (plugin->getName().find("static_layer") != std::string::npos)   
    {
      std::shared_ptr<nav2_costmap_2d::StaticLayer> costmap; 
      costmap = std::static_pointer_cast<nav2_costmap_2d::StaticLayer>(plugin);
      unsigned char* grid = costmap->getCharMap();
      for (auto& lift_cost : origin_cost_) {
          grid[lift_cost.first] = nav2_costmap_2d::LETHAL_OBSTACLE; 
	            RCLCPP_INFO(get_logger(),"mark OK");
      }
    }        
  }
  RCLCPP_INFO(get_logger(), "mark static map over!");
}

void PlannerServer::resetPoints()
{
  RCLCPP_INFO(get_logger(), "begin reset static map...");
  if(origin_cost_.empty()){
    return;
    RCLCPP_WARN(get_logger(), "the origin cost is empty");
  }
  std::vector<std::shared_ptr<nav2_costmap_2d::Layer> >* plugins = costmap_ros_->getLayeredCostmap()->getPlugins();
  for (std::vector<std::shared_ptr<nav2_costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) 
  { 
    std::shared_ptr<nav2_costmap_2d::Layer>& plugin = *pluginp;
    if (plugin->getName().find("static_layer") != std::string::npos) { 
      std::shared_ptr<nav2_costmap_2d::StaticLayer> costmap; 
      costmap = std::static_pointer_cast<nav2_costmap_2d::StaticLayer>(plugin);
      unsigned char* grid = costmap->getCharMap();
      for (auto& lift_cost : origin_cost_) {
        grid[lift_cost.first] = lift_cost.second;
      }
    }        
  }
  origin_cost_.clear();
  RCLCPP_INFO(get_logger(),"reset static map over!");
}

}  // namespace nav2_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_planner::PlannerServer)
