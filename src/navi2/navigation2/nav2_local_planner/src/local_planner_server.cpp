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
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_local_planner/local_planner_server.hpp"

using namespace std::chrono_literals;

namespace nav2_local_planner
{

LocalPlannerServer::LocalPlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("nav2_local_planner", "", true, options),
  gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
  planner_id_("GridBased"),
  planner_plugin_("nav2_smac_planner/SmacPlanner2D"),
  costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_id", planner_id_);
  declare_parameter("expected_planner_frequency", 1.0);
  declare_parameter("path_distance", 4.0);
  declare_parameter("bezier_transition_threshold", 0.523);
  declare_parameter("bezier_move_points_num", 10);

  get_parameter("planner_id", planner_id_);
  declare_parameter(planner_id_ + ".plugin", planner_plugin_);

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
  costmap_node_options_arguments.push_back(std::string("__ns:=") + nav2_util::add_namespaces(std::string{get_namespace()}, "planner_costmap"));
  costmap_node_options_arguments.push_back("--ros-args");
  costmap_node_options_arguments.push_back("-r");
  costmap_node_options_arguments.push_back("planner_costmap:" + std::string("__node:=") + "planner_costmap");
  if (!param_file.empty()) {
    costmap_node_options_arguments.push_back("--params-file");
    costmap_node_options_arguments.push_back(param_file);
  }
  auto costmap_node_options = rclcpp::NodeOptions().arguments(costmap_node_options_arguments);
  for (const auto &i: costmap_node_options.arguments()) {
    RCLCPP_INFO(get_logger(), "planner_costmap arguments: %s", i.c_str());
  }

  // Setup the global costmap
  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "planner_costmap", std::string{get_namespace()}, "planner_costmap", costmap_node_options);

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  history_path_.poses.clear();
  origin_global_path_.poses.clear();
}

LocalPlannerServer::~LocalPlannerServer()
{
  planner_ = nullptr;
  costmap_thread_.reset();
}

nav2_util::CallbackReturn
LocalPlannerServer::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->on_configure(state);
  costmap_ = costmap_ros_->getCostmap();

  RCLCPP_DEBUG(
    get_logger(), "Costmap size: %d,%d",
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  tf_ = costmap_ros_->getTfBuffer();

  auto node = shared_from_this();
  clock_ = node->get_clock();

  try {
    planner_plugin_ = nav2_util::get_plugin_type_param(
      node, planner_id_);
    planner_ =
      gp_loader_.createUniqueInstance(planner_plugin_);
    RCLCPP_INFO(
      get_logger(), "Created local planner plugin %s of type %s",
      planner_id_.c_str() , planner_plugin_.c_str());
    planner_->configure(node, planner_id_, tf_, costmap_ros_);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      get_logger(), "Failed to create local planner. Exception: %s",
      ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  get_parameter("bezier_transition_threshold", bezier_transition_threshold_);
  get_parameter("bezier_move_points_num", bezier_move_points_num_);

  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  get_parameter("path_distance", path_distance_);

  // Initialize pubs & subs
  local_plan_publisher_ = create_publisher<nav_msgs::msg::Path>("local_plan_path", 1);
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("local_plan_global", 1);
  obs_publisher_ = create_publisher<std_msgs::msg::Float64>("/obstacle", 1);

  // Create the action servers for local path plan
  action_server_local_plan_ = std::make_unique<ActionServerLocalPlan>(
    rclcpp_node_,
    "local_path_plan",
    std::bind(&LocalPlannerServer::computePlan, this));
  history_path_.poses.clear();
  origin_global_path_.poses.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalPlannerServer::on_activate(const rclcpp_lifecycle::State & state)
{
  auto node = shared_from_this();
  is_path_valid_service_ = node->create_service<nav2_msgs::srv::IsPathValid>(
  "is_path_valid",
  std::bind(
    &LocalPlannerServer::isPathValid, this,
    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  local_plan_publisher_->on_activate();
  obs_publisher_->on_activate();
  action_server_local_plan_->activate();
  costmap_ros_->on_activate(state);
  planner_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalPlannerServer::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_local_plan_->deactivate();
  plan_publisher_->on_deactivate();
  local_plan_publisher_->on_deactivate();
  obs_publisher_->on_deactivate();
  costmap_ros_->on_deactivate(state);
  planner_->deactivate();
  history_path_.poses.clear();
  origin_global_path_.poses.clear();
  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
LocalPlannerServer::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_local_plan_.reset();
  plan_publisher_.reset();
  local_plan_publisher_.reset();
  obs_publisher_.reset();
  tf_.reset();
  costmap_ros_->on_cleanup(state);
  planner_->cleanup();
  history_path_.poses.clear();
  origin_global_path_.poses.clear();
  costmap_ = nullptr;

  return nav2_util::CallbackReturn::SUCCESS;
}

void LocalPlannerServer::isPathValid(
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
    double check_dis = request->check_distance;
    double length = 0.0;
    for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
      if (!costmap_->worldToMap(
          request->path.poses[i].pose.position.x,
          request->path.poses[i].pose.position.y, mx, my))
      {
        continue;
      }
      
      unsigned int cost = costmap_->getCost(mx, my);
      if (i == closest_point_index)
      {
        length = 0.0;
      } else {
        length += nav2_util::geometry_utils::euclidean_distance(
          request->path.poses[i],
          request->path.poses[i - 1]);
      }
      if (length > check_dis)
      {
        response->is_valid = true;
        return;
      }

      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        response->is_valid = false;
        return;
      }
    }
  }
}

nav2_util::CallbackReturn
LocalPlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  history_path_.poses.clear();
  origin_global_path_.poses.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool LocalPlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

void LocalPlannerServer::waitForCostmap()
{
  // Don't compute a plan until costmap is valid (after clear costmap)
  rclcpp::Rate r(100);
  while (!costmap_ros_->isCurrent()) {
    r.sleep();
  }
}

template<typename T>
bool LocalPlannerServer::isCancelRequested(
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
void LocalPlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}

template<typename T>
  bool LocalPlannerServer::transformPosesToGlobalFrame(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    const double transform_length,
    const nav_msgs::msg::Path & global_path,
    const double near_to_goal_length,
    geometry_msgs::msg::PoseStamped & curr_start,
    nav_msgs::msg::Path & curr_path,
    bool & near_to_goal)
{
  curr_path.poses.clear();
  near_to_goal = false;
  if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start))
  {
    RCLCPP_WARN(
      get_logger(), "Could not transform the current pose in the costmap frame");
    action_server->terminate_current();
    return false;
  }
  if (global_path.poses.empty())
  {
    RCLCPP_WARN(
      get_logger(), "origin path is empty");
    action_server->terminate_current();
    return false;
  }
  std::string cost_map_frame =  costmap_ros_->getGlobalFrameID();
  std::string global_frame = global_path.poses.front().header.frame_id;
  nav_msgs::msg::Path transform_path;
  if (cost_map_frame == global_frame)
  {
    transform_path = global_path;
  } else {
    for(geometry_msgs::msg::PoseStamped pose : global_path.poses) {
      pose.header = global_path.header;
      geometry_msgs::msg::PoseStamped transform_pose;
      if (!costmap_ros_->transformPoseToGlobalFrame(pose, transform_pose))
      {
        RCLCPP_WARN(
          get_logger(), "Could not transform the global path in the costmap frame");
        action_server->terminate_current();
        return false;
      }
      transform_path.poses.emplace_back(transform_pose);
    }
  }

  if (transform_path.poses.empty()) {
    RCLCPP_WARN(
      get_logger(), "transform path is empty");
    action_server->terminate_current();
    return false;
  }
  transform_path.header.frame_id = transform_path.poses.front().header.frame_id;
  transform_path.header.stamp = clock_->now();
  if (transform_length <= 0.0)
  {
    curr_path = transform_path;
    return true;
  }
  curr_path.header.frame_id = transform_path.poses.front().header.frame_id;
  curr_path.header.stamp = clock_->now();
  double min_distance = __DBL_MAX__;
  int nearest_idx = 0;
  for (int i = 0; i < static_cast<int>(transform_path.poses.size()); i++) {
    double dis =  nav2_util::geometry_utils::euclidean_distance(
      transform_path.poses.at(i), curr_start);
    if (dis < min_distance)
    {
      min_distance = dis;
      nearest_idx = i;
    }
  }

  if (nearest_idx < 0 || nearest_idx >= static_cast<int>(transform_path.poses.size()))
  {
    RCLCPP_WARN(
      get_logger(), "Could not find nearest point in transform path");
    action_server->terminate_current();
    return false;
  }
  double remain_path_length =
   nav2_util::geometry_utils::calculate_path_length(transform_path, static_cast<size_t>(nearest_idx));

  if (remain_path_length <= near_to_goal_length)
  {
    curr_path = transform_path;
    near_to_goal = true;
    return true;
  }

  // Find points over max distance so we don't need.
  double s = 0.0;
  int end_idx = nearest_idx;
  for (int i = nearest_idx; i < static_cast<int>(transform_path.poses.size()); ++i) {
    if (nearest_idx < 0) {
      break;
    }
    if (i == nearest_idx) {
      s = 0.0;
    } else {
      double dist_poses =
        nav2_util::geometry_utils::euclidean_distance(transform_path.poses.at(i - 1), transform_path.poses.at(i));
      s += dist_poses;
    }
    end_idx = i;
    if (s >= transform_length) {
      break;
    }
  }

  if (end_idx < nearest_idx || end_idx >= static_cast<int>(transform_path.poses.size()))
  {
    RCLCPP_WARN(
      get_logger(), "Could not get end idx");
    action_server->terminate_current();
    return false;
  }

  for (int i = nearest_idx; i <= end_idx; i++)
  {
    curr_path.poses.emplace_back(transform_path.poses.at(i));
  }

  return (!curr_path.poses.empty());
}

template<typename T>
bool LocalPlannerServer::validatePath(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  const geometry_msgs::msg::PoseStamped & goal,
  const nav_msgs::msg::Path & path)
{
  if (path.poses.size() == 0) {
    RCLCPP_WARN(
      get_logger(), "Planning algorithm %s failed to generate a valid"
      " path to (%.2f, %.2f)", planner_plugin_.c_str(),
      goal.pose.position.x, goal.pose.position.y);
    action_server->terminate_current();
    return false;
  }

  if (costmap_)
  {
    unsigned int mx, my;
    for (auto pose : path.poses)
    {
      if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
      {
        auto cost = costmap_->getCost(mx, my);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          RCLCPP_WARN(get_logger(), "local plan path passing through unknown areas or obstacles");
          return false;
        }
      } 
    }
  }
  return true;
}

void
LocalPlannerServer::computePlan()
{
  RCLCPP_INFO(
    get_logger(), "Local Planner computePlan Running!");
  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_local_plan_->get_current_goal();
  auto result = std::make_shared<ActionLocalPlan::Result>();

  try {
    if (isServerInactive(action_server_local_plan_) || isCancelRequested(action_server_local_plan_)) {
      history_path_.poses.clear();
      origin_global_path_.poses.clear();
      return;
    }

    waitForCostmap();

    getPreemptedGoalIfRequested(action_server_local_plan_, goal);

    updateGlobalPath(history_path_, goal->global_path);

    if (origin_global_path_.poses.empty())
    {
      RCLCPP_ERROR(
        get_logger(), "Ref line of Local Planner is empty");
      action_server_local_plan_->terminate_current();
      return;
    }
    
    //  get current robot pose
    geometry_msgs::msg::PoseStamped current_pose;
    if (!getRobotPose(current_pose)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to Get Robot Pose For Local Planner");
      action_server_local_plan_->terminate_current();
      return;
    }

    // Transform them into the global frame
    nav_msgs::msg::Path truncate_path;
    bool near_to_goal = false;
    if (!transformPosesToGlobalFrame(action_server_local_plan_,
          goal->truncate_distance, origin_global_path_, goal->near_to_goal_length,
          current_pose, truncate_path, near_to_goal)) {
      return;
    }

    if (near_to_goal)
    {
      RCLCPP_INFO(
        get_logger(), "Is near to goal");
      result->planning_time = steady_clock_.now() - start_time;
      result->result_path = history_path_.poses.empty() ? origin_global_path_ : history_path_;;
      publishPlan(plan_publisher_, result->result_path);
      history_path_ = result->result_path;
      action_server_local_plan_->succeeded_current(result);
      return;
    }
    // publish obstacle
    std_msgs::msg::Float64 tmp;
    tmp.data = 1.0;
    obs_publisher_->publish(tmp);
    geometry_msgs::msg::PoseStamped goal_pose;
    if (!getGoalPose(truncate_path, goal_pose))
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to Get Goal Pose For Local Planner");
      action_server_local_plan_->terminate_current();
      return;
    }
    auto local_path = getPlan(current_pose, goal_pose);
    if (!validatePath(action_server_local_plan_, goal_pose, local_path)) {
      RCLCPP_ERROR(
        get_logger(), "Local Planner: path is not valid");
      return;
    }
    // Publish the plan for visualization purposes
    publishPlan(local_plan_publisher_, local_path);

    // Stitch local path to global path
    if (!stitchGlobalPath(origin_global_path_, local_path, result->result_path))
    {
      action_server_local_plan_->terminate_current();
      RCLCPP_ERROR(
        get_logger(), "Local Planner: stitchGlobalPath failed");
      return;
    }
    publishPlan(plan_publisher_, result->result_path);
    history_path_ = result->result_path;
    auto cycle_duration = steady_clock_.now() - start_time;
    result->planning_time = cycle_duration;
    action_server_local_plan_->succeeded_current(result);
    RCLCPP_INFO(
      get_logger(), "Local Planner action_server_local_plan_ success!");
    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Local Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to plan calculation: \"%s\"",
      planner_plugin_.c_str(), ex.what());
    action_server_local_plan_->terminate_current();
  }
}

void LocalPlannerServer::updateGlobalPath(
  const nav_msgs::msg::Path & last_path,
  const nav_msgs::msg::Path & new_path) 
{
  if (origin_global_path_.poses.empty() || 
      last_path.poses.empty() || new_path.poses.empty())
  {
    origin_global_path_ = new_path;
    return;
  }

  if (isDifferentOfTwoPath(last_path, new_path, 0.01, 30.0))
  {
    origin_global_path_ = new_path;
    return;
  }
}

nav_msgs::msg::Path
LocalPlannerServer::getPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
    "(%.2f, %.2f).", start.pose.position.x, start.pose.position.y,
    goal.pose.position.x, goal.pose.position.y);

  if (planner_ != nullptr) {
    return planner_->createPlan(start, goal);
  } else {
    RCLCPP_ERROR(
      get_logger(), "planner %s is not a valid planner. ",
      planner_plugin_.c_str());
  }

  return nav_msgs::msg::Path();
}

bool LocalPlannerServer::isDifferentOfTwoPath(
  const nav_msgs::msg::Path & path1,
  const nav_msgs::msg::Path & path2,
  const double dis_tolerance,
  const double check_dis) {
  if (path1.poses.empty() || path2.poses.empty())
  {
    return true;
  }
  
  double sum_length1 = nav2_util::geometry_utils::calculate_path_length(path1, 0);
  double sum_length2 = nav2_util::geometry_utils::calculate_path_length(path2, 0);
  if (std::fabs(sum_length1 - sum_length2) > dis_tolerance)
  {
    RCLCPP_INFO(get_logger(), "Two path length diff: %f.", std::fabs(sum_length1 - sum_length2));
    return true;
  }
  unsigned int start_check_idx1 = path1.poses.size() - 1;
  unsigned int start_check_idx2 = path2.poses.size() - 1;
  double length1 = 0.0;
  double length2 = 0.0;
  while (start_check_idx1 > 0 && start_check_idx2 > 0)
  {
    if (start_check_idx1 > 0 && start_check_idx1 < path1.poses.size() &&
        start_check_idx2 > 0 && start_check_idx2 < path2.poses.size())
    {
      double distance = nav2_util::geometry_utils::euclidean_distance(
          path1.poses.at(start_check_idx1), path2.poses.at(start_check_idx2));
      if (distance > dis_tolerance)
      {
        return true;
      }
      if (start_check_idx1 == path1.poses.size() - 1)
      {
        length1 = 0.0;
      } else {
        length1 += nav2_util::geometry_utils::euclidean_distance(
          path1.poses.at(start_check_idx1), path1.poses.at(start_check_idx1 + 1));
      }
      if (start_check_idx2 == path2.poses.size() - 1)
      {
        length2 = 0.0;
      } else {
        length2 += nav2_util::geometry_utils::euclidean_distance(
          path2.poses.at(start_check_idx2), path2.poses.at(start_check_idx2 + 1));
      }
      double length = std::min(length1, length2);
      if (length > check_dis)
      {
        break;
      }

    }
    start_check_idx1--;
    start_check_idx2--;
  }

  return false;
}

bool LocalPlannerServer::getGoalPose(
  const nav_msgs::msg::Path & path,
  geometry_msgs::msg::PoseStamped & goal) {

  if (path.poses.empty())
  {
    RCLCPP_INFO(
      get_logger(), "local planner truncate_path is empty");    
    return false;
  }
  if (!costmap_)
  {
    RCLCPP_INFO(
      get_logger(), "cost map is not valid");    
    return false;
  }
  size_t end_index = path.poses.size();
  for (size_t i = 0; i < path.poses.size(); i++)
  {
    auto pose = path.poses.at(i);
    unsigned int mx, my;
    if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
      end_index = i;
      break;
    }
    if (mx >= costmap_->getSizeInCellsX() ||
        my >= costmap_->getSizeInCellsY())
    {
      end_index = i;
      break;
    }
  }

  if (end_index <= 0)
  {
    return false;
  }
  goal.header.frame_id = path.header.frame_id;
  goal = (end_index >= path.poses.size()) ? path.poses.back() : path.poses.at(end_index - 1);
  return true;

}

bool LocalPlannerServer::stitchGlobalPath(
  const nav_msgs::msg::Path & global_path,
  const nav_msgs::msg::Path & local_path,
  nav_msgs::msg::Path & result_path) {

  if (global_path.poses.empty())
  {
    return false;
  }

  if (local_path.poses.empty())
  {
    result_path = global_path;
    return true;
  }

  std::string local_frame = local_path.poses.front().header.frame_id;
  std::string global_frame = global_path.poses.front().header.frame_id;
  result_path.poses.clear();
  result_path.header.frame_id = global_frame;
  if (local_frame != global_frame)
  {
    for (auto pose : local_path.poses)
    {
      geometry_msgs::msg::PoseStamped trans_pose = tf_->transform(
        pose, global_frame);
      result_path.poses.emplace_back(trans_pose);
    }

  } else {
    result_path = local_path;
  }

  if (result_path.poses.size() < 2)
  {
    return false;
  }

  double nearest_dis = __DBL_MAX__;
  auto start_pose = result_path.poses.back();
  size_t start_idx = 0;
  for (size_t i = 0; i < global_path.poses.size(); i++)
  {
    double dis =  nav2_util::geometry_utils::euclidean_distance(
      global_path.poses.at(i), start_pose);
    if (dis < nearest_dis)
    {
      start_idx = i;
      nearest_dis = dis;
    }
  }
  if (start_idx >= (global_path.poses.size() - 1))
  {
    return true;
  }

  double g_x = global_path.poses.at(start_idx + 1).pose.position.x -
    global_path.poses.at(start_idx).pose.position.x;
  double g_y = global_path.poses.at(start_idx + 1).pose.position.y -
    global_path.poses.at(start_idx).pose.position.y;
  double l_x = result_path.poses.back().pose.position.x -
    result_path.poses.at(result_path.poses.size() - 2).pose.position.x;
  double l_y = result_path.poses.back().pose.position.y -
    result_path.poses.at(result_path.poses.size() - 2).pose.position.y;

  double delta_dis = std::sqrt(g_x * g_x + g_y * g_y);
  double g_theta = std::atan2(g_y, g_x);
  double l_theta = std::atan2(l_y, l_x);
  double threshold = bezier_transition_threshold_;
  if (std::fabs(g_theta - l_theta) >= threshold || nearest_dis > 2.0 * delta_dis)
  {
    RCLCPP_INFO(
      get_logger(), "stitch local path to global: need to use beizer!");
    size_t move_num = bezier_move_points_num_;
    size_t local_idx = (result_path.poses.size() <= (move_num + 1))
      ? 0 : (result_path.poses.size() - move_num - 1);
    size_t global_idx = ((start_idx + move_num) >= global_path.poses.size())
      ? (global_path.poses.size() - 1) : (start_idx + move_num);
    double start_theta;
    if (local_idx >= (result_path.poses.size() - 1))
    {
      start_theta = l_theta;
    } else {
      double start_delta_x = result_path.poses.at(local_idx + 1).pose.position.x -
        result_path.poses.at(local_idx).pose.position.x;
      double start_delta_y = result_path.poses.at(local_idx + 1).pose.position.y -
        result_path.poses.at(local_idx).pose.position.y;
      start_theta = std::atan2(start_delta_y, start_delta_x);
    }
    double end_theta;
    if (global_idx >= (global_path.poses.size() - 1))
    {
      double end_delta_x = global_path.poses.back().pose.position.x -
        global_path.poses.at(global_path.poses.size() - 2).pose.position.x;
      double end_delta_y = global_path.poses.back().pose.position.y -
        global_path.poses.at(global_path.poses.size() - 2).pose.position.y;
      end_theta = std::atan2(end_delta_y, end_delta_x);
    } else {
      double end_delta_x = global_path.poses.at(global_idx + 1).pose.position.x -
        global_path.poses.at(global_idx).pose.position.x;
      double end_delta_y = global_path.poses.at(global_idx + 1).pose.position.y -
        global_path.poses.at(global_idx).pose.position.y;
      end_theta = std::atan2(end_delta_y, end_delta_x);
    }

    auto start_bezier_pose = result_path.poses.at(local_idx);
    auto end_bezier_pose = global_path.poses.at(global_idx);
    double sum_dis = nav2_util::geometry_utils::euclidean_distance(
      start_bezier_pose, end_bezier_pose);
    size_t interp_cnt = static_cast<size_t>(sum_dis / delta_dis);
    geometry_msgs::msg::PoseStamped pt1, pt2;
    pt1.pose.position.x = start_bezier_pose.pose.position.x +
      sum_dis * 0.4 * std::cos(start_theta);
    pt1.pose.position.y = start_bezier_pose.pose.position.y +
      sum_dis * 0.4 * std::sin(start_theta);
    pt2.pose.position.x = end_bezier_pose.pose.position.x -
      sum_dis * 0.4 * std::cos(end_theta);
    pt2.pose.position.y = end_bezier_pose.pose.position.y -
      sum_dis * 0.4 * std::sin(end_theta);
    nav_msgs::msg::Path bezier_path;
    bezier_path.poses.clear();
    geometry_msgs::msg::PoseStamped last_pose = start_bezier_pose;
    for (size_t i = 1; i <= interp_cnt; i++)
    {
      double t = (static_cast<double>(i) / static_cast<double>(interp_cnt));
      geometry_msgs::msg::PoseStamped bezier_pose =
        calcCubicBezier(start_bezier_pose, pt1, pt2, end_bezier_pose, t);
      double delta_x = bezier_pose.pose.position.x - last_pose.pose.position.x;
      double delta_y = bezier_pose.pose.position.y - last_pose.pose.position.y;
      double theta = std::atan2(delta_y, delta_x);
      bezier_pose.pose.orientation.x = 0.0;
      bezier_pose.pose.orientation.y = 0.0;
      bezier_pose.pose.orientation.z = std::sin(theta / 2.0);
      bezier_pose.pose.orientation.w = std::cos(theta / 2.0);
      bezier_pose.header = global_path.poses.front().header;
      bezier_path.poses.emplace_back(bezier_pose);
      last_pose = bezier_pose;
    }
    result_path.poses.erase((result_path.poses.begin() + local_idx), result_path.poses.end());
    result_path.poses.insert(result_path.poses.end(),
      bezier_path.poses.begin(), bezier_path.poses.end());
    result_path.poses.insert(result_path.poses.end(),
      (global_path.poses.begin() + global_idx), global_path.poses.end());
  } else {
    result_path.poses.insert(result_path.poses.end(),
      (global_path.poses.begin() + start_idx + 1), global_path.poses.end());
  }

  return (!result_path.poses.empty());
}

geometry_msgs::msg::PoseStamped LocalPlannerServer::calcCubicBezier(
  const geometry_msgs::msg::PoseStamped & p0,
  const geometry_msgs::msg::PoseStamped & p1,
  const geometry_msgs::msg::PoseStamped & p2,
  const geometry_msgs::msg::PoseStamped & p3,
  const double t)
{
  if (t > 1.0)
  {
    return p3;
  } else if (t <= 0.0)
  {
    return p0;
  }

  geometry_msgs::msg::PoseStamped pose;
  double delta_t = 1.0 - t;
  pose.pose.position.x = p0.pose.position.x * delta_t * delta_t * delta_t +
    3.0 * p1.pose.position.x * delta_t * delta_t * t +
    3.0 * p2.pose.position.x * delta_t * t * t +
    p3.pose.position.x * t * t * t;
  pose.pose.position.y = p0.pose.position.y * delta_t * delta_t * delta_t +
    3.0 * p1.pose.position.y * delta_t * delta_t * t +
    3.0 * p2.pose.position.y * delta_t * t * t +
    p3.pose.position.y * t * t * t;
  return pose;
}

bool LocalPlannerServer::getRobotPose(geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    return false;
  }
  pose = current_pose;
  return true;
}

void
LocalPlannerServer::publishPlan(
  const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr& publisher,
  const nav_msgs::msg::Path & path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (publisher->is_activated() && publisher->get_subscription_count() > 0) {
    publisher->publish(std::move(msg));
  }
}

}  // namespace nav2_local_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_local_planner::LocalPlannerServer)
