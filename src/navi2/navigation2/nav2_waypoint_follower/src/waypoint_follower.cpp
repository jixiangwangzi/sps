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

#include "nav2_waypoint_follower/waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace nav2_waypoint_follower
{

WaypointFollower::WaypointFollower(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("WaypointFollower", "", false, options),
  waypoint_task_executor_loader_("nav2_waypoint_follower",
    "nav2_core::WaypointTaskExecutor"),
  default_ids_{"wait_at_waypoint"},
  default_types_{"nav2_waypoint_follower::WaitAtWaypoint"}
{
  RCLCPP_INFO(get_logger(), "Creating");

  declare_parameter("stop_on_failure", true);
  declare_parameter("loop_rate", 20);
  declare_parameter("waypoint_task_executor_plugins", default_ids_);
  get_parameter("waypoint_task_executor_plugins", waypoint_task_executor_ids_);
  if (waypoint_task_executor_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      nav2_util::declare_parameter_if_not_declared(
        this, std::string(default_ids_[i] + ".plugin"),
        rclcpp::ParameterValue(default_types_[i])
      );
      nav2_util::declare_parameter_if_not_declared(
        this, std::string(default_ids_[i] + ".waypoint_type"),
        rclcpp::ParameterValue(0)
      );
    }
  }

  for (auto & waypoint_task_executor_id : waypoint_task_executor_ids_) {
    nav2_util::declare_parameter_if_not_declared(this, std::string(waypoint_task_executor_id + ".waypoint_type"), rclcpp::ParameterValue(0));
  }

}

WaypointFollower::~WaypointFollower()
{
  waypoint_task_executors_.clear();
  waypoint_type_map_waypoint_task_executor_type_.clear();
}

nav2_util::CallbackReturn
WaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  auto node = shared_from_this();

  waypoint_task_executor_types_.resize(waypoint_task_executor_ids_.size());

  stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
  loop_rate_ = get_parameter("loop_rate").as_int();


  callback_group_ = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);

    nav_through_poses_client_ = rclcpp_action::create_client<ClientT>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "navigate_through_poses", callback_group_
  );

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "follow_waypoints", std::bind(&WaypointFollower::followWaypoints, this)
  );

  for (size_t i = 0; i != waypoint_task_executor_ids_.size(); i++) {
    try {
      waypoint_task_executor_types_[i] = nav2_util::get_plugin_type_param(node, waypoint_task_executor_ids_[i]);
      RCLCPP_INFO(get_logger(), "Creating waypoint task executor plugin %s of type %s", waypoint_task_executor_ids_[i].c_str(), waypoint_task_executor_types_[i].c_str());
      auto waypoint_task_executor = waypoint_task_executor_loader_.createUniqueInstance(waypoint_task_executor_types_[i]);
      waypoint_task_executor->initialize(node, waypoint_task_executor_ids_[i]);
      waypoint_task_executors_.insert({waypoint_task_executor_ids_[i], std::move(waypoint_task_executor)});
      int waypoint_type = get_parameter(waypoint_task_executor_ids_[i] + ".waypoint_type").as_int();
      waypoint_type_map_waypoint_task_executor_type_.insert({waypoint_type, waypoint_task_executor_ids_[i]});
      RCLCPP_INFO(get_logger(), "Waypoint type %d ====> %s!", waypoint_type, waypoint_task_executor_ids_[i].c_str());
      RCLCPP_INFO(get_logger(), "Created waypoint task executor plugin success!");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(get_logger(), "Failed to create waypoint task executor. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_callback_group(callback_group_, get_node_base_interface());
  action_client_thread_ = std::make_unique<nav2_util::NodeThread>(executor);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_.reset();
  nav_through_poses_client_.reset();
  waypoint_task_executors_.clear();
  waypoint_type_map_waypoint_task_executor_type_.clear();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
WaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void
WaypointFollower::followWaypoints()
{
  auto goal = action_server_->get_current_goal();
  auto feedback = std::make_shared<ActionT::Feedback>();
  auto result = std::make_shared<ActionT::Result>();

  // Check if request is valid
  if (!action_server_ || !action_server_->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Received follow waypoint request with %i waypoints %i types:", static_cast<int>(goal->poses.size()), static_cast<int>(goal->types.size()));

  if (goal->poses.size() == 0) {
    action_server_->succeeded_current(result);
    return;
  }

  if (!goal->types.empty() && goal->poses.size() != goal->types.size()) {
    action_server_->terminate_current(result);
    return;
  }

  if (goal->poses.size() == goal->types.size()) {
    for (size_t i = 0; i < goal->poses.size(); ++i) {
      RCLCPP_INFO(get_logger(), "\tType: %d, goal: (%lf, %lf)", goal->types[i], goal->poses[i].pose.position.x, goal->poses[i].pose.position.y);
    }
  }

  rclcpp::WallRate r(loop_rate_);
  uint32_t goal_index = 0;
  bool new_goal = true;

  while (rclcpp::ok()) {
    // Check if asked to stop processing action
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Receive cancel request, cancel navigate through poses!");
      // auto cancel_future = nav_through_poses_client_->async_cancel_all_goals();
      auto cancel_future = nav_through_poses_client_->async_cancel_goal(nav_through_poses_goal_handle_);
      RCLCPP_INFO(get_logger(), "Sent cancel request to navigate through poses server!");
      cancel_future.wait();
      RCLCPP_INFO(get_logger(), "Cancel navigate through poses success!");
      action_server_->terminate_all();
      return;
    }

    // Check if asked to process another action
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
      goal = action_server_->accept_pending_goal();
      goal_index = 0;
      new_goal = true;
    }

    // Check if we need to send a new goal
    if (new_goal) {
      new_goal = false;
      ClientT::Goal client_goal;
      if (goal->types.empty()) {
        for (; goal_index < goal->poses.size(); ++goal_index) {
          client_goal.poses.push_back(goal->poses[goal_index]);
        }
      }
      else {
        for (; goal_index < goal->poses.size(); ++goal_index) {
          if (goal->types[goal_index] != goal->TMP_STOP) {
            client_goal.poses.push_back(goal->poses[goal_index]);
          }
          else {
            client_goal.poses.push_back(goal->poses[goal_index]);
            break;
          }
        }
      }
      RCLCPP_INFO(get_logger(), "=========================================");
      RCLCPP_INFO(get_logger(), "Send this goals to navigate through poses:");
      for (const auto& tmp_goal: client_goal.poses) {
        RCLCPP_INFO(get_logger(), "\t{%.2f, %.2f}", tmp_goal.pose.position.x, tmp_goal.pose.position.y);
      }
      RCLCPP_INFO(get_logger(), "now goal_index: %d", goal_index);
      RCLCPP_INFO(get_logger(), "=========================================");

      auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
      send_goal_options.result_callback =
        std::bind(&WaypointFollower::resultCallback, this, std::placeholders::_1);
      send_goal_options.goal_response_callback =
        std::bind(&WaypointFollower::goalResponseCallback, this, std::placeholders::_1);
      future_goal_handle_ =
        nav_through_poses_client_->async_send_goal(client_goal, send_goal_options);
      std::string goal_uuid_string;
      for (const auto &c: future_goal_handle_.get()->get_goal_id()) {
        goal_uuid_string += std::to_string(c);
      }
      RCLCPP_INFO(get_logger(), "current goal id: %s", goal_uuid_string.c_str());
      current_goal_status_ = ActionStatus::PROCESSING;
    }

    feedback->current_waypoint = goal_index;
    action_server_->publish_feedback(feedback);

    if (current_goal_status_ == ActionStatus::FAILED) {
      failed_ids_.push_back(goal_index);

      if (stop_on_failure_) {
        RCLCPP_WARN(
          get_logger(), "Failed to process waypoint %i in waypoint "
          "list and stop on failure is enabled."
          " Terminating action.", goal_index);
        result->missed_waypoints = failed_ids_;
        action_server_->terminate_current(result);
        failed_ids_.clear();
        return;
      } else {
        RCLCPP_INFO(
          get_logger(), "Failed to process waypoint %i,"
          " moving to next.", goal_index);
      }
    } else if (current_goal_status_ == ActionStatus::SUCCEEDED) {
      int type;
      if (goal->types.empty() || goal_index >= goal->types.size()) {
        type = 0;
      }
      else {
        type = goal->types[goal_index];
      }
      RCLCPP_INFO(
        get_logger(), "Succeeded processing waypoint %i, type %d, processing waypoint task execution",
        goal_index, type
      );

      std::string waypoint_task_executor_id;
      if (waypoint_type_map_waypoint_task_executor_type_.find(type) != waypoint_type_map_waypoint_task_executor_type_.end()) {
        waypoint_task_executor_id = waypoint_type_map_waypoint_task_executor_type_[type];
      } else {
        waypoint_task_executor_id = "";
        RCLCPP_WARN(get_logger(), "No waypoint task executor id find by given waypoint type %d", type);
      }

      if (waypoint_task_executor_id.empty()) {
        RCLCPP_INFO(get_logger(), "No task to handle on waypoint %i, moving to next.", goal_index);
      }
      else {
        bool is_task_executed = waypoint_task_executors_[waypoint_task_executor_id]->processAtWaypoint(
          goal->poses[goal_index], goal_index
        );
        RCLCPP_INFO(
          get_logger(), "Task execution at waypoint %i %s", goal_index,
          is_task_executed ? "succeeded" : "failed!"
        );
        // if task execution was failed and stop_on_failure_ is on , terminate action
        if (!is_task_executed && stop_on_failure_) {
          failed_ids_.push_back(goal_index);
          RCLCPP_WARN(
            get_logger(),
            "Failed to execute task at waypoint %i "
            " stop on failure is enabled."
            " Terminating action.", goal_index
          );
          result->missed_waypoints = failed_ids_;
          action_server_->terminate_current(result);
          failed_ids_.clear();
          return;
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Handled task execution on waypoint %i,"
            " moving to next.", goal_index
          );
        }
      }
    }

    if (current_goal_status_ != ActionStatus::PROCESSING &&
      current_goal_status_ != ActionStatus::UNKNOWN)
    {
      // Update server state
      goal_index++;
      new_goal = true;
      if (goal_index >= goal->poses.size()) {
        RCLCPP_INFO(
          get_logger(), "Completed all %lu waypoints requested.",
          goal->poses.size()
        );
        result->missed_waypoints = failed_ids_;
        action_server_->succeeded_current(result);
        failed_ids_.clear();
        return;
      }
    } else {
      RCLCPP_INFO_EXPRESSION(
        get_logger(),
        (static_cast<int>(now().seconds()) % 30 == 0),
        "Processing waypoint %i...", goal_index
      );
    }

    r.sleep();
  }
}

void
WaypointFollower::resultCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult & result)
{
  nav_through_poses_goal_handle_.reset();
  std::string goal_uuid_string;
  for (const auto &c: result.goal_id) {
    goal_uuid_string += std::to_string(c);
  }
  RCLCPP_INFO(get_logger(), "result callback goal id: %s, goal result code: %d", goal_uuid_string.c_str(), (char) result.code);
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      current_goal_status_ = ActionStatus::SUCCEEDED;
      return;
    case rclcpp_action::ResultCode::ABORTED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      current_goal_status_ = ActionStatus::FAILED;
      return;
    default:
      current_goal_status_ = ActionStatus::UNKNOWN;
      return;
  }
}

void
WaypointFollower::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr & goal)
{
  nav_through_poses_goal_handle_ = goal;
  std::string goal_uuid_string;
  for (const auto &c: goal->get_goal_id()) {
    goal_uuid_string += std::to_string(c);
  }
  RCLCPP_INFO(get_logger(), "goal response callback goal id %s", goal_uuid_string.c_str());
  if (!goal) {
    RCLCPP_ERROR(
      get_logger(),
      "navigate_to_pose action client failed to send goal to server.");
    current_goal_status_ = ActionStatus::FAILED;
  }
  else {
    current_goal_status_ = ActionStatus::PROCESSING;
    RCLCPP_INFO(get_logger(), "navigate_to_pose action client send goal to server success.");
  }
}

}  // namespace nav2_waypoint_follower

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_waypoint_follower::WaypointFollower)
