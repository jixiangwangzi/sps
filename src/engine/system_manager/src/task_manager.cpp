#include <chrono>
#include <filesystem>
#include <fstream>

#include "system_manager/bt/robot_bt.h"
#include "system_manager/bt_node_impl.hpp"

using namespace std::chrono_literals;

namespace naviengine
{

inline std::string TaskManager::toString(double val, int precision)
{
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << val;
    return out.str();
}

TaskManager::TaskManager() : Node("TaskManager")
{
    ROS_INFO("hello this is TaskManager");
    client_init();
    action_client_init();
    publish_init();
    subscription_init();
    parameter_init();

    if (!main_drop_switch_)
    {
        switchVisualDrop(false);
    }
}

TaskManager::~TaskManager()
{
}

void TaskManager::client_init()
{
    navi_pause_control_client_ = std::make_shared<NaviPauseControlServiceClient>();
}

void TaskManager::action_client_init()
{
    navi_lift_goal_ = nav2_msgs::action::NavigateToPose::Goal();
    navi_lift_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(get_node_base_interface(),
                                                                        get_node_graph_interface(),
                                                                        get_node_logging_interface(),
                                                                        get_node_waitables_interface(),
                                                                        "/navigate_to_lift");

    navi_to_pose_goal_ = nav2_msgs::action::NavigateToPose::Goal();
    navi_to_pose_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(get_node_base_interface(),
                                                                        get_node_graph_interface(),
                                                                        get_node_logging_interface(),
                                                                        get_node_waitables_interface(),
                                                                        "/navigate_to_pose");

    navi_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();
    navi_through_poses_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(get_node_base_interface(),
                                                                              get_node_graph_interface(),
                                                                              get_node_logging_interface(),
                                                                              get_node_waitables_interface(),
                                                                              "/navigate_through_poses");

    follow_waypoints_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
    follow_waypoints_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(get_node_base_interface(),
                                                                         get_node_graph_interface(),
                                                                         get_node_logging_interface(),
                                                                         get_node_waitables_interface(),
                                                                         "/follow_waypoints");
}

void TaskManager::publish_init()
{
    navi_type_pub_ = this->create_publisher<std_msgs::msg::Int32>("/navi_type",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    visual_drop_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>("/VisualDropSwitch",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void TaskManager::subscription_init()
{
    waypoints_wait_resume_sub_ = this->create_subscription<cm_msgs::msg::WaitResumeFeedback>(
        "/waypoint_follower/wait_resume_feedback", 1, std::bind(&TaskManager::waypoints_wait_resume_callback, this, std::placeholders::_1));
}

void TaskManager::parameter_init()
{
    this->declare_parameter<bool>("main_drop_switch", main_drop_switch_);
    this->get_parameter("main_drop_switch", main_drop_switch_);
    ROS_INFO("ParameterInit main_drop_switch: %d ", main_drop_switch_);

      // Setup callback for changes to parameters.
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(get_node_base_interface(),
                                                                         get_node_topics_interface(),
                                                                         get_node_graph_interface(),
                                                                         get_node_services_interface());

    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&TaskManager::on_parameter_event_callback, this, std::placeholders::_1));
}

void TaskManager::on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (auto & changed_parameter : event->changed_parameters) {
        const auto & type = changed_parameter.value.type;
        const auto & name = changed_parameter.name;
        const auto & value = changed_parameter.value;
        ROS_INFO("on_parameter_event_callback  name: %s ", name.c_str());  

        if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
        //   if (name == plugin_name_ + ".minimum_turning_radius") {
        //     smoother_params_.max_curvature = 1.0f / value.double_value;
        //   } else if (name == plugin_name_ + ".w_curve") {
        //     smoother_params_.curvature_weight = value.double_value;
        //   } else if (name == plugin_name_ + ".w_dist") {
        //     smoother_params_.distance_weight = value.double_value;
        //   } else if (name == plugin_name_ + ".w_smooth") {
        //     smoother_params_.smooth_weight = value.double_value;
        //   } else if (name == plugin_name_ + ".w_cost") {
        //     smoother_params_.costmap_weight = value.double_value;
        //   } else if (name == plugin_name_ + ".w_cost_cusp_multiplier") {
        //     smoother_params_.cusp_costmap_weight = value.double_value * smoother_params_.costmap_weight;
        //   } else if (name == plugin_name_ + ".cusp_zone_length") {
        //     smoother_params_.cusp_zone_length = value.double_value;
        //   }
        } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
            if (name == "main_drop_switch") {
                main_drop_switch_ = value.bool_value;
                ROS_INFO("on_parameter_event_callback  main_drop_switch_: %d ", main_drop_switch_);
            } 
        } else if (type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        //   if (name == plugin_name_ + ".path_downsampling_factor") {
        //     smoother_params_.path_downsampling_factor = value.integer_value;
        //   } else if (name == plugin_name_ + ".path_upsampling_factor") {
        //     smoother_params_.path_upsampling_factor = value.integer_value;
        //   }
        }

    }
}

void TaskManager::waypoints_wait_resume_callback(const cm_msgs::msg::WaitResumeFeedback::SharedPtr msg)
{
    int state = msg->state;
    if(waypoints_wait_resume_state_ != state)
    {
        waypoints_wait_resume_state_ = state;
        ROS_INFO("TaskManager::waypoints_wait_resume_callback state: %d, current_waypoint_index: %d, current_pose: {%.2f, %.2f}",
            state, msg->current_waypoint_index,
            msg->current_pose.pose.position.x,
            msg->current_pose.pose.position.y);

        if(follow_waypoints_handle_)
        {
            NaviStateInfo navi_state_info;
            if(state == cm_msgs::msg::WaitResumeFeedback::WAITING)
            {
                feed_back_nav_state_ = false;

                navi_state_info.navi_state = NAVISTATE::NAVI_STATE_PAUSE;
                publishNavState(navi_state_info);
            }
            else
            {
                navi_state_info.navi_state = NAVISTATE::NAVI_STATE_RUNNING;
                publishNavState(navi_state_info);

                feed_back_nav_state_ = true;
            }
        }
    }
}

void TaskManager::registNavStateCallback(const NavStateFun &nav_cb)
{
    nav_state_cb_ = std::move(nav_cb);
}

void TaskManager::publishNavState(const NaviStateInfo navi_state_info)
{
    nav_state_cb_(navi_state_info);
}

void TaskManager::changeMsgPoseToStructPose(const geometry_msgs::msg::Pose &msg_pose, Pose &pose)
{
    pose.position.x = msg_pose.position.x;
    pose.position.y = msg_pose.position.y;
    pose.position.z = msg_pose.position.z;
    pose.orientation.x = msg_pose.orientation.x;
    pose.orientation.y = msg_pose.orientation.y;
    pose.orientation.z = msg_pose.orientation.z;
    pose.orientation.w = msg_pose.orientation.w;
}

bool TaskManager::naviLiftImpl(const std::string &task_id,
                                 const ENAVITYPE &navi_type,
                                 const geometry_msgs::msg::PoseStamped &pose)
{
    ROS_INFO("TaskManager::naviLiftImpl task_id:%s, navi_type:%d", task_id.c_str(), (int)navi_type);
    switchVisualDrop(false);
    publishNaviType(navi_type);
    switchGoalOrientation(navi_type);

    auto is_action_server_ready = navi_lift_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        ROS_INFO("TaskManager::naviLiftImpl navigate_to_lift action server is not available.");
        return false;
    }

    navi_lift_goal_.pose = std::move(pose);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&TaskManager::naviLiftFeedbackCallback,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2,
                                                    navi_lift_goal_);
    send_goal_options.goal_response_callback =
        std::bind(&TaskManager::naviLiftGoalRespCallback, this, std::placeholders::_1, navi_lift_goal_);
    send_goal_options.result_callback =
        std::bind(&TaskManager::naviLiftResultCallback, this, std::placeholders::_1, navi_lift_goal_);


    auto future_goal_handle = navi_lift_action_client_->async_send_goal(navi_lift_goal_, send_goal_options);
    ROS_INFO("TaskManager::naviLiftImpl entry exit");
    return true;
}

void TaskManager::naviLiftFeedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback,
    const nav2_msgs::action::NavigateToPose::Goal &goal)  // NOLINT
{
    double now_stamp = now().seconds();
    if (feed_back_nav_state_ && (now_stamp - last_feed_back_time_ > 2))
    {
        std::string goal_uuid_string;
        for (const auto &c : goal_handle->get_goal_id())
        {
            goal_uuid_string += std::to_string(c);
        }

        ROS_INFO("TaskManager::naviLiftFeedbackCallback Goal id \"%s\" RUNNING ...", goal_uuid_string.c_str());
        ROS_INFO(
            "TaskManager::naviLiftFeedbackCallback Current pose: {%s, %s} --> Goal pose {%s, %s}, ETA: %s, Distance remaining: "
            "%s, Time taken: %s, Recoveries: %s",
            toString(feedback->current_pose.pose.position.x, 2).c_str(),
            toString(feedback->current_pose.pose.position.y, 2).c_str(),
            toString(goal.pose.pose.position.x, 2).c_str(),
            toString(goal.pose.pose.position.y, 2).c_str(),
            toString(rclcpp::Duration(feedback->estimated_time_remaining).seconds(), 2).c_str(),
            toString(feedback->distance_remaining, 2).c_str(),
            toString(rclcpp::Duration(feedback->navigation_time).seconds(), 2).c_str(),
            toString(feedback->number_of_recoveries).c_str());



        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_RUNNING;
        navi_state_info.navigation_time = rclcpp::Duration(feedback->navigation_time).seconds();
        navi_state_info.estimated_time_remaining = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        navi_state_info.distance_remaining = feedback->distance_remaining;
        navi_state_info.recovery_count = feedback->number_of_recoveries;

        publishNavState(navi_state_info);
        last_feed_back_time_ = now_stamp;
    }
}

void TaskManager::naviLiftGoalRespCallback(
    const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> &future,
    const nav2_msgs::action::NavigateToPose::Goal &goal)
{
    NAVISTATE navi_state;
    navi_lift_handle_ = future.get();
    if (!navi_lift_handle_)
    {
        navi_state = NAVISTATE::NAVI_STATE_FAILED;
        ROS_ERROR(
            "TaskManager::naviLiftGoalRespCallback action client failed to send goal {%f, %f} use behavior_tree %s to server.",
            goal.pose.pose.position.x,
            goal.pose.pose.position.y,
            goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviLift FAILED!");
    }
    else
    {
        navi_state = NAVISTATE::NAVI_STATE_ACTIVE;
        ROS_INFO("TaskManager::naviLiftGoalRespCallback action client send goal {%f, %f} use behavior_tree %s to server success!",
                 goal.pose.pose.position.x,
                 goal.pose.pose.position.y,
                 goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviLift ACTIVE!");
    }

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    if(navi_state == NAVISTATE::NAVI_STATE_ACTIVE)
    {
        feed_back_nav_state_ = true;
    }
}

void TaskManager::naviLiftResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result,
    const nav2_msgs::action::NavigateToPose::Goal &goal)
{
    navi_lift_handle_.reset();
    std::string goal_uuid_string;
    for (const auto &c : result.goal_id)
    {
        goal_uuid_string += std::to_string(c);
    }
    ROS_INFO("TaskManager::naviLiftResultCallback goal id: \"%s\", pose: {%f, %f}, behavior_tree: %s, code: %d!",
             goal_uuid_string.c_str(),
             goal.pose.pose.position.x,
             goal.pose.pose.position.y,
             goal.behavior_tree.c_str(),
             (int)result.code);

    NAVISTATE navi_state;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::UNKNOWN:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviLiftResult UNKNOWN!");
            break;
        case rclcpp_action::ResultCode::SUCCEEDED:
            navi_state = NAVISTATE::NAVI_STATE_SUCCESS;
            ROS_INFO("TaskManager::naviLiftResult SUCCESS!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navi_state = NAVISTATE::NAVI_STATE_CANCELED;
            ROS_INFO("TaskManager::naviLiftResult CANCELED!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navi_state = NAVISTATE::NAVI_STATE_FAILED;
            ROS_INFO("TaskManager::naviLiftResult FAILED!");
            break;
        default:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviLiftResult FAILED!");
            break;
    }

    feed_back_nav_state_ = false;

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    switchVisualDrop(false);
}

ERESULT TaskManager::cancelNaviLift()
{
    if (navi_lift_handle_)
    {
        ROS_INFO("TaskManager::cancelNaviLift ...");
        auto future_cancel = navi_lift_action_client_->async_cancel_goal(navi_lift_handle_);
        navi_lift_handle_.reset();

        feed_back_nav_state_ = false;
        return E_STOPNAVI_SUCCESS;
    }
    else
    {
        ROS_WARN("TaskManager::cancelNaviLift no NaviLift running ...");
        return E_NAVICMD_NO_ACTIVE_TASK;
    }
}

bool TaskManager::naviToPoseImpl(const std::string &task_id,
                                 const ENAVITYPE &navi_type,
                                 const geometry_msgs::msg::PoseStamped &pose)
{
    ROS_INFO("TaskManager::naviToPoseImpl task_id:%s, navi_type:%d", task_id.c_str(), (int)navi_type);
    switchVisualDrop(main_drop_switch_);
    publishNaviType(navi_type);
    switchGoalOrientation(navi_type);

    auto is_action_server_ready = navi_to_pose_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        ROS_INFO("TaskManager::naviToPoseImpl navigate_to_pose action server is not available.");
        return false;
    }

    navi_to_pose_goal_.pose = std::move(pose);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&TaskManager::naviToPoseFeedbackCallback,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2,
                                                    navi_to_pose_goal_);
    send_goal_options.goal_response_callback =
        std::bind(&TaskManager::naviToPoseGoalRespCallback, this, std::placeholders::_1, navi_to_pose_goal_);
    send_goal_options.result_callback =
        std::bind(&TaskManager::naviToPoseResultCallback, this, std::placeholders::_1, navi_to_pose_goal_);


    auto future_goal_handle = navi_to_pose_action_client_->async_send_goal(navi_to_pose_goal_, send_goal_options);
    ROS_INFO("TaskManager::naviToPoseImpl entry exit");
    return true;
}

void TaskManager::naviToPoseFeedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback,
    const nav2_msgs::action::NavigateToPose::Goal &goal)  // NOLINT
{
    double now_stamp = now().seconds();
    if (feed_back_nav_state_ && (now_stamp - last_feed_back_time_ > 2))
    {
        std::string goal_uuid_string;
        for (const auto &c : goal_handle->get_goal_id())
        {
            goal_uuid_string += std::to_string(c);
        }

        ROS_INFO("TaskManager::naviToPoseFeedbackCallback Goal id \"%s\" RUNNING ...", goal_uuid_string.c_str());
        ROS_INFO(
            "TaskManager::naviToPoseFeedbackCallback Current pose: {%s, %s} --> Goal pose {%s, %s}, ETA: %s, Distance remaining: "
            "%s, Time taken: %s, Recoveries: %s",
            toString(feedback->current_pose.pose.position.x, 2).c_str(),
            toString(feedback->current_pose.pose.position.y, 2).c_str(),
            toString(goal.pose.pose.position.x, 2).c_str(),
            toString(goal.pose.pose.position.y, 2).c_str(),
            toString(rclcpp::Duration(feedback->estimated_time_remaining).seconds(), 2).c_str(),
            toString(feedback->distance_remaining, 2).c_str(),
            toString(rclcpp::Duration(feedback->navigation_time).seconds(), 2).c_str(),
            toString(feedback->number_of_recoveries).c_str());


        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_RUNNING;
        navi_state_info.navigation_time = rclcpp::Duration(feedback->navigation_time).seconds();
        navi_state_info.estimated_time_remaining = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        navi_state_info.distance_remaining = feedback->distance_remaining;
        navi_state_info.recovery_count = feedback->number_of_recoveries;

        publishNavState(navi_state_info);
        last_feed_back_time_ = now_stamp;
    }
}

void TaskManager::naviToPoseGoalRespCallback(
    const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> &future,
    const nav2_msgs::action::NavigateToPose::Goal &goal)
{
    NAVISTATE navi_state;
    navi_to_pose_goal_handle_ = future.get();
    if (!navi_to_pose_goal_handle_)
    {
        navi_state = NAVISTATE::NAVI_STATE_FAILED;
        ROS_ERROR(
            "TaskManager::naviToPoseGoalRespCallback action client failed to send goal {%f, %f} use behavior_tree %s to server.",
            goal.pose.pose.position.x,
            goal.pose.pose.position.y,
            goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviToPose FAILED!");
    }
    else
    {
        navi_state = NAVISTATE::NAVI_STATE_ACTIVE;
        ROS_INFO("TaskManager::naviToPoseGoalRespCallback action client send goal {%f, %f} use behavior_tree %s to server success!",
                 goal.pose.pose.position.x,
                 goal.pose.pose.position.y,
                 goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviToPose ACTIVE!");
    }

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    if(navi_state == NAVISTATE::NAVI_STATE_ACTIVE)
    {
        feed_back_nav_state_ = true;
    }
}

void TaskManager::naviToPoseResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result,
    const nav2_msgs::action::NavigateToPose::Goal &goal)
{
    navi_to_pose_goal_handle_.reset();
    std::string goal_uuid_string;
    for (const auto &c : result.goal_id)
    {
        goal_uuid_string += std::to_string(c);
    }
    ROS_INFO("TaskManager::naviToPoseResultCallback goal id: \"%s\", pose: {%f, %f}, behavior_tree: %s, code: %d!",
             goal_uuid_string.c_str(),
             goal.pose.pose.position.x,
             goal.pose.pose.position.y,
             goal.behavior_tree.c_str(),
             (int)result.code);

    NAVISTATE navi_state;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::UNKNOWN:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviToPoseResult UNKNOWN!");
            break;
        case rclcpp_action::ResultCode::SUCCEEDED:
            navi_state = NAVISTATE::NAVI_STATE_SUCCESS;
            ROS_INFO("TaskManager::naviToPoseResult SUCCESS!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navi_state = NAVISTATE::NAVI_STATE_CANCELED;
            ROS_INFO("TaskManager::naviToPoseResult CANCELED!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navi_state = NAVISTATE::NAVI_STATE_FAILED;
            ROS_INFO("TaskManager::naviToPoseResult FAILED!");
            break;
        default:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviToPoseResult FAILED!");
            break;
    }

    feed_back_nav_state_ = false;

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    switchVisualDrop(false);
}

ERESULT TaskManager::cancelNaviToPose()
{
    if (navi_to_pose_goal_handle_)
    {
        ROS_INFO("TaskManager::cancelNaviToPose ...");
        auto future_cancel = navi_to_pose_action_client_->async_cancel_goal(navi_to_pose_goal_handle_);
        navi_to_pose_goal_handle_.reset();

        feed_back_nav_state_ = false;
        return E_STOPNAVI_SUCCESS;
    }
    else
    {
        ROS_WARN("TaskManager::cancelNaviToPose no NaviToPose running ...");
        return E_NAVICMD_NO_ACTIVE_TASK;
    }
}

bool TaskManager::naviThroughPosesImpl(const std::string &task_id,
                                       const ENAVITYPE &navi_type,
                                       const std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    ROS_INFO("TaskManager::naviThroughPosesImpl task_id:%s, navi_type:%d", task_id.c_str(), (int)navi_type);
    switchVisualDrop(main_drop_switch_);
    naviCmdControl(true);
    publishNaviType(navi_type);
    switchGoalOrientation(navi_type);

    auto is_action_server_ready = navi_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        ROS_ERROR("TaskManager::naviThroughPosesImpl navigate_through_poses action server is not available.");
        return false;
    }

    navi_through_poses_goal_.poses = poses;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

    send_goal_options.feedback_callback = std::bind(&TaskManager::naviThroughPosesFeedbackCallback,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2,
                                                    navi_through_poses_goal_);
    send_goal_options.goal_response_callback = std::bind(
        &TaskManager::naviThroughPosesGoalRespCallback, this, std::placeholders::_1, navi_through_poses_goal_);
    send_goal_options.result_callback =
        std::bind(&TaskManager::naviThroughPosesResultCallback, this, std::placeholders::_1, navi_through_poses_goal_);


    auto future_goal_handle =
        navi_through_poses_action_client_->async_send_goal(navi_through_poses_goal_, send_goal_options);
    ROS_INFO("TaskManager::naviThroughPosesImpl entry exit");
    return true;
}

void TaskManager::naviThroughPosesFeedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback,
    const nav2_msgs::action::NavigateThroughPoses::Goal &goal)
{
    double now_stamp = now().seconds();
    if (feed_back_nav_state_ && (now_stamp - last_feed_back_time_ > 2))
    {
        std::string goal_uuid_string;
        for (const auto &c : goal_handle->get_goal_id())
        {
            goal_uuid_string += std::to_string(c);
        }

        ROS_INFO("TaskManager::naviThroughPosesFeedback Goal id \"%s\" RUNNING ...", goal_uuid_string.c_str());
        ROS_INFO(
            "TaskManager::naviThroughPosesFeedback Current pose: {%s, %s}, ETA: %s, Distance remaining: %s, Time taken: %s, "
            "Recoveries: %s, poses nums: %s",
            toString(feedback->current_pose.pose.position.x, 2).c_str(),
            toString(feedback->current_pose.pose.position.y, 2).c_str(),
            toString(rclcpp::Duration(feedback->estimated_time_remaining).seconds(), 2).c_str(),
            toString(feedback->distance_remaining, 2).c_str(),
            toString(rclcpp::Duration(feedback->navigation_time).seconds(), 2).c_str(),
            toString(feedback->number_of_recoveries).c_str(),
            toString(feedback->number_of_poses_remaining).c_str());


        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_RUNNING;
        navi_state_info.navigation_time = rclcpp::Duration(feedback->navigation_time).seconds();
        navi_state_info.estimated_time_remaining = rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        navi_state_info.distance_remaining = feedback->distance_remaining;
        navi_state_info.recovery_count = feedback->number_of_recoveries;
        navi_state_info.number_of_poses_remaining = feedback->number_of_poses_remaining;
        publishNavState(navi_state_info);

        last_feed_back_time_ = now_stamp;
    }
}

void TaskManager::naviThroughPosesGoalRespCallback(
    const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr>
        &future,
    const nav2_msgs::action::NavigateThroughPoses::Goal &goal)
{
    NAVISTATE navi_state;
    std::string description;
    bool active;
    navi_through_poses_handle_ = future.get();
    if (!navi_through_poses_handle_)
    {
        navi_state = NAVISTATE::NAVI_STATE_FAILED;
        description = "naviThroughPosesGoalResp failed";
        active = false;
        ROS_ERROR("TaskManager::naviThroughPosesGoalRespCallback action client failed, use behavior_tree %s to server.",
                  goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviThroughPoses FAILED!");
    }
    else
    {
        navi_state = NAVISTATE::NAVI_STATE_ACTIVE;
        description = "naviThroughPosesGoalResp active";
        active = true;
        ROS_INFO("TaskManager::naviThroughPosesGoalRespCallback action client active, use behavior_tree %s to server success!",
                 goal.behavior_tree.c_str());
        ROS_ERROR("TaskManager::naviThroughPoses ACTIVE!");
    }

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    if(navi_state == NAVISTATE::NAVI_STATE_ACTIVE)
    {
        feed_back_nav_state_ = true;
    }
}

void TaskManager::naviThroughPosesResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result,
    const nav2_msgs::action::NavigateThroughPoses::Goal &goal)
{
    navi_through_poses_handle_.reset();
    std::string goal_uuid_string;
    for (const auto &c : result.goal_id)
    {
        goal_uuid_string += std::to_string(c);
    }
    ROS_INFO("TaskManager::naviThroughPosesResultCallback goal id: \"%s\", behavior_tree: %s, code: %d!",
             goal_uuid_string.c_str(),
             goal.behavior_tree.c_str(),
             (int)result.code);

    NAVISTATE navi_state;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::UNKNOWN:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviThroughPosesResult UNKNOWN!");
            break;
        case rclcpp_action::ResultCode::SUCCEEDED:
            navi_state = NAVISTATE::NAVI_STATE_SUCCESS;
            ROS_INFO("TaskManager::naviThroughPosesResult SUCCESS!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navi_state = NAVISTATE::NAVI_STATE_CANCELED;
            ROS_INFO("TaskManager::naviThroughPosesResult CANCELED!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navi_state = NAVISTATE::NAVI_STATE_FAILED;
            ROS_INFO("TaskManager::naviThroughPosesResult FAILED!");
            break;
        default:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::naviThroughPosesResult FAILED!");
            break;
    }

    feed_back_nav_state_ = false;

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    switchVisualDrop(false);
}

ERESULT TaskManager::cancelNaviThroughPoses()
{
    if (navi_through_poses_handle_)
    {
        ROS_INFO("TaskManager::cancelNaviThroughPoses ...");
        auto future_cancel = navi_through_poses_action_client_->async_cancel_goal(navi_through_poses_handle_);
        navi_through_poses_handle_.reset();
        feed_back_nav_state_ = false;
        return E_STOPNAVI_SUCCESS;
    }
    else
    {
        ROS_WARN("TaskManager::cancelNaviThroughPoses No NaviThroughPoses running ...");
        return E_NAVICMD_NO_ACTIVE_TASK;
    }
}

bool TaskManager::followWaypointsImpl(const std::string &task_id,
                                      const ENAVITYPE &navi_type,
                                      const std::vector<geometry_msgs::msg::PoseStamped> &poses,
                                      const std::vector<uint8_t> &poses_type)
{
    ROS_INFO("TaskManager::followWaypointsImpl task_id:%s, navi_type:%d, poses.size:%ld, poses_type.size:%ld",
              task_id.c_str(), (int)navi_type, poses.size(), poses_type.size());
    waypoints_wait_resume_state_ = 0;
    switchVisualDrop(main_drop_switch_);
    naviCmdControl(true);
    publishNaviType(navi_type);
    switchGoalOrientation(navi_type);

    auto is_action_server_ready = follow_waypoints_action_client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        ROS_ERROR("TaskManager::followWaypointsImpl follow_waypoints action server is not available.");
        return false;
    }

    follow_waypoints_goal_.poses = std::move(poses);
    follow_waypoints_goal_.types = std::move(poses_type);

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();

    send_goal_options.feedback_callback = std::bind(&TaskManager::followWaypointsFeedbackCallback,
                                                    this,
                                                    std::placeholders::_1,
                                                    std::placeholders::_2,
                                                    follow_waypoints_goal_);
    send_goal_options.goal_response_callback = std::bind(
        &TaskManager::followWaypointsGoalRespCallback, this, std::placeholders::_1, follow_waypoints_goal_);
    send_goal_options.result_callback =
        std::bind(&TaskManager::followWaypointsResultCallback, this, std::placeholders::_1, follow_waypoints_goal_);


    auto future_goal_handle =
        follow_waypoints_action_client_->async_send_goal(follow_waypoints_goal_, send_goal_options);
    ROS_INFO("TaskManager::followWaypointsImpl entry exit");
    return true;
}

void TaskManager::followWaypointsFeedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback,
    const nav2_msgs::action::FollowWaypoints::Goal &goal)
{
    double now_stamp = now().seconds();
    if (feed_back_nav_state_ && (now_stamp - last_feed_back_time_ > 2))
    {
        std::string goal_uuid_string;
        for (const auto &c : goal_handle->get_goal_id())
        {
            goal_uuid_string += std::to_string(c);
        }

        ROS_INFO("TaskManager::followWaypointsFeedbackCallback Goal id \"%s\" RUNNING ...", goal_uuid_string.c_str());
        ROS_INFO("TaskManager::followWaypointsFeedbackCallback Current waypoint: %d --> Goal pose {%s, %s}",
              feedback->current_waypoint,
              toString(goal.poses.back().pose.position.x, 2).c_str(),
              toString(goal.poses.back().pose.position.y, 2).c_str());


        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_RUNNING;
        publishNavState(navi_state_info);

        last_feed_back_time_ = now_stamp;
    }
}

void TaskManager::followWaypointsGoalRespCallback(
    const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr>
        &future,
    const nav2_msgs::action::FollowWaypoints::Goal &goal)
{
    NAVISTATE navi_state;
    std::string description;
    bool active;
    follow_waypoints_handle_ = future.get();
    if (!follow_waypoints_handle_)
    {
        navi_state = NAVISTATE::NAVI_STATE_FAILED;
        description = "followWaypointsGoalResp failed";
        active = false;
        ROS_ERROR("TaskManager::followWaypointsGoalRespCallback action client send goal {%f, %f} to server success!",
                  goal.poses.back().pose.position.x, goal.poses.back().pose.position.y);
        ROS_ERROR("TaskManager::followWaypoints FAILED!");
    }
    else
    {
        navi_state = NAVISTATE::NAVI_STATE_ACTIVE;
        description = "followWaypointsGoalResp active";
        active = true;
        ROS_INFO("TaskManager::followWaypointsGoalRespCallback action client send goal {%f, %f} to server success!",
                  goal.poses.back().pose.position.x, goal.poses.back().pose.position.y);
        ROS_ERROR("TaskManager::followWaypoints ACTIVE!");
    }

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    if(navi_state == NAVISTATE::NAVI_STATE_ACTIVE)
    {
        feed_back_nav_state_ = true;
    }
}

void TaskManager::followWaypointsResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult &result,
    const nav2_msgs::action::FollowWaypoints::Goal &goal)
{
    follow_waypoints_handle_.reset();
    std::string goal_uuid_string;
    for (const auto &c : result.goal_id)
    {
        goal_uuid_string += std::to_string(c);
    }
    ROS_INFO("TaskManager::followWaypointsResultCallback goal id: \"%s\", pose: {%f, %f}, code: %d!",
             goal_uuid_string.c_str(),
             goal.poses.back().pose.position.x,
             goal.poses.back().pose.position.y,
             (int)result.code);

    NAVISTATE navi_state;
    switch (result.code)
    {
        case rclcpp_action::ResultCode::UNKNOWN:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::followWaypointsResult UNKNOWN!");
            break;
        case rclcpp_action::ResultCode::SUCCEEDED:
            navi_state = NAVISTATE::NAVI_STATE_SUCCESS;
            ROS_INFO("TaskManager::followWaypointsResult SUCCESS!");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            navi_state = NAVISTATE::NAVI_STATE_CANCELED;
            ROS_INFO("TaskManager::followWaypointsResult CANCELED!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            navi_state = NAVISTATE::NAVI_STATE_FAILED;
            ROS_INFO("TaskManager::followWaypointsResult FAILED!");
            break;
        default:
            navi_state = NAVISTATE::NAVI_STATE_IDLE;
            ROS_WARN("TaskManager::followWaypointsResult FAILED!");
            break;
    }

    feed_back_nav_state_ = false;

    NaviStateInfo navi_state_info;
    navi_state_info.navi_state = navi_state;
    publishNavState(navi_state_info);

    switchVisualDrop(false);
}

ERESULT TaskManager::cancelFollowWaypoints()
{
    if (follow_waypoints_handle_)
    {
        ROS_INFO("TaskManager::cancelFollowWaypoints ...");
        auto future_cancel = follow_waypoints_action_client_->async_cancel_goal(follow_waypoints_handle_);
        follow_waypoints_handle_.reset();
        feed_back_nav_state_ = false;
        return E_STOPNAVI_SUCCESS;
    }
    else
    {
        ROS_WARN("TaskManager::cancelFollowWaypoints No FollowWaypoints running ...");
        return E_NAVICMD_NO_ACTIVE_TASK;
    }
}

ERESULT TaskManager::pauseNaviTask()
{
    ERESULT navi_cmd_result = E_NAVICMD_FAILED;
    if (navi_to_pose_goal_handle_ || navi_through_poses_handle_ || navi_lift_handle_ || follow_waypoints_handle_)
    {
        ROS_INFO("TaskManager::pauseNaviTask ...");
        if(naviCmdControl(true))
        {
            navi_cmd_result = E_NAVICMD_SUCCESS;
            feed_back_nav_state_ = false;
        }
        else
        {
            navi_cmd_result = E_NAVICMD_FAILED;
        }
    }
    else
    {
        ROS_WARN("TaskManager::pauseNaviTask no task running");
        navi_cmd_result = E_NAVICMD_NO_ACTIVE_TASK;
    }

    if(navi_cmd_result == E_NAVICMD_SUCCESS)
    {
        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_PAUSE;
        publishNavState(navi_state_info);
    }

    return navi_cmd_result;
}

ERESULT TaskManager::resumeNaviTask()
{
    ERESULT navi_cmd_result = E_NAVICMD_FAILED;
    if (navi_to_pose_goal_handle_ || navi_through_poses_handle_ || navi_lift_handle_ || follow_waypoints_handle_)
    {
        ROS_INFO("TaskManager::resumeNaviTask ...");
        if(naviCmdControl(false))
        {
            navi_cmd_result = E_NAVICMD_SUCCESS;
        }
        else
        {
            navi_cmd_result = E_NAVICMD_FAILED;
        }
    }
    else
    {
        ROS_WARN("TaskManager::resumeNaviTask no task paused");
        navi_cmd_result = E_NAVICMD_NO_PAUSED_TASK;
    }

    if(navi_cmd_result == E_NAVICMD_SUCCESS)
    {
        NaviStateInfo navi_state_info;
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_ACTIVE;
        publishNavState(navi_state_info);

        feed_back_nav_state_ = true;
    }

    return navi_cmd_result;
}

bool TaskManager::naviCmdControl(bool cmd_pause)
{
    bool result = false;
    if (navi_to_pose_goal_handle_ || navi_through_poses_handle_ || navi_lift_handle_ || follow_waypoints_handle_)
    {
        auto navi_pause_control_req = std::make_shared<std_srvs::srv::SetBool::Request>();
        navi_pause_control_req->data = cmd_pause;

        auto navi_pause_control_res = std::make_shared<std_srvs::srv::SetBool::Response>();
        if (navi_pause_control_client_->call(navi_pause_control_req, navi_pause_control_res))
        {
            result = true;
        }
        else
        {
            result = false;
        }
        ROS_INFO("TaskManager::naviCmdControl cmd_pause:%d, result:%d", cmd_pause, result);
    }

    return result;
}

void TaskManager::publishNaviType(const ENAVITYPE &navi_type)
{
    ROS_INFO("TaskManager::publishNaviType navi_type:%d", navi_type);
    std_msgs::msg::Int32 nav_type_data;
    nav_type_data.data = navi_type == ENAVITYPE::NAVI_TEMP ? int(ENAVITYPE::NAVI_NORMAL) : int(navi_type);
    navi_type_pub_->publish(nav_type_data);
}

void TaskManager::robotAlarmHandle(bool alarm_state)
{
    ROS_INFO("TaskManager::robotAlarmHandle alarm_state:%d", alarm_state);
    if (navi_to_pose_goal_handle_ || navi_through_poses_handle_ || navi_lift_handle_ || follow_waypoints_handle_)
    {
        if(alarm_state)
        {
            feed_back_nav_state_ = false;

            NaviStateInfo navi_state_info;
            navi_state_info.navi_state = NAVISTATE::NAVI_STATE_EMERGENCY;
            publishNavState(navi_state_info);

            pauseNaviTask();
        }
        else
        {
            resumeNaviTask();
        }
    }
}

void TaskManager::switchVisualDrop(bool flag)
{
    std_msgs::msg::Bool switch_data;
    switch_data.data = flag;
    visual_drop_switch_pub_->publish(switch_data);
    ROS_INFO("TaskManager::switchVisualDrop flag:%d", flag);
}


void TaskManager::switchGoalOrientation(const ENAVITYPE &navi_type)
{
    ROS_INFO("TaskManager::switchGoalOrientation navi_type:%d", navi_type);
    std::string node_name = std::string("/planner_server");
    std::vector<rclcpp::Parameter> v_params;
    v_params.clear();
    rclcpp::Parameter goal_orientation("GridBased.use_final_approach_orientation",
                                       rclcpp::ParameterValue(navi_type == NAVI_IGNORE_ANGLE));
    v_params.emplace_back(goal_orientation);

    updateParameters(node_name, v_params);
}

int TaskManager::updateParameters(std::string& destNodeName,
   std::vector<rclcpp::Parameter>& params)
{
    ROS_INFO("TaskManager::updateParameters nodeName:%s, params size : %lu",
            destNodeName.c_str(), params.size());
    auto param_client =
        std::make_shared<rclcpp::AsyncParametersClient>(this, destNodeName);
    //std::stringstream ss;

    while (!param_client->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            ROS_ERROR("TaskManager::updateParameters Interrupted while waiting for the service.");
            return 1;
        }
        ROS_WARN("TaskManager::updateParameters update %s params failed", destNodeName.c_str());
        return -1;
    }

    if(!param_client->service_is_ready())
    {
        ROS_WARN("TaskManager::updateParameters update %s params failed", destNodeName.c_str());
        return -2;
    }

    //ss.str("");
    //ss.clear(std::stringstream::goodbit);
    auto set_parameters_result = param_client->set_parameters(params);

    // for(auto &result : set_parameters_result.get())
    // {
    //     ROS_WARN("setting params");
    //     if(!result.successful){
    //         // 如果设置的字段不存在，会引发此错误
    //         ROS_ERROR("Failed to set parameter: %s", result.reason.c_str());
    //     }
    // }

    auto res = set_parameters_result.wait_for(std::chrono::milliseconds(30));

    ROS_WARN("TaskManager::updateParameters update %s params successful", destNodeName.c_str());
    return 0;
}

}  // namespace naviengine