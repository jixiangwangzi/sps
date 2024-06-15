#ifndef TASK_MANAGER_HPP_
#define TASK_MANAGER_HPP_

#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sps_common_msgs/msg/battery_state.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"
#include "service_client/navi_pause_control.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "cm_msgs/msg/wait_resume_feedback.hpp"
#include "system_manager/define.h"

using NaviLiftHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NaviToPoseGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
using NaviThroughPosesHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
using FollowWaypointsHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;

namespace naviengine
{

class TaskManager : public rclcpp::Node
{
public:
    TaskManager();
    ~TaskManager();

    static inline std::string toString(double val, int precision = 0);

    void registNavStateCallback(const NavStateFun &nav_cb);

    void publishNavState(const NaviStateInfo navi_state_info);

    bool naviLiftImpl(const std::string &task_id,
                        const ENAVITYPE &navi_type,
                        const geometry_msgs::msg::PoseStamped &pose);
    ERESULT cancelNaviLift();

    bool naviToPoseImpl(const std::string &task_id,
                        const ENAVITYPE &navi_type,
                        const geometry_msgs::msg::PoseStamped &pose);
    ERESULT cancelNaviToPose();

    bool naviThroughPosesImpl(const std::string &task_id,
                              const ENAVITYPE &navi_type,
                              const std::vector<geometry_msgs::msg::PoseStamped> &poses);
    ERESULT cancelNaviThroughPoses();

    bool followWaypointsImpl(const std::string &task_id,
                             const ENAVITYPE &navi_type,
                             const std::vector<geometry_msgs::msg::PoseStamped> &poses,
                             const std::vector<uint8_t> &poses_type);
    ERESULT cancelFollowWaypoints();

    ERESULT pauseNaviTask();

    ERESULT resumeNaviTask();

    void robotAlarmHandle(bool alarm_state);

private:
    double last_feed_back_time_ = 0.0;
    bool feed_back_nav_state_ = false;

    bool main_drop_switch_ = true;

    int waypoints_wait_resume_state_ = 0;

    NavStateFun nav_state_cb_;

    std::shared_ptr<NaviPauseControlServiceClient> navi_pause_control_client_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr navi_type_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_drop_switch_pub_;

    rclcpp::Subscription<cm_msgs::msg::WaitResumeFeedback>::SharedPtr waypoints_wait_resume_sub_;

    void client_init();
    void subscription_init();
    void action_client_init();
    void publish_init();
    void parameter_init();

    void waypoints_wait_resume_callback(const cm_msgs::msg::WaitResumeFeedback::SharedPtr event);

    void on_parameter_event_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    // Subscription for parameter change
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    NaviLiftHandle::SharedPtr navi_lift_handle_;
    nav2_msgs::action::NavigateToPose::Goal navi_lift_goal_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navi_lift_action_client_;

    NaviToPoseGoalHandle::SharedPtr navi_to_pose_goal_handle_;
    nav2_msgs::action::NavigateToPose::Goal navi_to_pose_goal_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navi_to_pose_action_client_;

    NaviThroughPosesHandle::SharedPtr navi_through_poses_handle_;
    nav2_msgs::action::NavigateThroughPoses::Goal navi_through_poses_goal_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr navi_through_poses_action_client_;

    FollowWaypointsHandle::SharedPtr follow_waypoints_handle_;
    nav2_msgs::action::FollowWaypoints::Goal follow_waypoints_goal_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr follow_waypoints_action_client_;

    void naviLiftFeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback,
        const nav2_msgs::action::NavigateToPose::Goal &goal);
    void naviLiftGoalRespCallback(
        const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> &future,
        const nav2_msgs::action::NavigateToPose::Goal &goal);
    void naviLiftResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result,
        const nav2_msgs::action::NavigateToPose::Goal &goal);

    void naviToPoseFeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback,
        const nav2_msgs::action::NavigateToPose::Goal &goal);
    void naviToPoseGoalRespCallback(
        const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> &future,
        const nav2_msgs::action::NavigateToPose::Goal &goal);
    void naviToPoseResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result,
        const nav2_msgs::action::NavigateToPose::Goal &goal);

    void naviThroughPosesFeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback,
        const nav2_msgs::action::NavigateThroughPoses::Goal &goal);
    void naviThroughPosesGoalRespCallback(
        const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr>
            &future,
        const nav2_msgs::action::NavigateThroughPoses::Goal &goal);
    void naviThroughPosesResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult &result,
        const nav2_msgs::action::NavigateThroughPoses::Goal &goal);

    void followWaypointsFeedbackCallback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle,
        const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback,
        const nav2_msgs::action::FollowWaypoints::Goal &goal);
    void followWaypointsGoalRespCallback(
        const std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr>
            &future,
        const nav2_msgs::action::FollowWaypoints::Goal &goal);
    void followWaypointsResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult &result,
        const nav2_msgs::action::FollowWaypoints::Goal &goal);

    void changeMsgPoseToStructPose(const geometry_msgs::msg::Pose &w_pose, Pose &cur_pose);
    bool naviCmdControl(bool cmd_pause);
    void publishNaviType(const ENAVITYPE &navi_type);
    void switchVisualDrop(bool flag);
    void switchGoalOrientation(const ENAVITYPE &navi_type);
    int updateParameters(std::string& destNodeName, std::vector<rclcpp::Parameter>& params);
};

}  // namespace naviengine

#endif
