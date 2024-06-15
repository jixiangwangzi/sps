#ifndef NAVI_THROUGH_POSES_HPP_
#define NAVI_THROUGH_POSES_HPP_


#include "rclcpp/rclcpp.hpp"
//#include "sps_common_msgs/srv/navi_poses.hpp"
#include "cm_msgs/srv/navi_poses.hpp"
#include "system_manager/robot_naviengine.h"


class NaviThroughPosesService
{
public:
    NaviThroughPosesService(rclcpp::Node& node,
                            rclcpp::CallbackGroup::SharedPtr group_service,
                            naviengine::RobotNaviEngine& engine);
    NaviThroughPosesService() = default;

private:
    void ServiceCallback(cm_msgs::srv::NaviPoses::Request::SharedPtr request,
                         cm_msgs::srv::NaviPoses::Response::SharedPtr response);


    void changeMsgPoseToStructPose(const geometry_msgs::msg::Pose& msg_pose, Pose& pose);

private:
    rclcpp::Service<cm_msgs::srv::NaviPoses>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;
};


#endif