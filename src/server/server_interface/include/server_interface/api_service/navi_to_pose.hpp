#ifndef NAVI_TO_POSE_HPP_
#define NAVI_TO_POSE_HPP_


#include "rclcpp/rclcpp.hpp"
//#include "sps_common_msgs/srv/navi_to_pose.hpp"
#include "cm_msgs/srv/navi_to_pose.hpp"
#include "system_manager/robot_naviengine.h"


class NaviToPoseService
{
public:
    NaviToPoseService(rclcpp::Node& node,
                      rclcpp::CallbackGroup::SharedPtr group_service,
                      naviengine::RobotNaviEngine& engine);
    NaviToPoseService() = default;

private:
    void ServiceCallback(cm_msgs::srv::NaviToPose::Request::SharedPtr request,
                         cm_msgs::srv::NaviToPose::Response::SharedPtr response);


private:
    rclcpp::Service<cm_msgs::srv::NaviToPose>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;
};


#endif