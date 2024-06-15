#ifndef NAVI_POSES_MILEAGE_HPP_
#define NAVI_POSES_MILEAGE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "cm_msgs/srv/get_poses_mileage.hpp"
#include "system_manager/robot_naviengine.h"


class NaviPosesMileageService
{
public:
    NaviPosesMileageService(rclcpp::Node& node,
                      rclcpp::CallbackGroup::SharedPtr group_service,
                      naviengine::RobotNaviEngine& engine);
    NaviPosesMileageService() = default;

private:
    void ServiceCallback(cm_msgs::srv::GetPosesMileage::Request::SharedPtr request,
                         cm_msgs::srv::GetPosesMileage::Response::SharedPtr response);


private:
    rclcpp::Service<cm_msgs::srv::GetPosesMileage>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;
};


#endif