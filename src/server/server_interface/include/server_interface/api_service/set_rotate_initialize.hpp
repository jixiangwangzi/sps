#ifndef SET_ROTATE_INITIALIZE_HPP_
#define SET_ROTATE_INITIALIZE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/set_pose_initialize.hpp"
#include "system_manager/robot_naviengine.h"


class SetRotateInitializeService {
  public:
    SetRotateInitializeService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetRotateInitializeService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SetPoseInitialize::Request::SharedPtr request,
                        sps_common_msgs::srv::SetPoseInitialize::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SetPoseInitialize>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif