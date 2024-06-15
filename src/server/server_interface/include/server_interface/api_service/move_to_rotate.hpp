#ifndef MOVE_TO_ROTATE_HPP_
#define MOVE_TO_ROTATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/move_to.hpp"
#include "system_manager/robot_naviengine.h"


class MoveToRotateService {
  public:
    MoveToRotateService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    MoveToRotateService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::MoveTo::Request::SharedPtr request,
                        sps_common_msgs::srv::MoveTo::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::MoveTo>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif