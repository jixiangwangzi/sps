#ifndef CHARGING_TO_DOCK_HPP_
#define CHARGING_TO_DOCK_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/charging_to_dock.hpp"
#include "system_manager/robot_naviengine.h"


class ChargingToDockService {
  public:
    ChargingToDockService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    ChargingToDockService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::ChargingToDock::Request::SharedPtr request,
                        sps_common_msgs::srv::ChargingToDock::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::ChargingToDock>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif