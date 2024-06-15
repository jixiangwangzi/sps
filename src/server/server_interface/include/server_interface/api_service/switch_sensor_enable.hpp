#ifndef SWITCH_SENSOR_ENABLE_HPP_
#define SWITCH_SENSOR_ENABLE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/switch_sensor.hpp"
#include "system_manager/robot_naviengine.h"


class SwitchSensorEnableService {
  public:
    SwitchSensorEnableService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SwitchSensorEnableService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SwitchSensor::Request::SharedPtr request,
                        sps_common_msgs::srv::SwitchSensor::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SwitchSensor>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif