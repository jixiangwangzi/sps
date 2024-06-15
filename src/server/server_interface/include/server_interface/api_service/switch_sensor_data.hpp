#ifndef SWITCH_SENSOR_DATA_HPP_
#define SWITCH_SENSOR_DATA_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/switch_sensor.hpp"
#include "system_manager/robot_naviengine.h"


class SwitchSensorDataService {
  public:
    SwitchSensorDataService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SwitchSensorDataService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SwitchSensor::Request::SharedPtr request,
                        sps_common_msgs::srv::SwitchSensor::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SwitchSensor>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif