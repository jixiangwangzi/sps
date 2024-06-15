#include "server_interface/api_service/switch_sensor_data.hpp"

SwitchSensorDataService::SwitchSensorDataService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine) : engine_(engine)
{
    server_ = node.create_service<sps_common_msgs::srv::SwitchSensor>("/switch_sensor_data",
                                             std::bind(&SwitchSensorDataService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SwitchSensorDataService::ServiceCallback(sps_common_msgs::srv::SwitchSensor::Request::SharedPtr request,
                            sps_common_msgs::srv::SwitchSensor::Response::SharedPtr response)
{
    ROS_INFO("charging_to_dock_service header.stamp: %d", request->header.stamp.sec);
    ERESULT res = engine_.SwitchSensorData((SENSORTYPE)request->sensor_type, request->state);
    response->result = res;
}

