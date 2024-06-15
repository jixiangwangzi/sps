#include "server_interface/api_service/charging_to_dock.hpp"

ChargingToDockService::ChargingToDockService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::ChargingToDock>("/charging_to_dock",
                                             std::bind(&ChargingToDockService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void ChargingToDockService::ServiceCallback(sps_common_msgs::srv::ChargingToDock::Request::SharedPtr request,
                            sps_common_msgs::srv::ChargingToDock::Response::SharedPtr response) {
    ROS_INFO("charging_to_dock_service header.stamp: %d", request->header.stamp.sec);
    ERESULT res = engine_.ChargingToDock(request->task_id, request->ctrl_cmd);
    response->result = res;
}

