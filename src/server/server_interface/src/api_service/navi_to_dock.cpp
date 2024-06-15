#include "server_interface/api_service/navi_to_dock.hpp"

NaviToDockService::NaviToDockService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::ChargingToDock>("/task_navi_to_dock",
                                             std::bind(&NaviToDockService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviToDockService::ServiceCallback(sps_common_msgs::srv::ChargingToDock::Request::SharedPtr request,
                            sps_common_msgs::srv::ChargingToDock::Response::SharedPtr response) {
    ROS_INFO("task_navi_to_dock_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

