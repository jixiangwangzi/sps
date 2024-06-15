#include "server_interface/api_service/cancel_mapping.hpp"

CancelMappingService::CancelMappingService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::CmdCtrl>("/cancel_mapping",
                                             std::bind(&CancelMappingService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void CancelMappingService::ServiceCallback(sps_common_msgs::srv::CmdCtrl::Request::SharedPtr request,
                            sps_common_msgs::srv::CmdCtrl::Response::SharedPtr response) {
    ROS_INFO("cancel_mapping_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

