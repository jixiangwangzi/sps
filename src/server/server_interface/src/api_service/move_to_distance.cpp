#include "server_interface/api_service/move_to_distance.hpp"

MoveToDistanceService::MoveToDistanceService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::MoveTo>("/move_to_distance",
                                             std::bind(&MoveToDistanceService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void MoveToDistanceService::ServiceCallback(sps_common_msgs::srv::MoveTo::Request::SharedPtr request,
                            sps_common_msgs::srv::MoveTo::Response::SharedPtr response) {
    ROS_INFO("move_to_distance_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

