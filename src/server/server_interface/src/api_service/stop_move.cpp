#include "server_interface/api_service/stop_move.hpp"

StopMoveService::StopMoveService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::CmdCtrl>("/stop_move",
                                             std::bind(&StopMoveService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void StopMoveService::ServiceCallback(sps_common_msgs::srv::CmdCtrl::Request::SharedPtr request,
                            sps_common_msgs::srv::CmdCtrl::Response::SharedPtr response) 
{
    ROS_INFO("stop_move_service header.stamp: %d", request->header.stamp.sec);
    ERESULT res = engine_.StopMove();
    response->result = 100;
}

