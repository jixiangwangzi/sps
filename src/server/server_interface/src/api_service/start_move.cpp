#include "server_interface/api_service/start_move.hpp"

StartMoveService::StartMoveService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::StartMove>("/start_move",
                                             std::bind(&StartMoveService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void StartMoveService::ServiceCallback(sps_common_msgs::srv::StartMove::Request::SharedPtr request,
                            sps_common_msgs::srv::StartMove::Response::SharedPtr response) 
{
    ROS_INFO("charging_to_dock_service header.stamp: %d", request->header.stamp.sec);
    Twist twist;
    twist.vx = request->twist.linear.x;
    twist.vy =request->twist.linear.y;
    twist.vz =request->twist.angular.z;
    ERESULT res = engine_.StartMove(twist, request->collision_check);
    response->result = res;
}

