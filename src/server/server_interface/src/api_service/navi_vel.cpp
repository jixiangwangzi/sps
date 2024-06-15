#include "server_interface/api_service/navi_vel.hpp"

NaviVelService::NaviVelService(rclcpp::Node &node,
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::SpsNaviVel>("/sps_set_navi_vel",
                                             std::bind(&NaviVelService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviVelService::ServiceCallback(cm_msgs::srv::SpsNaviVel::Request::SharedPtr request,
                            cm_msgs::srv::SpsNaviVel::Response::SharedPtr response)
{
    ROS_INFO("navi_vel_service linear_max: %f, angular_max: %f", request->linear_max, request->angular_max);
    ERESULT res = engine_.SetMaxNaviVel(request->linear_max, request->angular_max);
    response->result = res;
    ROS_INFO("navi_vel_service result: %d", res);
}

