#include "server_interface/api_service/set_rotate_initialize.hpp"

SetRotateInitializeService::SetRotateInitializeService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::SetPoseInitialize>("/set_rotate_initialize",
                                             std::bind(&SetRotateInitializeService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SetRotateInitializeService::ServiceCallback(sps_common_msgs::srv::SetPoseInitialize::Request::SharedPtr request,
                            sps_common_msgs::srv::SetPoseInitialize::Response::SharedPtr response) {
    ROS_INFO("set_rotate_initialize_service");
    Pose pose;
    ERESULT res = engine_.SetLocalization(pose, (SENSORMATCHTYPE) request->sensor_match_type, GLOBAL_POSE);
    response->result = res;
}

