#include "server_interface/api_service/save_map.hpp"

SaveMapService::SaveMapService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::SaveMap>("/save_map",
                                             std::bind(&SaveMapService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SaveMapService::ServiceCallback(sps_common_msgs::srv::SaveMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SaveMap::Response::SharedPtr response) {
    ROS_INFO("save_map_service");
    ROS_INFO("request.name %s", request->full_name.c_str());
    char task_id_ch[32] = {'\0'};
    sprintf(task_id_ch, "SaveMap_%f", rclcpp::Time().seconds());
    ROS_INFO("save_map_service task id ==> %s", task_id_ch);
    ERESULT res = engine_.SaveMap(task_id_ch, request->full_name);

    response->result = res;
}

