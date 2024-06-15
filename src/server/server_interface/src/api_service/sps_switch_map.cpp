#include "server_interface/api_service/sps_switch_map.hpp"

SpsSwitchMapService::SpsSwitchMapService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::SpsLoadMap>("/sps_switch_map",
                                             std::bind(&SpsSwitchMapService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SpsSwitchMapService::ServiceCallback(cm_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            cm_msgs::srv::SpsLoadMap::Response::SharedPtr response) {
    ROS_INFO("sps_switch_map_service header.stamp: %d", request->header.stamp.sec);
    char task_id_ch[32] = {'\0'};
    sprintf(task_id_ch, "LoadMap_%f", rclcpp::Time().seconds());
    //sprintf(task_id_ch, "LoadMap_%d", request->header.stamp.sec);
    ROS_INFO("SpsSwitchMapService task id ==> %s, map_path: %s", task_id_ch, request->map_path.c_str());
    ERESULT res = engine_.LoadMap(task_id_ch, request->map_path);
    response->result = res;

    ROS_INFO("SpsSwitchMapService result: %d", res);
}

