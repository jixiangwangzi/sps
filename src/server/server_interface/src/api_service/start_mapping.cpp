#include "server_interface/api_service/start_mapping.hpp"

StartMappingService::StartMappingService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::StartMapping>("/start_mapping",
                                             std::bind(&StartMappingService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void StartMappingService::ServiceCallback(sps_common_msgs::srv::StartMapping::Request::SharedPtr request,
                            sps_common_msgs::srv::StartMapping::Response::SharedPtr response) {
    ROS_INFO("start_mapping_service");
    char task_id_ch[32] = {'\0'};
    sprintf(task_id_ch, "StartMap_%f", rclcpp::Time().seconds());
    ROS_INFO("sps_switch_map task id ==> %s", task_id_ch);
    ERESULT res = engine_.StartBuildMap("task_id_ch", (EMAPPINGTYPE) request->mapping_type, (EMAPPINGMODE) request->mapping_mode);
    response->result = res;
}

