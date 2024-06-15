#include "server_interface/api_service/save_record_path.hpp"

SaveRecordPathService::SaveRecordPathService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::SaveMap>("/save_record_path",
                                             std::bind(&SaveRecordPathService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SaveRecordPathService::ServiceCallback(sps_common_msgs::srv::SaveMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SaveMap::Response::SharedPtr response) {
    ROS_INFO("save_record_path_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

