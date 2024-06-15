#include "server_interface/api_service/navi_path.hpp"

NaviPathService::NaviPathService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::NaviPath>("/task_navi_path",
                                             std::bind(&NaviPathService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviPathService::ServiceCallback(sps_common_msgs::srv::NaviPath::Request::SharedPtr request,
                            sps_common_msgs::srv::NaviPath::Response::SharedPtr response) {
    ROS_INFO("task_navi_path_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

