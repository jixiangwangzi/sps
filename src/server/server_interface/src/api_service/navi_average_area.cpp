#include "server_interface/api_service/navi_average_area.hpp"

NaviAverageAreaService::NaviAverageAreaService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::NaviAverageArea>("/task_navi_average_area",
                                             std::bind(&NaviAverageAreaService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviAverageAreaService::ServiceCallback(sps_common_msgs::srv::NaviAverageArea::Request::SharedPtr request,
                            sps_common_msgs::srv::NaviAverageArea::Response::SharedPtr response) {
    ROS_INFO("task_navi_average_area_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

