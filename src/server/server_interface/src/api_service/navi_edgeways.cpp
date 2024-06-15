#include "server_interface/api_service/navi_edgeways.hpp"

NaviEdgewaysService::NaviEdgewaysService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::NaviEdgeways>("/task_navi_edgeways",
                                             std::bind(&NaviEdgewaysService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviEdgewaysService::ServiceCallback(sps_common_msgs::srv::NaviEdgeways::Request::SharedPtr request,
                            sps_common_msgs::srv::NaviEdgeways::Response::SharedPtr response) {
    ROS_INFO("task_navi_edgeways_service");
    //engine_.StartBuildMap("789456", E_NORMAL_MAPPING_MODE);
    response->result = 100;
}

