#include "server_interface/api_service/get_navi_path.hpp"

GetNaviPathService::GetNaviPathService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine):engine_(engine) 
{
    server_ = node.create_service<sps_common_msgs::srv::GetNaviPath>("/get_navi_path",
                                             std::bind(&GetNaviPathService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void GetNaviPathService::ServiceCallback(sps_common_msgs::srv::GetNaviPath::Request::SharedPtr request,
                            sps_common_msgs::srv::GetNaviPath::Response::SharedPtr response) 
{
    ROS_INFO("get_navi_path task id ==> %s", request->task_id.c_str());
    ERESULT res = engine_.ReportNaviPath(request->task_id);
    response->result = res;
}

