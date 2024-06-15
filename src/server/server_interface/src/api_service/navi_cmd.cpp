#include "server_interface/api_service/navi_cmd.hpp"

NaviCmdService::NaviCmdService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::TaskNaviCmd>("/task_navi_cmd",
                                             std::bind(&NaviCmdService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviCmdService::ServiceCallback(cm_msgs::srv::TaskNaviCmd::Request::SharedPtr request,
                            cm_msgs::srv::TaskNaviCmd::Response::SharedPtr response) 
{
    ROS_INFO("task_navi_cmd_service header.stamp: %d, ctrl_cmd: %d", request->header.stamp.sec, request->ctrl_cmd);
    ERESULT res = engine_.NaviCmd(request->task_id, (NAVICMD)request->ctrl_cmd);
    response->result = res;
    ROS_INFO("task_navi_cmd_service result: %d", res);
}

