#include "system_manager/service/navi_to_pose.hpp"



SystemNaviToPoseService::SystemNaviToPoseService(rclcpp::Node &node)
{
    server_ = node.create_service<sps_common_msgs::srv::NaviToPose>("/system_navi_to_pose",
                                             std::bind(&SystemNaviToPoseService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2));
}


SystemNaviToPoseService::~SystemNaviToPoseService()
{
}

void SystemNaviToPoseService::ServiceCallback(sps_common_msgs::srv::NaviToPose::Request::SharedPtr request,
                            sps_common_msgs::srv::NaviToPose::Response::SharedPtr response)
{
    ROS_INFO("task_navi_to_pose_service");

    response->result = 100;

}



