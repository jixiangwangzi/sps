#include "server_interface/api_service/set_pose_rotate_initialize.hpp"

SetPoseRotateInitializeService::SetPoseRotateInitializeService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<sps_common_msgs::srv::SetPoseInitialize>("/set_pose_rotate_initialize",
                                             std::bind(&SetPoseRotateInitializeService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SetPoseRotateInitializeService::ServiceCallback(sps_common_msgs::srv::SetPoseInitialize::Request::SharedPtr request,
                            sps_common_msgs::srv::SetPoseInitialize::Response::SharedPtr response) {
    ROS_INFO("set_pose_rotate_initialize_service");
    Pose pose;
    PixelPose pixel_pose;
    pixel_pose.x = request->pose.position.x;
    pixel_pose.y = request->pose.position.y;
    pixel_pose.theta = request->pose.position.z;
    if(request->pose_type == GRID_POSE)
    {
         engine_.ChangePixelToWord(pixel_pose, pose);

    } else 
    {
        pose.position.x = request->pose.position.x;
        pose.position.y = request->pose.position.y;
        pose.position.z = request->pose.position.z;
        pose.orientation.x = request->pose.orientation.x;
        pose.orientation.y = request->pose.orientation.y;
        pose.orientation.z = request->pose.orientation.z;
        pose.orientation.w = request->pose.orientation.w;
    }

    ERESULT res = engine_.SetLocalization(pose, (SENSORMATCHTYPE) request->sensor_match_type, LOCAL_POSE);
    response->result = res;

}
