#include "server_interface/api_service/set_pose_initialize.hpp"

SetPoseInitializeService::SetPoseInitializeService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::SetPoseInitialize>("/set_pose_initialize",
                                             std::bind(&SetPoseInitializeService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void SetPoseInitializeService::ServiceCallback(cm_msgs::srv::SetPoseInitialize::Request::SharedPtr request,
                            cm_msgs::srv::SetPoseInitialize::Response::SharedPtr response)
{
    ROS_INFO("set_pose_initialize_service header.stamp: %d, pose_type: %d", request->header.stamp.sec, request->pose_type);
    PoiType poi_type = UNKOWN_TYPE;
    Pose pose;
    PixelPose pixel_pose;
    pixel_pose.x = request->pose.position.x;
    pixel_pose.y = request->pose.position.y;
    pixel_pose.theta = request->pose.position.z;
    ROS_INFO("set_pose_initialize_service pixel_pose(%f, %f, %f) ", pixel_pose.x, pixel_pose.y, pixel_pose.theta);
    if(request->pose_type == GRID_POSE)
    {
        engine_.QueryPointType(pixel_pose, poi_type);
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

    ERESULT res = engine_.SetPoseinitialize(pose, (SENSORMATCHTYPE) request->sensor_match_type, INIT_POSE, poi_type);
    response->result = res;

    ROS_INFO("set_pose_initialize_service result: %d", res);
}



