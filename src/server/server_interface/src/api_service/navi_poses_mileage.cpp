#include "server_interface/api_service/navi_poses_mileage.hpp"

NaviPosesMileageService::NaviPosesMileageService(rclcpp::Node& node,
                                     rclcpp::CallbackGroup::SharedPtr group_service,
                                     naviengine::RobotNaviEngine& engine)
    : engine_(engine)
{
    server_ = node.create_service<cm_msgs::srv::GetPosesMileage>(
        "/navi_poses_mileage",
        std::bind(&NaviPosesMileageService::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        group_service);
}

void NaviPosesMileageService::ServiceCallback(cm_msgs::srv::GetPosesMileage::Request::SharedPtr request,
                                        cm_msgs::srv::GetPosesMileage::Response::SharedPtr response)
{
    ROS_INFO("NaviPosesMileageService request.pose_type=%d", request->pose_type);
    Pose start_word_pose, target_word_pose;
    PixelPose start_pixel_pose, target_pixel_pose;
    if (request->pose_type == GRID_POSE)
    {
        start_pixel_pose.x = request->start_pose.position.x;
        start_pixel_pose.y = request->start_pose.position.y;
        start_pixel_pose.theta = request->start_pose.position.z;
        engine_.ChangePixelToWord(start_pixel_pose, start_word_pose);

        target_pixel_pose.x = request->target_pose.position.x;
        target_pixel_pose.y = request->target_pose.position.y;
        target_pixel_pose.theta = request->target_pose.position.z;
        engine_.ChangePixelToWord(target_pixel_pose, target_word_pose);

        ROS_INFO("NaviPosesMileageService start_pixel_pose (%f, %f, %f), target_pixel_pose (%f, %f, %f)",
                 start_pixel_pose.x, start_pixel_pose.y, start_pixel_pose.theta,
                 target_pixel_pose.x, target_pixel_pose.y, target_pixel_pose.theta);
    }
    else
    {
        start_word_pose.position.x = request->start_pose.position.x;
        start_word_pose.position.y = request->start_pose.position.y;
        start_word_pose.position.z = request->start_pose.position.z;
        start_word_pose.orientation.x = request->start_pose.orientation.x;
        start_word_pose.orientation.y = request->start_pose.orientation.y;
        start_word_pose.orientation.z = request->start_pose.orientation.z;
        start_word_pose.orientation.w = request->start_pose.orientation.w;

        target_word_pose.position.x = request->target_pose.position.x;
        target_word_pose.position.y = request->target_pose.position.y;
        target_word_pose.position.z = request->target_pose.position.z;
        target_word_pose.orientation.x = request->target_pose.orientation.x;
        target_word_pose.orientation.y = request->target_pose.orientation.y;
        target_word_pose.orientation.z = request->target_pose.orientation.z;
        target_word_pose.orientation.w = request->target_pose.orientation.w;
    }

    ROS_INFO("NaviPosesMileageService start_word_pose.position {%f, %f, %f}, start_word_pose.orientation {%f, %f, %f, %f}",
             start_word_pose.position.x, start_word_pose.position.y, start_word_pose.position.z,
             start_word_pose.orientation.x, start_word_pose.orientation.y, start_word_pose.orientation.z, start_word_pose.orientation.w);
    ROS_INFO("NaviPosesMileageService target_word_pose.position {%f, %f, %f}, target_word_pose.orientation {%f, %f, %f, %f}",
             target_word_pose.position.x, target_word_pose.position.y, target_word_pose.position.z,
             target_word_pose.orientation.x, target_word_pose.orientation.y, target_word_pose.orientation.z, target_word_pose.orientation.w);

    double mileage = engine_.GetNaviPosesMileage(start_word_pose, target_word_pose);
    if(mileage > 0)
    {
        response->result = cm_msgs::srv::GetPosesMileage::Response::SUCCESS;
        response->mileage = mileage;
    }
    else
    {
        response->result = cm_msgs::srv::GetPosesMileage::Response::FAILED;
    }

    ROS_INFO("NaviPosesMileageService result: %d, mileage: %f", response->result, mileage);
}
