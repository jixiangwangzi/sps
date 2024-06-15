#include "server_interface/api_service/navi_through_poses.hpp"

NaviThroughPosesService::NaviThroughPosesService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine)
    :engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::NaviPoses>("/task_navi_through_poses",
                                             std::bind(&NaviThroughPosesService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
}

void NaviThroughPosesService::ServiceCallback(cm_msgs::srv::NaviPoses::Request::SharedPtr request,
                            cm_msgs::srv::NaviPoses::Response::SharedPtr response)
{
    ROS_INFO("task_navi_through_poses_service header.stamp: %d", request->header.stamp.sec);
    Header header;
    header.sec = request->header.stamp.sec;
    header.nsec = request->header.stamp.nanosec;
    header.frame_id = request->header.frame_id;
    Twist twist;
    twist.vx = request->twist.linear.x;
    twist.vy =request->twist.linear.y;
    twist.vz =request->twist.angular.z;
    ROS_INFO("NaviThroughPosesService request.type=%d, request.pose_type=%d", request->type, request->pose_type);
    ENAVITYPE type;
    switch (request->type) {
        case cm_msgs::srv::NaviPoses::Request::NORMAL_NAVI:
            type = NAVI_NORMAL;
            break;
        case cm_msgs::srv::NaviPoses::Request::CHARGE_NAVI:
            type = NAVI_GOCHARGE;
            break;
        case cm_msgs::srv::NaviPoses::Request::LIFT_NAVI:
            type = NAVI_GOINLIFT;
            break;
        case cm_msgs::srv::NaviPoses::Request::LIFT_NAVI_OUT:
            type = NAVI_GOOUTLIFT;
            break;
        case cm_msgs::srv::NaviPoses::Request::IGNORE_ANGLE:
            type = NAVI_IGNORE_ANGLE;
            break;
        default:
            ROS_ERROR("NaviThroughPosesService: unknown navi type received");
            response->result = E_STARTNAVI_FAILED;
            return;
    }
    std::vector<Pose> points;
    std::vector<int> points_type;
    Pose pose;
    PixelPose pixel_pose;
    if(request->pose_type == GRID_POSE)
    {
        for (auto &tmp_navi_point: request->poses)
        {
            pixel_pose.x = tmp_navi_point.position.x;
            pixel_pose.y = tmp_navi_point.position.y;
            pixel_pose.theta = tmp_navi_point.position.z;
            ROS_INFO("NaviThroughPosesService pixel_pose:(%f, %f, %f)", pixel_pose.x, pixel_pose.y, pixel_pose.theta);
            engine_.ChangePixelToWord(pixel_pose, pose);
            points.push_back(pose);

            ROS_INFO("NaviThroughPosesService::goal pose==> "
                "pose.position {%f, %f, %f}, pose.orientation {%f, %f, %f, %f}",
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }
    else
    {
        for (auto &tmp_navi_point: request->poses)
        {
            changeMsgPoseToStructPose(tmp_navi_point, pose);
            points.push_back(pose);
        }

    }

    points_type = request->poses_navi_type;
    for (auto& tmp_poi_type : points_type)
    {
        ROS_INFO("NaviThroughPosesService::poses_navi_type %d", tmp_poi_type);
    }

    ERESULT result = engine_.StartNaviThroughPoses(header, request->task_id, type, points, points_type, twist, request->bt_xml_filename, request->align_angle);
    response->result = result;

    ROS_INFO("task_navi_through_poses_service result: %d", result);
}



void NaviThroughPosesService::changeMsgPoseToStructPose(const geometry_msgs::msg::Pose& msg_pose, Pose& pose)
{
    pose.position.x = msg_pose.position.x;
    pose.position.y = msg_pose.position.y;
    pose.position.z = msg_pose.position.z;
    pose.orientation.x = msg_pose.orientation.x;
    pose.orientation.y = msg_pose.orientation.y;
    pose.orientation.z = msg_pose.orientation.z;
    pose.orientation.w = msg_pose.orientation.w;
}


