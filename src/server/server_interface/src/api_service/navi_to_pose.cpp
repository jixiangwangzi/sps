#include "server_interface/api_service/navi_to_pose.hpp"

NaviToPoseService::NaviToPoseService(rclcpp::Node& node,
                                     rclcpp::CallbackGroup::SharedPtr group_service,
                                     naviengine::RobotNaviEngine& engine)
    : engine_(engine)
{
    server_ = node.create_service<cm_msgs::srv::NaviToPose>(
        "/task_navi_to_pose",
        std::bind(&NaviToPoseService::ServiceCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        group_service);
}

void NaviToPoseService::ServiceCallback(cm_msgs::srv::NaviToPose::Request::SharedPtr request,
                                        cm_msgs::srv::NaviToPose::Response::SharedPtr response)
{
    ROS_INFO("task_navi_to_pose_service header.stamp: %d", request->header.stamp.sec);
    Header header;
    header.sec = request->header.stamp.sec;
    header.nsec = request->header.stamp.nanosec;
    header.frame_id = request->header.frame_id;
    Twist twist;
    twist.vx = request->twist.linear.x;
    twist.vy = request->twist.linear.y;
    twist.vz = request->twist.angular.z;
    ROS_INFO(
        "NaviToPoseService::naviToService request.type=%d, request.pose_type=%d", request->type, request->pose_type);
    ENAVITYPE type;
    switch (request->type)
    {
        case cm_msgs::srv::NaviToPose::Request::NORMAL_NAVI:
            type = NAVI_NORMAL;
            break;
        case cm_msgs::srv::NaviToPose::Request::CHARGE_NAVI:
            type = NAVI_GOCHARGE;
            break;
        case cm_msgs::srv::NaviToPose::Request::LIFT_NAVI:
            type = NAVI_GOINLIFT;
            break;
        case cm_msgs::srv::NaviToPose::Request::LIFT_NAVI_OUT:
            type = NAVI_GOOUTLIFT;
            break;
        case cm_msgs::srv::NaviToPose::Request::IGNORE_ANGLE:
            type = NAVI_IGNORE_ANGLE;
            break;
        default:
            ROS_ERROR("NaviModule: unknown navi type received");
            response->result = E_STARTNAVI_FAILED;
            return;
    }

    Pose pose;
    PixelPose pixel_pose;
    pixel_pose.x = request->pose.position.x;
    pixel_pose.y = request->pose.position.y;
    pixel_pose.theta = request->pose.position.z;
    ROS_INFO("NaviToPoseService::naviToService pixel_pose (%f, %f, %f)", pixel_pose.x, pixel_pose.y, pixel_pose.theta);
    if (request->pose_type == GRID_POSE)
    {
        engine_.ChangePixelToWord(pixel_pose, pose);
    }
    else
    {
        pose.position.x = request->pose.position.x;
        pose.position.y = request->pose.position.y;
        pose.position.z = request->pose.position.z;
        pose.orientation.x = request->pose.orientation.x;
        pose.orientation.y = request->pose.orientation.y;
        pose.orientation.z = request->pose.orientation.z;
        pose.orientation.w = request->pose.orientation.w;
    }

    ROS_INFO(
        "StartNavi::type==>%d, goal pose==> "
        "pose.position {%f, %f, %f}, pose.orientation {%f, %f, %f, %f}",
        type,
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);

    std::string lift_id;
    PoiType poi_type;
    if(type == NAVI_GOINLIFT || type == NAVI_GOOUTLIFT)
    {
        if((engine_.QueryLiftPointType(pixel_pose, poi_type, lift_id) == E_OK) && !lift_id.empty())
        {
            ROS_INFO("NaviToPoseService::naviToService lift poi_type: %d, lift_id: %s", poi_type, lift_id.c_str());
        }
        else
        {
            ROS_INFO("NaviToPoseService::naviToService get lift info failed!");
            response->result = E_STARTNAVI_FAILED;
            return;
        }
    }

    ERESULT result = engine_.StartNaviToPose(
        header, request->task_id, lift_id, type, pose, twist, request->bt_xml_filename, request->align_angle);

    response->result = result;

    ROS_INFO("task_navi_to_pose_service result: %d", result);
}
