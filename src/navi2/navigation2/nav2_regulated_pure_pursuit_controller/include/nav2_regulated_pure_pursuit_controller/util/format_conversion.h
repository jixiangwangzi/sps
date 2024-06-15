#ifndef __FORMAT_CONVERT_H__
#define __FORMAT_CONVERT_H__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

using namespace common;

geometry_msgs::msg::PoseStamped ConvertToPoseStamped(/*const geometry_msgs::msg::PoseStamped& pose,*/
                                                     const CurvePoint& point)
{
    geometry_msgs::msg::PoseStamped transform_pose;

    // transform_pose.header          = pose.header;
    transform_pose.pose.position.x = point.GetX();
    transform_pose.pose.position.y = point.GetY();
    transform_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, point.GetTheta());

    transform_pose.pose.orientation = tf2::toMsg(q);

    return transform_pose;
}

nav_msgs::msg::Path ConvertToRosPath(const std::vector<CurvePoint>& path)
{
    nav_msgs::msg::Path transform_path;

    for (auto& pt : path)
    {
        transform_path.poses.emplace_back(ConvertToPoseStamped(pt));
    }
}

#endif
