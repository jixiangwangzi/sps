#ifndef QUERY_LIFT_CHECK_POSE_HPP_
#define QUERY_LIFT_CHECK_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/get_lift_check_pixel_point.hpp"
#include "system_manager/define.h"

class QueryLiftCheckPoseServiceClient : public rclcpp::Node
{
public:
    QueryLiftCheckPoseServiceClient();

    bool call(sps_common_msgs::srv::GetLiftCheckPixelPoint::Request::SharedPtr& request,
              sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::GetLiftCheckPixelPoint>::SharedPtr client_;
};

#endif