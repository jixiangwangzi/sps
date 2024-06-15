#ifndef SET_AIDED_POSE_HPP_
#define SET_AIDED_POSE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/aided_pose.hpp"
#include "system_manager/define.h"

class SetAidedPoseServiceClient : public rclcpp::Node
{
public:
    SetAidedPoseServiceClient();

    bool call(sps_common_msgs::srv::AidedPose::Request::SharedPtr& request,
              sps_common_msgs::srv::AidedPose::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::AidedPose>::SharedPtr client_;
};

#endif