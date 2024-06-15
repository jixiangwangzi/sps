#ifndef PIXEL_TO_WORD_HPP_
#define PIXEL_TO_WORD_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/change_pixel_pose_to_word_pose.hpp"
#include "system_manager/define.h"

class PixelToWordServiceClient : public rclcpp::Node
{
public:
    PixelToWordServiceClient();

    bool call(sps_common_msgs::srv::ChangePixelPoseToWordPose::Request::SharedPtr& request,
              sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::ChangePixelPoseToWordPose>::SharedPtr client_;
};

#endif