#ifndef WORD_TO_PIXEL_HPP_
#define WORD_TO_PIXEL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/change_word_pose_to_pixel_pose.hpp"
#include "system_manager/define.h"

class WordToPixelServiceClient : public rclcpp::Node
{
public:
    WordToPixelServiceClient();

    bool call(sps_common_msgs::srv::ChangeWordPoseToPixelPose::Request::SharedPtr& request,
              sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::ChangeWordPoseToPixelPose>::SharedPtr client_;
};

#endif