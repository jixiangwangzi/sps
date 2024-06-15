#include "system_manager/service_client/word_to_pixel.hpp"

WordToPixelServiceClient::WordToPixelServiceClient() : Node("word_to_pixel_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::ChangeWordPoseToPixelPose>("/change_word_pose_to_pixel_pose");
}

bool WordToPixelServiceClient::call(sps_common_msgs::srv::ChangeWordPoseToPixelPose::Request::SharedPtr& request,
                                    sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("word_to_pixel service can not connected!");
        return false;
    }

    auto future_result = client_->async_send_request(request);

    ROS_INFO("Waiting for service complete");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        response = future_result.get();
        return true;
    }
    else
    {
        ROS_INFO("Failed to call word_to_pixel service");
        return false;
    }
}
