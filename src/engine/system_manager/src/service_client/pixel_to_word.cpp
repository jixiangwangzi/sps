#include "system_manager/service_client/pixel_to_word.hpp"
PixelToWordServiceClient::PixelToWordServiceClient() : Node("pixel_to_word_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::ChangePixelPoseToWordPose>("/change_pixel_pose_to_word_pose");
}

bool PixelToWordServiceClient::call(sps_common_msgs::srv::ChangePixelPoseToWordPose::Request::SharedPtr& request,
                                    sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("pixel_to_word service can not connected!");
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
        ROS_INFO("Failed to call pixel_to_word service");
        return false;
    }
}
