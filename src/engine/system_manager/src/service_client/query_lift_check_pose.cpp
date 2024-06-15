#include "system_manager/service_client/query_lift_check_pose.hpp"

QueryLiftCheckPoseServiceClient::QueryLiftCheckPoseServiceClient() : Node("query_lift_check_pose_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::GetLiftCheckPixelPoint>("/get_lift_check_pixel_point");
}

bool QueryLiftCheckPoseServiceClient::call(sps_common_msgs::srv::GetLiftCheckPixelPoint::Request::SharedPtr& request,
                                    sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("get_lift_check_pixel_point service can not connected!");
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
        ROS_INFO("Failed to call get_lift_check_pixel_point service");
        return false;
    }
}
