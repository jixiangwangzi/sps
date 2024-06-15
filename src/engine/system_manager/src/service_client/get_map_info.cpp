#include "system_manager/service_client/get_map_info.hpp"
GetMapInfoServiceClient::GetMapInfoServiceClient() : Node("get_map_info_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::GetMapInfo>("/get_map_info");
}

bool GetMapInfoServiceClient::call(sps_common_msgs::srv::GetMapInfo::Request::SharedPtr& request,
                                    sps_common_msgs::srv::GetMapInfo::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("get_map_info service can not connected!");
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
        ROS_INFO("Failed to call get_map_info service");
        return false;
    }
}
