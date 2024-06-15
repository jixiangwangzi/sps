#include "system_manager/service_client/set_slam_model_client.hpp"


SetSlamModelServiceClient::SetSlamModelServiceClient() : Node("set_slam_model_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::SetSlamModel>("/set_slam_model");
}

bool SetSlamModelServiceClient::call(sps_common_msgs::srv::SetSlamModel::Request::SharedPtr& request,
                                     sps_common_msgs::srv::SetSlamModel::Response::SharedPtr& response,
                                     const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("set_slam_model service can not connected!");
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
        ROS_INFO("Failed to call set_slam_model service");
        return false;
    }
}
