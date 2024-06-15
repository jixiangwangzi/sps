#include "system_manager/service_client/navi_pause_control.hpp"

NaviPauseControlServiceClient::NaviPauseControlServiceClient() : Node("navi_pause_control_service_node")
{
    client_ = this->create_client<std_srvs::srv::SetBool>("/pause_nav_task");
}

bool NaviPauseControlServiceClient::call(std_srvs::srv::SetBool::Request::SharedPtr& request,
                                    std_srvs::srv::SetBool::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("pause_nav_task service can not connected!");
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
        ROS_INFO("Failed to call pause_nav_task service");
        return false;
    }
}
