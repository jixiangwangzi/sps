#include "system_manager/service_client/get_charge_pile.hpp"
GetChargePileServiceClient::GetChargePileServiceClient() : Node("get_charge_pile_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::GetDefaultChargePile>("/get_default_charge_pile");
}

bool GetChargePileServiceClient::call(sps_common_msgs::srv::GetDefaultChargePile::Request::SharedPtr& request,
                                    sps_common_msgs::srv::GetDefaultChargePile::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("get_default_charge_pile service can not connected!");
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
        ROS_INFO("Failed to call get_default_charge_pile service");
        return false;
    }
}
