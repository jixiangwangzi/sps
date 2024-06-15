#include "system_manager/service_client/load_navi_map.hpp"
LoadNaviMapServiceClient::LoadNaviMapServiceClient() : Node("load_navi_map_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::SpsLoadMap>("/load_navi_map");
}

bool LoadNaviMapServiceClient::call(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr& request,
                                    sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("load_navi_map service can not connected!");
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
        ROS_INFO("Failed to call load_navi_map service");
        return false;
    }
}
