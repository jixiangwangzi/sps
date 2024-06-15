#include "system_manager/service_client/publish_mapping_map.hpp"

PublishMappingMapServiceClient::PublishMappingMapServiceClient() : Node("publish_mapping_map_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::PublishMapData>("/publish_mapping_map");
}

bool PublishMappingMapServiceClient::call(sps_common_msgs::srv::PublishMapData::Request::SharedPtr& request,
                                    sps_common_msgs::srv::PublishMapData::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("publish_mapping_map service can not connected!");
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
        ROS_INFO("Failed to call publish_mapping_map service");
        return false;
    }
}
