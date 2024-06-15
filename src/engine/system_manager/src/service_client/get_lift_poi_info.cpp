#include "system_manager/service_client/get_lift_poi_info.hpp"

GetLiftPoiInfoServiceClient::GetLiftPoiInfoServiceClient() : Node("get_lift_poi_info_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::GetLiftPoiInfo>("/get_lift_poi_info");
}

bool GetLiftPoiInfoServiceClient::call(sps_common_msgs::srv::GetLiftPoiInfo::Request::SharedPtr& request,
                                    sps_common_msgs::srv::GetLiftPoiInfo::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("get_lift_poi_info service can not connected!");
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
        ROS_INFO("Failed to call get_lift_poi_info service");
        return false;
    }
}
