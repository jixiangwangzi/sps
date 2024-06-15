#include "system_manager/service_client/query_poi_type.hpp"

QueryPoiTypeServiceClient::QueryPoiTypeServiceClient() : Node("query_poi_type_service_node")
{
    client_ = this->create_client<sps_common_msgs::srv::QueryPoiTypeByPose>("/query_poi_type_by_pose");
}

bool QueryPoiTypeServiceClient::call(sps_common_msgs::srv::QueryPoiTypeByPose::Request::SharedPtr& request,
                                    sps_common_msgs::srv::QueryPoiTypeByPose::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("query_poi_type_by_pose service can not connected!");
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
        ROS_INFO("Failed to call query_poi_type_by_pose service");
        return false;
    }
}
