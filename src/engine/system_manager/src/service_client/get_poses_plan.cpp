#include "system_manager/service_client/get_poses_plan.hpp"

GetPosesPlanServiceClient::GetPosesPlanServiceClient() : Node("get_poses_plan_service_node")
{
    client_ = this->create_client<nav_msgs::srv::GetPlan>("/make_plan");
}

bool GetPosesPlanServiceClient::call(nav_msgs::srv::GetPlan::Request::SharedPtr& request,
                                    nav_msgs::srv::GetPlan::Response::SharedPtr& response,
                                    const std::chrono::seconds timeout)
{
    if (!client_->wait_for_service(timeout))
    {
        ROS_INFO("make_plan service can not connected!");
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
        ROS_INFO("Failed to call make_plan service");
        return false;
    }
}
