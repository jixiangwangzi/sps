#include "server_interface/api_service/set_param_data.hpp"

SetParamDataService::SetParamDataService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine): Node("set_param_data_service_node"), engine_(engine) 
{
    server_ = node.create_service<sps_common_msgs::srv::SetParamData>("/set_param_data",
                                             std::bind(&SetParamDataService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
    set_param_data_client_ = this->create_client<sps_common_msgs::srv::SetParamData>("/param_data");
}

void SetParamDataService::ServiceCallback(sps_common_msgs::srv::SetParamData::Request::SharedPtr request,
                            sps_common_msgs::srv::SetParamData::Response::SharedPtr response)
{
    ROS_INFO("set_param_data_service header.stamp: %d", request->header.stamp.sec);
    ROS_INFO("set_params_data_service node_name: %s, param_name: %s", request->node_name.c_str(), request->param_name.c_str());

    auto param_data_srv_req = std::make_shared<sps_common_msgs::srv::SetParamData::Request>();

    param_data_srv_req->node_name = request->node_name;
    param_data_srv_req->param_name = request->param_name;
    param_data_srv_req->param_data = request->param_data;

    if(!set_param_data_client_->wait_for_service(std::chrono::seconds(5))) {
        ROS_WARN("param_data service can not connected!");
        return;
    }

    auto future_result = set_param_data_client_->async_send_request(param_data_srv_req);

    ROS_INFO("Waiting for service complete");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto param_data_srv_res = future_result.get();
        response->result = E_PARAMDATA_SUCCESS;
 
    }
    else
    {
        response->result = E_PARAMDATA_FAILED;
        ROS_INFO("Failed to call set_param_data_service");

    }
}

