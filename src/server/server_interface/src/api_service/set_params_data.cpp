#include "server_interface/api_service/set_params_data.hpp"

SetParamsDataService::SetParamsDataService(rclcpp::Node &node, 
                                            rclcpp::CallbackGroup::SharedPtr group_service,
                                            naviengine::RobotNaviEngine& engine): Node("params_data_service_node"), engine_(engine) {
    server_ = node.create_service<cm_msgs::srv::SpsParamsData>("/set_params_data",
                                             std::bind(&SetParamsDataService::ServiceCallback, this,std::placeholders::_1, std::placeholders::_2),
                                             rmw_qos_profile_services_default,
                                             group_service);
    set_params_data_client_ = this->create_client<sps_common_msgs::srv::ParamsData>("/params_data");
}

void SetParamsDataService::ServiceCallback(cm_msgs::srv::SpsParamsData::Request::SharedPtr request,
                            cm_msgs::srv::SpsParamsData::Response::SharedPtr response)
{
    ROS_INFO("set_params_data_service ctrl_cmd: %d, json_params: %s", request->ctrl_cmd, request->json_params.c_str());

    auto params_data_srv_req = std::make_shared<sps_common_msgs::srv::ParamsData::Request>();

    params_data_srv_req->ctrl_cmd = request->ctrl_cmd;
    params_data_srv_req->json_params = request->json_params;

    if(!set_params_data_client_->wait_for_service(std::chrono::seconds(5))) {
        ROS_WARN("params_data service can not connected!");
        return;
    }

    auto future_result = set_params_data_client_->async_send_request(params_data_srv_req);

    ROS_INFO("Waiting for service complete");
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto params_data_srv_res = future_result.get();
        response->result = E_PARAMDATA_SUCCESS;
        response->json_report = params_data_srv_res->json_report;
 
    }
    else
    {
        response->result = E_PARAMDATA_FAILED;
        ROS_INFO("Failed to call set_params_data_service");

    }
}
