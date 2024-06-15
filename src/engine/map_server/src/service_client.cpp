#include "map_server/service_client.hpp"

#include <chrono>

rclcpp::Logger CLIENT_LOG = rclcpp::get_logger("SeviceClient");

using namespace std::chrono_literals;

SeviceClient::SeviceClient() : Node("SeviceClient") {
    RCLCPP_INFO(CLIENT_LOG, "hello this is SeviceClient");
    client_init();
}

SeviceClient::~SeviceClient() {
}

void SeviceClient::client_init() {
    set_aided_pose_client_ = this->create_client<sps_common_msgs::srv::AidedPose>("/set_aided_pose");
    load_map_client_ = this->create_client<sps_common_msgs::srv::SpsLoadMap>("/load_map");
    update_map_client_ = this->create_client<sps_common_msgs::srv::UpdateMap>("/update_map");
    mode_service_client_ = this->create_client<sps_common_msgs::srv::Mode>("/mode_type");
}

bool SeviceClient::aided_pose_client(sps_common_msgs::srv::AidedPose::Request::SharedPtr &request,
                                    sps_common_msgs::srv::AidedPose::Response::SharedPtr &response,
                                    const std::chrono::milliseconds timeout) {

    RCLCPP_INFO(CLIENT_LOG, "SeviceClient::aided_pose_client");
    if(!set_aided_pose_client_->wait_for_service(timeout)) {
        RCLCPP_WARN(CLIENT_LOG, "set_aided_pose service can not connected!");
        response->result = sps_common_msgs::srv::AidedPose::Response::FAILED;
        response->description = "set_aided_pose service can not connected";
        return false;
    }

    auto future_result = set_aided_pose_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        response = future_result.get();
        return true;
    } else {
        RCLCPP_WARN(CLIENT_LOG, "aided_pose_client failed to call set_aided_pose service");
        response->result = sps_common_msgs::srv::AidedPose::Response::FAILED;
        response->description = "failed to call set_aided_pose service";
        return false;
    }
}

bool SeviceClient::load_map_client(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr &request,
                    sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr &response,
                    const std::chrono::milliseconds timeout) {

    RCLCPP_INFO(CLIENT_LOG, "SeviceClient::load_map_client");
    if(!load_map_client_->wait_for_service(timeout)) {
        RCLCPP_WARN(CLIENT_LOG, "load_map service can not connected!");
        response->result = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;
        response->description = "load_map service can not connected";
        return false;
    }

    auto future_result = load_map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        response = future_result.get();
        return true;
    } else {
        RCLCPP_WARN(CLIENT_LOG, "load_map_client failed to call load_map service");
        response->result = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;
        response->description = "failed to call load_map service";
        return false;
    }
}

bool SeviceClient::update_map_client(sps_common_msgs::srv::UpdateMap::Request::SharedPtr &request,
                                    sps_common_msgs::srv::UpdateMap::Response::SharedPtr &response,
                                    const std::chrono::milliseconds timeout) {

    RCLCPP_INFO(CLIENT_LOG, "SeviceClient::update_map_client");
    if(!update_map_client_->wait_for_service(timeout)) {
        RCLCPP_WARN(CLIENT_LOG, "update_map service can not connected!");
        response->result = sps_common_msgs::srv::UpdateMap::Response::FAILED;
        response->description = "update_map service can not connected";
        return false;
    }

    auto future_result = update_map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        response = future_result.get();
        return true;
    } else {
        RCLCPP_WARN(CLIENT_LOG, "update_map_client failed to call update_map service");
        response->result = sps_common_msgs::srv::UpdateMap::Response::FAILED;
        response->description = "failed to call update_map service";
        return false;
    }
}

bool SeviceClient::mode_service_client(sps_common_msgs::srv::Mode::Request::SharedPtr &request,
                                        sps_common_msgs::srv::Mode::Response::SharedPtr &response,
                                        const std::chrono::milliseconds timeout) {

    RCLCPP_INFO(CLIENT_LOG, "SeviceClient::mode_service_client");
    if(!mode_service_client_->wait_for_service(timeout)) {
        RCLCPP_WARN(CLIENT_LOG, "mode_type service can not connected!");
        response->result = sps_common_msgs::srv::Mode::Response::FAILED;
        response->feedback = "mode_type service can not connected";
        return false;
    }

    auto future_result = mode_service_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, timeout) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        response = future_result.get();
        return true;
    } else {
        RCLCPP_WARN(CLIENT_LOG, "mode_service_client failed to call mode_type service");
        response->result = sps_common_msgs::srv::Mode::Response::FAILED;
        response->feedback = "failed to call mode_type service";
        return false;
    }
}