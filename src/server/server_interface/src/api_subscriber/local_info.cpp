#include "server_interface/api_subscriber/local_info.hpp"
LocalInfoSubscriber::LocalInfoSubscriber(rclcpp::Node &node) 
{
    subscriber_ = node.create_subscription<sps_common_msgs::msg::LocalState>("/local_info", 1,
                                            std::bind(&LocalInfoSubscriber::msg_callback, this, std::placeholders::_1));
}

void LocalInfoSubscriber::msg_callback(const sps_common_msgs::msg::LocalState::SharedPtr msg)
{
    data_ = msg;
}

void LocalInfoSubscriber::ParseData(sps_common_msgs::msg::LocalState::SharedPtr & data) {
    data = data_;
}