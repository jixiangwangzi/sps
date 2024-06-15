#include "system_manager/subscriber/map_server_info.hpp"
SystemMapInfoSubscriber::SystemMapInfoSubscriber(rclcpp::Node &node)
{
    subscriber_ = node.create_subscription<sps_common_msgs::msg::MapInfo>("/system_map_info", 1,
                                            std::bind(&SystemMapInfoSubscriber::msg_callback, this, std::placeholders::_1));
}

void SystemMapInfoSubscriber::msg_callback(const sps_common_msgs::msg::MapInfo::SharedPtr msg)
{
    data_ = msg;
}

void SystemMapInfoSubscriber::ParseData(sps_common_msgs::msg::MapInfo::SharedPtr & data) {
    data = data_;
}