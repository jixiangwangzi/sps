#include "server_interface/api_subscriber/map_server_info.hpp"
MapInfoSubscriber::MapInfoSubscriber(rclcpp::Node &node) 
{
    subscriber_ = node.create_subscription<sps_common_msgs::msg::MapInfo>("/map_info", 1,
                                            std::bind(&MapInfoSubscriber::msg_callback, this, std::placeholders::_1));
}

void MapInfoSubscriber::msg_callback(const sps_common_msgs::msg::MapInfo::SharedPtr msg)
{
    data_ = msg;
}

void MapInfoSubscriber::ParseData(sps_common_msgs::msg::MapInfo::SharedPtr & data) {
    data = data_;
}