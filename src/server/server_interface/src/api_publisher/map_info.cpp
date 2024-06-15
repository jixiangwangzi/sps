#include "server_interface/api_publisher/map_info.hpp"
MapInfoPublisher::MapInfoPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::MapInfo>("/sps_map_info", 1);
}

void MapInfoPublisher::Publish(sps_common_msgs::msg::MapInfo& data)
{

    publisher_->publish(data);
}

bool MapInfoPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
