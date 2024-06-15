#include "server_interface/api_publisher/lidar2d_data.hpp"
 Lidar2dDataPublisher:: Lidar2dDataPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::LaserFrame>("/lidar2d_data", 1);
}

void  Lidar2dDataPublisher::Publish(sps_common_msgs::msg::LaserFrame& data)
{

    publisher_->publish(data);
}

bool  Lidar2dDataPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
