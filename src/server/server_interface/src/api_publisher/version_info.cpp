#include "server_interface/api_publisher/version_info.hpp"
VersionInfoPublisher::VersionInfoPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::VersionInfo>("/version_info", 1);
}

void VersionInfoPublisher::Publish(sps_common_msgs::msg::VersionInfo& data)
{

    publisher_->publish(data);
}

bool VersionInfoPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
