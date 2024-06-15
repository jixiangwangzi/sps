#include "system_manager/publisher/cloud_twist_mux.hpp"
CloudTwistMuxPublisher::CloudTwistMuxPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("/cloud_twist_mux", 1);
}

void CloudTwistMuxPublisher::Publish(geometry_msgs::msg::TwistStamped& data)
{

    publisher_->publish(data);
}

bool CloudTwistMuxPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
