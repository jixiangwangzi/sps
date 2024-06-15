#include "system_manager/publisher/cloud_twist_chassis.hpp"


CloudTwistChassisPublisher::CloudTwistChassisPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("/cloud_twist", 1);
}

void CloudTwistChassisPublisher::Publish(geometry_msgs::msg::TwistStamped& data)
{

    publisher_->publish(data);
}

bool CloudTwistChassisPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
