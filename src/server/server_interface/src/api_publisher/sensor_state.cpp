#include "server_interface/api_publisher/sensor_state.hpp"
SensorStatePublisher::SensorStatePublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::SensorState>("/sensor_state", 1);
}

void SensorStatePublisher::Publish(sps_common_msgs::msg::SensorState& data)
{


    publisher_->publish(data);
}

bool SensorStatePublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
