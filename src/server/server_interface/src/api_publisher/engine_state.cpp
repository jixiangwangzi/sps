#include "server_interface/api_publisher/engine_state.hpp"
EngineStatePublisher::EngineStatePublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::EngineState>("/engine_state", 1);
}

void EngineStatePublisher::Publish(sps_common_msgs::msg::EngineState& data)
{

    publisher_->publish(data);
}

bool EngineStatePublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
