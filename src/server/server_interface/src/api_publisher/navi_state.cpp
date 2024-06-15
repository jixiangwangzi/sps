#include "server_interface/api_publisher/navi_state.hpp"
NaviStatePublisher::NaviStatePublisher(rclcpp::Node& node)
{
    publisher_ = node.create_publisher<cm_msgs::msg::SpsNaviState>("/sps_navi_state", 1);
}

void NaviStatePublisher::Publish(cm_msgs::msg::SpsNaviState& data)
{
    publisher_->publish(data);
}

bool NaviStatePublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
