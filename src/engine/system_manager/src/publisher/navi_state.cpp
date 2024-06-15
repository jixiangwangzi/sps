#include "system_manager/publisher/navi_state.hpp"
SystemNaviStatePublisher::SystemNaviStatePublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<sps_common_msgs::msg::SpsNaviState>("/system_navi_state", 1);
}

void SystemNaviStatePublisher::Publish(sps_common_msgs::msg::SpsNaviState& data)
{

    publisher_->publish(data);
}

bool SystemNaviStatePublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
