#include "server_interface/api_publisher/local_state.hpp"
LocalStatePublisher::LocalStatePublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<cm_msgs::msg::LocalState>("/local_state", 1);
}

void LocalStatePublisher::Publish(sps_common_msgs::msg::LocalState& data)
{
    cm_msgs::msg::LocalState cm_local_state;
    cm_local_state.header = data.header;
    cm_local_state.localization_state = data.localization_state;
    cm_local_state.map_uuid = data.map_uuid;
    cm_local_state.map_name = data.map_name;
    cm_local_state.confidence = data.confidence;
    cm_local_state.threshold = data.threshold;
    cm_local_state.map_update = data.map_update;
    cm_local_state.grid_x = data.grid_x;
    cm_local_state.grid_y = data.grid_y;
    cm_local_state.grid_theta = data.grid_theta;
    cm_local_state.world_x = data.world_x;
    cm_local_state.world_y = data.world_y;
    cm_local_state.world_theta = data.world_theta;

    publisher_->publish(cm_local_state);
}

bool LocalStatePublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
