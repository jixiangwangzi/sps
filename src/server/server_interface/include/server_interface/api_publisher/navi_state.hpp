
#ifndef NAVI_STATE_HPP_
#define NAVI_STATE_HPP_


#include "cm_msgs/msg/sps_navi_state.hpp"
#include "rclcpp/rclcpp.hpp"

class NaviStatePublisher
{
public:
    NaviStatePublisher(rclcpp::Node& node);

    void Publish(cm_msgs::msg::SpsNaviState& data);

    bool HasSubscribers();

private:
    rclcpp::Publisher<cm_msgs::msg::SpsNaviState>::SharedPtr publisher_;
};

#endif