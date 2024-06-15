
#ifndef SYSTEM_NAVI_STATE_HPP_
#define SYSTEM_NAVI_STATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/sps_navi_state.hpp"
#include "system_manager/define.h"

class SystemNaviStatePublisher
{
  public:
    SystemNaviStatePublisher(rclcpp::Node &node);
    SystemNaviStatePublisher() = default;

    void Publish(sps_common_msgs::msg::SpsNaviState& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::SpsNaviState>::SharedPtr publisher_;
};

#endif