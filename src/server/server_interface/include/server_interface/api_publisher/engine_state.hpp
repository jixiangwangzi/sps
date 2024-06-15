
#ifndef ENGINE_STATE_HPP_
#define ENGINE_STATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/engine_state.hpp"

class EngineStatePublisher
{
  public:
    EngineStatePublisher(rclcpp::Node &node);
    EngineStatePublisher() = default;

    void Publish(sps_common_msgs::msg::EngineState& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::EngineState>::SharedPtr publisher_;
};

#endif