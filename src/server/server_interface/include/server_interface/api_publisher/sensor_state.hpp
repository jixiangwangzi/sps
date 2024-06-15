
#ifndef SENSOR_STATE_HPP_
#define SENSOR_STATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/sensor_state.hpp"

class SensorStatePublisher
{
  public:
    SensorStatePublisher(rclcpp::Node &node);

    void Publish(sps_common_msgs::msg::SensorState& date);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::SensorState>::SharedPtr publisher_;
};

#endif