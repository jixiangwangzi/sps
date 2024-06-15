
#ifndef CLOUD_TWIST_CHASSIS_HPP_
#define CLOUD_TWIST_CHASSIS_HPP_


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CloudTwistChassisPublisher
{
  public:
    CloudTwistChassisPublisher(rclcpp::Node &node);
    CloudTwistChassisPublisher() = default;

    void Publish(geometry_msgs::msg::TwistStamped& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

#endif