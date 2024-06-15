
#ifndef LIDAR2D_DATA_HPP_
#define LIDAR2D_DATA_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/laser_frame.hpp"

class Lidar2dDataPublisher
{
  public:
    Lidar2dDataPublisher(rclcpp::Node &node);

    void Publish(sps_common_msgs::msg::LaserFrame& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::LaserFrame>::SharedPtr publisher_;
};

#endif