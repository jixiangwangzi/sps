
#ifndef VWESION_INFO_HPP_
#define VWESION_INFO_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/version_info.hpp"

class VersionInfoPublisher
{
  public:
    VersionInfoPublisher(rclcpp::Node &node);

    void Publish(sps_common_msgs::msg::VersionInfo& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::VersionInfo>::SharedPtr publisher_;
};

#endif