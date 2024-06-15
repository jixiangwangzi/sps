
#ifndef MAP_INFO_HPP_
#define MAP_INFO_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/map_info.hpp"

class MapInfoPublisher
{
  public:
    MapInfoPublisher(rclcpp::Node &node);

    void Publish(sps_common_msgs::msg::MapInfo& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<sps_common_msgs::msg::MapInfo>::SharedPtr publisher_;
};

#endif