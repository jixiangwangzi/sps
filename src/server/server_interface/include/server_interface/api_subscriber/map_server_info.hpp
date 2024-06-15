

#ifndef MAP_SERVER_INFO_HPP_
#define MAP_SERVER_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/map_info.hpp"
class MapInfoSubscriber 
{
  public:
    MapInfoSubscriber(rclcpp::Node &node);
    MapInfoSubscriber() = default;
    sps_common_msgs::msg::MapInfo::SharedPtr data_;
    void ParseData(sps_common_msgs::msg::MapInfo::SharedPtr & data);

  private:
    void msg_callback(const sps_common_msgs::msg::MapInfo::SharedPtr msg);

  private:
    rclcpp::Subscription<sps_common_msgs::msg::MapInfo>::SharedPtr subscriber_;

};
#endif