

#ifndef SYSTEM_MAP_SERVER_INFO_HPP_
#define SYSTEM_MAP_SERVER_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/map_info.hpp"
#include "system_manager/define.h"
class SystemMapInfoSubscriber
{
  public:
    SystemMapInfoSubscriber( rclcpp::Node &node);
    SystemMapInfoSubscriber() = default;

    void ParseData(sps_common_msgs::msg::MapInfo::SharedPtr & data);

  private:
    void msg_callback(const sps_common_msgs::msg::MapInfo::SharedPtr msg);

  private:
    rclcpp::Subscription<sps_common_msgs::msg::MapInfo>::SharedPtr subscriber_;
    sps_common_msgs::msg::MapInfo::SharedPtr data_;

};
#endif