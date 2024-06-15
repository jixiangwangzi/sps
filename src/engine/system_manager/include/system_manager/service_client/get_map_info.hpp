#ifndef GET_MAP_INFO_HPP_
#define GET_MAP_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/get_map_info.hpp"
#include "system_manager/define.h"

class GetMapInfoServiceClient : public rclcpp::Node
{
public:
    GetMapInfoServiceClient();

    bool call(sps_common_msgs::srv::GetMapInfo::Request::SharedPtr& request,
              sps_common_msgs::srv::GetMapInfo::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::GetMapInfo>::SharedPtr client_;
};

#endif