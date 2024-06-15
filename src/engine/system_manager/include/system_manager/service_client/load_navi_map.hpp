#ifndef LOAD_NAVI_MAP_HPP_
#define LOAD_NAVI_MAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/sps_load_map.hpp"
#include "system_manager/define.h"

class LoadNaviMapServiceClient : public rclcpp::Node
{
public:
    LoadNaviMapServiceClient();

    bool call(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr& request,
              sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::SpsLoadMap>::SharedPtr client_;
};

#endif