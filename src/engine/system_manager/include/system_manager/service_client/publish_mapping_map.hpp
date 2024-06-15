#ifndef PUBLISH_MAPPING_MAP_HPP_
#define PUBLISH_MAPPING_MAP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/publish_map_data.hpp"
#include "system_manager/define.h"

class PublishMappingMapServiceClient : public rclcpp::Node
{
public:
    PublishMappingMapServiceClient();

    bool call(sps_common_msgs::srv::PublishMapData::Request::SharedPtr& request,
              sps_common_msgs::srv::PublishMapData::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::PublishMapData>::SharedPtr client_;
};

#endif