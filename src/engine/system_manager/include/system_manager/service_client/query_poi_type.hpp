#ifndef QUERY_POI_TYPE_HPP_
#define QUERY_POI_TYPE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/query_poi_type_by_pose.hpp"
#include "system_manager/define.h"

class QueryPoiTypeServiceClient : public rclcpp::Node
{
public:
    QueryPoiTypeServiceClient();

    bool call(sps_common_msgs::srv::QueryPoiTypeByPose::Request::SharedPtr& request,
              sps_common_msgs::srv::QueryPoiTypeByPose::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::QueryPoiTypeByPose>::SharedPtr client_;
};

#endif