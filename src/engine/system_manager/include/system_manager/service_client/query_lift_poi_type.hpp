#ifndef QUERY_LIFT_POI_TYPE_HPP_
#define QUERY_LIFT_POI_TYPE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/query_lift_poi_type.hpp"
#include "system_manager/define.h"

class QueryLiftPoiTypeServiceClient : public rclcpp::Node
{
public:
    QueryLiftPoiTypeServiceClient();

    bool call(sps_common_msgs::srv::QueryLiftPoiType::Request::SharedPtr& request,
              sps_common_msgs::srv::QueryLiftPoiType::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::QueryLiftPoiType>::SharedPtr client_;
};

#endif