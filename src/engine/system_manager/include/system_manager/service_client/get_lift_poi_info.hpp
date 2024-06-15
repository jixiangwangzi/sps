#ifndef GET_LIFT_POI_INFO_HPP_
#define GET_LIFT_POI_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/get_lift_poi_info.hpp"
#include "system_manager/define.h"

class GetLiftPoiInfoServiceClient : public rclcpp::Node
{
public:
    GetLiftPoiInfoServiceClient();

    bool call(sps_common_msgs::srv::GetLiftPoiInfo::Request::SharedPtr& request,
              sps_common_msgs::srv::GetLiftPoiInfo::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::GetLiftPoiInfo>::SharedPtr client_;
};

#endif