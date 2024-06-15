#ifndef NAVI_AVERAGE_AREA_HPP_
#define NAVI_AVERAGE_AREA_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_average_area.hpp"
#include "system_manager/robot_naviengine.h"


class NaviAverageAreaService {
  public:
    NaviAverageAreaService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviAverageAreaService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviAverageArea::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviAverageArea::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviAverageArea>::SharedPtr server_; 
    naviengine::RobotNaviEngine& engine_;

};


#endif