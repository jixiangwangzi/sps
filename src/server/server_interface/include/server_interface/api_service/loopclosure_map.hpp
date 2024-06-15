#ifndef LOOPCLOSURE_MAP_HPP_
#define LOOPCLOSURE_MAP_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/loopclosure_map.hpp"
#include "system_manager/robot_naviengine.h"


class LoopclosureMapService {
  public:
    LoopclosureMapService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    LoopclosureMapService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::LoopclosureMap::Request::SharedPtr request,
                        sps_common_msgs::srv::LoopclosureMap::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::LoopclosureMap>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif