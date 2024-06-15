#ifndef SAVE_MAP_HPP_
#define SAVE_MAP_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/save_map.hpp"
#include "system_manager/robot_naviengine.h"


class SaveMapService {
  public:
    SaveMapService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SaveMapService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SaveMap::Request::SharedPtr request,
                        sps_common_msgs::srv::SaveMap::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SaveMap>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif