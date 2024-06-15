#ifndef SPS_SWITCH_MAP_HPP_
#define SPS_SWITCH_MAP_HPP_


#include "rclcpp/rclcpp.hpp"
//#include "sps_common_msgs/srv/sps_load_map.hpp"
#include "cm_msgs/srv/sps_load_map.hpp"
#include "system_manager/robot_naviengine.h"


class SpsSwitchMapService {
  public:
    SpsSwitchMapService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SpsSwitchMapService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                        cm_msgs::srv::SpsLoadMap::Response::SharedPtr response);

  private:
    rclcpp::Service<cm_msgs::srv::SpsLoadMap>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif