#ifndef NAVI_TO_PATH_HPP_
#define NAVI_TO_PATH_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_path.hpp"
#include "system_manager/robot_naviengine.h"


class NaviPathService {
  public:
    NaviPathService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviPathService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviPath::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviPath::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviPath>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif