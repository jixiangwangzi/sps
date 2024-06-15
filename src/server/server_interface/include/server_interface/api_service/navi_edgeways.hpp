#ifndef NAVI_EDGEWAYS_HPP_
#define NAVI_EDGEWAYS_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_edgeways.hpp"
#include "system_manager/robot_naviengine.h"


class NaviEdgewaysService {
  public:
    NaviEdgewaysService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviEdgewaysService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviEdgeways::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviEdgeways::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviEdgeways>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif