#ifndef NAVI_THROUGH_ANCHORS_HPP_
#define NAVI_THROUGH_ANCHORS_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_poses.hpp"
#include "system_manager/robot_naviengine.h"


class NaviThroughAnchorsService {
  public:
    NaviThroughAnchorsService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviThroughAnchorsService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviPoses::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviPoses::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviPoses>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif