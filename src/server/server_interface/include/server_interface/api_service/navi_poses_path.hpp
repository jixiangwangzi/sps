#ifndef NAVI_POSES_PATH_HPP_
#define NAVI_POSES_PATH_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_poses.hpp"
#include "system_manager/robot_naviengine.h"


class NaviPosesPathService {
  public:
    NaviPosesPathService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviPosesPathService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviPoses::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviPoses::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviPoses>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif