#ifndef NAVI_VEL_HPP_
#define NAVI_VEL_HPP_


#include "rclcpp/rclcpp.hpp"
#include "cm_msgs/srv/sps_navi_vel.hpp"
#include "system_manager/robot_naviengine.h"


class NaviVelService {
  public:
    NaviVelService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviVelService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::SpsNaviVel::Request::SharedPtr request,
                        cm_msgs::srv::SpsNaviVel::Response::SharedPtr response);

  private:
    rclcpp::Service<cm_msgs::srv::SpsNaviVel>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif