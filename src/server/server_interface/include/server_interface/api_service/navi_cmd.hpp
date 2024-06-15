#ifndef NAVI_CMD_HPP_
#define NAVI_CMD_HPP_


#include "rclcpp/rclcpp.hpp"
//#include "sps_common_msgs/srv/task_navi_cmd.hpp"
#include "cm_msgs/srv/task_navi_cmd.hpp"
#include "system_manager/robot_naviengine.h"


class NaviCmdService {
  public:
    NaviCmdService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    NaviCmdService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::TaskNaviCmd::Request::SharedPtr request,
                        cm_msgs::srv::TaskNaviCmd::Response::SharedPtr response);

  private:
    rclcpp::Service<cm_msgs::srv::TaskNaviCmd>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif