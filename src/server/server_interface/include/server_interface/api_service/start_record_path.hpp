#ifndef START_RECORD_PATH_HPP_
#define START_RECORD_PATH_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/cmd_ctrl.hpp"
#include "system_manager/robot_naviengine.h"


class StartRecordPathService {
  public:
    StartRecordPathService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    StartRecordPathService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::CmdCtrl::Request::SharedPtr request,
                        sps_common_msgs::srv::CmdCtrl::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::CmdCtrl>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif