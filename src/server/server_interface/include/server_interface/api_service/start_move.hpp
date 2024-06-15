#ifndef START_MOVE_HPP_
#define START_MOVE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/start_move.hpp"
#include "system_manager/robot_naviengine.h"


class StartMoveService {
  public:
    StartMoveService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    StartMoveService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::StartMove::Request::SharedPtr request,
                        sps_common_msgs::srv::StartMove::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::StartMove>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif