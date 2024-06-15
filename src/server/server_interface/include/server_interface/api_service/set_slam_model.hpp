#ifndef SET_SLAM_MODE_HPP_
#define SET_SLAM_MODE_HPP_


#include "cm_msgs/srv/sps_set_slam_model.hpp"
#include "system_manager/robot_naviengine.h"

class SetSlamModelService {
  public:
    SetSlamModelService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetSlamModelService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::SpsSetSlamModel::Request::SharedPtr request,
                        cm_msgs::srv::SpsSetSlamModel::Response::SharedPtr response);

  private:
    rclcpp::Service<cm_msgs::srv::SpsSetSlamModel>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif