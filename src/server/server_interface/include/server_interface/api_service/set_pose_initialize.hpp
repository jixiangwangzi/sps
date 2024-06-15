#ifndef SET_POSE_INITIALIZE_HPP_
#define SET_POSE_INITIALIZE_HPP_


//#include "sps_common_msgs/srv/set_pose_initialize.hpp"
#include "cm_msgs/srv/set_pose_initialize.hpp"
#include "system_manager/robot_naviengine.h"
//#include "geometry_msgs/msg/pose.hpp"
//#include "std_msgs/msg/header.hpp"

class SetPoseInitializeService {
  public:
    SetPoseInitializeService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetPoseInitializeService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::SetPoseInitialize::Request::SharedPtr request,
                        cm_msgs::srv::SetPoseInitialize::Response::SharedPtr response);

  private:
    rclcpp::Service<cm_msgs::srv::SetPoseInitialize>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif