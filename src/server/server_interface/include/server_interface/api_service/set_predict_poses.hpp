#ifndef SET_PREDICT_POSES_HPP_
#define SET_PREDICT_POSES_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/set_predict_poses.hpp"
#include "system_manager/robot_naviengine.h"


class SetPredictPosesService {
  public:
    SetPredictPosesService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetPredictPosesService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SetPredictPoses::Request::SharedPtr request,
                        sps_common_msgs::srv::SetPredictPoses::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SetPredictPoses>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif