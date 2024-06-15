#ifndef SET_SLAM_MODEL_HPP_
#define SET_SLAM_MODEL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/set_slam_model.hpp"
#include "system_manager/define.h"

class SetSlamModelServiceClient : public rclcpp::Node
{
public:
    SetSlamModelServiceClient();

    bool call(sps_common_msgs::srv::SetSlamModel::Request::SharedPtr& request,
              sps_common_msgs::srv::SetSlamModel::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::SetSlamModel>::SharedPtr client_;
};

#endif