#ifndef SYSTEM_NAVI_TO_POSE_HPP_
#define SYSTEM_NAVI_TO_POSE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/navi_to_pose.hpp"
#include "system_manager/define.h"



class SystemNaviToPoseService
{
  public:
    SystemNaviToPoseService(rclcpp::Node &node);
    SystemNaviToPoseService() = default;
    ~SystemNaviToPoseService();

  private:
    void ServiceCallback(sps_common_msgs::srv::NaviToPose::Request::SharedPtr request,
                        sps_common_msgs::srv::NaviToPose::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::NaviToPose>::SharedPtr server_;

};



#endif