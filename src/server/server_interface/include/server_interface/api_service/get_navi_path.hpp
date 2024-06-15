#ifndef GET_NAVI_PATH_HPP_
#define GET_NAVI_PATH_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/get_navi_path.hpp"
#include "system_manager/robot_naviengine.h"


class GetNaviPathService {
  public:
    GetNaviPathService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    GetNaviPathService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::GetNaviPath::Request::SharedPtr request,
                        sps_common_msgs::srv::GetNaviPath::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::GetNaviPath>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif