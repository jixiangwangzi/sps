#ifndef START_MAPPING_HPP_
#define START_MAPPING_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/start_mapping.hpp"
#include "system_manager/robot_naviengine.h"


class StartMappingService {
  public:
    StartMappingService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    StartMappingService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::StartMapping::Request::SharedPtr request,
                        sps_common_msgs::srv::StartMapping::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::StartMapping>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif