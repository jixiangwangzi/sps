#ifndef SAVE_RECORD_PATH_HPP_
#define SAVE_RECORD_PATH_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/save_map.hpp"
#include "system_manager/robot_naviengine.h"


class SaveRecordPathService {
  public:
    SaveRecordPathService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SaveRecordPathService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SaveMap::Request::SharedPtr request,
                        sps_common_msgs::srv::SaveMap::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SaveMap>::SharedPtr server_;
    naviengine::RobotNaviEngine& engine_;

};


#endif