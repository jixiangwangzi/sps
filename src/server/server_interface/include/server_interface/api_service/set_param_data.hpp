#ifndef SET_PARAM_DATA_HPP_
#define SET_PARAM_DATA_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/set_param_data.hpp"
#include "system_manager/robot_naviengine.h"


class SetParamDataService : public rclcpp::Node
{
  public:
    SetParamDataService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetParamDataService() = default;

  private:
    void ServiceCallback(sps_common_msgs::srv::SetParamData::Request::SharedPtr request,
                        sps_common_msgs::srv::SetParamData::Response::SharedPtr response);

  private:
    rclcpp::Service<sps_common_msgs::srv::SetParamData>::SharedPtr server_;
    rclcpp::Client<sps_common_msgs::srv::SetParamData>::SharedPtr set_param_data_client_;
    naviengine::RobotNaviEngine& engine_;

};


#endif