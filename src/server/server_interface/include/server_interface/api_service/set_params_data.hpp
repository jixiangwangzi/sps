#ifndef SET_PARAMS_DATA_HPP_
#define SET_PARAMS_DATA_HPP_


#include "rclcpp/rclcpp.hpp"
#include "cm_msgs/srv/sps_params_data.hpp"
#include "sps_common_msgs/srv/params_data.hpp"
#include "system_manager/robot_naviengine.h"


class SetParamsDataService : public rclcpp::Node
{
  public:
    SetParamsDataService(rclcpp::Node &node,
                    rclcpp::CallbackGroup::SharedPtr group_service,
                    naviengine::RobotNaviEngine& engine);
    SetParamsDataService() = default;

  private:
    void ServiceCallback(cm_msgs::srv::SpsParamsData::Request::SharedPtr request,
                        cm_msgs::srv::SpsParamsData::Response::SharedPtr response);
    void SetParamsDataCallback(rclcpp::Client<sps_common_msgs::srv::ParamsData>::SharedFuture response);

  private:
    rclcpp::Service<cm_msgs::srv::SpsParamsData>::SharedPtr server_;
    rclcpp::Client<sps_common_msgs::srv::ParamsData>::SharedPtr set_params_data_client_;
    naviengine::RobotNaviEngine& engine_;

};


#endif