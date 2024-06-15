#ifndef NAVI_PAUSE_CONTROL_HPP_
#define NAVI_PAUSE_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "system_manager/define.h"

class NaviPauseControlServiceClient : public rclcpp::Node
{
public:
    NaviPauseControlServiceClient();

    bool call(std_srvs::srv::SetBool::Request::SharedPtr& request,
              std_srvs::srv::SetBool::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
};

#endif