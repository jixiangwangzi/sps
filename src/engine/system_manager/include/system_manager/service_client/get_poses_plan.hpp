#ifndef GET_POSES_PLAN_HPP_
#define GET_POSES_PLAN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "system_manager/define.h"

class GetPosesPlanServiceClient : public rclcpp::Node
{
public:
    GetPosesPlanServiceClient();

    bool call(nav_msgs::srv::GetPlan::Request::SharedPtr& request,
              nav_msgs::srv::GetPlan::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr client_;
};

#endif