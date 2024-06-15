#ifndef GET_CHARGE_PILE_HPP_
#define GET_CHARGE_PILE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/srv/get_default_charge_pile.hpp"
#include "system_manager/define.h"

class GetChargePileServiceClient : public rclcpp::Node
{
public:
    GetChargePileServiceClient();

    bool call(sps_common_msgs::srv::GetDefaultChargePile::Request::SharedPtr& request,
              sps_common_msgs::srv::GetDefaultChargePile::Response::SharedPtr& response,
              const std::chrono::seconds timeout = std::chrono::seconds(5));


private:
    rclcpp::Client<sps_common_msgs::srv::GetDefaultChargePile>::SharedPtr client_;
};

#endif