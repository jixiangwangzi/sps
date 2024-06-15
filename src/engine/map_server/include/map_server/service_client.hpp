#ifndef SERVICE_CLIENT_HPP_
#define SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sps_common_msgs/srv/aided_pose.hpp"
#include "sps_common_msgs/srv/sps_load_map.hpp"
#include "sps_common_msgs/srv/update_map.hpp"
#include "sps_common_msgs/srv/mode.hpp"

class SeviceClient : public rclcpp::Node
{
public:
    SeviceClient();
    ~SeviceClient();

    bool aided_pose_client(sps_common_msgs::srv::AidedPose::Request::SharedPtr &request,
                            sps_common_msgs::srv::AidedPose::Response::SharedPtr &response,
                            const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    bool load_map_client(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr &request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr &response,
                            const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    bool update_map_client(sps_common_msgs::srv::UpdateMap::Request::SharedPtr &request,
                            sps_common_msgs::srv::UpdateMap::Response::SharedPtr &response,
                            const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
    bool mode_service_client(sps_common_msgs::srv::Mode::Request::SharedPtr &request,
                            sps_common_msgs::srv::Mode::Response::SharedPtr &response,
                            const std::chrono::milliseconds timeout = std::chrono::milliseconds(5000));
private:
    rclcpp::Client<sps_common_msgs::srv::AidedPose>::SharedPtr set_aided_pose_client_;
    rclcpp::Client<sps_common_msgs::srv::SpsLoadMap>::SharedPtr load_map_client_;
    rclcpp::Client<sps_common_msgs::srv::UpdateMap>::SharedPtr update_map_client_;
    rclcpp::Client<sps_common_msgs::srv::Mode>::SharedPtr mode_service_client_;

    void client_init();
};

#endif
