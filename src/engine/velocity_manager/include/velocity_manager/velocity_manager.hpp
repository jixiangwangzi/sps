#ifndef VELOCITY_MANAGER_HPP_
#define VELOCITY_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "std_msgs/msg/int32.hpp"

#include "yaml-cpp/yaml.h"

class VelocityManager : public rclcpp::Node
{

public:
    VelocityManager();
    ~VelocityManager();

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;

    std::string home_path_ = "/home/ginger";
    double navi_linear_vel_ = -1.0;
    double navi_rot_vel_ = -1.0;

    double max_linear_vel_ = 0.7;   //默认的导航速度，云端设置下来的
    double current_max_vel_ = 0.0;  //当前真实使用的导航速度,动态调整
    double max_rot_vel_ = 0.6;

    double lift_max_vel_ = 0.37;

    double area_linear_vel_ = -1.0;
    int navi_type_ = 0;

    void parameter_init();
    void publish_init();
    void service_init();
    void subscription_init();

    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr area_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navi_type_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navi_vel_cmd_sub_;

    void area_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void navi_type_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void navi_vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void loadVelConfig();
    void updateNaviVel(const double navi_vel);
};

#endif
