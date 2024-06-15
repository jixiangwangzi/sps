#ifndef MONITOR_MANAGER_HPP_
#define MONITOR_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sps_common_msgs/msg/localization_status.hpp"
#include "sps_common_msgs/msg/robot_state_alarm.hpp"
#include "cm_msgs/msg/sps_drop_collision.hpp"
#include "cm_msgs/msg/sps_hw_error.hpp"
#include "cm_msgs/msg/sps_rdm_alarm.hpp"


class MonitorManager : public rclcpp::Node
{

public:
    MonitorManager();
    ~MonitorManager();

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;

    int visual_drop_ = 0;
    bool emergency_ = false;
    bool HA_state_ = false;
    bool collision_ = false;
    int error_code_ = 0;

    int latest_loc_status_ = 0;
    int latest_loc_specific_causes_ = 0;

    bool FLAG_LIDAR_VISUAL_LOC_WEAK_ = false;
    bool FLAG_OUT_MAP_ = false;
    bool FLAG_LOC_JUMP_ = false;
    bool FLAG_LIDAR_DATA_LOST_ = false;
    bool FLAG_ODOM_DATA_LOST_ = false;
    bool FLAG_IMU_DATA_LOST_ = false;
    bool FLAG_VISUAL_POSE_DATA_LOST_ = false;
    bool FLAG_NAVI_LOST_DEFAULT_ = false;

    bool FLAG_LIDAR_LOC_WEAK_ = false;
    bool FLAG_VISUAL_LOC_WEAK_ = false;


    void parameter_init();
    void service_init();
    void publish_init();
    void subscription_init();

    rclcpp::Publisher<sps_common_msgs::msg::RobotStateAlarm>::SharedPtr robot_state_alarm_pub_;
    rclcpp::Publisher<cm_msgs::msg::SpsRdmAlarm>::SharedPtr robot_rdm_alarm_pub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr visual_drop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr HA_state_sub_;
    rclcpp::Subscription<cm_msgs::msg::SpsDropCollision>::SharedPtr drop_collision_sub_;
    rclcpp::Subscription<cm_msgs::msg::SpsHWError>::SharedPtr hw_error_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::LocalizationStatus>::SharedPtr local_status_sub_;

    void visual_drop_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void HA_state_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void drop_collision_callback(const cm_msgs::msg::SpsDropCollision::SharedPtr msg);
    void hw_error_callback(const cm_msgs::msg::SpsHWError::SharedPtr msg);
    void local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg);

    void clearLoclRdmAlarm();
    void publishRobotStateAlarm();
    void publishRobotRdmAlarm(const cm_msgs::msg::SpsRdmAlarm& rdm_alarm);
};

#endif
