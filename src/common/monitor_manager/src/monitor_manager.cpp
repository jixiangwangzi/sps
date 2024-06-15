#include "monitor_manager/monitor_manager.hpp"

#include <chrono>

rclcpp::Logger MONITOR_LOG = rclcpp::get_logger("MonitorManager");

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonitorManager>();
    rclcpp::executors::SingleThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

MonitorManager::MonitorManager() : Node("MonitorManager") {
    RCLCPP_INFO(MONITOR_LOG, "hello this is MonitorManager");
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parameter_init();
    service_init();
    publish_init();
    subscription_init();
}

MonitorManager::~MonitorManager() {
}

void MonitorManager::parameter_init() {
}

void MonitorManager::service_init() {
}

void MonitorManager::publish_init() {
    robot_state_alarm_pub_ = this->create_publisher<sps_common_msgs::msg::RobotStateAlarm>("/robot_state_alarm",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    robot_rdm_alarm_pub_ = this->create_publisher<cm_msgs::msg::SpsRdmAlarm>("/robot_rdm_alarm",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void MonitorManager::subscription_init() {
    visual_drop_sub_ = this->create_subscription<std_msgs::msg::Int32>("/VisualDrop", 1,
                                            std::bind(&MonitorManager::visual_drop_callback, this, std::placeholders::_1));
    e_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>("/EStop", 1,
                                            std::bind(&MonitorManager::e_stop_callback, this, std::placeholders::_1));
    HA_state_sub_ = this->create_subscription<std_msgs::msg::Bool>("/sps_HA_state", 1,
                                            std::bind(&MonitorManager::HA_state_callback, this, std::placeholders::_1));
    drop_collision_sub_ = this->create_subscription<cm_msgs::msg::SpsDropCollision>("/SpsDropCollision", 1,
                                            std::bind(&MonitorManager::drop_collision_callback, this, std::placeholders::_1));
    hw_error_sub_ = this->create_subscription<cm_msgs::msg::SpsHWError>("/SpsHWError", 1,
                                            std::bind(&MonitorManager::hw_error_callback, this, std::placeholders::_1));
    local_status_sub_ = this->create_subscription<sps_common_msgs::msg::LocalizationStatus>("/localization_status", 1, 
                                            std::bind(&MonitorManager::local_status_callback, this, std::placeholders::_1));
}

void MonitorManager::visual_drop_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(MONITOR_LOG, "visual_drop_callback msg->data:%d", msg->data);
    if(visual_drop_ != msg->data) {
        visual_drop_ = msg->data;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        if(visual_drop_ > 0) {
            RCLCPP_INFO(MONITOR_LOG, "visual_drop_callback visual drop is triggered");
            rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_DROP_DANGER;
            rdm_alarm.device_name = "Navigation";
            rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
            rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_MAJOR);
        } else if(visual_drop_ == 0) {
            RCLCPP_INFO(MONITOR_LOG, "visual_drop_callback visual drop is release");
            rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_DROP_DANGER;
            rdm_alarm.device_name = "Navigation";
            rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
            rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        }
        publishRobotStateAlarm();
        publishRobotRdmAlarm(rdm_alarm);
    }
}

void MonitorManager::e_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if(emergency_ != msg->data) {
        emergency_ = msg->data;
        if(emergency_) {
            RCLCPP_INFO(MONITOR_LOG, "e_stop_callback emergency stop pressed");
        } else {
            RCLCPP_INFO(MONITOR_LOG, "e_stop_callback emergency stop released");
        }
        publishRobotStateAlarm();
    }
}

void MonitorManager::HA_state_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if(HA_state_ != msg->data) {
        HA_state_ = msg->data;
        if(HA_state_) {
            RCLCPP_INFO(MONITOR_LOG, "HA_state_callback HA start");
        } else {
            RCLCPP_INFO(MONITOR_LOG, "HA_state_callback HA stop");
        }
        publishRobotStateAlarm();
    }
}

void MonitorManager::drop_collision_callback(const cm_msgs::msg::SpsDropCollision::SharedPtr msg) {
    if (msg->is_collision.size() == 1) {
        if(collision_ != msg->is_collision[0]) {
            collision_ = msg->is_collision[0];
            if(collision_) {
                RCLCPP_INFO(MONITOR_LOG, "drop_collision_callback bump detected");
            } else {
                RCLCPP_INFO(MONITOR_LOG, "drop_collision_callback bump clear");
            }
            publishRobotStateAlarm();
        }
    }
}

void MonitorManager::hw_error_callback(const cm_msgs::msg::SpsHWError::SharedPtr msg) {
    RCLCPP_INFO(MONITOR_LOG, "hw_error_callback error_code: %d, description: %s", msg->error_code, msg->description.c_str());
    if(error_code_ != msg->error_code) {
        error_code_ = msg->error_code;
        publishRobotStateAlarm();
    }
}

void MonitorManager::local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg) {
    if(latest_loc_status_ != msg->status || latest_loc_specific_causes_ != msg->specific_causes) {
        RCLCPP_INFO(MONITOR_LOG, "local_status_callback status: %d, specific_causes: %d", msg->status, msg->specific_causes);
        latest_loc_status_ = msg->status;
        latest_loc_specific_causes_ = msg->specific_causes;

        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        if(latest_loc_status_ == sps_common_msgs::msg::LocalizationStatus::LOST) {
            switch (latest_loc_specific_causes_) {
                case sps_common_msgs::msg::LocalizationStatus::LIDAR_VISUAL_LOC_WEAK:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(1);
                    FLAG_LIDAR_VISUAL_LOC_WEAK_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::OUT_MAP:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_OUT_MAP_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::LOC_JUMP:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(3);
                    FLAG_LOC_JUMP_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::LIDAR_DATA_LOST:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LIDAR_ABNORMAL;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_LIDAR_DATA_LOST_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::ODOM_DATA_LOST:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_ODOM_ABNORMAL;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_ODOM_DATA_LOST_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::IMU_DATA_LOST:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_IMU_ABNORMAL;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_IMU_DATA_LOST_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::VISUAL_POSE_DATA_LOST:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_HDMAP_LOAD_VISON_MAP_FAIL;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_VISUAL_POSE_DATA_LOST_ = true;
                    break;
                default:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CRITICAL);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(4);
                    FLAG_NAVI_LOST_DEFAULT_ = true;
                    break;
            }
            publishRobotRdmAlarm(rdm_alarm);
        } else if(latest_loc_status_ == sps_common_msgs::msg::LocalizationStatus::WEAK) {
            switch (latest_loc_specific_causes_) {
                case sps_common_msgs::msg::LocalizationStatus::LIDAR_LOC_WEAK:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOC_WEAK;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_WARNING);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(1);
                    FLAG_LIDAR_LOC_WEAK_ = true;
                    break;
                case sps_common_msgs::msg::LocalizationStatus::VISUAL_LOC_WEAK:
                    rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOC_WEAK;
                    rdm_alarm.device_name = "Navigation";
                    rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_WARNING);
                    rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
                    rdm_alarm.alarm_spec.push_back(2);
                    FLAG_VISUAL_LOC_WEAK_ = true;
                    break;
            }
            publishRobotRdmAlarm(rdm_alarm);
        } else if(latest_loc_status_ == sps_common_msgs::msg::LocalizationStatus::RUNNING) {
            clearLoclRdmAlarm();
        }
    }
}

void MonitorManager::clearLoclRdmAlarm() {
    RCLCPP_INFO(MONITOR_LOG, "clearLoclRdmAlarm");

    if(FLAG_LIDAR_VISUAL_LOC_WEAK_) {
        FLAG_LIDAR_VISUAL_LOC_WEAK_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(1);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_OUT_MAP_) {
        FLAG_OUT_MAP_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_LOC_JUMP_) {
        FLAG_LOC_JUMP_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(3);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_LIDAR_DATA_LOST_) {
        FLAG_LIDAR_DATA_LOST_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LIDAR_ABNORMAL;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_ODOM_DATA_LOST_) {
        FLAG_ODOM_DATA_LOST_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_ODOM_ABNORMAL;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_IMU_DATA_LOST_) {
        FLAG_IMU_DATA_LOST_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_IMU_ABNORMAL;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_EQUIPMENT);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_VISUAL_POSE_DATA_LOST_) {
        FLAG_VISUAL_POSE_DATA_LOST_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_HDMAP_LOAD_VISON_MAP_FAIL;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_NAVI_LOST_DEFAULT_) {
        FLAG_NAVI_LOST_DEFAULT_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOST;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(4);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_LIDAR_LOC_WEAK_) {
        FLAG_LIDAR_LOC_WEAK_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOC_WEAK;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(1);
        publishRobotRdmAlarm(rdm_alarm);
    }

    if(FLAG_VISUAL_LOC_WEAK_) {
        FLAG_VISUAL_LOC_WEAK_ = false;
        cm_msgs::msg::SpsRdmAlarm rdm_alarm;
        rdm_alarm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_LOC_WEAK;
        rdm_alarm.device_name = "Navigation";
        rdm_alarm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
        rdm_alarm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
        rdm_alarm.alarm_spec.push_back(2);
        publishRobotRdmAlarm(rdm_alarm);
    }
}

void MonitorManager::publishRobotStateAlarm() {
    RCLCPP_INFO(MONITOR_LOG, "publishRobotStateAlarm visual_drop_:%d, emergency_:%d, HA_state_:%d, collision_:%d, error_code_:%d",
                visual_drop_, emergency_, HA_state_, collision_, error_code_);

    sps_common_msgs::msg::RobotStateAlarm robotStateAlam;
    robotStateAlam.visual_drop_state = visual_drop_ > 0 ? true : false;
    robotStateAlam.emergency_state = emergency_;
    robotStateAlam.collision_state = collision_;
    robotStateAlam.hw_error_state = error_code_ > 0 ? true : false;
    robotStateAlam.ha_state = HA_state_;
    robot_state_alarm_pub_->publish(robotStateAlam);
}

void MonitorManager::publishRobotRdmAlarm(const cm_msgs::msg::SpsRdmAlarm& rdm_alarm) {
    RCLCPP_INFO(MONITOR_LOG, "publishRobotRdmAlarm");
    robot_rdm_alarm_pub_->publish(rdm_alarm);
}