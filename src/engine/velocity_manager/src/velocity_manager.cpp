#include "velocity_manager/velocity_manager.hpp"

#include <chrono>

rclcpp::Logger VEL_LOG = rclcpp::get_logger("VelManager");

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityManager>();
    rclcpp::executors::SingleThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

VelocityManager::VelocityManager() : Node("VelocityManager") {
    RCLCPP_INFO(VEL_LOG, "hello this is VelocityManager");
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parameter_init();
    publish_init();
    service_init();
    subscription_init();

    loadVelConfig();
    updateNaviVel(max_linear_vel_);
}

VelocityManager::~VelocityManager() {
}

void VelocityManager::parameter_init() {
    if (!this->has_parameter("home_path"))  {
        this->declare_parameter("home_path", home_path_);
    }
    this->get_parameter("home_path", home_path_);

    if (!this->has_parameter("/moveSpeedHari/naviGearSpeed")) {
        this->declare_parameter("/moveSpeedHari/naviGearSpeed", navi_linear_vel_);
    }
    this->get_parameter("/moveSpeedHari/naviGearSpeed", navi_linear_vel_);

    if (!this->has_parameter("/moveSpeedHari/naviTurnSpeed")) {
        this->declare_parameter("/moveSpeedHari/naviTurnSpeed", navi_rot_vel_);
    }
    this->get_parameter("/moveSpeedHari/naviTurnSpeed", navi_rot_vel_);
}

void VelocityManager::publish_init() {
    speed_limit_publisher_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void VelocityManager::service_init() {
}

void VelocityManager::subscription_init() {
    area_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/area_nav_vel", 1, std::bind(&VelocityManager::area_vel_callback, this, std::placeholders::_1));
    navi_type_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/navi_type", 1, std::bind(&VelocityManager::navi_type_callback, this, std::placeholders::_1));
    navi_vel_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/navi_vel_cmd", 1, std::bind(&VelocityManager::navi_vel_cmd_callback, this, std::placeholders::_1));
}

void VelocityManager::loadVelConfig() {
    RCLCPP_INFO(VEL_LOG, "loadVelConfig navi_linear_vel_:%f, navi_rot_vel_:%f, home_path_:%s",
                navi_linear_vel_, navi_rot_vel_, home_path_.c_str());

    bool er_lin = false;
    bool er_rot = false;
    if (navi_linear_vel_ > 0.0 && navi_linear_vel_ <= 1.0) {
        max_linear_vel_ = navi_linear_vel_;
        er_lin = true;
    }

    if (navi_rot_vel_ > 0.0 && navi_rot_vel_ <= 1.0) {
        max_rot_vel_ = navi_rot_vel_;
        er_rot = true;
    }

    if (!er_lin || !er_rot)
    {
        std::string vel_config_path(home_path_ + "/ginlt_param/robot_config_hari.yaml");
        try {
            YAML::Node navi_vel_node = YAML::LoadFile(vel_config_path);
            if (navi_vel_node["moveSpeedHari"].IsDefined())
            {
                if (navi_vel_node["moveSpeedHari"]["naviGearSpeed"].IsDefined())
                {
                    double navi_linear_vel = navi_vel_node["moveSpeedHari"]["naviGearSpeed"].as<double>();
                    if (navi_linear_vel > 0.0 && navi_linear_vel <= 1.0)
                    {
                        max_linear_vel_ = navi_linear_vel;
                    }
                }
                if (navi_vel_node["moveSpeedHari"]["naviTurnSpeed"].IsDefined())
                {
                    double navi_rot_vel = navi_vel_node["moveSpeedHari"]["naviTurnSpeed"].as<double>();
                    if (navi_rot_vel > 0.0 && navi_rot_vel <= 1.0)
                    {
                        max_rot_vel_ = navi_rot_vel;
                    }
                }
                RCLCPP_INFO(VEL_LOG, "loadVelConfig max_linear_vel_:%f, max_rot_vel_:%f", max_linear_vel_, max_rot_vel_);
            }
        } catch (YAML::Exception &e) {
            RCLCPP_ERROR(VEL_LOG, "loadVelConfig cannot open navigation vel config file: %s", e.msg.c_str());
        }
    }
}

void VelocityManager::area_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if(navi_type_ != 1 && navi_type_ != 3) {
        if(area_linear_vel_ != msg->linear.x) {
            RCLCPP_INFO(VEL_LOG, "area_vel_callback linear.x:%f", msg->linear.x);
            area_linear_vel_ = msg->linear.x;
            if (area_linear_vel_ <= 0.0 || area_linear_vel_ > 2.0) {
                updateNaviVel(max_linear_vel_);
            } else {
                updateNaviVel(area_linear_vel_);
            }
        }
    }
}

void VelocityManager::navi_type_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    navi_type_ = msg->data;
    RCLCPP_INFO(VEL_LOG, "navi_type_callback navi_type_:%d", navi_type_);

    if(navi_type_ == 1 || navi_type_ == 3) {
        updateNaviVel(lift_max_vel_);
    } else {
        updateNaviVel(max_linear_vel_);
    }
}

void VelocityManager::navi_vel_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    RCLCPP_INFO(VEL_LOG, "navi_vel_cmd_callback linear.x:%f, max_linear_vel_:%f", msg->linear.x, max_linear_vel_);
    max_linear_vel_ = msg->linear.x;
    updateNaviVel(msg->linear.x);
}

void VelocityManager::updateNaviVel(const double navi_vel) {
    RCLCPP_INFO(VEL_LOG, "updateNaviVel navi_vel:%f, current_max_vel_:%f", navi_vel, current_max_vel_);
    current_max_vel_ = navi_vel;
    nav2_msgs::msg::SpeedLimit navi_speed_limit;
    navi_speed_limit.speed_limit = navi_vel;
    speed_limit_publisher_->publish(navi_speed_limit);
}
