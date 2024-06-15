#ifndef SEMANTIC_AREA_SERVER_HPP_
#define SEMANTIC_AREA_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sps_common_msgs/srv/get_semantics_areas.hpp"
#include "sps_common_msgs/msg/special_areas.hpp"
#include "sps_common_msgs/msg/semantics_area.hpp"
#include "sps_common_msgs/msg/sps_navi_state.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"
#include "sps_common_msgs/msg/localization_status.hpp"

#include "cm_msgs/msg/sps_ultrasonic.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class SemanticAreaServer : public rclcpp::Node
{

public:
    SemanticAreaServer();
    ~SemanticAreaServer();

private:

    enum SonarWeight
    {
        L_WEIGHT = 1,
        M_WEIGHT = 3,
        R_WEIGHT = 5,
    };

    enum BlockedSonar
    {
        NO_BLOCKED = 0,
        L_BLOCKED = 1,
        M_BLOCKED = 3,
        R_BLOCKED = 5,
        LM_BLOCKED = 4,
        LR_BLOCKED = 6,
        MR_BLOCKED = 8,
        LMR_BLOCKED = 9,
    };

    rclcpp::CallbackGroup::SharedPtr callback_group_service_;

    geometry_msgs::msg::Pose latest_loc_pose_;
    std::int32_t latest_loc_status_ = 0;
    nav_msgs::msg::OccupancyGrid occ_map_;
    rclcpp::TimerBase::SharedPtr area_timer_;
    std::vector<sps_common_msgs::msg::SemanticsArea> semantics_areas_;
    bool area_monitor_ = false;

    int pub_sonar_ = 5;
    int default_use_sonar_count_ = 5; // 5: 使用所有超声； 3： 使用前1+后2的超声； 2：使用后面2个超声； 0：关闭所有超声
    std::string sonar_frame_ = "sonar_link";
    double side_detect_range_ = 0.15;
    double detect_range_ = 0.37;

    int navi_type_ = 0;

    rclcpp::Client<sps_common_msgs::srv::GetSemanticsAreas>::SharedPtr get_semantics_areas_client_;

    rclcpp::Publisher<sps_common_msgs::msg::SpecialAreas>::SharedPtr special_areas_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr area_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sonar_scan_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::SpsNaviState>::SharedPtr navi_state_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::LocalizationStatus>::SharedPtr local_status_sub_;
    rclcpp::Subscription<cm_msgs::msg::SpsUltrasonic>::SharedPtr sps_ultrasonic_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navi_type_sub_;

    void transforms_init();
    void client_init();
    void publish_init();
    void subscription_init();

    void occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void navi_state_callback(const sps_common_msgs::msg::SpsNaviState::SharedPtr msg);
    void local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg);
    void ultrasonic_callback(const cm_msgs::msg::SpsUltrasonic::SharedPtr msg);
    void navi_type_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void semantics_areas_callback(rclcpp::Client<sps_common_msgs::srv::GetSemanticsAreas>::SharedFuture response);
    void areaTimerCallback();
    void getSemanticsAreas();
    bool isInRectangle(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2, 
                              const geometry_msgs::msg::Point32& p3, const geometry_msgs::msg::Point32& p4, 
                              const geometry_msgs::msg::Point32& p);
    float getCross(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2, 
                          const geometry_msgs::msg::Point32& p);
    void setPubSonar(int pub_sonar);
};

#endif
