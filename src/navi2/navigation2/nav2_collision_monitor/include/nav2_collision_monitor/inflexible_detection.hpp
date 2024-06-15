#ifndef NAV2_COLLISION_MONITOR__INFLEXIBLE_DETECTION_H_
#define NAV2_COLLISION_MONITOR__INFLEXIBLE_DETECTION_H_
//project stuff


#include <vector>
#include <list>
#include <string>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include "cm_msgs/msg/sps_ultrasonic.hpp"

namespace nav2_collision_monitor {

enum ControlState {
    STRAIGHT,
    LEFTTURN,
    RIGHTTURN,
    ROTATION,
};

class InflexibleDetection {

public:
    InflexibleDetection(    
        const nav2_util::LifecycleNode::WeakPtr & node,
        const std::string & name,
        const std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    ~InflexibleDetection();

    void Initialize(std::string name);
    bool configure();
    bool getParameters();

    void SetUltradatas(const sensor_msgs::msg::ChannelFloat32 ultra_datas);

    bool CheckSurroundSafety(const double& check_windows_x,
                             const double& check_windows_y);
    bool CheckRangeSafety(const geometry_msgs::msg::Twist& cmd,const std::string &car_type);
    bool CheckSurroundSafety();

    void GetLidarPose();

    void activate();
    void deactivate();



private:
    void LiteUltrasonicCallback(cm_msgs::msg::SpsUltrasonic::ConstSharedPtr msg);
    void RangesMsgCallback(sensor_msgs::msg::ChannelFloat32::ConstSharedPtr ranges_message);
    void LaserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    void DistRemainMsgCallback(std_msgs::msg::Float32::ConstSharedPtr msg);

private:
    //laser_geometry::LaserProjection projector_; 
    rclcpp::Subscription<cm_msgs::msg::SpsUltrasonic>::SharedPtr lite_ultrasonic_sub_;
    rclcpp::Subscription<sensor_msgs::msg::ChannelFloat32>::SharedPtr ultras_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_remaining_sub_;
    nav2_util::LifecycleNode::WeakPtr node_;
    sensor_msgs::msg::ChannelFloat32 ultra_datas_;
    sensor_msgs::msg::LaserScan scan_datas_;
    sensor_msgs::msg::PointCloud2 lidar_cloud_;
    sensor_msgs::msg::PointCloud lidar_cloud_1;
    cm_msgs::msg::SpsUltrasonic lite_ultrasonic_datas_;

    std::string name_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("collision_monitor")};

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_base_tf_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    double distance_remaining_;
    double distance_remaining_threshold_;

    //has configure yaml.param
    
    bool ultras_detection_open_;
    std::string ultras_sub_topic_;
    std::string laser1_topic_;
    double emergency_stop_distance_;
    double omega_;
    double forward_emergency_stop_dist_;
    double side_forward_emergency_stop_dist_; //for ginger_lite
    double rotate_forward_emergency_stop_dist_;
    double turn_emergency_stop_dist_;
    double obstacle_detect_distance_;
    double min_range_threshold_;
    double range_sensor_emergency_stop_dist_;
    double scan_forward_dist_;
    double scan_side_dist_;
    double check_window_x_;
    double check_window_y_;
    std::string base_frame_;
    std::string lidar_frame_;
    bool visualize_lidar_cloud_;

    //no configure yaml
    double robot_inscribed_radius_;
    double robot_circumscribed_radius_;
    double robot_radius_aug_;
    double granularity_;
    double laser_uncertainty_;
    double diff_mode_min_turning_radius_;
    double straight_threshold_;
    int max_fatal_point_num_;
    int num_sonar_;
    int max_num_sonar_buffer_size_;
};

}//namespace nav2_collision_monitor
#endif //NAV2_COLLISION_MONITOR__INFLEXIBLE_DETECTION_H_
