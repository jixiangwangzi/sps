#ifndef SPSCARTOGRAPHERLOC_H
#define SPSCARTOGRAPHERLOC_H

#include "cartographer_ros/interface_loc_ros.h"
#include "lidar_slam/SpsSlam.h"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sps_common_msgs/msg/initial_slam_pose.hpp"
#include "sps_common_msgs/msg/localization_status.hpp"
#include "sps_common_msgs/srv/aided_pose.hpp"
namespace sps
{

class SpsCartographerLoc : public SpsSlam
{
public:
    SpsCartographerLoc(RosNodeHandlePtr &nh, RosTf2BufferPtr &buffer, RosTf2BroadPtr &cast);
    ~SpsCartographerLoc();

    bool setOccMap(const nav_msgs::msg::OccupancyGrid &occ_map);
    int setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& intial_pose) override;
    int SetInitPoseSelf(const sps_common_msgs::msg::InitialSlamPose& initial_pose_msg) override;

    bool naviTypeCB(const std_msgs::msg::Int32& msg) override;
    void ImuCallBack(const sensor_msgs::msg::Imu::ConstPtr msg);
private:
    void PublishLocConfidence();
    rclcpp::Node::SharedPtr carto_handle_;
    cartographer_ros::CartographerLocRosInterface* carto_ros_ = NULL;
    
    int laser_count_ = 0.0;
    int visual_count_ = 0.0;
    double last_stamp_imu_;
    double last_stamp_init_ = 0.0;
    bool fisrt_flag = true;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::TimerBase::SharedPtr loc_confidence_timers_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Publisher<sps_common_msgs::msg::LocalizationStatus>::SharedPtr loc_status_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
//     ros::Subscriber imu_sub_;
//     ros::WallTimer loc_confidence_timers_;
//     ros::CallbackQueue carto_queue_;
//     ros::AsyncSpinner spinner_;
    nav_msgs::msg::OccupancyGrid map_;
};

} //namespace


#endif //SPSCARTOGRAPHER_H
