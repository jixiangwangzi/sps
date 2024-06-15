#ifndef SPS_SLAM_H
#define SPS_SLAM_H

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <glog/logging.h>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
// 自定义的消息类型
#include "sps_common_msgs/msg/initial_slam_pose.hpp"
// #include "slam2/msg/location_state.hpp"

#include <memory>




namespace sps
{
using RosNodeHandlePtr = std::shared_ptr<rclcpp::Node>;
using RosTf2BufferPtr = std::shared_ptr<tf2_ros::Buffer>;
using RosTf2BroadPtr = std::shared_ptr<tf2_ros::TransformBroadcaster>;
using Tf2MessageFilter = tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>;
using MessageFilterSub = message_filters::Subscriber<sensor_msgs::msg::LaserScan>;
using Tf2MessageFilterPtr = std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>;
using MessageFilterSubPtr = std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>;
using namespace std::chrono_literals;

class SpsSlam
{
public:
    SpsSlam(RosNodeHandlePtr &nh, RosTf2BufferPtr &buffer, RosTf2BroadPtr &cast)
            : nh_(nh), tf2_buffer_(buffer), tf2_cast_(cast), lastSendLocLostRdm_(0.0), flagInLocLostRdm_(false)
    {
        // scan_sub_ = std::make_shared<MessageFilterSub>(*nh_, "/scan", 10);
        // scan_filter_ = std::make_shared<Tf2MessageFilter>(*scan_sub_, *tf2_buffer_, "/odom", 10, *nh_);
        // scan_filter_->registerCallback(&SpsSlam::laserCallback, this);
        LOG(INFO) << "SpsSlam start!";
        loc_confidence_pub_ = nh_->create_publisher<std_msgs::msg::Float32>("/loc_confidence", 10);
        mapping_data_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("/update_occmap", 1);
        // location_state_pub_ = nh_->advertise<ginger_msgs::LocationState>("/location_state", 1, true);
        //TODO:
        robot_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("hd_map_init_pose", 1);
        LOG(INFO) << "SpsSlam end!";
    }


    virtual ~SpsSlam() = default;

    // virtual void laserCallback(const sensor_msgs::msg::LaserScan::ConstPtr &msg) { };

    void publishLocConfidence(const float& loc_confidence) 
    {
        std_msgs::msg::Float32 confidence_value;
        confidence_value.data = loc_confidence;
        loc_confidence_pub_->publish(confidence_value);
    }

    void startUpdateMap()
    {
        update_map_ = true;
    }
    void stopUpdateMap()
    {
        update_map_ = false;
    }

    void PublishOccMap(const nav_msgs::msg::OccupancyGrid& occ_data)
    {
        if (update_map_) {
            mapping_data_pub_->publish(occ_data);
        }
    }

    //localization function
    virtual bool setOccMap(const nav_msgs::msg::OccupancyGrid &map) {
        RCLCPP_ERROR(nh_->get_logger(),"SpsSlam: No set occ map");
        return false;
    };
    virtual int setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
        RCLCPP_ERROR(nh_->get_logger(),"SpsSlam: No initial pose");
        return -1;
    };

    virtual int SetInitPoseSelf(const sps_common_msgs::msg::InitialSlamPose& initial_pose_msg){
        RCLCPP_ERROR(nh_->get_logger(),"SpsSlam: No initial pose");
        return -1;       
    }

    // virtual bool trigGlobalLocalization(const int& reloc_command) {
    //     RCLCPP_ERROR(nh_->get_logger(),"SpsSlam: No global localization");
    //     return false;
    // };

    // virtual void SetRelocalizationState(const std_msgs::msg::Int32& reloc_state_msg){
    //     ;
    // };

    //获取导航状态
    virtual bool naviTypeCB(const std_msgs::msg::Int32& msg){}

//     bool PublishLocalizationState(const int loc_state) {
//         if (loc_state == 0) {
//             ginger_msgs::LocationState state;
//             state.stamp = ros::Time::now();
//             state.state = ginger_msgs::LocationState::LOCATION_LOSS;
//             location_state_pub_.publish(state);
//         }
//         else if (loc_state == 1) {
//             ginger_msgs::LocationState state;
//             state.stamp = ros::Time::now();
//             state.state = ginger_msgs::LocationState::LOCATION_RESTORE;
//             location_state_pub_.publish(state);
//         }
//     };

    // publish robot pose (map->base_link)
    void PublishRobotPose() 
    {
        LOG(INFO) << "PublishRobotPose()";
        try
        {
            //TODO:
            geometry_msgs::msg::TransformStamped tmp_m_b = tf2_buffer_->lookupTransform(
                                "map", "base_link", tf2::TimePoint(),500ms);
            geometry_msgs::msg::PoseWithCovarianceStamped robot_m_b;
            robot_m_b.pose.pose.position.x = tmp_m_b.transform.translation.x;
            robot_m_b.pose.pose.position.y = tmp_m_b.transform.translation.y;
            robot_m_b.pose.pose.position.z = tmp_m_b.transform.translation.z;
            robot_m_b.pose.pose.orientation = tmp_m_b.transform.rotation;
            robot_m_b.header.frame_id = "map";
            robot_m_b.header.stamp = tmp_m_b.header.stamp;
            robot_pose_pub_->publish(robot_m_b);
        }
        catch(tf2::TransformException e)
        {
            RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 50, "SpsSlam: cannot get tf: %s\n", e.what());
        }

    };

     // get robot pose (map->base_link)
    geometry_msgs::msg::PoseWithCovarianceStamped GetRobotPose() {
        rclcpp::Time now = nh_->now();
        geometry_msgs::msg::PoseWithCovarianceStamped robot_m_b;
        try
        {
            geometry_msgs::msg::TransformStamped tmp_m_b = tf2_buffer_->lookupTransform("map", "base_link", tf2::TimePoint(),500ms);
            robot_m_b.pose.pose.position.x = tmp_m_b.transform.translation.x;
            robot_m_b.pose.pose.position.y = tmp_m_b.transform.translation.y;
            robot_m_b.pose.pose.position.z = tmp_m_b.transform.translation.z;
            robot_m_b.pose.pose.orientation = tmp_m_b.transform.rotation;
            robot_m_b.header.frame_id = "map";
            robot_m_b.header.stamp = tmp_m_b.header.stamp;
        }
        catch(tf2::TransformException e)
        {
            RCLCPP_ERROR_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 50, "SpsSlam: cannot get tf: %s\n", e.what()); 
        }
        return robot_m_b;
    };   


protected:
    RosNodeHandlePtr nh_;
    RosTf2BroadPtr tf2_cast_;
    RosTf2BufferPtr tf2_buffer_;
    MessageFilterSubPtr scan_sub_;
    Tf2MessageFilterPtr scan_filter_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapping_data_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loc_confidence_pub_;
    // rclcpp::Publisher<> location_state_pub_;                                       //TODO::待吉祥定义好消息后补充
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_pub_;
    double lastSendLocLostRdm_;
    bool flagInLocLostRdm_;

    bool update_map_ = true;

    void pubFakeTF()
    {
        geometry_msgs::msg::TransformStamped fake_tf;
        rclcpp::Time transform_expiration = nh_->now();
        fake_tf.header.stamp = transform_expiration;
        fake_tf.header.frame_id = "map";
        fake_tf.child_frame_id = "odom";
        fake_tf.transform.translation.x = 0.0;
        fake_tf.transform.translation.y = 0.0;
        fake_tf.transform.translation.z = 0.0;
        fake_tf.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
        tf2_cast_->sendTransform(fake_tf);
    }
};

}

#endif //SPS_SLAM_H
