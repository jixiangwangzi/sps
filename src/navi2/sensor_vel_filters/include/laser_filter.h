#ifndef LASER_FILTER_H_
#define LASER_FILTER_H_

//#include <filters/filter.hpp>
//#include <filters/filter_base.hpp>

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
//#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"

namespace sensor_vel_filters
{

struct Pose
{
  double x;  // x-coordinate of pose
  double y;  // y-coordinate of pose
  double theta;  // rotation angle of pose
};

struct Velocity
{
  double x;  // x-component of linear velocity
  double y;  // y-component of linear velocity
  double tw;  // z-component of angular twist
};
//public filters::FilterBase<sensor_msgs::msg::LaserScan> ,
class LaserScanFilter 
{

public:
    LaserScanFilter(
      const nav2_util::LifecycleNode::WeakPtr & node,
      const std::string & filter_name);

    ~LaserScanFilter();

    void configure();

    void activate();

    void deactivate();

    void publish();

protected:
    void getParameters();

    void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    void odomvelCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void calDist(const Velocity vel, float &dist );
    
    void projectState(const double &dt, Pose &pose, Velocity &velocity);

    
private:
    nav2_util::LifecycleNode::WeakPtr node_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("sensor_filter")};
    std::string filter_name_;
    std::string topic_in_;
    std::string topic_out_;
    float replacement_value_;
    Velocity vel_;
    double diff_mode_min_turning_radius_;
    double omega_;
    double truncate_range_;
    sensor_msgs::msg::LaserScan scan_msg_;
    double simutime_;

    //rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_filter_;

};

} // namespace sensor_vel_filters

#endif