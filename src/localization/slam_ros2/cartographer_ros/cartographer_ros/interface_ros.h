#ifndef CARTOGRAPHER_INTERFACE_ROS_H
#define CARTOGRAPHER_INTERFACE_ROS_H

//
#include <rclcpp/rclcpp.hpp>
// #include "ros/console.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <iostream>
#include <memory>
namespace cartographer_ros
{
class CartographerRosInterface
{
public:
    // CartographerRosInterface(std::shared_ptr<tf2_ros::Buffer> buffer);
    CartographerRosInterface(const rclcpp::Node::SharedPtr node_handle, std::shared_ptr<tf2_ros::Buffer> buffer, const int& slam_model);
    ~CartographerRosInterface();

    bool SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map);
    int SetInitialPose(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& initial_pose);
    bool SetFlagMatchMap(const bool& msg);
    bool SetRelocalizationState(const int& reloc_state);
    bool naviTypeCB(const int& naviType);
    bool GetFlagInWall();
    double GetLocConfidence();
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> GetOccMap();
// private:
    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    // ros::Duration transform_tolerance_;

    // ros::NodeHandle node_handle_;
    rclcpp::Node::SharedPtr node_handle_;
    // cartographer_ros::OccupancyGridNode* occ_grid_node_ = NULL;
    // cartographer_ros::Node* carto_node_ = NULL;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // cartographer_ros::NodeOptions node_options_;
    // cartographer_ros::TrajectoryOptions trajectory_options_;

};

}

#endif
