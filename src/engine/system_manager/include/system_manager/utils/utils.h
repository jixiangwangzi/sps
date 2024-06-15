#ifndef  _UTILS_H_

#include "define.h"
#include "rclcpp/rclcpp.hpp"

inline geometry_msgs::PoseWithCovarianceStamped& Pose2PoseWithCovarianceStameped(Pose& poseFrom, geometry_msgs::PoseWithCovarianceStamped& poseTo) {
    poseTo.pose.pose.position.x = poseFrom.position.x; 
    poseTo.pose.pose.position.y = poseFrom.position.y; 
    poseTo.pose.pose.position.z = poseFrom.position.z; 
    poseTo.pose.pose.orientation.x = poseFrom.orientation.x; 
    poseTo.pose.pose.orientation.y = poseFrom.orientation.y; 
    poseTo.pose.pose.orientation.z = poseFrom.orientation.z; 
    poseTo.pose.pose.orientation.w = poseFrom.orientation.w; 

    return poseTo;
} 

inline Pose& PoseWithCovarianceStamped2Pose(geometry_msgs::PoseWithCovarianceStamped& poseFrom, Pose& poseTo) {

    poseTo.pose.x = poseFrom.pose.pose.position.x;
    poseTo.pose.y = poseFrom.pose.pose.position.y;
    poseTo.pose.z = poseFrom.pose.pose.position.z;
    poseTo.orientation.x = poseFrom.pose.pose.orientation.x;
    poseTo.orientation.y = poseFrom.pose.pose.orientation.y;
    poseTo.orientation.z = poseFrom.pose.pose.orientation.z;
    poseTo.orientation.w = poseFrom.pose.pose.orientation.w;

    return poseTo;
}

#endif


