#ifndef POINTCLOUD_FILTER_H_
#define POINTCLOUD_FILTER_H_


#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace sensor_vel_filters
{
class PointCloudFilter :  public rclcpp::Node
{

public:
    PointCloudFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~PointCloudFilter();

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_novel_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_origin_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pass_filter_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_statis_outlier_filter_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_voxel_grid_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_crop_filter_cloud_;

    void DataCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};
} // namespace sensor_vel_filters

#endif