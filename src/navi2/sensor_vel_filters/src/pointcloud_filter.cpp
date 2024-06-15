#include "pointcloud_filter.h"

using namespace std;
using namespace sensor_vel_filters;

PointCloudFilter::PointCloudFilter(const rclcpp::NodeOptions & options)
        : Node("pclsub", options)
{
    sub_novel_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points", rclcpp::QoS(10), std::bind(&PointCloudFilter::DataCallback, this, std::placeholders::_1));
    pub_origin_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/origin_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_pass_filter_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pass_filter_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_statis_outlier_filter_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/statis_outlier_filter_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_voxel_grid_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_grid_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    pub_crop_filter_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/crop_filter_cloud", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

PointCloudFilter::~PointCloudFilter()
{
    sub_novel_.reset();
    pub_origin_cloud_.reset();
    pub_pass_filter_cloud_.reset();
    pub_statis_outlier_filter_cloud_.reset();
    pub_voxel_grid_cloud_.reset();
    pub_crop_filter_cloud_.reset();
}

void PointCloudFilter::DataCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(msg->data.empty()){
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *cloud);
    //RCLCPP_INFO(this->get_logger(), "cloud_origin : points_size(%d,%d)",msg->height,msg->width);
    //received origin pointcloud and pub
    //pub_origin_cloud_->publish(*msg);

    //crop_box_filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_crop_filter = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::CropBox<pcl::PointXYZ> crop;
    sensor_msgs::msg::PointCloud2 cloud_crop_filter_msg;
    cloud_crop_filter_msg.header.frame_id = "rslidar"; //depth_scan_frame  camera_depth_optical_frame
    cloud_crop_filter_msg.header.stamp = now();
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(-2.0,-2.0,-0.5,1.0)); 
    //crop.setMin(Eigen::Vector4f(-2.0,-1.2,0.1,1.0)); 
    crop.setMax(Eigen::Vector4f(2.0,2.0,1.5,1.0));
    //crop.setKeepOrganized(true);
    crop.setNegative(false);
    crop.setUserFilterValue(0.1f);
    crop.filter(*cloud_crop_filter);
    //RCLCPP_INFO(this->get_logger(), "cloud_crop_filter1 : points_size(%zu)",cloud_crop_filter->points.size());

    if(cloud_crop_filter->points.empty()){
        return ;
    }
    //RCLCPP_INFO(this->get_logger(), "cloud_crop_filter1 : points_size(%d,%d)",cloud_crop_filter->height,cloud_crop_filter->width);
    //pcl::toROSMsg(*cloud_crop_filter,cloud_crop_filter_msg);
    //pub_crop_filter_cloud_->publish(cloud_crop_filter_msg);


    //pass_through_filter
    #if 0 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass_filter = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    sensor_msgs::msg::PointCloud2 cloud_pass_filter_msg;
    cloud_pass_filter_msg.header.frame_id = "camera2_depth_optical_frame";
    cloud_pass_filter_msg.header.stamp = now();
    pass.setInputCloud(cloud_crop_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.5,-0.01);
    pass.filter(*cloud_pass_filter);
    RCLCPP_INFO(this->get_logger(), "cloud_pass_filter : points_size(%d,%d)",cloud_pass_filter->height,cloud_pass_filter->width);
    pcl::toROSMsg(*cloud_pass_filter,cloud_pass_filter_msg);
    pub_pass_filter_cloud_->publish(cloud_pass_filter_msg);
    if(cloud_pass_filter->points.empty()){
        return;
    }
    #endif

    //voxel_grid_filter
    sensor_msgs::msg::PointCloud2 cloud_voxel_grid_filter_msg;
    cloud_voxel_grid_filter_msg.header.frame_id = "rslidar";
    cloud_voxel_grid_filter_msg.header.stamp = now();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_grid_filter = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel_grid.setInputCloud(cloud_crop_filter);
    voxel_grid.setLeafSize(0.05f,0.05f,0.05f);
    voxel_grid.filter(*cloud_voxel_grid_filter);
    if(cloud_voxel_grid_filter->points.empty()){
        return;
    }
    //RCLCPP_INFO(this->get_logger(), "cloud_voxel_grid_filter : points_size(%d,%d)",cloud_voxel_grid_filter->height,cloud_voxel_grid_filter->width);
    //pcl::toROSMsg(*cloud_voxel_grid_filter,cloud_voxel_grid_filter_msg);
    //pub_voxel_grid_cloud_->publish(cloud_voxel_grid_filter_msg);

    //static_outlier filter
    sensor_msgs::msg::PointCloud2 cloud_statis_outlier_filter_msg;
    cloud_statis_outlier_filter_msg.header.frame_id = "rslidar";
    cloud_statis_outlier_filter_msg.header.stamp = now();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statis_outlier;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statis_outlier_filter = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    statis_outlier.setInputCloud(cloud_voxel_grid_filter);
    statis_outlier.setMeanK(5);
    statis_outlier.setStddevMulThresh(1.0);
    statis_outlier.filter(*cloud_statis_outlier_filter);
    if(cloud_statis_outlier_filter->points.empty()){
        return;
    }
    //RCLCPP_INFO(this->get_logger(), "cloud_statis_outlier_filter : points_size(%d,%d)",cloud_statis_outlier_filter->height,cloud_statis_outlier_filter->width);
    pcl::toROSMsg(*cloud_statis_outlier_filter,cloud_statis_outlier_filter_msg);
    //RCLCPP_WARN(rclcpp::get_logger("laser"),"pintcloud3 size = %zu",cloud_statis_outlier_filter_msg.data.size());
    pub_statis_outlier_filter_cloud_->publish(cloud_statis_outlier_filter_msg);        
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_vel_filters::PointCloudFilter)
