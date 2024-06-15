#include "pointcloud_filter.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sensor_vel_filters::PointCloudFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}