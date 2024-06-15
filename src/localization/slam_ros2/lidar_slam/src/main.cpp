#include "lidar_slam/slam_node.h"
#include <glog/logging.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    // FLAGS_log_dir = "/home/ginger";
    // google::InitGoogleLogging(argv[0]);
    // google::ParseCommandLineFlags(&argc, &argv, true);
    // FLAGS_logtostderr =1;


    rclcpp::init(argc,argv);

    google::AllowCommandLineReparsing();
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);
    FLAGS_logtostderr =1;
    int mode = -1;

    auto nh = std::make_shared<rclcpp::Node>("lidar_slam_node");
    if (!nh->has_parameter("mapping_model")) 
    {
        nh->declare_parameter("mapping_model", mode);
    }
    nh->get_parameter("mapping_model", mode);
    sps::SlamNode slam_node(nh);

    rclcpp::spin(nh);
    rclcpp::shutdown();
    return 0;
}
