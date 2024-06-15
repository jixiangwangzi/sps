#include "sensor_filter.h"


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<sensor_vel_filters::SensorFilter>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
