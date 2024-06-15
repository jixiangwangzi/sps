#include "server_interface/server_interface_core.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exector;

    auto node = std::make_shared<ServerInterfaceCore>("server_interface_core", exector);

    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}