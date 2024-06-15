#include "server_interface/api_publisher/slam_report.hpp"
SlamReportPublisher::SlamReportPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<cm_msgs::msg::SpsSlamReport>("/sps_slam_report", 1);
}

void SlamReportPublisher::Publish(cm_msgs::msg::SpsSlamReport& data)
{

    publisher_->publish(data);
}

bool SlamReportPublisher::HasSubscribers()
{
    return publisher_->get_subscription_count() != 0;
}
