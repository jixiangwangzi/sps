
#ifndef SLAM_REPORT_HPP_
#define SLAM_REPORT_HPP_


#include "rclcpp/rclcpp.hpp"
#include "cm_msgs/msg/sps_slam_report.hpp"

class SlamReportPublisher
{
  public:
    SlamReportPublisher(rclcpp::Node &node);

    void Publish(cm_msgs::msg::SpsSlamReport& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<cm_msgs::msg::SpsSlamReport>::SharedPtr publisher_;
};

#endif