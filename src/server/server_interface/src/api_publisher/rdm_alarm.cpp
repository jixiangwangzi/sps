#include "server_interface/api_publisher/rdm_alarm.hpp"

RdmAlarmPublisher::RdmAlarmPublisher(rclcpp::Node &node)
{

    publisher_ = node.create_publisher<cm_msgs::msg::SpsRdmAlarm>("/sps_rdm_alarm", 1);
}

void RdmAlarmPublisher::Publish(cm_msgs::msg::SpsRdmAlarm& data)
{

    publisher_->publish(data);
}

bool RdmAlarmPublisher::HasSubscribers() {
    return publisher_->get_subscription_count() != 0;
}
