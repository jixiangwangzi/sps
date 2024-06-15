
#ifndef RDM_ALARM_HPP_
#define RDM_ALARM_HPP_


#include "rclcpp/rclcpp.hpp"
//#include "sps_common_msgs/msg/rdm_alarm.hpp"
#include "cm_msgs/msg/sps_rdm_alarm.hpp"

class RdmAlarmPublisher
{
  public:
    RdmAlarmPublisher(rclcpp::Node &node);

    void Publish(cm_msgs::msg::SpsRdmAlarm& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<cm_msgs::msg::SpsRdmAlarm>::SharedPtr publisher_;
};

#endif