

#ifndef LOCAL_INFO_HPP_
#define LOCAL_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/local_state.hpp"
class LocalInfoSubscriber 
{
  public:
    LocalInfoSubscriber(rclcpp::Node &node);
    LocalInfoSubscriber() = default;
    sps_common_msgs::msg::LocalState::SharedPtr data_;
    void ParseData(sps_common_msgs::msg::LocalState::SharedPtr & data);

  private:
    void msg_callback(const sps_common_msgs::msg::LocalState::SharedPtr msg);

  private:
    rclcpp::Subscription<sps_common_msgs::msg::LocalState>::SharedPtr subscriber_;

};
#endif