
#ifndef LOCAL_STATE_HPP_
#define LOCAL_STATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/local_state.hpp"
#include "cm_msgs/msg/local_state.hpp"

class LocalStatePublisher 
{
  public:
    LocalStatePublisher(rclcpp::Node &node);

    void Publish(sps_common_msgs::msg::LocalState& data);

    bool HasSubscribers();

  private:
    rclcpp::Publisher<cm_msgs::msg::LocalState>::SharedPtr publisher_;
};

#endif