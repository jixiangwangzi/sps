#ifndef NAVIENGINE_NODE_HPP_
#define NAVIENGINE_NODE_HPP_

#include "define.h"
#include "system_manager/bt/robot_bt.h"
#include "publisher/cloud_twist_chassis.hpp"
#include "publisher/cloud_twist_mux.hpp"
#include "publisher/navi_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "service/navi_to_pose.hpp"
#include "service_client/pixel_to_word.hpp"
#include "service_client/query_poi_type.hpp"
#include "service_client/query_lift_poi_type.hpp"
#include "subscriber/map_server_info.hpp"

#include "sps_common_msgs/msg/robot_state_alarm.hpp"

namespace naviengine
{
class NaviEngineNode : public rclcpp::Node
{
public:
    NaviEngineNode(RobotBt* robotBt);
    ~NaviEngineNode();

    ERESULT ChangePixelToWord(const PixelPose &pixel_pose, Pose &pose);
    ERESULT QueryPointType(const PixelPose &pixel_pose, PoiType &poi_type);
    ERESULT QueryLiftPointType(const PixelPose &pixel_pose, PoiType &poi_type, std::string &lift_id);
    ERESULT StartMove(const Twist &twist, const bool collision_check);
    ERESULT StopMove();
    void SetNaviTaskId(const std::string &task_id);
    bool IsCompletedTask(const std::string &task_id);
    bool IsAlarmState();

private:
    RobotBt *robotBt_;

    void ParameterInit();
    void ServiceInit();
    void ClientInit();
    void PublishInit();
    void SubscriberInit();
    void TwistPublish();
    void TwistPublishThread();

    void robot_state_alarm_callback(const sps_common_msgs::msg::RobotStateAlarm::SharedPtr msg);

    std::shared_ptr<PixelToWordServiceClient> pixel_to_word_server_client_;
    std::shared_ptr<QueryPoiTypeServiceClient> query_poi_type_server_client_;
    std::shared_ptr<QueryLiftPoiTypeServiceClient> query_lift_poi_type_server_client_;

    std::shared_ptr<SystemNaviToPoseService> navi_to_pose_server_;

    std::shared_ptr<SystemNaviStatePublisher> navi_state_pub_;
    std::shared_ptr<CloudTwistMuxPublisher> cloud_twist_mux_pub_;
    std::shared_ptr<CloudTwistChassisPublisher> cloud_twist_chassis_pub_;

    std::shared_ptr<SystemMapInfoSubscriber> map_info_sub_;

    rclcpp::Subscription<sps_common_msgs::msg::RobotStateAlarm>::SharedPtr robot_state_alarm_sub_;


    rclcpp::TimerBase::SharedPtr publish_twist_timer_;
    std::shared_ptr<std::thread> publish_twist_thread_;

    geometry_msgs::msg::TwistStamped twist_stamped_;

    bool continuous_twist_ctrl_ = false;

    bool collision_check_ = true;

    double cloud_twist_pub_rate_ = 10;

    std::string completed_task_id_;

    bool visual_drop_state_ = false;
    bool emergency_state_ = false;
    bool HA_state_ = false;
    bool collision_state_ = false;
    bool hw_error_state_ = false;
};

}

#endif
