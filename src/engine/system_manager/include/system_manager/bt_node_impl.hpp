#ifndef BT_NODE_IMPL_HPP_
#define BT_NODE_IMPL_HPP_

#include "cm_msgs/msg/battery_state.hpp"
#include "cm_msgs/msg/initial_slam_pose.hpp"
#include "cm_msgs/msg/sps_rdm_alarm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "service_client/load_navi_map.hpp"
#include "service_client/set_aided_pose.hpp"
#include "service_client/query_lift_check_pose.hpp"
#include "service_client/get_lift_poi_info.hpp"
#include "service_client/get_poses_plan.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "system_manager/define.h"
#include "system_manager/local_manager.hpp"
#include "system_manager/task_manager.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace naviengine
{

class BTNodeImpl : public rclcpp::Node
{
public:
    BTNodeImpl(std::shared_ptr<LocalManager> localManager, std::shared_ptr<TaskManager> taskManager);
    ~BTNodeImpl();

    typedef enum
    {
        NAVI_TO_POSE = 0,
        NAVI_WAYPOINTS = 1,
        NAVI_LIFT = 2
    } ETASKTYPE;

    static inline std::string toString(double val, int precision = 0);

    ERESULT loadMap(std::string map_path);
    ERESULT setWorldPose(const Pose& w_pose, const LOCALINITTYPE& pose_type, const SENSORMATCHTYPE& match_type, const PoiType& poi_type);
    ERESULT startNaviToPose(const Header& header,
                            const std::string& task_id,
                            const std::string& lift_id,
                            const ENAVITYPE& navi_type,
                            const Pose& pose,
                            const Twist& twist,
                            const std::string& bt_xml_filename,
                            const bool align_angle);
    ERESULT startNaviThroughPoses(const Header& header,
                                  const std::string& task_id,
                                  const ENAVITYPE& navi_type,
                                  const std::vector<Pose>& points,
                                  const std::vector<int>& points_type,
                                  const Twist& twist,
                                  const std::string& bt_xml_filename,
                                  const bool align_angle);

    ERESULT pauseNavi(const std::string& task_id);
    ERESULT resumeNavi(const std::string& task_id);
    ERESULT stopNavi(const std::string& task_id);
    ERESULT setSlamModel(const int slam_model);
    ERESULT setMaxNaviVel(const double linear_max, const double angular_max);
    double getNaviPosesMileage(const Pose &start_pose, const Pose &target_pose);
    void robotAlarmHandle(bool alarm_state);
    bool IsChargingLine();

private:
    std::shared_ptr<LocalManager> localManager_;
    std::shared_ptr<TaskManager> taskManager_;
    std::string task_id_;
    ENAVITYPE navi_type_;
    ETASKTYPE task_type_;
    geometry_msgs::msg::PoseStamped navi_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_poses_;

    int charge_state_ = -1;  //未充电（-1）,线充（0）,桩充（1）

    int init_pose_wait_timeout_ = 6;

    bool record_pose_state_ = false;
    rclcpp::TimerBase::SharedPtr record_pose_timer_;
    std::vector<geometry_msgs::msg::Pose> robot_history_world_poses_;

    bool trapped_check_state_ = false;
    bool trapped_reported_ = false;

    int check_move_distance_interval_ = 30;                // 默认检查30秒内的最大移动距离;
    float max_move_distance_threshold_ = 0.5;              // 默认检查30秒内的最大移动距离;
    float check_move_distance_from_goal_threshold_ = 2.0;  // 默认在距离目标点2米以外的距离才进行检查;

    float liftin_distance_thresh_ = 0.6;                   // 0.6m;

    std::string navi_lift_id_;  // 当前机器人使用的电梯id， 多电梯场景
    int liftin_step_ = 0;  // 进电梯步骤， 1：进梯点->电梯门前检测点； 2：电梯门前检测点->电梯内点; 0:非电梯导航
    bool liftin_status_ = false;
    geometry_msgs::msg::PoseStamped lift_inner_pose_;
    rclcpp::TimerBase::SharedPtr liftin_check_timer_;
    std::vector<geometry_msgs::msg::Pose> lift_door_poi_;

    geometry_msgs::msg::Pose end_pose_;
    rclcpp::TimerBase::SharedPtr trapped_check_timer_;
    std::deque<geometry_msgs::msg::Pose> trapped_check_poses_;

    void parameter_init();
    void publish_init();
    void client_init();
    void subscription_init();

    void NavStateCallback(NaviStateInfo navi_state_info);

    std::shared_ptr<LoadNaviMapServiceClient> load_map_client_;
    std::shared_ptr<QueryLiftCheckPoseServiceClient> query_lift_check_pose_client_;
    std::shared_ptr<GetLiftPoiInfoServiceClient> get_lift_poi_info_client_;
    std::shared_ptr<GetPosesPlanServiceClient> get_poses_plan_client_;

    //rclcpp::Publisher<cm_msgs::msg::InitialSlamPose>::SharedPtr slam_initial_pose_pub_;
    rclcpp::Publisher<cm_msgs::msg::SpsRdmAlarm>::SharedPtr robot_rdm_alarm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navi_vel_cmd_pub_;

    rclcpp::Subscription<cm_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;

    void battery_state_callback(const cm_msgs::msg::BatteryState::SharedPtr msg);
    void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    bool changeWordToPixel(const geometry_msgs::msg::Pose& w_pose, sps_common_msgs::msg::PixelPose& pixel_pose);
    bool changePixelToWord(const sps_common_msgs::msg::PixelPose& pixel_pose, geometry_msgs::msg::Pose& w_pose);
    void changeStructPoseToMsgPose(const Pose& pose, geometry_msgs::msg::Pose& msg_pose);

    bool getLiftCheckPoi(const std::string& lift_id, sps_common_msgs::msg::PixelPose& pixel_pose);
    bool naviGoInLift();
    bool getLiftPoiInfo(const std::string& lift_id);
    bool checkLiftInsideState(double robot_x, double robot_y, double liftin_distance_thresh);
    void liftinChcekCallback();

    void setTrappedCheckState(bool state);
    void trappedCheckTimerCallback();

    void setRecordPoseState(bool state);
    double calculateDistance(const std::vector<geometry_msgs::msg::Pose> poses);
    double calculateDistance(const std::vector<geometry_msgs::msg::PoseStamped> poses);
    void recordPoseTimerCallback();
};

}  // namespace naviengine

#endif
