#ifndef MAP_SERVER_CORE_HPP_
#define MAP_SERVER_CORE_HPP_

#include <jsoncpp/json/json.h>

#include "api_publisher/engine_state.hpp"
#include "api_publisher/lidar2d_data.hpp"
#include "api_publisher/local_state.hpp"
#include "api_publisher/map_info.hpp"
#include "api_publisher/slam_report.hpp"
#include "api_publisher/navi_state.hpp"
#include "api_publisher/rdm_alarm.hpp"
#include "api_publisher/sensor_state.hpp"
#include "api_publisher/version_info.hpp"
#include "api_service/cancel_mapping.hpp"
#include "api_service/cancel_record_path.hpp"
#include "api_service/charging_to_dock.hpp"
#include "api_service/get_navi_path.hpp"
#include "api_service/loopclosure_map.hpp"
#include "api_service/move_to_distance.hpp"
#include "api_service/move_to_rotate.hpp"
#include "api_service/navi_average_area.hpp"
#include "api_service/navi_cmd.hpp"
#include "api_service/navi_edgeways.hpp"
#include "api_service/navi_path.hpp"
#include "api_service/navi_poses_path.hpp"
#include "api_service/navi_through_anchors.hpp"
#include "api_service/navi_through_poses.hpp"
#include "api_service/navi_to_dock.hpp"
#include "api_service/navi_to_pose.hpp"
#include "api_service/save_map.hpp"
#include "api_service/save_record_path.hpp"
#include "api_service/set_param_data.hpp"
#include "api_service/set_params_data.hpp"
#include "api_service/set_pose_initialize.hpp"
#include "api_service/set_pose_rotate_initialize.hpp"
#include "api_service/set_predict_pose.hpp"
#include "api_service/set_predict_poses.hpp"
#include "api_service/set_rotate_initialize.hpp"
#include "api_service/sps_switch_map.hpp"
#include "api_service/sps_update_map.hpp"
#include "api_service/start_mapping.hpp"
#include "api_service/start_move.hpp"
#include "api_service/start_record_path.hpp"
#include "api_service/stop_move.hpp"
#include "api_service/switch_sensor_data.hpp"
#include "api_service/switch_sensor_enable.hpp"
#include "api_service/set_slam_model.hpp"
#include "api_service/navi_vel.hpp"
#include "api_service/navi_poses_mileage.hpp"
#include "api_subscriber/map_server_info.hpp"
#include "api_subscriber/local_info.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sps_common_msgs/msg/bit_map.hpp"
#include "sps_common_msgs/msg/initial_slam_pose.hpp"
#include "sps_common_msgs/msg/lift_poi.hpp"
#include "sps_common_msgs/msg/map_info.hpp"
#include "sps_common_msgs/msg/local_state.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"
#include "sps_common_msgs/msg/sps_navi_state.hpp"
#include "sps_common_msgs/msg/virtual_wall.hpp"
#include "cm_msgs/msg/sps_rdm_alarm.hpp"
#include "system_manager/robot_engine_observer.h"
#include "system_manager/robot_naviengine.h"



class ServerInterfaceCore : public rclcpp::Node, public naviengine::IRobotEngineObserver
{
public:
    ServerInterfaceCore(const std::string name, rclcpp::executors::MultiThreadedExecutor& exector);
    ~ServerInterfaceCore();
    ERESULT OnUpdateNaviPath(const NaviPathInfo navi_path) override;
    ERESULT OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id) override;
    ERESULT OnUpdateNaviState(const NaviStateInfo navi_state) override;
    ERESULT OnNaviState(const std::string& task_id, ENAVITYPE type, NAVISTATE state) override;



private:
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    std::string robot_naviengine_bt_;
    naviengine::RobotNaviEngine& engine_;


    void ParameterInit();
    void ApiServiceInit();
    void ApiPublishInit();
    void ApiSubscriberInit();
    void SpsApiPublish();



    std::shared_ptr<NaviToPoseService> navi_to_pose_server_;
    std::shared_ptr<NaviThroughPosesService> navi_through_poses_server_;
    std::shared_ptr<NaviToDockService> navi_to_dock_server_;
    std::shared_ptr<NaviPathService> navi_path_server_;
    std::shared_ptr<NaviPosesPathService> navi_poses_path_server_;
    std::shared_ptr<NaviEdgewaysService> navi_edgeways_server_;
    std::shared_ptr<NaviThroughAnchorsService> navi_through_anchors_server_;
    std::shared_ptr<NaviAverageAreaService> navi_average_area_server_;
    std::shared_ptr<ChargingToDockService> charging_to_dock_server_;
    std::shared_ptr<NaviCmdService> navi_cmd_server_;
    std::shared_ptr<GetNaviPathService> get_navi_path_server_;
    std::shared_ptr<SetPoseInitializeService> set_pose_initialize_server_;
    std::shared_ptr<SetPoseRotateInitializeService> set_pose_rotate_initialize_server_;
    std::shared_ptr<SetRotateInitializeService> set_rotate_initialize_server_;
    std::shared_ptr<SetPredictPoseService> set_predict_pose_server_;
    std::shared_ptr<SetPredictPosesService> set_predict_poses_server_;
    std::shared_ptr<StartMappingService> start_mapping_server_;
    std::shared_ptr<CancelMappingService> cancel_mapping_server_;
    std::shared_ptr<LoopclosureMapService> loopclosure_map_server_;
    std::shared_ptr<SaveMapService> save_map_server_;
    std::shared_ptr<StartRecordPathService> start_record_path_server_;
    std::shared_ptr<CancelRecordPathService> cancel_record_path_server_;
    std::shared_ptr<SaveRecordPathService> save_record_path_server_;
    std::shared_ptr<SpsSwitchMapService> sps_switch_map_server_;
    std::shared_ptr<SpsUpdateMapService> sps_update_map_server_;
    std::shared_ptr<MoveToDistanceService> move_to_distance_server_;
    std::shared_ptr<MoveToRotateService> move_to_rotate_server_;
    std::shared_ptr<StartMoveService> start_move_server_;
    std::shared_ptr<StopMoveService> stop_move_server_;
    std::shared_ptr<SwitchSensorDataService> switch_sensor_data_server_;
    std::shared_ptr<SetParamDataService> set_param_data_server_;
    std::shared_ptr<SetParamsDataService> set_params_data_server_;
    std::shared_ptr<SwitchSensorEnableService> switch_sensor_enable_server_;
    std::shared_ptr<SetSlamModelService> set_slam_model_server_;
    std::shared_ptr<NaviVelService> navi_vel_server_;
    std::shared_ptr<NaviPosesMileageService> navi_poses_mileage_server_;


    std::shared_ptr<NaviStatePublisher> navi_state_pub_;
    std::shared_ptr<EngineStatePublisher> engine_state_pub_;
    std::shared_ptr<VersionInfoPublisher> version_info_pub_;
    std::shared_ptr<SensorStatePublisher> sensor_state_pub_;
    std::shared_ptr<Lidar2dDataPublisher> lidar2d_data_pub_;
    std::shared_ptr<SlamReportPublisher> slam_report_pub_;
    std::shared_ptr<LocalStatePublisher> local_state_pub_;
    std::shared_ptr<MapInfoPublisher> map_info_pub_;
    std::shared_ptr<RdmAlarmPublisher> rdm_alarm_pub_;
    // std::shared_ptr<Lidar2dDataPublisher> lidar3d_data_pub_;
    // std::shared_ptr<Lidar2dDataPublisher> camera_data_pub_;
    // std::shared_ptr<Lidar2dDataPublisher> ultra_data_pub_;


    std::shared_ptr<MapInfoSubscriber> map_info_sub_;
    sps_common_msgs::msg::MapInfo* map_info_sub_data_;

    std::shared_ptr<LocalInfoSubscriber> local_info_sub_;
    sps_common_msgs::msg::LocalState* local_info_sub_data_;

    rclcpp::Subscription<cm_msgs::msg::SpsRdmAlarm>::SharedPtr robot_rdm_alarm_sub_;
    void RobotRdmAlarmCallback(const cm_msgs::msg::SpsRdmAlarm::SharedPtr msg);

    void PublishNaviDrivingDistance(const std::string &task_id, const double &driving_distance);

    cm_msgs::msg::SpsNaviState sps_navi_state_;


    rclcpp::TimerBase::SharedPtr publish_timer_;
};



#endif
