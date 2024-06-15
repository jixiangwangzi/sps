#include "server_interface/server_interface_core.hpp"

using namespace std::chrono_literals;

ServerInterfaceCore::ServerInterfaceCore(const std::string name, rclcpp::executors::MultiThreadedExecutor& exector)
    : Node(name), engine_(naviengine::RobotNaviEngine::GetInstance())
{
    RCLCPP_INFO(this->get_logger(), "hello this is %s.", name.c_str());
    engine_.Init(*this, exector);

    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ParameterInit();
    ApiServiceInit();
    ApiPublishInit();
    ApiSubscriberInit();
}

ServerInterfaceCore::~ServerInterfaceCore()
{
}

void ServerInterfaceCore::ParameterInit()
{
    this->declare_parameter<std::string>("robot_naviengine_bt", robot_naviengine_bt_);
    this->get_parameter("robot_naviengine_bt", robot_naviengine_bt_);
    ROS_INFO("ParameterInit robot_naviengine_bt_: %s ", robot_naviengine_bt_.c_str());
}

void ServerInterfaceCore::ApiServiceInit()
{
    navi_to_pose_server_ = std::make_shared<NaviToPoseService>(*this, callback_group_service_, engine_);
    navi_through_poses_server_ = std::make_shared<NaviThroughPosesService>(*this, callback_group_service_, engine_);
    // navi_to_dock_server_ = std::make_shared<NaviToDockService>(*this, callback_group_service_, engine_);
    // navi_path_server_ = std::make_shared<NaviPathService>(*this, callback_group_service_, engine_);
    // navi_poses_path_server_ = std::make_shared<NaviPosesPathService>(*this, callback_group_service_, engine_);
    // navi_edgeways_server_ = std::make_shared<NaviEdgewaysService>(*this, callback_group_service_, engine_);
    // navi_through_anchors_server_ = std::make_shared<NaviThroughAnchorsService>(*this, callback_group_service_,
    // engine_); navi_average_area_server_ = std::make_shared<NaviAverageAreaService>(*this, callback_group_service_,
    // engine_); charging_to_dock_server_ = std::make_shared<ChargingToDockService>(*this, callback_group_service_,
    // engine_);
    navi_cmd_server_ = std::make_shared<NaviCmdService>(*this, callback_group_service_, engine_);
    get_navi_path_server_ = std::make_shared<GetNaviPathService>(*this, callback_group_service_, engine_);
    set_pose_initialize_server_ = std::make_shared<SetPoseInitializeService>(*this, callback_group_service_, engine_);
    set_pose_rotate_initialize_server_ =
        std::make_shared<SetPoseRotateInitializeService>(*this, callback_group_service_, engine_);
    set_rotate_initialize_server_ =
        std::make_shared<SetRotateInitializeService>(*this, callback_group_service_, engine_);
    set_predict_pose_server_ = std::make_shared<SetPredictPoseService>(*this, callback_group_service_, engine_);
    set_predict_poses_server_ = std::make_shared<SetPredictPosesService>(*this, callback_group_service_, engine_);
    // start_mapping_server_ = std::make_shared<StartMappingService>(*this, callback_group_service_, engine_);
    // cancel_mapping_server_ = std::make_shared<CancelMappingService>(*this, callback_group_service_, engine_);
    // loopclosure_map_server_ = std::make_shared<LoopclosureMapService>(*this, callback_group_service_, engine_);
    // save_map_server_ = std::make_shared<SaveMapService>(*this, callback_group_service_, engine_);
    // start_record_path_server_ = std::make_shared<StartRecordPathService>(*this, callback_group_service_, engine_);
    // cancel_record_path_server_ = std::make_shared<CancelRecordPathService>(*this, callback_group_service_, engine_);
    // save_record_path_server_ = std::make_shared<SaveRecordPathService>(*this, callback_group_service_, engine_);
    sps_switch_map_server_ = std::make_shared<SpsSwitchMapService>(*this, callback_group_service_, engine_);
    sps_update_map_server_ = std::make_shared<SpsUpdateMapService>(*this, callback_group_service_, engine_);
    move_to_distance_server_ = std::make_shared<MoveToDistanceService>(*this, callback_group_service_, engine_);
    move_to_rotate_server_ = std::make_shared<MoveToRotateService>(*this, callback_group_service_, engine_);
    start_move_server_ = std::make_shared<StartMoveService>(*this, callback_group_service_, engine_);
    stop_move_server_ = std::make_shared<StopMoveService>(*this, callback_group_service_, engine_);
    switch_sensor_data_server_ = std::make_shared<SwitchSensorDataService>(*this, callback_group_service_, engine_);
    set_param_data_server_ = std::make_shared<SetParamDataService>(*this, callback_group_service_, engine_);
    set_params_data_server_ = std::make_shared<SetParamsDataService>(*this, callback_group_service_, engine_);
    switch_sensor_enable_server_ = std::make_shared<SwitchSensorEnableService>(*this, callback_group_service_, engine_);
    set_slam_model_server_ = std::make_shared<SetSlamModelService>(*this, callback_group_service_, engine_);
    navi_vel_server_ = std::make_shared<NaviVelService>(*this, callback_group_service_, engine_);
    navi_poses_mileage_server_ = std::make_shared<NaviPosesMileageService>(*this, callback_group_service_, engine_);
}

void ServerInterfaceCore::ApiPublishInit()
{
    engine_state_pub_ = std::make_shared<EngineStatePublisher>(*this);
    version_info_pub_ = std::make_shared<VersionInfoPublisher>(*this);
    sensor_state_pub_ = std::make_shared<SensorStatePublisher>(*this);
    lidar2d_data_pub_ = std::make_shared<Lidar2dDataPublisher>(*this);
    navi_state_pub_ = std::make_shared<NaviStatePublisher>(*this);
    slam_report_pub_ = std::make_shared<SlamReportPublisher>(*this);
    local_state_pub_ = std::make_shared<LocalStatePublisher>(*this);
    map_info_pub_ = std::make_shared<MapInfoPublisher>(*this);
    rdm_alarm_pub_ = std::make_shared<RdmAlarmPublisher>(*this);
    publish_timer_ = this->create_wall_timer(1s, std::bind(&ServerInterfaceCore::SpsApiPublish, this));
}

void ServerInterfaceCore::ApiSubscriberInit()
{
    RCLCPP_INFO(this->get_logger(), "api_subscription_init");
    map_info_sub_ = std::make_shared<MapInfoSubscriber>(*this);
    local_info_sub_ = std::make_shared<LocalInfoSubscriber>(*this);

    robot_rdm_alarm_sub_ = this->create_subscription<cm_msgs::msg::SpsRdmAlarm>("/robot_rdm_alarm", 1,
                                            std::bind(&ServerInterfaceCore::RobotRdmAlarmCallback, this, std::placeholders::_1));
}



void ServerInterfaceCore::SpsApiPublish()
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    sps_common_msgs::msg::EngineState engineState;
    engineState.engine_state = engine_.GetState();
    engine_state_pub_->Publish(engineState);

    if (local_state_pub_ && local_info_sub_->data_)
    {
        local_info_sub_data_ = local_info_sub_->data_.get();
        local_state_pub_->Publish(*local_info_sub_data_);
        RCLCPP_INFO_THROTTLE(
            rclcpp::get_logger("ServerInterfaceCore"), clock, 5000, "SpsApiPublish local_info_sub_data_");
    }
}

void ServerInterfaceCore::PublishNaviDrivingDistance(const std::string &task_id, const double &driving_distance)
{
    std::string report_string;
    Json::FastWriter fw;
    Json::Value root;
    Json::Value param;

    root["action"] = Json::Value("reportNaviMileage");
    param["opId"] = Json::Value(task_id);
    char driving_distance_fmt_ch[64];
    sprintf(driving_distance_fmt_ch, "%0.2f", driving_distance);
    param["mileage"] = Json::Value(driving_distance_fmt_ch);

    root["param"] = Json::Value(param);
    report_string = fw.write(root);

    cm_msgs::msg::SpsSlamReport slam_report_msg;
    slam_report_msg.json_report = report_string;
    slam_report_pub_->Publish(slam_report_msg);

    ROS_INFO("ServerInterfaceCore::PublishNaviDrivingDistance report_string: %s", report_string.c_str());
}

ERESULT ServerInterfaceCore::OnUpdateNaviPath(const NaviPathInfo navi_path_info)
{
    ROS_INFO("ServerInterfaceCore OnUpdateNaviPath entry");
    std::string report_string;
    Json::FastWriter fw;
    Json::Value root;
    Json::Value param;
    Json::Value mapData;
    Json::Value currPoi;
    Json::Value targetPoi;

    root["action"] = Json::Value();
    root["param"] = Json::Value();
    param["opId"] = Json::Value(navi_path_info.task_id);
    param["mapData"] = Json::Value();
    mapData["mapId"] = Json::Value(navi_path_info.map_id);
    mapData["mapName"] = Json::Value(navi_path_info.map_name);
    mapData["mapType"] = Json::Value();
    mapData["description"] = Json::Value();
    mapData["mapVersion"] = Json::Value();
    mapData["path"] = Json::Value();
    mapData["currPoi"] = Json::Value();
    mapData["targetPoi"] = Json::Value();

    for (const auto navi_path_point_ : navi_path_info.path)
    {
        Json::Value point;
        point["x"] = Json::Value(navi_path_point_.x);
        point["y"] = Json::Value(navi_path_point_.y);
        point["yaw"] = Json::Value(navi_path_point_.theta);
        mapData["path"].append(point);
    }

    currPoi["x"] = Json::Value(navi_path_info.curr_poi.x);
    currPoi["y"] = Json::Value(navi_path_info.curr_poi.y);
    currPoi["yaw"] = Json::Value(navi_path_info.curr_poi.theta);
    mapData["currPoi"] = Json::Value(currPoi);

    targetPoi["x"] = Json::Value(navi_path_info.target_poi.x);
    targetPoi["y"] = Json::Value(navi_path_info.target_poi.y);
    targetPoi["yaw"] = Json::Value(navi_path_info.target_poi.theta);
    mapData["targetPoi"] = Json::Value(targetPoi);

    param["mapData"] = Json::Value(mapData);

    root["action"] = Json::Value("reportNaviPath");
    root["param"] = Json::Value(param);
    report_string = fw.write(root);
    // ROS_INFO("cm_naviPathParse: %s", report_string.c_str());

    cm_msgs::msg::SpsSlamReport slam_report_msg;
    slam_report_msg.json_report = report_string;
    slam_report_msg.header.stamp = rclcpp::Time();
    slam_report_pub_->Publish(slam_report_msg);

    return E_OK;
}

ERESULT ServerInterfaceCore::OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id)
{
    ROS_INFO("ServerInterfaceCore OnUpdateRetentionStatus is_tapped:%d", is_tapped);
    std::string report_string;
    Json::FastWriter fw;
    Json::Value root;
    Json::Value param;
    Json::Value mapData;
    Json::Value currPoi;

    if (is_tapped)
    {
        root["action"] = Json::Value("reportRetention");
        param["content"] = Json::Value("Be trapped...");
    }
    else
    {
        root["action"] = Json::Value("reportRetentionRecovery");
        param["content"] = Json::Value("Out of trap...");
    }
    param["opId"] = Json::Value(task_id);
    param["robotId"] = Json::Value("");
    param["level"] = Json::Value(1);

    root["param"] = Json::Value(param);
    report_string = fw.write(root);
    // ROS_INFO("PublishNaviRetentionStatus: %s", report_string.c_str());

    cm_msgs::msg::SpsSlamReport slam_report_msg;
    slam_report_msg.json_report = report_string;
    slam_report_pub_->Publish(slam_report_msg);

    return E_OK;
}

ERESULT ServerInterfaceCore::OnUpdateNaviState(const NaviStateInfo navi_state_info)
{
    sps_navi_state_.task_id = navi_state_info.task_id;
    sps_navi_state_.navi_type = static_cast<uint8>(navi_state_info.navi_type);
    sps_navi_state_.navi_state = static_cast<uint8>(navi_state_info.navi_state);
    sps_navi_state_.navi_map = navi_state_info.navi_map;
    sps_navi_state_.navi_path = navi_state_info.navi_path;
    sps_navi_state_.navi_detailed_state = static_cast<uint8>(navi_state_info.navi_detailed_state);
    sps_navi_state_.linear_speed = navi_state_info.linear_speed;
    sps_navi_state_.angular_speed = navi_state_info.angular_speed;
    sps_navi_state_.distance_remaining = navi_state_info.distance_remaining;
    sps_navi_state_.estimated_time_remaining = navi_state_info.estimated_time_remaining;
    sps_navi_state_.mileage = navi_state_info.mileage;

    sps_navi_state_.header.stamp = rclcpp::Time();
    navi_state_pub_->Publish(sps_navi_state_);

    ROS_INFO(
        "ServerInterfaceCore OnUpdateNaviState nav_state:%d, task_id:%s, navi_type:%d, navi_map:%s, navi_path:%s, "
        "navi_detailed_state:%d, linear_speed:%f, angular_speed:%f, distance_remaining:%f, "
        "estimated_time_remaining:%f, mileage:%f",
        sps_navi_state_.navi_state,
        sps_navi_state_.task_id.c_str(),
        sps_navi_state_.navi_type,
        sps_navi_state_.navi_map.c_str(),
        sps_navi_state_.navi_path.c_str(),
        sps_navi_state_.navi_detailed_state,
        sps_navi_state_.linear_speed,
        sps_navi_state_.angular_speed,
        sps_navi_state_.distance_remaining,
        sps_navi_state_.estimated_time_remaining,
        sps_navi_state_.mileage);

    if(navi_state_info.navi_state == NAVISTATE::NAVI_STATE_SUCCESS ||
        navi_state_info.navi_state == NAVISTATE::NAVI_STATE_FAILED ||
        navi_state_info.navi_state == NAVISTATE::NAVI_STATE_CANCELED)
    {
        PublishNaviDrivingDistance(navi_state_info.task_id, navi_state_info.mileage);
    }

    return E_OK;
}

ERESULT ServerInterfaceCore::OnNaviState(const std::string& task_id, ENAVITYPE type, NAVISTATE state)
{
    ROS_INFO("ServerInterfaceCore OnNaviState entry");
    sps_navi_state_.task_id = task_id;
    sps_navi_state_.navi_type = static_cast<uint8>(type);
    sps_navi_state_.navi_state = static_cast<uint8>(state);
    sps_navi_state_.header.stamp = rclcpp::Time();
    navi_state_pub_->Publish(sps_navi_state_);
    ROS_INFO("ServerInterfaceCore OnNaviState exit");
    return E_OK;
}

void ServerInterfaceCore::RobotRdmAlarmCallback(const cm_msgs::msg::SpsRdmAlarm::SharedPtr msg)
{
    ROS_INFO("ServerInterfaceCore RobotRdmAlarmCallback");
    cm_msgs::msg::SpsRdmAlarm rdm_alarm = *msg;
    rdm_alarm_pub_->Publish(rdm_alarm);
}