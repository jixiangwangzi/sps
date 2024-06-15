#include <chrono>
#include <filesystem>
#include <fstream>

#include "system_manager/bt/robot_bt.h"
#include "system_manager/bt_node_impl.hpp"

rclcpp::Logger NODEIMPL_LOG = rclcpp::get_logger("BtNodeImpl");

using namespace std::chrono_literals;

namespace naviengine
{

BTNodeImpl::BTNodeImpl(std::shared_ptr<LocalManager> localManager, std::shared_ptr<TaskManager> taskManager)
    : Node("BTNodeImpl")
{
    ROS_INFO("hello this is BTNodeImpl");
    parameter_init();
    client_init();
    subscription_init();
    publish_init();

    localManager_ = localManager;
    taskManager_ = taskManager;
    NavStateFun navi_state_fun = std::bind(&BTNodeImpl::NavStateCallback, this, std::placeholders::_1);
    taskManager_->registNavStateCallback(navi_state_fun);
}

BTNodeImpl::~BTNodeImpl()
{
}

void BTNodeImpl::parameter_init()
{
    this->declare_parameter<float>("check_move_distance_from_goal_threshold", check_move_distance_from_goal_threshold_);
    this->get_parameter("check_move_distance_from_goal_threshold", check_move_distance_from_goal_threshold_);

    this->declare_parameter<int>("check_move_distance_interval", check_move_distance_interval_);
    this->get_parameter("check_move_distance_interval", check_move_distance_interval_);

    this->declare_parameter<float>("max_move_distance_threshold", max_move_distance_threshold_);
    this->get_parameter("max_move_distance_threshold", max_move_distance_threshold_);
}

void BTNodeImpl::publish_init()
{
    //slam_initial_pose_pub_ = this->create_publisher<cm_msgs::msg::InitialSlamPose>("/initialpose_self", 1);
    robot_rdm_alarm_pub_ = this->create_publisher<cm_msgs::msg::SpsRdmAlarm>("/robot_rdm_alarm",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    navi_vel_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/navi_vel_cmd",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void BTNodeImpl::client_init()
{
    load_map_client_ = std::make_shared<LoadNaviMapServiceClient>();
    query_lift_check_pose_client_ = std::make_shared<QueryLiftCheckPoseServiceClient>();
    get_lift_poi_info_client_ = std::make_shared<GetLiftPoiInfoServiceClient>();
    get_poses_plan_client_ = std::make_shared<GetPosesPlanServiceClient>();
}

void BTNodeImpl::subscription_init()
{
    battery_state_sub_ = this->create_subscription<cm_msgs::msg::BatteryState>(
        "/BatteryState", 1, std::bind(&BTNodeImpl::battery_state_callback, this, std::placeholders::_1));
    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 1, std::bind(&BTNodeImpl::global_path_callback, this, std::placeholders::_1));
}

void BTNodeImpl::battery_state_callback(const cm_msgs::msg::BatteryState::SharedPtr msg)
{
    if (charge_state_ != msg->chargeset_type)
    {
        ROS_INFO("BTNodeImpl::battery_state_callback msg->chargeset_type:%d", msg->chargeset_type);
        charge_state_ = msg->chargeset_type;
        if (charge_state_ == DOCK || charge_state_ == BOTH)
        {
            localManager_->prepareInitPose(CHARGING_PILE);
        }
    }
}

void BTNodeImpl::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    ROS_INFO("BTNodeImpl::global_path_callback poses.size:%ld", msg->poses.size());
    if (msg->poses.empty())
    {
        return;
    }

    NaviPathInfo navi_path_info;
    navi_path_info.path.clear();
    for (const auto &w_pose : msg->poses)
    {
        sps_common_msgs::msg::PixelPose pose;
        changeWordToPixel(w_pose.pose, pose);
        PixelPose pixel_pose;
        pixel_pose.x = pose.x;
        pixel_pose.y = pose.y;
        pixel_pose.theta = pose.theta;
        navi_path_info.path.push_back(pixel_pose);
    }
    sps_common_msgs::msg::PixelPose curr_pose;
    localManager_->getRobotPose(curr_pose);
    navi_path_info.curr_poi.x = curr_pose.x;
    navi_path_info.curr_poi.y = curr_pose.y;
    navi_path_info.curr_poi.theta = curr_pose.theta;

    sps_common_msgs::msg::PixelPose target_pose;
    if(task_type_ == ETASKTYPE::NAVI_TO_POSE || task_type_ == ETASKTYPE::NAVI_LIFT)
    {
        changeWordToPixel(navi_pose_.pose, target_pose);
    }
    else if(task_type_ == ETASKTYPE::NAVI_WAYPOINTS)
    {
        changeWordToPixel(waypoints_poses_.back().pose, target_pose);
    }
    navi_path_info.target_poi.x = (float)target_pose.x;
    navi_path_info.target_poi.y = (float)target_pose.y;
    navi_path_info.target_poi.theta = (float)target_pose.theta;

    navi_path_info.task_id = task_id_;
    navi_path_info.map_id = localManager_->getMapInfo("id");
    navi_path_info.map_name = localManager_->getMapInfo("name");

    RobotBt::GetInstance()->OnUpdateNaviPath(navi_path_info);
}

void BTNodeImpl::NavStateCallback(NaviStateInfo navi_state_info)
{
    navi_state_info.navi_type = navi_type_;
    navi_state_info.task_id = task_id_;

    if(navi_state_info.navi_state == NAVISTATE::NAVI_STATE_SUCCESS &&
        navi_type_ == NAVI_GOINLIFT && liftin_step_ == 1)
    {
        liftin_step_ = 2;
        naviGoInLift();
        return;
    }

    if(navi_type_ == NAVI_GOINLIFT && liftin_step_ == 2 && liftin_status_ &&
        (navi_state_info.navi_state == NAVISTATE::NAVI_STATE_FAILED ||
        navi_state_info.navi_state == NAVISTATE::NAVI_STATE_CANCELED))
    {
        navi_state_info.navi_state = NAVISTATE::NAVI_STATE_SUCCESS;
    }

    if(navi_state_info.navi_state == NAVISTATE::NAVI_STATE_ACTIVE ||
        navi_state_info.navi_state == NAVISTATE::NAVI_STATE_RUNNING)
    {
        if(!record_pose_state_)
        {
            setRecordPoseState(true);
        }

        if(!trapped_check_state_)
        {
            setTrappedCheckState(true);
        }
    }
    else
    {
        if(trapped_check_state_)
        {
            setTrappedCheckState(false);
        }

        if(navi_state_info.navi_state == NAVISTATE::NAVI_STATE_SUCCESS ||
            navi_state_info.navi_state == NAVISTATE::NAVI_STATE_FAILED ||
            navi_state_info.navi_state == NAVISTATE::NAVI_STATE_CANCELED)
        {
            setRecordPoseState(false);
            double distance = calculateDistance(robot_history_world_poses_);
            navi_state_info.mileage = distance;
        }
    }

    RobotBt::GetInstance()->OnUpdateNaviState(navi_state_info);
}

bool BTNodeImpl::IsChargingLine()
{
    return charge_state_ == LINE || charge_state_ == BOTH;
}

ERESULT BTNodeImpl::loadMap(std::string map_path)
{
    localManager_->setDepthScanState(true);

    ERESULT result = E_LOADMAP_FAILED;

    auto load_map_req = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Request>();
    load_map_req->header.frame_id = "map";
    load_map_req->header.stamp = rclcpp::Time();
    load_map_req->map_path = map_path;

    auto load_map_res = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Response>();
    if (load_map_client_->call(load_map_req, load_map_res, std::chrono::seconds(8)))
    {
        result = (ERESULT)load_map_res->result;
        ROS_INFO(
            "BTNodeImpl::loadMap result: %d, description: %s", load_map_res->result, load_map_res->description.c_str());

        if (load_map_res->result == sps_common_msgs::srv::SpsLoadMap::Response::SUCCESS && localManager_->updateMapInfo())
        {
            localManager_->prepareInitPose(SWITCH_MAP_WITH_SAVED_POSE);
        }
    }

    return result;
}

ERESULT BTNodeImpl::setWorldPose(const Pose& w_pose, const LOCALINITTYPE& pose_type, const SENSORMATCHTYPE& match_type, const PoiType& poi_type)
{
    ERESULT result = E_SETPOSE_FAILED;

    geometry_msgs::msg::Pose init_pose;
    init_pose.position.x = w_pose.position.x;
    init_pose.position.y = w_pose.position.y;
    init_pose.position.z = w_pose.position.z;
    init_pose.orientation.x = w_pose.orientation.x;
    init_pose.orientation.y = w_pose.orientation.y;
    init_pose.orientation.z = w_pose.orientation.z;
    init_pose.orientation.w = w_pose.orientation.w;

    SCENETYPE scene_type;
    if(poi_type == LIFT_INNER_POI)
    {
        scene_type = LIFT_INTERIOR;
    }
    else
    {
        scene_type = CLOUD_INSTRUCTION;
    }
    result = localManager_->setWorldPose(init_pose, pose_type, match_type, scene_type);

    if(result == E_OK)
    {
        for (int i = 0; i < init_pose_wait_timeout_; ++i)
        {
            if (localManager_->isInitComplete())
            {
                ROS_INFO("BTNodeImpl::setWorldPose set pose successfully!");
                return result;
            }
            rclcpp::Rate(1).sleep();
        }
    }

    return result;
}

bool BTNodeImpl::changeWordToPixel(const geometry_msgs::msg::Pose& w_pose, sps_common_msgs::msg::PixelPose& pixel_pose)
{
    return localManager_->changeWordPoseToPixelPose(w_pose, pixel_pose);
}

bool BTNodeImpl::changePixelToWord(const sps_common_msgs::msg::PixelPose& pixel_pose, geometry_msgs::msg::Pose& w_pose)
{
    return localManager_->changePixelPoseToWordPose(pixel_pose, w_pose);
}

void BTNodeImpl::changeStructPoseToMsgPose(const Pose& pose, geometry_msgs::msg::Pose& msg_pose)
{
    msg_pose.position.x = pose.position.x;
    msg_pose.position.y = pose.position.y;
    msg_pose.position.z = pose.position.z;
    msg_pose.orientation.x = pose.orientation.x;
    msg_pose.orientation.y = pose.orientation.y;
    msg_pose.orientation.z = pose.orientation.z;
    msg_pose.orientation.w = pose.orientation.w;
}

bool BTNodeImpl::getLiftCheckPoi(const std::string& lift_id, sps_common_msgs::msg::PixelPose& pixel_pose)
{
    ROS_INFO("BTNodeImpl::getLiftCheckPoi lift_id:%s", lift_id.c_str());
    auto get_lift_check_poi_req = std::make_shared<sps_common_msgs::srv::GetLiftCheckPixelPoint::Request>();
    get_lift_check_poi_req->header.frame_id = "map";
    get_lift_check_poi_req->header.stamp = rclcpp::Time();
    get_lift_check_poi_req->lift_id = lift_id;

    auto get_lift_check_poi_res = std::make_shared<sps_common_msgs::srv::GetLiftCheckPixelPoint::Response>();
    if (query_lift_check_pose_client_->call(get_lift_check_poi_req, get_lift_check_poi_res))
    {
        ROS_INFO(
            "BTNodeImpl::getLiftCheckPoi result: %d, description: %s", get_lift_check_poi_res->result, get_lift_check_poi_res->description.c_str());

        if (get_lift_check_poi_res->result == sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SUCCESS)
        {
            pixel_pose = get_lift_check_poi_res->pixel_pose;
            return true;
        }
    }
    return false;
}

bool BTNodeImpl::getLiftPoiInfo(const std::string& lift_id)
{
    ROS_INFO("BTNodeImpl::getLiftPoiInfo");
    auto get_lift_poi_info_req = std::make_shared<sps_common_msgs::srv::GetLiftPoiInfo::Request>();
    get_lift_poi_info_req->header.frame_id = "map";
    get_lift_poi_info_req->header.stamp = rclcpp::Time();
    get_lift_poi_info_req->lift_id = lift_id;

    auto get_lift_poi_info_res = std::make_shared<sps_common_msgs::srv::GetLiftPoiInfo::Response>();
    if (get_lift_poi_info_client_->call(get_lift_poi_info_req, get_lift_poi_info_res))
    {
        ROS_INFO(
            "BTNodeImpl::getLiftPoiInfo result: %d, description: %s", get_lift_poi_info_res->result, get_lift_poi_info_res->description.c_str());

        if (get_lift_poi_info_res->result == sps_common_msgs::srv::GetLiftPoiInfo::Response::SUCCESS)
        {
            lift_door_poi_ = get_lift_poi_info_res->door_poi;
            return true;
        }
    }
    return false;
}

bool BTNodeImpl::naviGoInLift()
{
    ROS_INFO("BTNodeImpl::naviGoInLift liftin_step_:%d", liftin_step_);
    liftin_status_ = false;
    if(liftin_step_ == 1)
    {
        sps_common_msgs::msg::PixelPose pixel_pose;
        if(getLiftPoiInfo(navi_lift_id_) && getLiftCheckPoi(navi_lift_id_, pixel_pose))
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map";
            pose_stamped.header.stamp = rclcpp::Time();
            if(changePixelToWord(pixel_pose, pose_stamped.pose))
            {
                navi_pose_ = pose_stamped;
                return taskManager_->naviLiftImpl(task_id_, navi_type_, pose_stamped);
            }
        }
    }
    else if(liftin_step_ == 2)
    {
        navi_pose_ = lift_inner_pose_;
        bool result = taskManager_->naviLiftImpl(task_id_, navi_type_, lift_inner_pose_);
        if(result)
        {
            liftin_check_timer_ = this->create_wall_timer(1s, std::bind(&BTNodeImpl::liftinChcekCallback, this));
        }
        return result;
    }
    return false;
}

ERESULT BTNodeImpl::startNaviToPose(const Header& header,
                                    const std::string& task_id,
                                    const std::string& lift_id,
                                    const ENAVITYPE& navi_type,
                                    const Pose& pose,
                                    const Twist& twist,
                                    const std::string& bt_xml_filename,
                                    const bool align_angle)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Time();
    changeStructPoseToMsgPose(pose, pose_stamped.pose);

    task_id_ = task_id;
    navi_type_ = navi_type;
    navi_pose_ = pose_stamped;
    navi_lift_id_ = lift_id;
    liftin_step_ = 0;
    liftin_status_ = false;

    if(navi_type == NAVI_GOINLIFT || navi_type == NAVI_GOOUTLIFT)
    {
        if(!lift_id.empty())
        {
            task_type_ = ETASKTYPE::NAVI_LIFT;
            if(navi_type == NAVI_GOOUTLIFT)
            {
                if (taskManager_->naviLiftImpl(task_id, navi_type, pose_stamped))
                {
                    return E_STARTNAVI_SUCCESS;
                }
            }
            else
            {
                liftin_step_ = 1;
                lift_inner_pose_ = pose_stamped;
                if(naviGoInLift())
                {
                    return E_STARTNAVI_SUCCESS;
                }
            }
        }
    }
    else
    {
        task_type_ = ETASKTYPE::NAVI_TO_POSE;
        if (taskManager_->naviToPoseImpl(task_id, navi_type, pose_stamped))
        {
            return E_STARTNAVI_SUCCESS;
        }
    }

    return E_STARTNAVI_FAILED;
}

ERESULT BTNodeImpl::startNaviThroughPoses(const Header& header,
                                          const std::string& task_id,
                                          const ENAVITYPE& navi_type,
                                          const std::vector<Pose>& points,
                                          const std::vector<int>& points_type,
                                          const Twist& twist,
                                          const std::string& bt_xml_filename,
                                          const bool align_angle)
{
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    for (auto& tmp_navi_point : points)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = rclcpp::Time();
        changeStructPoseToMsgPose(tmp_navi_point, pose_stamped.pose);
        poses.push_back(pose_stamped);
    }

    std::vector<uint8_t> poses_type;
    for (auto& tmp_poi_type : points_type)
    {
        poses_type.push_back((uint8_t)tmp_poi_type);
    }

    task_id_ = task_id;
    navi_type_ = navi_type;
    task_type_ = ETASKTYPE::NAVI_WAYPOINTS;
    waypoints_poses_ = poses;
    liftin_step_ = 0;
    liftin_status_ = false;
    if (taskManager_->followWaypointsImpl(task_id, navi_type, poses, poses_type))
    {
        return E_STARTNAVI_SUCCESS;
    }

    return E_STARTNAVI_FAILED;
}

ERESULT BTNodeImpl::pauseNavi(const std::string& task_id)
{
    return taskManager_->pauseNaviTask();
}

ERESULT BTNodeImpl::resumeNavi(const std::string& task_id)
{
    return taskManager_->resumeNaviTask();
}

ERESULT BTNodeImpl::stopNavi(const std::string& task_id)
{
    if (task_type_ == ETASKTYPE::NAVI_TO_POSE)
    {
        return taskManager_->cancelNaviToPose();
    }
    else if (task_type_ == ETASKTYPE::NAVI_WAYPOINTS)
    {
        return taskManager_->cancelFollowWaypoints();
    }
    else if (task_type_ == ETASKTYPE::NAVI_LIFT)
    {
        return taskManager_->cancelNaviLift();
    }
    else
    {
        ROS_WARN("BTNodeImpl::stopNavi fail, curr task_type_:%d", task_type_);
        return E_STOPNAVI_FAILED;
    }
}

ERESULT BTNodeImpl::setSlamModel(const int slam_model)
{
    return localManager_->setSlamModel(slam_model);
}

ERESULT BTNodeImpl::setMaxNaviVel(const double linear_max, const double angular_max)
{
    geometry_msgs::msg::Twist max_vel;
    max_vel.linear.x = linear_max;
    max_vel.angular.z = angular_max;
    navi_vel_cmd_pub_->publish(max_vel);
    return E_OK;
}

double BTNodeImpl::getNaviPosesMileage(const Pose &start_pose, const Pose &target_pose)
{
    double mileage = 0.0;
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.header.stamp = rclcpp::Time();
    changeStructPoseToMsgPose(start_pose, start.pose);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Time();
    changeStructPoseToMsgPose(target_pose, goal.pose);

    auto get_plan_req = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    get_plan_req->start = start;
    get_plan_req->goal = goal;

    auto get_plan_res = std::make_shared<nav_msgs::srv::GetPlan::Response>();
    if (get_poses_plan_client_->call(get_plan_req, get_plan_res))
    {
        std::vector<geometry_msgs::msg::PoseStamped> plan_poses = get_plan_res->plan.poses;
        ROS_INFO("BTNodeImpl::getNaviPosesMileage plan_poses.size(): %ld", plan_poses.size());

        if (plan_poses.size() > 0)
        {
            mileage = calculateDistance(plan_poses);
        }
    }

    return mileage;
}

void BTNodeImpl::robotAlarmHandle(bool alarm_state)
{
    taskManager_->robotAlarmHandle(alarm_state);
}

bool BTNodeImpl::checkLiftInsideState(double robot_x, double robot_y, double liftin_distance_thresh)
{
    bool inside = false;
    // 以门左边的点为原点求解向量
    if (lift_door_poi_.empty())
    {
        ROS_ERROR("BTNodeImpl::checkLiftInsideState lift_door_poi_ is empty");
        return inside;
    }
    double vec_door_x = lift_door_poi_[1].position.x - lift_door_poi_[0].position.x;
    double vec_door_y = lift_door_poi_[1].position.y - lift_door_poi_[0].position.y;
    double vec_robot_x = robot_x - lift_door_poi_[0].position.x;
    double vec_robot_y = robot_y - lift_door_poi_[0].position.y;
    // ROS_INFO("Lift door: (%0.3f, %0.3f), (%0.3f, %0.3f)", lift_door_poi_[0].position.x,
    //             lift_door_poi_[0].position.y, lift_door_poi_[1].position.x,
    //             lift_door_poi_[1].position.y);
    //向量叉乘
    double cross_value = vec_door_x * vec_robot_y - vec_door_y * vec_robot_x;

    if (cross_value > 0.0)  // >0说明机器人在门里面
    {
        double vec_door_norm = std::sqrt(vec_robot_x * vec_robot_x + vec_robot_y * vec_robot_y);
        double robot_dist = cross_value / vec_door_norm;
        ROS_INFO("BTNodeImpl::checkLiftInsideState robot's distance to lift door: %0.3f, threshold: %0.3f", robot_dist, liftin_distance_thresh);
        if (robot_dist > liftin_distance_thresh)
        {
            inside = true;
        }
    }
    return inside;
}

void BTNodeImpl::liftinChcekCallback()
{
    if (liftin_step_ == 2)
    {  // lift in
        geometry_msgs::msg::Pose cur_pose;
        if (!localManager_->getRobotPose(cur_pose))
        {
            ROS_WARN("BTNodeImpl::liftinChcekCallback getRobotPose failed!");
            return;
        }

        if (checkLiftInsideState(cur_pose.position.x, cur_pose.position.y, liftin_distance_thresh_))
        {
            liftin_status_ = true;
            ROS_INFO("BTNodeImpl::liftinChcekCallback Reach lift safe line successfully");
            taskManager_->cancelNaviLift();
            liftin_check_timer_->cancel();
        }
        else
        {
            liftin_status_ = false;
        }
    }
}

void BTNodeImpl::setRecordPoseState(bool state)
{
    ROS_INFO("BTNodeImpl::setRecordPoseState state:%d", state);
    record_pose_state_ = state;
    if(state)
    {
        robot_history_world_poses_.clear();
        record_pose_timer_ = this->create_wall_timer(1s, std::bind(&BTNodeImpl::recordPoseTimerCallback, this));
    }
    else
    {
        record_pose_timer_->cancel();
    }
}

void BTNodeImpl::recordPoseTimerCallback()
{
    geometry_msgs::msg::Pose pose;
    if (!localManager_->getRobotPose(pose))
    {
        ROS_WARN("BTNodeImpl::recordPoseTimerCallback getRobotPose failed!");
        return;
    }

    robot_history_world_poses_.push_back(pose);
}

double BTNodeImpl::calculateDistance(const std::vector<geometry_msgs::msg::Pose> poses)
{
    if (poses.empty()) return 0.0;
    double distance = 0.0;
    for (int i = 0; i < poses.size() - 1; ++i) {
        double diff_x = poses[i].position.x - poses[i + 1].position.x;
        double diff_y = poses[i].position.y - poses[i + 1].position.y;
        distance += std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
    }
    ROS_INFO("BTNodeImpl::CalculateDistance poses.size():%ld, distance:%f", poses.size(), distance);
    return distance;
}

double BTNodeImpl::calculateDistance(const std::vector<geometry_msgs::msg::PoseStamped> poses)
{
    if (poses.empty()) return 0.0;

    double distance = 0.0;
    for (int i = 0; i < poses.size() - 1; ++i) {
        double diff_x = poses[i].pose.position.x - poses[i + 1].pose.position.x;
        double diff_y = poses[i].pose.position.y - poses[i + 1].pose.position.y;
        distance += std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));
    }
    ROS_INFO("BTNodeImpl::CalculateDistance poses.size():%ld, distance:%f", poses.size(), distance);
    return distance;
}

void BTNodeImpl::setTrappedCheckState(bool state)
{
    ROS_INFO("BTNodeImpl::setTrappedCheckState state:%d", state);
    trapped_check_state_ = state;
    if(state)
    {
        if(task_type_ == ETASKTYPE::NAVI_TO_POSE || task_type_ == ETASKTYPE::NAVI_LIFT)
        {
            end_pose_ = navi_pose_.pose;
        }
        else if(task_type_ == ETASKTYPE::NAVI_WAYPOINTS)
        {
            end_pose_ = waypoints_poses_.back().pose;
        }

        trapped_check_poses_.clear();
        trapped_check_timer_ = this->create_wall_timer(1s, std::bind(&BTNodeImpl::trappedCheckTimerCallback, this));
    }
    else
    {
        trapped_check_timer_->cancel();
    }
}

void BTNodeImpl::trappedCheckTimerCallback()
{
    geometry_msgs::msg::Pose robot_curr_pose;
    if (!localManager_->getRobotPose(robot_curr_pose))
    {
        ROS_WARN("BTNodeImpl::trappedCheckTimerCallback getRobotPose failed!");
        return;
    }

    //auto distance_from_current_to_end =
    //    (float)sqrt(pow((robot_curr_pose.position.x - end_pose_.position.x), 2) + pow((robot_curr_pose.position.y - end_pose_.position.y), 2));
    //if (distance_from_current_to_end < check_move_distance_from_goal_threshold_)
    //{
    //    return;
    //}

    // ROS_INFO("robot_curr_pose: {%f, %f, %f}", robot_curr_pose.x, robot_curr_pose.y, robot_curr_pose.theta);
    if (trapped_check_poses_.size() < check_move_distance_interval_)
    {  // 记录15秒内的位置信息
        trapped_check_poses_.push_back(robot_curr_pose);
    }
    else
    {
        trapped_check_poses_.pop_front();  // 删除15秒之前的位置信息
        trapped_check_poses_.push_back(robot_curr_pose);
        float max_move_meter_distance = 0;
        std::string history_move_distances_string;
        std::string history_points_string;
        char tmp[32];
        for (const auto &item : trapped_check_poses_)
        {
            auto move_distance =
                (float)sqrt(pow((robot_curr_pose.position.x - item.position.x), 2) + pow((robot_curr_pose.position.y - item.position.y), 2));
            memset(tmp, '\0', sizeof(tmp));
            sprintf(tmp, "%.2f ", move_distance);
            history_move_distances_string += tmp;
            memset(tmp, '\0', sizeof(tmp));
            sprintf(tmp, "{%.2f, %.2f} ", item.position.x, item.position.y);
            history_points_string += tmp;
            if (move_distance > max_move_meter_distance)
            {
                max_move_meter_distance = move_distance;
            }
        }

        if (!trapped_reported_ && max_move_meter_distance < max_move_distance_threshold_)
        {  // 判断15秒内最大移动距离是否超过50厘米
            // 告警机器人受困
            RobotBt::GetInstance()->OnUpdateRetentionStatus(true, task_id_);

            cm_msgs::msg::SpsRdmAlarm sps_rdm;
            sps_rdm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_TRAPPED;
            sps_rdm.device_name = "Navigation";
            sps_rdm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
            sps_rdm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_MAJOR);
            robot_rdm_alarm_pub_->publish(sps_rdm);
            trapped_reported_ = true;
        }
        else if (trapped_reported_ && max_move_meter_distance > max_move_distance_threshold_)
        {
            // 解除机器人受困告警
            RobotBt::GetInstance()->OnUpdateRetentionStatus(false, task_id_);

            cm_msgs::msg::SpsRdmAlarm sps_rdm;
            sps_rdm.alarm_code = cm_msgs::msg::SpsRdmAlarm::ALM_NAVI_TRAPPED;
            sps_rdm.device_name = "Navigation";
            sps_rdm.alarm_type.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_PROCESSING_ERROR);
            sps_rdm.alarm_severity.push_back(cm_msgs::msg::SpsRdmAlarm::ALM_CLEARED);
            robot_rdm_alarm_pub_->publish(sps_rdm);
            trapped_reported_ = false;
        }
    }
}

}  // namespace naviengine