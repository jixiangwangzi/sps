#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "system_manager/local_manager.hpp"
#include "yaml-cpp/yaml.h"

rclcpp::Logger LOCALM_LOG = rclcpp::get_logger("LocalManager");

using namespace std::chrono_literals;

namespace naviengine
{

LocalManager::LocalManager() : Node("LocalManager")
{
    ROS_INFO("hello this is LocalManager");
    parameter_init();
    client_init();
    subscription_init();
    publish_init();

    publish_thread_ = std::make_shared<std::thread>(&LocalManager::writeRobotPose, this);

    load_default_map_timer_ = this->create_wall_timer(5s, std::bind(&LocalManager::loadDefaultMapTimerCallback, this));
}

LocalManager::~LocalManager()
{
    if (publish_thread_)
    {
        publish_thread_->join();
    }
}

void LocalManager::parameter_init()
{
    this->declare_parameter<std::string>("home_path", home_path_);
    this->get_parameter("home_path", home_path_);

    this->declare_parameter<double>("static_pose_upload_fps", static_pose_upload_fps_);
    this->get_parameter("static_pose_upload_fps", static_pose_upload_fps_);
}

void LocalManager::publish_init()
{
    pixel_pose_pub_ = this->create_publisher<sps_common_msgs::msg::PixelPose>(
        "/pixel_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    local_info_pub_ = this->create_publisher<sps_common_msgs::msg::LocalState>(
        "/local_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    depth_scan_switch_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/sps_depth_scan_switch", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void LocalManager::client_init()
{
    get_map_info_client_ = std::make_shared<GetMapInfoServiceClient>();
    get_charge_pile_client_ = std::make_shared<GetChargePileServiceClient>();
    set_aided_pose_client_ = std::make_shared<SetAidedPoseServiceClient>();
    load_map_client_ = std::make_shared<LoadNaviMapServiceClient>();
    set_slam_model_client_ = std::make_shared<SetSlamModelServiceClient>();
    publish_mapping_map_client_ = std::make_shared<PublishMappingMapServiceClient>();
}

void LocalManager::subscription_init()
{
    local_status_sub_ = this->create_subscription<sps_common_msgs::msg::LocalizationStatus>(
        "/localization_status", 1, std::bind(&LocalManager::local_status_callback, this, std::placeholders::_1));
    init_state_sub_ = this->create_subscription<sps_common_msgs::msg::InitialSlamState>(
        "/initial_slam_state", 1, std::bind(&LocalManager::init_state_callback, this, std::placeholders::_1));
    shutdown_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "/shutdown_cmd", 1, std::bind(&LocalManager::shutdown_callback, this, std::placeholders::_1));
    update_occ_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/update_occmap", 1, std::bind(&LocalManager::update_occ_map_callback, this, std::placeholders::_1));
}

void LocalManager::loadDefaultMapTimerCallback()
{
    load_default_map_timer_->cancel();

    if (loadDefaultMap() && updateMapInfo())
    {
        prepareInitPose(START_UP_WITH_SAVED_POSE);
    }

    setDepthScanState(true);
}

bool LocalManager::loadDefaultMap()
{
    std::string maplist_file{home_path_ + "/ginger_maps/release/maplist.json"};
    std::string map_path;
    ROS_INFO("LocalManager::loadDefaultMap maplist_file: %s", maplist_file.c_str());

    YAML::Node maplist;
    try
    {
        maplist = YAML::LoadFile(maplist_file);
        ROS_INFO("LocalManager::loadDefaultMap load maplist file successfully");
    }
    catch (YAML::Exception &e)
    {
        ROS_WARN("LocalManager::loadDefaultMap cant open map list file: %s", e.msg.c_str());
        return false;
    }
    try
    {
        auto default_map = maplist["currmap"];
        if (default_map.IsNull())
        {
            ROS_WARN("LocalManager::loadDefaultMap no default map defined");
            return false;
        }
        auto uuid = default_map["uuid"].as<std::string>();
        if (uuid.empty())
        {
            ROS_WARN("LocalManager::loadDefaultMap no uuid in default map");
            return false;
        }
        auto name = default_map["name"].as<std::string>();
        if (name.empty())
        {
            ROS_WARN("LocalManager::loadDefaultMap no name in default map");
            return false;
        }

        map_path = std::string{home_path_ + "/ginger_maps/release/" + uuid + "/" + name};
        ROS_INFO("LocalManager::loadDefaultMap map_path: %s", map_path.c_str());
    }
    catch (YAML::Exception &e)
    {
        ROS_WARN("LocalManager::loadDefaultMap error occurred when parsing maplist file: %s, Path: %s",
                 e.msg.c_str(),
                 maplist_file.c_str());
        return false;
    }

    auto load_map_req = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Request>();
    load_map_req->header.frame_id = "map";
    load_map_req->header.stamp = rclcpp::Time();
    load_map_req->map_path = map_path;

    auto load_map_res = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Response>();
    if (load_map_client_->call(load_map_req, load_map_res, std::chrono::seconds(8)))
    {
        ROS_INFO("LocalManager::loadDefaultMap result: %d, description: %s",
                 load_map_res->result,
                 load_map_res->description.c_str());
        if (load_map_res->result == sps_common_msgs::srv::SpsLoadMap::Response::SUCCESS)
        {
            return true;
        }
    }
    return false;
}

void LocalManager::local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg)
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_INFO_THROTTLE(LOCALM_LOG,
                         clock,
                         4000,
                         "local_status_callback status:%d, score:%f, spec_causes:%d",
                         msg->status,
                         msg->score,
                         msg->specific_causes);
    latest_loc_pose_ = msg->pose;
    latest_loc_status_ = (LOCALTYPE)msg->status;
    latest_loc_confidence_ = msg->score;
    latest_loc_map_update_ = msg->need_map_update;
}

void LocalManager::init_state_callback(const sps_common_msgs::msg::InitialSlamState::SharedPtr msg)
{
    if(initial_state_ != msg->initial_state)
    {
        ROS_INFO("LocalManager::init_state_callback initial_state_:%d, msg->initial_state:%d",
                initial_state_, msg->initial_state);
        initial_state_ = msg->initial_state;
    }
}

void LocalManager::shutdown_callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    int cmd = msg->data;
    ROS_INFO("LocalManager::shutdown_callback cmd:%d", cmd);
    // 1表示关机，2表示重启
    if (cmd == 1 || cmd == 2)
    {
        shutdown_ = true;
    }
}

void LocalManager::update_occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr occ_map) {
    ROS_INFO("LocalManager::update_occ_map_callback width:%d, height:%d", occ_map->info.width, occ_map->info.height);
    if(occ_map->info.width > 0 && occ_map->info.height > 0)
    {
        map_uuid_ = "mapping";
        map_name_ = "mapping";
        origin_x_ = occ_map->info.origin.position.x;
        origin_y_ = occ_map->info.origin.position.y;
        grid_width_ = occ_map->info.width;
        grid_height_ = occ_map->info.height;
    }
}

bool LocalManager::isLocalError()
{
    ROS_INFO("LocalManager::isLocalError latest_loc_status_:%d, curr_slam_model_:%d", latest_loc_status_, curr_slam_model_);
    if(curr_slam_model_ == SLAM_LOC)
    {
        if(latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::RUNNING &&
            latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::WEAK)
        {
            return true;
        }

        if(!isInitComplete())
        {
            return true;
        }
    }

    return false;
}

bool LocalManager::isInitComplete()
{
    ROS_INFO("LocalManager::isInitComplete initial_state_:%d, scene_type_:%d", initial_state_, scene_type_);
    if(initial_state_ < sps_common_msgs::msg::InitialSlamState::BUILD_CONSTRAINT)
    {
        return false;
    }
    else if(initial_state_ == sps_common_msgs::msg::InitialSlamState::BUILD_CONSTRAINT)
    {
        if(scene_type_ != LIFT_INTERIOR)
        {
            return false;
        }
    }

    return true;
}

bool LocalManager::getChargePilePose()
{
    auto get_charge_pile_req = std::make_shared<sps_common_msgs::srv::GetDefaultChargePile::Request>();
    get_charge_pile_req->header.frame_id = "map";
    get_charge_pile_req->header.stamp = rclcpp::Time();

    auto get_charge_pile_res = std::make_shared<sps_common_msgs::srv::GetDefaultChargePile::Response>();
    if (get_charge_pile_client_->call(get_charge_pile_req, get_charge_pile_res))
    {
        ROS_INFO("LocalManager::getChargePilePose result: %d, description: %s",
                 get_charge_pile_res->result,
                 get_charge_pile_res->description.c_str());

        if (get_charge_pile_res->result == sps_common_msgs::srv::GetDefaultChargePile::Response::SUCCESS)
        {
            initial_pose_ = get_charge_pile_res->pixel_pose;
            ROS_INFO("LocalManager::getChargePilePose initial_pose_: {%f, %f, %f}",
                     initial_pose_.x,
                     initial_pose_.y,
                     initial_pose_.theta);
            return true;
        }
    }
    return false;
}

void LocalManager::setPosePixelZero()
{
    sps_common_msgs::msg::PixelPose pixel_zero_pose;
    pixel_zero_pose.x = 5.0;
    pixel_zero_pose.y = 5.0;
    pixel_zero_pose.theta = 0.0;
    geometry_msgs::msg::Pose word_zero_pose;
    if (changePixelPoseToWordPose(pixel_zero_pose, word_zero_pose))
    {
        ROS_INFO("LocalManager::setPosePixelZero set (0,0,0) as initial pose");
        setWorldPose(word_zero_pose, INIT_POSE, NONE, UNKNOWN);
    }
}

void LocalManager::prepareInitPose(SCENETYPE scene_type)
{
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        bool res = false;
        if (scene_type == CHARGING_PILE)
        {
            res = getChargePilePose();
        }

        if (!res)
        {
            res = readRobotPoseFromFile();
        }

        ROS_INFO("LocalManager::prepareInitPose res:%d, scene_type:%d", res, (int)scene_type);

        if (res)
        {
            geometry_msgs::msg::Pose word_pose;
            if (changePixelPoseToWordPose(initial_pose_, word_pose))
            {
                setWorldPose(word_pose, INIT_POSE, ALL, scene_type);
            }
        }
        else
        {
            setPosePixelZero();
        }
    }
    else
    {
        ROS_WARN("LocalManager::loadDefaultMap no map info");
    }
}

bool LocalManager::updateMapInfo()
{
    bool result = false;
    map_uuid_ = "";
    map_name_ = "";
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    resolution_ = 0.0;
    grid_width_ = 0;
    grid_height_ = 0;

    auto get_map_info_req = std::make_shared<sps_common_msgs::srv::GetMapInfo::Request>();
    get_map_info_req->header.frame_id = "map";
    get_map_info_req->header.stamp = rclcpp::Time();

    auto get_map_info_res = std::make_shared<sps_common_msgs::srv::GetMapInfo::Response>();
    if (get_map_info_client_->call(get_map_info_req, get_map_info_res))
    {
        ROS_INFO("LocalManager::updateMapInfo result: %d, description: %s",
                 get_map_info_res->result,
                 get_map_info_res->description.c_str());

        if (get_map_info_res->result == sps_common_msgs::srv::GetMapInfo::Response::SUCCESS)
        {
            sps_common_msgs::msg::MapInfo mapInfo = get_map_info_res->map_info;
            map_uuid_ = mapInfo.map_uuid;
            map_name_ = mapInfo.map_name;
            origin_x_ = mapInfo.origin_x;
            origin_y_ = mapInfo.origin_y;
            resolution_ = mapInfo.resolution;
            grid_width_ = mapInfo.grid_width;
            grid_height_ = mapInfo.grid_height;
            result = true;
            ROS_INFO("LocalManager::updateMapInfo map_uuid_: %s, map_name_: %s", map_uuid_.c_str(), map_name_.c_str());
        }
    }
    return result;
}

std::string LocalManager::getMapInfo(const std::string &key)
{
    if (key == "id")
    {
        return map_uuid_;
    }
    else if (key == "name")
    {
        return map_name_;
    }
    else
    {
        return "";
    }
}

ERESULT LocalManager::setSlamModel(const int slam_model)
{
    ERESULT result = E_STARTBUILDMAP_FAILED;
    ROS_INFO("LocalManager::setSlamModel slam_model:%d, curr_slam_model_:%d", slam_model, curr_slam_model_);

    auto set_slam_model_req = std::make_shared<sps_common_msgs::srv::SetSlamModel::Request>();

    set_slam_model_req->header.frame_id = "map";
    set_slam_model_req->header.stamp = rclcpp::Time();
    set_slam_model_req->slam_mode = slam_model;

    auto set_slam_model_res = std::make_shared<sps_common_msgs::srv::SetSlamModel::Response>();
    if (set_slam_model_client_->call(set_slam_model_req, set_slam_model_res, std::chrono::seconds(5)))
    {
        ROS_INFO("LocalManager::setSlamModel result: %d, description: %s",
                 set_slam_model_res->result,
                 set_slam_model_res->description.c_str());

        result = (ERESULT)set_slam_model_res->result;
    }

    if(result == E_STARTBUILDMAP_SUCCESS)
    {
        if(slam_model == SLAM_LOC && curr_slam_model_ == SLAM_IDLE)
        {
            loadMappingMap();
            setDepthScanState(true);
        }
        curr_slam_model_ = slam_model;
    }

    return result;
}

void LocalManager::loadMappingMap()
{
    ROS_INFO("LocalManager::loadMappingMap");
    auto publish_mapping_map_req = std::make_shared<sps_common_msgs::srv::PublishMapData::Request>();
    publish_mapping_map_req->header.frame_id = "map";
    publish_mapping_map_req->header.stamp = rclcpp::Time();

    auto publish_mapping_map_res = std::make_shared<sps_common_msgs::srv::PublishMapData::Response>();
    if (publish_mapping_map_client_->call(publish_mapping_map_req, publish_mapping_map_res, std::chrono::seconds(5)))
    {
        ROS_INFO("LocalManager::loadMappingMap result: %d, description: %s",
                 publish_mapping_map_res->result,
                 publish_mapping_map_res->description.c_str());
    }
}

void LocalManager::setDepthScanState(const bool &state)
{
    ROS_INFO("LocalManager::setDepthScanState state:%d", state);
    std_msgs::msg::Bool msg;
    msg.data = state;
    depth_scan_switch_pub_->publish(msg);
}

ERESULT LocalManager::setWorldPose(const geometry_msgs::msg::Pose &init_pose,
                                   const LOCALINITTYPE &pose_type,
                                   const SENSORMATCHTYPE &match_type,
                                   const SCENETYPE &scene_type)
{
    ERESULT result = E_SETPOSE_FAILED;
    ROS_INFO("LocalManager::setWorldPose init_pose: {%f, %f, %f}, match_type: %d, scene_type: %d",
             init_pose.position.x,
             init_pose.position.y,
             tf2::getYaw<geometry_msgs::msg::Quaternion>(init_pose.orientation),
             match_type,
             scene_type);

    auto aided_pose_req = std::make_shared<sps_common_msgs::srv::AidedPose::Request>();
    std::vector<geometry_msgs::msg::Pose> pose_vector;
    pose_vector.push_back(init_pose);

    aided_pose_req->header.frame_id = "map";
    aided_pose_req->header.stamp = rclcpp::Time();
    aided_pose_req->poses = pose_vector;
    aided_pose_req->pose_type = (int)pose_type;
    aided_pose_req->match_type = (int)match_type;
    aided_pose_req->scene_type = (int)scene_type;

    auto aided_pose_res = std::make_shared<sps_common_msgs::srv::AidedPose::Response>();
    if (set_aided_pose_client_->call(aided_pose_req, aided_pose_res, std::chrono::seconds(5)))
    {
        ROS_INFO("LocalManager::setWorldPose result: %d, description: %s",
                 aided_pose_res->result,
                 aided_pose_res->description.c_str());

        result = (ERESULT)aided_pose_res->result;
        scene_type_ = scene_type;
    }

    return result;
}

bool LocalManager::changePixelPoseToWordPose(const sps_common_msgs::msg::PixelPose &p_pose,
                                             geometry_msgs::msg::Pose &w_pose)
{
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        w_pose.position.x = p_pose.x * resolution_ + origin_x_;
        w_pose.position.y = (grid_height_ - p_pose.y - 1) * resolution_ + origin_y_;

        double yaw = (450 - p_pose.theta) * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        w_pose.orientation = tf2::toMsg(q);

        return true;
    }
    else
    {
        ROS_ERROR("LocalManager::ChangePixelPoseToWordPose map info is empty");
        return false;
    }
}

bool LocalManager::changeWordPoseToPixelPose(const geometry_msgs::msg::Pose &w_pose,
                                             sps_common_msgs::msg::PixelPose &p_pose)
{
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        int x = static_cast<int>((w_pose.position.x - origin_x_) / resolution_);
        int y = static_cast<int>(grid_height_ - (w_pose.position.y - origin_y_) / resolution_ - 1);
        double yaw = tf2::getYaw<geometry_msgs::msg::Quaternion>(w_pose.orientation);

        p_pose.stamp = rclcpp::Time();

        int theta = (static_cast<int>(yaw * 180 / M_PI) + 360) % 360;
        p_pose.x = static_cast<float>(x);
        p_pose.y = static_cast<float>(y);
        p_pose.theta = static_cast<float>((450 - theta) % 360);

        return true;
    }
    else
    {
        ROS_ERROR("LocalManager::ChangeWordPoseToPixelPose map info is empty");
        return false;
    }
}

bool LocalManager::getRobotPose(geometry_msgs::msg::Pose& w_pose)
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        w_pose = latest_loc_pose_;
        return true;
    }
    else
    {
        RCLCPP_INFO_THROTTLE(LOCALM_LOG, clock, 4000, "LocalManager::getRobotPose w_pose map info is empty");
        return false;
    }
}

bool LocalManager::getRobotPose(sps_common_msgs::msg::PixelPose &pose)
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        /**if(latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::RUNNING &&
            latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::WEAK) {
            RCLCPP_INFO_THROTTLE(LOCALM_LOG, clock, 4000, "getRobotPose PixelPose fail, loc_status is not ok");
            return false;
        }**/

        changeWordPoseToPixelPose(latest_loc_pose_, pose);

        pose.cur_mapid = map_uuid_;
        pose.loc_confidence = latest_loc_confidence_;
        pose.loc_threshold = 0.6;

        //在log中记录机器人的轨迹
        RCLCPP_INFO_THROTTLE(
            LOCALM_LOG,
            clock,
            2000,
            "Debug_for_pose: pixel: %0.2f, %0.2f, %0.2f, world: %0.2f, %0.2f, w x h: %d x %d, ori: %0.2f, %0.2f",
            pose.x,
            pose.y,
            pose.theta,
            latest_loc_pose_.position.x,
            latest_loc_pose_.position.y,
            grid_width_,
            grid_height_,
            origin_x_,
            origin_y_);

        return true;
    }
    else
    {
        RCLCPP_INFO_THROTTLE(LOCALM_LOG, clock, 4000, "LocalManager::getRobotPose PixelPose map info is empty");
        return false;
    }
}

bool LocalManager::getLocalInfo(sps_common_msgs::msg::LocalState &localInfo)
{
    rclcpp::Clock clock(RCL_ROS_TIME);
    if (grid_width_ > 0 && grid_height_ > 0)
    {
        /**if(latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::RUNNING &&
            latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::WEAK) {
            RCLCPP_INFO_THROTTLE(LOCALM_LOG, clock, 4000, "getLocalInfo fail, loc_status is not ok");
            return false;
        }**/

        int grid_x = static_cast<int>((latest_loc_pose_.position.x - origin_x_) / resolution_);
        int grid_y = static_cast<int>(grid_height_ - (latest_loc_pose_.position.y - origin_y_) / resolution_ - 1);
        double yaw = tf2::getYaw<geometry_msgs::msg::Quaternion>(latest_loc_pose_.orientation);

        int theta = (static_cast<int>(yaw * 180 / M_PI) + 360) % 360;
        float grid_theta = static_cast<float>((450 - theta) % 360);

        localInfo.header.frame_id = "map";
        localInfo.header.stamp = rclcpp::Time();
        localInfo.localization_state = latest_loc_status_;
        localInfo.map_uuid = map_uuid_;
        localInfo.map_name = map_name_;
        localInfo.confidence = latest_loc_confidence_;
        localInfo.threshold = 0.6;
        localInfo.map_update = latest_loc_map_update_;
        localInfo.grid_x = grid_x;
        localInfo.grid_y = grid_y;
        localInfo.grid_theta = grid_theta;
        localInfo.world_x = latest_loc_pose_.position.x;
        localInfo.world_y = latest_loc_pose_.position.y;
        localInfo.world_theta = yaw;

        //在log中记录机器人的轨迹
        RCLCPP_INFO_THROTTLE(
            LOCALM_LOG,
            clock,
            2000,
            "Debug_for_pose: pixel: %d, %d, %0.2f, world: %0.2f, %0.2f, %0.2f, w x h: %d x %d, ori: %0.2f, %0.2f",
            localInfo.grid_x,
            localInfo.grid_y,
            localInfo.grid_theta,
            localInfo.world_x,
            localInfo.world_y,
            localInfo.world_theta,
            grid_width_,
            grid_height_,
            origin_x_,
            origin_y_);

        return true;
    }
    else
    {
        RCLCPP_INFO_THROTTLE(LOCALM_LOG, clock, 4000, "LocalManager::getLocalInfo map info is empty");
        return false;
    }
}

bool LocalManager::readRobotPoseFromFile()
{
    // load initial pose
    bool valid = false;
    initial_pose_.x = 0.0;
    initial_pose_.y = 0.0;
    initial_pose_.theta = 0.0;
    YAML::Node pose_node;
    posefile_lock_.lock();
    std::string pose_file;
    try
    {
        pose_file = home_path_ + "/.cur_pose.yaml";
        ROS_INFO("LocalManager::readRobotPoseFromFile pose_file: %s", pose_file.c_str());
        pose_node = YAML::LoadFile(pose_file);
        ROS_INFO("LocalManager::readRobotPoseFromFile LoadFile content: \n%s", Dump(pose_node).c_str());
        ROS_INFO("===========================================================\n\n");

        std::string mapId = map_uuid_;
        std::string mapName = map_name_;
        if (pose_node["map_uuid"].as<std::string>() == mapId && pose_node["map_name"].as<std::string>() == mapName)
        {
            initial_pose_.x = pose_node["robot_pose"][0].as<float>();
            initial_pose_.y = pose_node["robot_pose"][1].as<float>();
            initial_pose_.theta = pose_node["robot_pose"][2].as<float>();
            valid = true;
            ROS_INFO("LocalManager::readRobotPoseFromFile initial pose from pose file: %f, %f, %f",
                     initial_pose_.x,
                     initial_pose_.y,
                     initial_pose_.theta);
        }
        else
        {
            ROS_ERROR("LocalManager::readRobotPoseFromFile map uuid & name is differente: %s, %s",
                      pose_node["map_uuid"].as<std::string>().c_str(),
                      mapId.c_str());
        }
    }
    catch (const YAML::Exception &e)
    {
        ROS_WARN("LocalManager::readRobotPoseFromFile can't parse pose file: %s\n ERROR: %s %s\n",
                 pose_file.c_str(),
                 e.msg.c_str(),
                 e.what());
    }
    posefile_lock_.unlock();
    return valid;
}

void LocalManager::writeRobotPose()
{
    rclcpp::Rate rate(static_pose_upload_fps_);
    std::string pose_file(home_path_ + "/.cur_pose.yaml");
    // YAML::Node robot_pose; // = YAML::LoadFile(pose_file);

    while (rclcpp::ok())
    {
        sps_common_msgs::msg::LocalState localInfo;
        if (!shutdown_ && getLocalInfo(localInfo))
        {
            local_info_pub_->publish(localInfo);

            posefile_lock_.lock();
            std::ofstream fout(pose_file);
            if (fout.is_open())
            {
                YAML::Emitter out;
                out << YAML::BeginMap;
                out << YAML::Key << "map_uuid";
                out << YAML::Value << map_uuid_;
                out << YAML::Key << "map_name";
                out << YAML::Value << map_name_;
                out << YAML::Key << "map_time";
                out << YAML::Value << localInfo.header.stamp.sec;

                out << YAML::Key << "robot_pose";
                out << YAML::Value << YAML::BeginSeq << localInfo.grid_x << localInfo.grid_y << localInfo.grid_theta
                    << YAML::EndSeq;
                out << YAML::EndMap;

                fout << out.c_str();
            }
            fout.close();
            sync();
            posefile_lock_.unlock();
        }
        rate.sleep();
    }
}

}  // namespace naviengine