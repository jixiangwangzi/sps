#include "map_server/map_server_core.hpp"

#include <filesystem>
#include <fstream>
#include <chrono>
#include "yaml-cpp/yaml.h"

rclcpp::Logger MAPCORE_LOG = rclcpp::get_logger("MapServerCore");

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapServerCore>();
    rclcpp::executors::SingleThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}

MapServerCore::MapServerCore() : Node("MapServerCore") {
    RCLCPP_INFO(MAPCORE_LOG, "hello this is MapServerCore");
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    transforms_init();
    parameter_init();
    client_init();
    service_init();
    publish_init();
    subscription_init();
    map_running_ = true;

    mapDatabase = MapDatabase::getInstance();
    mapDatabase->init(home_path_);
    poseManager = PoseManager::getInstance();
    liftManager = LiftManager::getInstance();
    service_client_ = std::make_shared<SeviceClient>();

    //publish_thread_ = std::make_shared<std::thread>(&MapServerCore::writeRobotPose, this);

    //loadDefaultMap();
}

void MapServerCore::loadDefaultMap() {
    sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Request>();
    sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Response>();
    load_default_map_service(request, response);
    publishInitialPose();
}

MapServerCore::~MapServerCore() {
    map_running_ = false;
    //if (publish_thread_) {
    //    publish_thread_->join();
    //}
}

void MapServerCore::transforms_init() {

}

void MapServerCore::parameter_init() {
    this->declare_parameter<std::string>("home_path", home_path_);
    this->get_parameter("home_path", home_path_);

    this->declare_parameter<double>("static_pose_upload_fps", static_pose_upload_fps_);
    this->get_parameter("static_pose_upload_fps", static_pose_upload_fps_);
}

void MapServerCore::client_init() {
    set_aided_pose_client_ = this->create_client<sps_common_msgs::srv::AidedPose>("/set_aided_pose");
    load_map_client_ = this->create_client<sps_common_msgs::srv::SpsLoadMap>("/load_map");
    update_map_client_ = this->create_client<sps_common_msgs::srv::UpdateMap>("/update_map");
    mode_service_client_ = this->create_client<sps_common_msgs::srv::Mode>("/mode_type");
}

void MapServerCore::service_init() {
    load_2d_test_map_server_ = this->create_service<sps_common_msgs::srv::SpsLoadMap>("/load_2d_test_map",
                                            std::bind(&MapServerCore::load_2d_test_map_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    load_default_map_server_ = this->create_service<sps_common_msgs::srv::SpsLoadMap>("/load_default_map",
                                            std::bind(&MapServerCore::load_default_map_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    load_navi_map_server_ = this->create_service<sps_common_msgs::srv::SpsLoadMap>("/load_navi_map",
                                            std::bind(&MapServerCore::load_navi_map_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_map_info_server_ = this->create_service<sps_common_msgs::srv::GetMapInfo>("/get_map_info",
                                            std::bind(&MapServerCore::get_map_info_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    publish_map_data_server_ = this->create_service<sps_common_msgs::srv::PublishMapData>("/publish_map_data",
                                            std::bind(&MapServerCore::publish_map_data_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_curr_robot_pose_server_ = this->create_service<sps_common_msgs::srv::GetCurrRobotPose>("/get_curr_robot_pose",
                                            std::bind(&MapServerCore::get_curr_robot_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    change_word_pose_to_pixel_pose_server_ = this->create_service<sps_common_msgs::srv::ChangeWordPoseToPixelPose>("/change_word_pose_to_pixel_pose",
                                            std::bind(&MapServerCore::change_word_pose_to_pixel_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    change_pixel_pose_to_word_pose_server_ = this->create_service<sps_common_msgs::srv::ChangePixelPoseToWordPose>("/change_pixel_pose_to_word_pose",
                                            std::bind(&MapServerCore::change_pixel_pose_to_word_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    query_pose_by_name_server_ = this->create_service<sps_common_msgs::srv::QueryPoseByName>("/query_pose_by_name",
                                            std::bind(&MapServerCore::query_pose_by_name_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    query_poi_type_by_pose_server_ = this->create_service<sps_common_msgs::srv::QueryPoiTypeByPose>("/query_poi_type_by_pose",
                                            std::bind(&MapServerCore::query_poi_type_by_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    goal_valid_check_server_ = this->create_service<sps_common_msgs::srv::GoalValidCheck>("/goal_valid_check",
                                            std::bind(&MapServerCore::goal_valid_check_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    query_charge_pile_by_name_server_ = this->create_service<sps_common_msgs::srv::QueryChargePileByName>("/query_charge_pile_by_name",
                                            std::bind(&MapServerCore::query_charge_pile_by_name_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    set_default_charge_pile_server_ = this->create_service<sps_common_msgs::srv::SetDefaultChargePile>("/set_default_charge_pile",
                                            std::bind(&MapServerCore::set_default_charge_pile_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_default_charge_pile_server_ = this->create_service<sps_common_msgs::srv::GetDefaultChargePile>("/get_default_charge_pile",
                                            std::bind(&MapServerCore::get_default_charge_pile_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_nearest_charge_pile_server_ = this->create_service<sps_common_msgs::srv::GetNearestChargePile>("/get_nearest_charge_pile",
                                            std::bind(&MapServerCore::get_nearest_charge_pile_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    publish_initial_pixel_pose_zero_server_ = this->create_service<sps_common_msgs::srv::PublishInitialPixelPoseZero>("/publish_initial_pixel_pose_zero",
                                            std::bind(&MapServerCore::publish_initial_pixel_pose_zero_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    //get_semantic_area_info_server_ = this->create_service<sps_common_msgs::srv::GetSemanticAreaInfo>("/get_semantic_area_info",
    //                                        std::bind(&MapServerCore::get_semantic_area_info_service, this, std::placeholders::_1, std::placeholders::_2),
    //                                        rmw_qos_profile_services_default,
    //                                        callback_group_service_);
    get_semantic_area_info_server_ = this->create_service<sps_common_msgs::srv::GetSemanticsAreas>("/get_semantic_area_info",
                                            std::bind(&MapServerCore::get_semantic_area_info_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    publish_lift_point_server_ = this->create_service<sps_common_msgs::srv::PublishLiftPoint>("/publish_lift_point",
                                            std::bind(&MapServerCore::publish_lift_point_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_lift_poi_info_server_ = this->create_service<sps_common_msgs::srv::GetLiftPoiInfo>("/get_lift_poi_info",
                                            std::bind(&MapServerCore::get_lift_poi_info_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    query_lift_poi_type_server_ = this->create_service<sps_common_msgs::srv::QueryLiftPoiType>("/query_lift_poi_type",
                                            std::bind(&MapServerCore::query_lift_poi_type_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    point_lift_inside_check_server_ = this->create_service<sps_common_msgs::srv::PointLiftInsideCheck>("/point_lift_inside_check",
                                            std::bind(&MapServerCore::point_lift_inside_check_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_robot_lift_pose_server_ = this->create_service<sps_common_msgs::srv::GetRobotLiftPose>("/get_robot_lift_pose",
                                            std::bind(&MapServerCore::get_robot_lift_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_robot_map_pose_server_ = this->create_service<sps_common_msgs::srv::GetRobotMapPose>("/get_robot_map_pose",
                                            std::bind(&MapServerCore::get_robot_map_pose_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_lift_check_pixel_point_server_ = this->create_service<sps_common_msgs::srv::GetLiftCheckPixelPoint>("/get_lift_check_pixel_point",
                                            std::bind(&MapServerCore::get_lift_check_pixel_point_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    get_lift_out_angle_server_ = this->create_service<sps_common_msgs::srv::GetLiftOutAngle>("/get_lift_out_angle",
                                            std::bind(&MapServerCore::get_lift_out_angle_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    update_env_map_server_ = this->create_service<sps_common_msgs::srv::UpdateMap>("/update_env_map",
                                            std::bind(&MapServerCore::update_env_map_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
    publish_mapping_map_server_ = this->create_service<sps_common_msgs::srv::PublishMapData>("/publish_mapping_map",
                                            std::bind(&MapServerCore::publish_mapping_map_service, this, std::placeholders::_1, std::placeholders::_2),
                                            rmw_qos_profile_services_default,
                                            callback_group_service_);
}

void MapServerCore::publish_init() {
    pixel_pose_pub_ = this->create_publisher<sps_common_msgs::msg::PixelPose>("/pixel_pose",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    //initialpose_pub_ = this->create_publisher<cm_msgs::msg::InitialSlamPose>("/initialpose_self",
    //                           rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()); //使用Ros1版本定位
    virtual_walls_pub_ = this->create_publisher<cm_msgs::msg::Virtualwalls>("/virtualwalls",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    bit_map_pub_ = this->create_publisher<cm_msgs::msg::BitMap>("/bit_map",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occ_map",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    map_updates_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/updates",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    static_occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/static_occ_map",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    static_map_info_pub_ = this->create_publisher<nav_msgs::msg::MapMetaData>("/occ_map_info",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    lift_poi_pub_ = this->create_publisher<sps_common_msgs::msg::LiftPoi>("/lift_poi",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    update_navi_map_pub_ = this->create_publisher<sps_common_msgs::msg::UpdateNaviMap>("/UpdateNaviMap_uplink",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    map_info_pub_ = this->create_publisher<sps_common_msgs::msg::MapInfo>("/map_info",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    mapping_occ_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapping_map",
                               rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

void MapServerCore::subscription_init() {
    update_occ_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/update_occmap", 1,
                                            std::bind(&MapServerCore::update_occ_map_callback, this, std::placeholders::_1));
    //update_tag_pose_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/tag_poses_list", 1,
    //                                        std::bind(&MapServerCore::update_tag_pose_callback, this, std::placeholders::_1));
    //loc_confidence_sub_ = this->create_subscription<std_msgs::msg::Float32>("/loc_confidence", 10,
    //                                        std::bind(&MapServerCore::loc_confidence_callback, this, std::placeholders::_1));
    battery_state_sub_ = this->create_subscription<sps_common_msgs::msg::BatteryState>("/BatteryState", 1,
                                            std::bind(&MapServerCore::battery_state_callback, this, std::placeholders::_1));
    update_navi_map_sub_ = this->create_subscription<sps_common_msgs::msg::UpdateNaviMap>("/UpdateNaviMap_downlink", 1,
                                            std::bind(&MapServerCore::update_navi_map_callback, this, std::placeholders::_1));
    local_status_sub_ = this->create_subscription<sps_common_msgs::msg::LocalizationStatus>("/localization_status", 1,
                                            std::bind(&MapServerCore::local_status_callback, this, std::placeholders::_1));
}

void MapServerCore::load_2d_test_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response) {
    std::string mapPath = request->map_path;
    RCLCPP_INFO(MAPCORE_LOG, "load_2d_test_map_service mapPath: %s", mapPath.c_str());
    response->result = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;

    if(mapPath.empty()) {
        response->description = "map_path is empty";
        return;
    } else if(!std::filesystem::exists(mapPath)) {
        response->description = "map_path is not exist";
        return;
    } else {
        int result = mapDatabase->loadTestMap(mapPath);
        if(result > 0) {
            map_path_ = mapPath;
            publishMapAll();
        }
        loadMapResultToResponse(result, response->result, response->description);
    }
}

void MapServerCore::load_default_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response) {
    std::string maplist_file{home_path_ + "/ginger_maps/release/maplist.json"};
    std::string map_path;
    RCLCPP_INFO(MAPCORE_LOG, "load_default_map_service maplist_file: %s", maplist_file.c_str());
    response->result = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;

    YAML::Node maplist;
    try {
        maplist = YAML::LoadFile(maplist_file);
        RCLCPP_INFO(MAPCORE_LOG, "load_default_map_service load maplist file successfully");
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPCORE_LOG, "load_default_map_service cant open map list file: %s", e.msg.c_str());
        //publishFakeMap();
        response->description = "cant open map list file";
        return;
    }
    try {
        auto default_map = maplist["currmap"];
        if (default_map.IsNull()) {
            RCLCPP_ERROR(MAPCORE_LOG, "load_default_map_service no default map defined");
            //publishFakeMap();
            response->description = "no default map defined";
            return;
        }
        auto uuid = default_map["uuid"].as<std::string>();
        if (uuid.empty()) {
            RCLCPP_ERROR(MAPCORE_LOG, "load_default_map_service no uuid in default map");
            //publishFakeMap();
            response->description = "no uuid in default map";
            return;
        }
        auto name = default_map["name"].as<std::string>();
        if (name.empty()) {
            RCLCPP_ERROR(MAPCORE_LOG, "load_default_map_service no name in default map");
            //publishFakeMap();
            response->description = "no name in default map";
            return;
        }

        map_path = std::string{home_path_ + "/ginger_maps/release/" + uuid + "/" + name};
        RCLCPP_INFO(MAPCORE_LOG, "load_default_map_service map_path: %s", map_path.c_str());
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPCORE_LOG, "load_default_map_service error occurred when parsing maplist file: %s, Path: %s", e.msg.c_str(), maplist_file.c_str());
        //publishFakeMap();
        response->description = "error occurred when parsing maplist file";
        return;
    }

    loadNaviMapImpl(map_path, LOCAL_LOAD_MAP, response->result, response->description);
    RCLCPP_INFO(MAPCORE_LOG, "load_default_map_service result:%d, description:%s", response->result, response->description.c_str());
    return;
}

void MapServerCore::load_navi_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response) {
    std::string mapPath = request->map_path;
    RCLCPP_INFO(MAPCORE_LOG, "load_navi_map_service mapPath: %s", mapPath.c_str());
    response->result = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;

    if(mapPath.empty()) {
        response->description = "map_path is empty";
        return;
    } else if(!std::filesystem::exists(mapPath)) {
        response->description = "map_path is not exist";
        return;
    } else {
        if (update_map_flag_) {
            update_map_flag_ = false;
            sps_common_msgs::msg::UpdateNaviMap update_map_res;
            update_map_res.stamp = rclcpp::Time();
            update_map_res.map_uuid = update_map_uuid_;
            update_map_res.map_name = update_map_name_;
            update_map_res.map_version = update_map_version_;
            update_map_res.link_type = sps_common_msgs::msg::UpdateNaviMap::UPLINK;
            update_map_res.result = sps_common_msgs::msg::UpdateNaviMap::SUCCESS;
            update_navi_map_pub_->publish(update_map_res);
            RCLCPP_INFO(MAPCORE_LOG, "load_navi_map_service finish to uplink update map result");
        }

        loadNaviMapImpl(mapPath, CLOUD_LOAD_MAP, response->result, response->description);
        RCLCPP_INFO(MAPCORE_LOG, "load_navi_map_service result:%d, description:%s", response->result, response->description.c_str());
        return;
    }
}

void MapServerCore::get_map_info_service(sps_common_msgs::srv::GetMapInfo::Request::SharedPtr request,
                            sps_common_msgs::srv::GetMapInfo::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_map_info_service");
    sps_common_msgs::msg::MapInfo mapInfo;
    if(mapDatabase->getMapInfo(mapInfo)) {
        response->map_info = mapInfo;
        response->result = sps_common_msgs::srv::GetMapInfo::Response::SUCCESS;
        response->description = "get map info success";
    } else {
        response->result = sps_common_msgs::srv::GetMapInfo::Response::FAILED;
        response->description = "get map info failed";
    }
}

void MapServerCore::publish_map_data_service(sps_common_msgs::srv::PublishMapData::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishMapData::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "publish_map_data_service");
    publishMapAll();
    response->result = sps_common_msgs::srv::PublishMapData::Response::SUCCESS;
    response->description = "publish map info success";
}

void MapServerCore::get_curr_robot_pose_service(sps_common_msgs::srv::GetCurrRobotPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetCurrRobotPose::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_curr_robot_pose_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(getRobotPose(pixel_pose)){
        response->pose = pixel_pose;
        response->result = sps_common_msgs::srv::GetCurrRobotPose::Response::SUCCESS;
        response->description = "get curr robot pose success";
    } else {
        response->result = sps_common_msgs::srv::GetCurrRobotPose::Response::FAILED;
        response->description = "get curr robot pose failed";
    }
}

void MapServerCore::change_word_pose_to_pixel_pose_service(sps_common_msgs::srv::ChangeWordPoseToPixelPose::Request::SharedPtr request,
                            sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "change_word_pose_to_pixel_pose_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(poseManager->changeWordPoseToPixelPose(request->word_pose, pixel_pose)){
        response->pixel_pose = pixel_pose;
        response->result = sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::SUCCESS;
        response->description = "change word pose to pixel pose success";
    } else {
        response->result = sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::FAILED;
        response->description = "change word pose to pixel pose failed";
    }
}

void MapServerCore::change_pixel_pose_to_word_pose_service(sps_common_msgs::srv::ChangePixelPoseToWordPose::Request::SharedPtr request,
                            sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "change_pixel_pose_to_word_pose_service");
    geometry_msgs::msg::Pose w_pose;
    if(poseManager->changePixelPoseToWordPose(request->pixel_pose, w_pose)){
        response->word_pose = w_pose;
        response->result = sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::SUCCESS;
        response->description = "change pixel pose to word pose success";
    } else {
        response->result = sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::FAILED;
        response->description = "change pixel pose to word pose failed";
    }
}

void MapServerCore::query_pose_by_name_service(sps_common_msgs::srv::QueryPoseByName::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryPoseByName::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "query_pose_by_name_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(poseManager->queryPoseByName(request->name, pixel_pose)){
        response->pixel_pose = pixel_pose;
        response->result = sps_common_msgs::srv::QueryPoseByName::Response::SUCCESS;
        response->description = "query pose by name success";
    } else {
        response->result = sps_common_msgs::srv::QueryPoseByName::Response::FAILED;
        response->description = "query pose by name failed";
    }
}

void MapServerCore::query_poi_type_by_pose_service(sps_common_msgs::srv::QueryPoiTypeByPose::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryPoiTypeByPose::Response::SharedPtr response) {
    PoiType poi_type = poseManager->queryPoiTypeByPose(request->pixel_pose);
    RCLCPP_INFO(MAPCORE_LOG, "query_poi_type_by_pose_service poi_type: %d", poi_type);
    response->poi_type = poi_type;
    response->result = sps_common_msgs::srv::QueryPoiTypeByPose::Response::SUCCESS;
    response->description = "query point type by pose success";
}

void MapServerCore::goal_valid_check_service(sps_common_msgs::srv::GoalValidCheck::Request::SharedPtr request,
                            sps_common_msgs::srv::GoalValidCheck::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "goal_valid_check_service");
    if(poseManager->checkPoseValid(request->pixel_pose)){
        response->result = sps_common_msgs::srv::GoalValidCheck::Response::POINT_OK;
        response->description = "goal is valid";
    } else {
        response->result = sps_common_msgs::srv::GoalValidCheck::Response::POINT_ERROR;
        response->description = "goal not valid";
    }
}

void MapServerCore::query_charge_pile_by_name_service(sps_common_msgs::srv::QueryChargePileByName::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryChargePileByName::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "query_charge_pile_by_name_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(poseManager->queryChargePileByName(request->name, pixel_pose)){
        response->pixel_pose = pixel_pose;
        response->result = sps_common_msgs::srv::QueryChargePileByName::Response::SUCCESS;
        response->description = "query charge pile by name success";
    } else {
        response->result = sps_common_msgs::srv::QueryChargePileByName::Response::FAILED;
        response->description = "query charge pile by name failed";
    }
}

void MapServerCore::set_default_charge_pile_service(sps_common_msgs::srv::SetDefaultChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::SetDefaultChargePile::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "set_default_charge_pile_service");
    if(poseManager->setDefaultChargePile(request->map_id, request->map_name, request->charger_name, request->pixel_pose)){
        response->result = sps_common_msgs::srv::SetDefaultChargePile::Response::SUCCESS;
        response->description = "set default charge pile success";
    } else {
        response->result = sps_common_msgs::srv::SetDefaultChargePile::Response::FAILED;
        response->description = "set default charge pile failed";
    }
}

void MapServerCore::get_default_charge_pile_service(sps_common_msgs::srv::GetDefaultChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::GetDefaultChargePile::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_default_charge_pile_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(poseManager->getDefaultChargePile(pixel_pose)){
        response->pixel_pose = pixel_pose;
        response->result = sps_common_msgs::srv::GetDefaultChargePile::Response::SUCCESS;
        response->description = "get default charge pile success";
    } else {
        response->result = sps_common_msgs::srv::GetDefaultChargePile::Response::FAILED;
        response->description = "get default charge pile failed";
    }
}

void MapServerCore::get_nearest_charge_pile_service(sps_common_msgs::srv::GetNearestChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::GetNearestChargePile::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_nearest_charge_pile_service");
    sps_common_msgs::msg::PixelPose curr_pose, charger_pose;
    if(getRobotPose(curr_pose)) {
        if(poseManager->getNearestChargePile(curr_pose, charger_pose)){
            response->pixel_pose = charger_pose;
            response->result = sps_common_msgs::srv::GetNearestChargePile::Response::SUCCESS;
            response->description = "get nearest charge pile success";
            return;
        }
    } else {
        response->result = sps_common_msgs::srv::GetNearestChargePile::Response::FAILED;
        response->description = "getRobotPose failed";
        return;
    }

    response->result = sps_common_msgs::srv::GetNearestChargePile::Response::FAILED;
    response->description = "get nearest charge pile failed";
}

void MapServerCore::publish_initial_pixel_pose_zero_service(sps_common_msgs::srv::PublishInitialPixelPoseZero::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishInitialPixelPoseZero::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "publish_initial_pixel_pose_zero_service");
    sps_common_msgs::msg::PixelPose pixel_zero_pose;
    pixel_zero_pose.x = 5.0;
    pixel_zero_pose.y = 5.0;
    pixel_zero_pose.theta = 0.0;
    geometry_msgs::msg::Pose zero_pose;
    poseManager->changePixelPoseToWordPose(pixel_zero_pose, zero_pose);

    //使用Ros1版本定位
    /**geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = rclcpp::Time();
    initial_pose.pose.pose = zero_pose;
    initial_pose.pose.covariance[6 * 0 + 0] = 0.2 * 0.2;
    initial_pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    initial_pose.pose.covariance[6 * 5 + 5] = (M_PI / 12.0) * (M_PI / 12.0);
    publishInitialPose(initial_pose, 1);**/

    setAidedPose(zero_pose, sps_common_msgs::srv::AidedPose::Request::INIT_POSE,
                sps_common_msgs::srv::AidedPose::Request::NONE);

    response->result = sps_common_msgs::srv::PublishInitialPixelPoseZero::Response::SUCCESS;
    response->description = "publish initial pixel pose zero success";
}
/**
void MapServerCore::get_semantic_area_info_service(sps_common_msgs::srv::GetSemanticAreaInfo::Request::SharedPtr request,
                            sps_common_msgs::srv::GetSemanticAreaInfo::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_semantic_area_info_service");
    double area_speed;
    AreaType area_type;
    sps_common_msgs::msg::SpecialAreas special_areas;
    if(mapDatabase->getSemanticAreaInfo(request->pose_x, request->pose_y, area_speed, area_type, special_areas)) {
        response->area_speed = area_speed;
        response->area_type = area_type;
        response->special_areas = special_areas;
        response->result = sps_common_msgs::srv::GetSemanticAreaInfo::Response::SUCCESS;
        response->description = "get semantic area info success";
    } else {
        response->result = sps_common_msgs::srv::GetSemanticAreaInfo::Response::FAILED;
        response->description = "get semantic area info failed";
    }
}
**/
void MapServerCore::get_semantic_area_info_service(sps_common_msgs::srv::GetSemanticsAreas::Request::SharedPtr request,
                            sps_common_msgs::srv::GetSemanticsAreas::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_semantic_area_info_service");
    std::vector<sps_common_msgs::msg::SemanticsArea> semantics_areas;
    if(mapDatabase->getSemanticAreaInfo(semantics_areas)) {
        response->areas = semantics_areas;
        response->result = sps_common_msgs::srv::GetSemanticAreaInfo::Response::SUCCESS;
        response->description = "get semantic area info success";
    } else {
        response->result = sps_common_msgs::srv::GetSemanticAreaInfo::Response::FAILED;
        response->description = "get semantic area info failed";
    }
}

void MapServerCore::publish_lift_point_service(sps_common_msgs::srv::PublishLiftPoint::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishLiftPoint::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "publish_lift_point_service");
    if(publishLiftPoiForNavi(request->lift_id, request->lift_safe_dist)) {
        response->result = sps_common_msgs::srv::PublishLiftPoint::Response::SUCCESS;
        response->description = "publish lift point success";
    } else {
        response->result = sps_common_msgs::srv::PublishLiftPoint::Response::FAILED;
        response->description = "publish lift point failed";
    }
}

void MapServerCore::get_lift_poi_info_service(sps_common_msgs::srv::GetLiftPoiInfo::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftPoiInfo::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_lift_poi_info_service lift_id:%s", request->lift_id.c_str());
    LiftPoi lift_poi;
    if(liftManager->getLiftPoiById(request->lift_id, lift_poi)) {
        response->name = lift_poi.name;
        response->group_id = lift_poi.group_id;
        response->inner_poi = lift_poi.inner_poi;
        response->out_poi = lift_poi.out_poi;
        response->entry_poi = lift_poi.entry_poi;
        response->door_poi = lift_poi.door_poi;
        response->corner_poi = lift_poi.corner_poi;

        response->pixel_out_poi = lift_poi.pixel_out_poi;
        response->pixel_inner_poi = lift_poi.pixel_inner_poi;
        response->pixel_entry_poi = lift_poi.pixel_entry_poi;
        response->pixel_door_poi = lift_poi.pixel_door_poi;
        response->pixel_corner_poi = lift_poi.pixel_door_poi;

        response->result = sps_common_msgs::srv::GetLiftPoiInfo::Response::SUCCESS;
        response->description = "get lift poi info success";
    } else {
        response->result = sps_common_msgs::srv::GetLiftPoiInfo::Response::FAILED;
        response->description = "get lift poi info failed";
    }
}

void MapServerCore::query_lift_poi_type_service(sps_common_msgs::srv::QueryLiftPoiType::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryLiftPoiType::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "query_lift_poi_type_service");
    std::string lift_id;
    LiftPoiType lift_poi_type;
    if(liftManager->queryLiftPoiType(request->pixel_pose, lift_id, lift_poi_type)) {
        response->lift_id = lift_id;
        response->lift_poi_type = lift_poi_type;
        response->result = sps_common_msgs::srv::QueryLiftPoiType::Response::SUCCESS;
        response->description = "query lift point type success";
    } else {
        response->lift_id = lift_id;
        response->lift_poi_type = lift_poi_type;
        response->result = sps_common_msgs::srv::QueryLiftPoiType::Response::FAILED;
        response->description = "query lift point type failed";
    }
}

void MapServerCore::point_lift_inside_check_service(sps_common_msgs::srv::PointLiftInsideCheck::Request::SharedPtr request,
                            sps_common_msgs::srv::PointLiftInsideCheck::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "point_lift_inside_check_service");
    if(liftManager->checkLiftInsideState(request->point, request->lift_id, request->liftin_distance_thresh)) {
        response->result = sps_common_msgs::srv::PointLiftInsideCheck::Response::INSIDE;
        response->description = "point is inside";
    } else {
        response->result = sps_common_msgs::srv::PointLiftInsideCheck::Response::OUTSIDE;
        response->description = "point is outside";
    }
}

void MapServerCore::get_robot_lift_pose_service(sps_common_msgs::srv::GetRobotLiftPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetRobotLiftPose::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_robot_lift_pose_service");
    std::vector<double> pose_lift_robot;
    geometry_msgs::msg::PoseStamped cur_robot_pose;
    if(getRobotPose(cur_robot_pose)) {
        if(liftManager->getRobotLiftPose2(request->lift_id, cur_robot_pose, pose_lift_robot)) {
            response->pose_lift_pose = pose_lift_robot;
            response->result = sps_common_msgs::srv::GetRobotLiftPose::Response::SUCCESS;
            response->description = "get robot lift pose success";
            return;
        }
    } else {
        response->result = sps_common_msgs::srv::GetRobotLiftPose::Response::FAILED;
        response->description = "getRobotPose failed";
        return;
    }

    response->result = sps_common_msgs::srv::GetRobotLiftPose::Response::FAILED;
    response->description = "get robot lift pose failed";
}

void MapServerCore::get_robot_map_pose_service(sps_common_msgs::srv::GetRobotMapPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetRobotMapPose::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_robot_map_pose_service");
    geometry_msgs::msg::PoseStamped cur_robot_pose;
    if(liftManager->getRobotMapPose2(request->lift_id, request->pose_lift_pose, cur_robot_pose)) {
        response->cur_robot_pose = cur_robot_pose;
        response->result = sps_common_msgs::srv::GetRobotMapPose::Response::SUCCESS;
        response->description = "get robot map pose success";
    } else {
        response->result = sps_common_msgs::srv::GetRobotMapPose::Response::FAILED;
        response->description = "get robot map pose failed";
    }
}

void MapServerCore::get_lift_check_pixel_point_service(sps_common_msgs::srv::GetLiftCheckPixelPoint::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_lift_check_pixel_point_service");
    sps_common_msgs::msg::PixelPose pixel_pose;
    if(liftManager->calcuLiftCheckPixelPoint(request->lift_id, pixel_pose)) {
        response->pixel_pose = pixel_pose;
        response->result = sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SUCCESS;
        response->description = "get lift check pixel point success";
    } else {
        response->result = sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::FAILED;
        response->description = "get lift check pixel point failed";
    }
}

void MapServerCore::get_lift_out_angle_service(sps_common_msgs::srv::GetLiftOutAngle::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftOutAngle::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "get_lift_out_angle_service");
    double liftout_angle;
    if(liftManager->getLiftoutAngle(request->lift_id, liftout_angle)) {
        response->liftout_angle = liftout_angle;
        response->result = sps_common_msgs::srv::GetLiftOutAngle::Response::SUCCESS;
        response->description = "get lift out angle success";
    } else {
        response->result = sps_common_msgs::srv::GetLiftOutAngle::Response::FAILED;
        response->description = "get lift out angle failed";
    }
}

void MapServerCore::update_env_map_service(sps_common_msgs::srv::UpdateMap::Request::SharedPtr request,
                            sps_common_msgs::srv::UpdateMap::Response::SharedPtr response) {
    std::string mapPath = request->path_name;
    RCLCPP_INFO(MAPCORE_LOG, "update_env_map_service mapPath: %s", mapPath.c_str());

    auto update_map_req = std::make_shared<sps_common_msgs::srv::UpdateMap::Request>();
    update_map_req->header.frame_id = "map";
    update_map_req->header.stamp = rclcpp::Time();
    update_map_req->path_name = mapPath;

    auto update_map_resp = std::make_shared<sps_common_msgs::srv::UpdateMap::Response>();
    service_client_->update_map_client(update_map_req, update_map_resp, std::chrono::milliseconds(5000));
    RCLCPP_INFO(MAPCORE_LOG, "update_env_map_service response ==> result: %d, description: %s",
                response->result, response->description.c_str());

    response->result = update_map_resp->result;
    response->description = update_map_resp->description;
}

void MapServerCore::publish_mapping_map_service(sps_common_msgs::srv::PublishMapData::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishMapData::Response::SharedPtr response) {
    RCLCPP_INFO(MAPCORE_LOG, "publish_mapping_map_service");
    nav_msgs::msg::OccupancyGrid occMap;
    if(mapDatabase->getOccMap(occMap)){
        mapping_occ_map_pub_->publish(occMap);
        response->result = sps_common_msgs::srv::PublishMapData::Response::SUCCESS;
        response->description = "publish mapping map success";
    } else {
        RCLCPP_WARN(MAPCORE_LOG, "publish_mapping_map_service no occ map");
        response->result = sps_common_msgs::srv::PublishMapData::Response::FAILED;
        response->description = "publish mapping map failed, no occ map";
    }
}

void MapServerCore::update_occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr occ_map) {
    RCLCPP_INFO(MAPCORE_LOG, "update_occ_map_callback width:%d, height:%d", occ_map->info.width, occ_map->info.height);
    if(occ_map->info.width > 0 && occ_map->info.height > 0) {
        cm_msgs::msg::BitMap bit_map;
        mapDatabase->updateMapDataInMapping(*occ_map, bit_map);

        //RVIZ
        if (occ_map) {
            occ_map_pub_->publish(*occ_map);
            static_occ_map_pub_->publish(*occ_map);
            static_map_info_pub_->publish(occ_map->info);
        }

        // 上传到RCU，显示
        bit_map_pub_->publish(bit_map);
    }
}

void MapServerCore::update_tag_pose_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    RCLCPP_INFO(MAPCORE_LOG, "update_tag_pose_callback");
    TagPoi tag_pose{};
    double roll, pitch, yaw;
    tf2::Quaternion RQ2;
    TagPosesPtr tag_poses;
    for (const auto &marker: msg->markers) {
        tf2::convert(marker.pose.orientation, RQ2);
        tf2::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);
        tag_pose.id = std::to_string(marker.id);
        tag_pose.x = marker.pose.position.x;
        tag_pose.y = marker.pose.position.y;
        tag_pose.z = marker.pose.position.z;
        tag_pose.roll = roll;
        tag_pose.pitch = pitch;
        tag_pose.yaw = yaw;
        tag_poses->push_back(tag_pose);
    }
    mapDatabase->updateTagPoses(*tag_poses);
}

void MapServerCore::local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 2000, "local_status_callback status:%d, score:%f, spec_causes:%d",
                         msg->status, msg->score, msg->specific_causes);
    latest_loc_pose_ = msg->pose;
    latest_loc_status_ = msg->status;
    latest_loc_confidence_ = msg->score;
}

void MapServerCore::loc_confidence_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    RCLCPP_INFO(MAPCORE_LOG, "loc_confidence_callback");
    latest_loc_confidence_ = msg->data;
}

void MapServerCore::battery_state_callback(const sps_common_msgs::msg::BatteryState::SharedPtr msg) {
    RCLCPP_INFO(MAPCORE_LOG, "battery_state_callback");
      // update map
    if (msg->current > 0 && update_map_flag_) {
        update_map_flag_ = false;
        int err = updateNaviMap();

        sps_common_msgs::msg::UpdateNaviMap update_map_res;
        if (err == -2) {
            update_map_res.result = sps_common_msgs::msg::UpdateNaviMap::MAP_NAME_INVALID;
        }
        else if (err < 0) {
            update_map_res.result = sps_common_msgs::msg::UpdateNaviMap::MAP_FILE_INVALID;
        }
        else {
            update_map_res.result = sps_common_msgs::msg::UpdateNaviMap::SUCCESS;

            //设定初始位置(更新地图: uuid name均不会改变)
            //publishInitialPose();
            RCLCPP_INFO(MAPCORE_LOG, "battery_state_callback update navi map successfully");
        }
        update_map_res.stamp = rclcpp::Time();
        update_map_res.map_uuid = update_map_uuid_;
        update_map_res.map_name = update_map_name_;
        update_map_res.map_version = update_map_version_;
        update_map_res.link_type = sps_common_msgs::msg::UpdateNaviMap::UPLINK;
        update_navi_map_pub_->publish(update_map_res);
        RCLCPP_INFO(MAPCORE_LOG, "battery_state_callback finish to uplink update map result");
    }
}

void MapServerCore::update_navi_map_callback(const sps_common_msgs::msg::UpdateNaviMap::SharedPtr msg) {
    RCLCPP_INFO(MAPCORE_LOG, "update_navi_map_callback");
    update_map_uuid_ = msg->map_uuid;
    update_map_name_ = msg->map_name;
    update_map_version_ = msg->map_version;
    update_map_flag_ = true;
}

void MapServerCore::loadNaviMapImpl(const std::string& mapPath, const std::int32_t& type, std::int32_t& loadMapRes, std::string& loadMapDes) {
    map_path_ = mapPath;
    std::int32_t result = sps_common_msgs::srv::SpsLoadMap::Response::SUCCESS;
    std::string description;
    localLoadMap(mapPath, type, result, description);
    if(result != sps_common_msgs::srv::SpsLoadMap::Response::SUCCESS) {
        loadMapRes = result;
        loadMapDes = description;
        return;
    } else {
        int res = mapDatabase->loadMap(mapPath);
        loadMapResultToResponse(res, result, description);
        if(res > 0) {
            //用于加载地图之后，设置初始位置
            //readRobotPoseFromFile();
            loadDefaultCharger();
            tagPoseModeCall();
            publishMapAll();
        }
        loadMapRes = result;
        loadMapDes = description;
    }
    return;
}

void MapServerCore::localLoadMap(const std::string& mapPath, const std::int32_t& type, std::int32_t& loadMapRes, std::string& loadMapDes) {
    auto load_map_req = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Request>();
    load_map_req->header.frame_id = "map";
    load_map_req->header.stamp = rclcpp::Time();
    load_map_req->map_path = mapPath;

    auto response = std::make_shared<sps_common_msgs::srv::SpsLoadMap::Response>();
    if(type == LOCAL_LOAD_MAP){
        service_client_->load_map_client(load_map_req, response, std::chrono::milliseconds(30000));
    } else {
        service_client_->load_map_client(load_map_req, response, std::chrono::milliseconds(8000));
    }
    RCLCPP_INFO(MAPCORE_LOG, "localLoadMap response ==> result: %d, description: %s",
                response->result, response->description.c_str());

    loadMapRes = response.get()->result;
    loadMapDes = response.get()->description;
}

void MapServerCore::loadMapResultToResponse(const int result, std::int32_t& loadMapRes, std::string& loadMapDes) {
    if(result > 0) {
        loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::SUCCESS;
        loadMapDes = "load map success";
        return;
    } else {
        switch (result){
            case -3:
                loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::NO_MAP_PNG;
                loadMapDes = "no map png";
                break;
            case -4:
                loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::NO_MAP_YAML;
                loadMapDes = "no map yaml";
                break;
            case -5:
                loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::MAP_PNG_UNVALID;
                loadMapDes = "map png unvalid";
                break;
            case -6:
                loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::MAP_YAML_UNVALID;
                loadMapDes = "map yaml unvalid";
                break;
            default:
                loadMapRes = sps_common_msgs::srv::SpsLoadMap::Response::FAILED;
                loadMapDes = "load map failed";
                break;
        }
    }
    return;
}

bool MapServerCore::loadDefaultCharger() {
    //读取设定的定位导航速度
    std::string vel_config_path(home_path_ + "/ginlt_param/robot_config_hari.yaml");
    RCLCPP_INFO(MAPCORE_LOG, "loadDefaultCharger vel_config_path: %s", vel_config_path.c_str());

    try {
        YAML::Node navi_vel_node = YAML::LoadFile(vel_config_path);
        if (navi_vel_node["chargePileDefault"].IsDefined()) {
            std::string mapId, chgSiteName;
            double x, y, rotation;
            if (navi_vel_node["chargePileDefault"]["mapId"].IsDefined()) {
                mapId = navi_vel_node["chargePileDefault"]["mapId"].as<std::string>();
            }
            if (navi_vel_node["chargePileDefault"]["chgSiteName"].IsDefined()) {
                chgSiteName = navi_vel_node["chargePileDefault"]["chgSiteName"].as<std::string>();
            }
            if (navi_vel_node["chargePileDefault"]["x"].IsDefined()) {
                x = navi_vel_node["chargePileDefault"]["x"].as<double>();
            }
            if (navi_vel_node["chargePileDefault"]["y"].IsDefined()) {
                y = navi_vel_node["chargePileDefault"]["y"].as<double>();
            }
            if (navi_vel_node["chargePileDefault"]["rotation"].IsDefined()) {
                rotation = navi_vel_node["chargePileDefault"]["rotation"].as<double>();
            }
            RCLCPP_INFO(MAPCORE_LOG, "loadDefaultCharger Get default charger from config file, mapId: %s, chgSiteName: %s, x: %f, y: %f, rotation: %f",
                   mapId.c_str(), chgSiteName.c_str(), x, y, rotation);

            sps_common_msgs::msg::PixelPose pose;
            pose.x = x;
            pose.y = y;
            pose.theta = rotation;
            return poseManager->setDefaultChargePile(mapId, "", chgSiteName, pose);
        } else {
            RCLCPP_INFO(MAPCORE_LOG, "loadDefaultCharger chargePileDefault not defined");
        }
    } catch (YAML::Exception &e) {
        RCLCPP_ERROR(MAPCORE_LOG, "loadDefaultCharger cannot open navigation vel config file: %s", e.msg.c_str());
    }

    return false;
}

bool MapServerCore::getRobotPose(geometry_msgs::msg::PoseStamped& cur_robot_pose) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    nav_msgs::msg::OccupancyGrid occMap;
    if(!mapDatabase->getOccMap(occMap)){
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PoseStamped occ_map_ data empty");
        return false;
    }

    /**geometry_msgs::msg::TransformStamped tfstamed;
    try {
        tfstamed = tf2_buffer_->lookupTransform("map", "base_link",
                                            tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PoseStamped lookupTransform %s", ex.what());
        return false;
    }

    cur_robot_pose.pose.position.x = tfstamed.transform.translation.x;
    cur_robot_pose.pose.position.y = tfstamed.transform.translation.y;
    cur_robot_pose.pose.orientation = tfstamed.transform.rotation;**/

    if(latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::RUNNING &&
        latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::WEAK) {
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PoseStamped fail, loc_status is not ok");
        return false;
    }

    cur_robot_pose.header.frame_id = "map";
    cur_robot_pose.header.stamp = rclcpp::Time();
    cur_robot_pose.pose = latest_loc_pose_;
    return true;
}

bool MapServerCore::getRobotPose(sps_common_msgs::msg::PixelPose &pose) {
    rclcpp::Clock clock(RCL_ROS_TIME);
    nav_msgs::msg::OccupancyGrid occMap;
    if(!mapDatabase->getOccMap(occMap)){
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PixelPose occ_map_ data empty");
        return false;
    }

    if(latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::RUNNING &&
        latest_loc_status_ != sps_common_msgs::msg::LocalizationStatus::WEAK) {
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PixelPose fail, loc_status is not ok");
        return false;
    }

    /**geometry_msgs::msg::TransformStamped tfstamed;
    try {
        tfstamed = tf2_buffer_->lookupTransform("map", "base_link",
                                            tf2::TimePointZero, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 4000, "getRobotPose PixelPose lookupTransform %s", ex.what());
        return false;
    }

    int x = static_cast<int>(
        (tfstamed.transform.translation.x - occMap.info.origin.position.x) /
        occMap.info.resolution);
    int y = static_cast<int>(
       occMap.info.height -
       (tfstamed.transform.translation.y - occMap.info.origin.position.y) /
           occMap.info.resolution - 1);
    double yaw =
       tf2::getYaw<geometry_msgs::msg::Quaternion>(tfstamed.transform.rotation);**/

    int x = static_cast<int>(
        (latest_loc_pose_.position.x - occMap.info.origin.position.x) /
        occMap.info.resolution);
    int y = static_cast<int>(
       occMap.info.height -
       (latest_loc_pose_.position.y - occMap.info.origin.position.y) /
           occMap.info.resolution - 1);
    double yaw =
       tf2::getYaw<geometry_msgs::msg::Quaternion>(latest_loc_pose_.orientation);

    pose.stamp = rclcpp::Time();
    pose.map_time = occMap.header.stamp;

    int theta = (static_cast<int>(yaw * 180 / M_PI) + 360) % 360;
    pose.x = static_cast<float>(x);
    pose.y = static_cast<float>(y);
    pose.theta = static_cast<float>((450 - theta) % 360);

    pose.cur_mapid = mapDatabase->getMapInfo("id");
    pose.loc_confidence = latest_loc_confidence_;
    pose.loc_threshold = 0.6;

    //在log中记录机器人的轨迹
    RCLCPP_INFO_THROTTLE(MAPCORE_LOG, clock, 2000, "Debug_for_pose: pixel: %0.2f, %0.2f, %0.2f, world: %0.2f, %0.2f, w x h: %d x %d, ori: %0.2f, %0.2f",
        pose.x, pose.y, pose.theta, latest_loc_pose_.position.x, latest_loc_pose_.position.y,
        occMap.info.width, occMap.info.height, occMap.info.origin.position.x, occMap.info.origin.position.y
    );

    return true;
}

//使用Ros1版本定位
/**bool MapServerCore::publishInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& cur_pose, int is_optimize) {
    cm_msgs::msg::InitialSlamPose slam_initial_pose;
    if (is_optimize == 1) {
        slam_initial_pose.need_optimize = cm_msgs::msg::InitialSlamPose::OPTIMIZE;
    } else if (is_optimize == 0) {
        slam_initial_pose.need_optimize = cm_msgs::msg::InitialSlamPose::NO_OPTIMIZE;
    }
    slam_initial_pose.initial_pose = cur_pose;
    RCLCPP_INFO(MAPCORE_LOG, "publishInitialPose initial_pose: {%f, %f, %f}, need_optimize: %d to slam",
           slam_initial_pose.initial_pose.pose.pose.position.x,
           slam_initial_pose.initial_pose.pose.pose.position.y,
           slam_initial_pose.initial_pose.pose.pose.position.z,
           slam_initial_pose.need_optimize);
    initialpose_pub_->publish(slam_initial_pose);
    return true;
}**/

bool MapServerCore::publishInitialPose() {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    bool res = getInitialPose(initial_pose);
    if (res) {
        setAidedPose(initial_pose.pose.pose, sps_common_msgs::srv::AidedPose::Request::INIT_POSE,
                sps_common_msgs::srv::AidedPose::Request::ALL);

        //使用Ros1版本定位
        //publishInitialPose(initial_pose, 1);
        //RCLCPP_INFO(MAPCORE_LOG, "publishInitialPose successfully");
    }
    else {
        RCLCPP_ERROR(MAPCORE_LOG, "publishInitialPose failed");
    }
    return res;
}

bool MapServerCore::getInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
    if(getLatestlPose(pose)) {
        RCLCPP_INFO(MAPCORE_LOG, "getInitialPose get latest pose as initial pose");
    }
    else if(getChargePile(pose)) {
        RCLCPP_INFO(MAPCORE_LOG, "getInitialPose get charge poi as initial pose");
    }
    else {
        pose.header.frame_id = "map";
        pose.header.stamp = rclcpp::Time();
        pose.pose.pose.position.x = 0.0;
        pose.pose.pose.position.y = 0.0;
        pose.pose.pose.position.z = 0.0;

        pose.pose.pose.orientation.w = 1.0;
        pose.pose.pose.orientation.x = 0.0;
        pose.pose.pose.orientation.y = 0.0;
        pose.pose.pose.orientation.z = 0.0;
        pose.pose.covariance[6 * 0 + 0] = 0.2 * 0.2;
        pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
        pose.pose.covariance[6 * 5 + 5] = (M_PI / 12.0) * (M_PI / 12.0);

        RCLCPP_INFO(MAPCORE_LOG, "getInitialPose set (0,0,0) in world coordinate system as initial pose");
    }
    return true;
}

bool MapServerCore::getLatestlPose(geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
    RCLCPP_INFO(MAPCORE_LOG, "getLatestlPose initial_pose_ in pixel map: {%f %f %f}",
           initial_pose_.x, initial_pose_.y, initial_pose_.theta);
    if (initial_pose_.x * initial_pose_.y < 0.01) {
        RCLCPP_WARN(MAPCORE_LOG, "getLatestlPose cannot get latest robot pose");
        return false;
    }

    poseManager->changePixelPoseToWordPose(initial_pose_, pose.pose.pose);
    pose.header.frame_id = "map";
    pose.header.stamp = rclcpp::Time();
    pose.pose.covariance[6 * 0 + 0] = 0.2 * 0.2;
    pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    pose.pose.covariance[6 * 5 + 5] = (M_PI / 12.0) * (M_PI / 12.0);
    return true;
}

bool MapServerCore::getChargePile(geometry_msgs::msg::PoseWithCovarianceStamped &pose) {
    if (!poseManager->getChargePile(pose.pose.pose)) {
        return false;
    }
    pose.header.frame_id = "map";
    pose.header.stamp = rclcpp::Time();
    pose.pose.covariance[6 * 0 + 0] = 0.2 * 0.2;
    pose.pose.covariance[6 * 1 + 1] = 0.5 * 0.5;
    pose.pose.covariance[6 * 5 + 5] = (M_PI / 12.0) * (M_PI / 12.0);
    return true;
}

bool MapServerCore::publishLiftPoiForNavi(const std::string& lift_id, double lift_safe_dist) {
    RCLCPP_INFO(MAPCORE_LOG, "PublishLiftPoiForNavi lift_id:%s, lift_safe_dist:%f", lift_id.c_str(), lift_safe_dist);
    //publish lift poi
    if (lift_id.empty()) return false;

    LiftPoi curr_lift;
    if(!liftManager->getLiftPoiById(lift_id, curr_lift)){
        RCLCPP_ERROR(MAPCORE_LOG, "PublishLiftPoiForNavi cannot find lift poi for navigation: %s", lift_id.c_str());
        return false;
    }

    sps_common_msgs::msg::LiftPoi lift_poi;
    lift_poi.map_id = mapDatabase->getMapInfo("id");
    lift_poi.map_name = mapDatabase->getMapInfo("name");
    lift_poi.lift_id = lift_id;

    // lift_poi.door_poi.resize(curr_lift.door_poi.size()*2);
    for (const auto& item : curr_lift.door_poi) {
        lift_poi.door_poi.push_back(item.position.x);
        lift_poi.door_poi.push_back(item.position.y);
    }
    // lift_poi.corner_poi.resize(curr_lift.corner_poi.size()*2);
    for (const auto& item : curr_lift.corner_poi) {
        lift_poi.corner_poi.push_back(item.position.x);
        lift_poi.corner_poi.push_back(item.position.y);
    }
    // lift_poi.entry_poi.resize(curr_lift.entry_poi.size()*2);
    for (const auto& item : curr_lift.entry_poi) {
        lift_poi.entry_poi.push_back(item.position.x);
        lift_poi.entry_poi.push_back(item.position.y);
        lift_poi.entry_poi.push_back(tf2::getYaw(item.orientation));
    }

    // lift_poi.inner_poi.resize(2);
    // lift_poi.out_poi.resize(2);
    lift_poi.inner_poi.push_back(curr_lift.inner_poi.position.x);
    lift_poi.inner_poi.push_back(curr_lift.inner_poi.position.y);
    lift_poi.inner_poi.push_back(tf2::getYaw(curr_lift.inner_poi.orientation));
    lift_poi.out_poi.push_back(curr_lift.out_poi.position.x);
    lift_poi.out_poi.push_back(curr_lift.out_poi.position.y);
    lift_poi.out_poi.push_back(tf2::getYaw(curr_lift.out_poi.orientation));

    lift_poi.safe_dist = (float)lift_safe_dist;

    lift_poi_pub_->publish(lift_poi);
    return true;
}

int MapServerCore::updateNaviMap() {
    std::string map_uuid = mapDatabase->getMapInfo("id");
    std::string map_name = mapDatabase->getMapInfo("name");
    if (update_map_uuid_ != map_uuid || update_map_name_ != map_name) {
        RCLCPP_WARN(MAPCORE_LOG, "updateNaviMap failed to update map: %s, %s || %s, %s",
                map_uuid.c_str(), map_name.c_str(), update_map_uuid_.c_str(), update_map_name_.c_str());
        return -2;
    }

    std::string map_path = map_path_;
    std::int32_t result;
    std::string description;
    loadNaviMapImpl(map_path, LOCAL_LOAD_MAP, result, description);
    if (result < 0) {
        RCLCPP_ERROR(MAPCORE_LOG, "updateNaviMap failed to update navi map");
    }

    return result;
}

void MapServerCore::tagPoseModeCall() {
    std::vector<geometry_msgs::msg::PoseStamped> poses_stamped;
    if(!mapDatabase->getPosesStamped(poses_stamped)) {
        RCLCPP_INFO(MAPCORE_LOG, "tagPoseModeCall poses_stamped is empty");
        return;
    }

    auto mode_srv_req = std::make_shared<sps_common_msgs::srv::Mode_Request>();
    // call service to zero begin navi mode
    mode_srv_req->mode_type = 1;  // ginger_msgs::ModeRequest::SLAM_LOC;
    mode_srv_req->tag_id_pose = poses_stamped;

    if(!mode_service_client_->wait_for_service(1s)) {
        RCLCPP_WARN(MAPCORE_LOG, "tagPoseModeCall service can not connected!");
        return;
    }

    mode_service_client_->async_send_request(mode_srv_req, std::bind(&MapServerCore::pose_mode_callback, this, std::placeholders::_1));
}

void MapServerCore::pose_mode_callback(rclcpp::Client<sps_common_msgs::srv::Mode>::SharedFuture response) {
    auto resp_data = response.get();
    RCLCPP_INFO(MAPCORE_LOG, "pose_mode_callback response ==> result: %d, feedback: %s",
               resp_data->result, resp_data->feedback.c_str());
}

void MapServerCore::setAidedPose(const geometry_msgs::msg::Pose& pose, const int pose_type, const int match_type) {
    auto aided_pose_req = std::make_shared<sps_common_msgs::srv::AidedPose::Request>();
    std::vector<geometry_msgs::msg::Pose> pose_vector;
    pose_vector.push_back(pose);

    aided_pose_req->header.frame_id = "map";
    aided_pose_req->header.stamp = rclcpp::Time();
    aided_pose_req->poses = pose_vector;
    aided_pose_req->pose_type = pose_type;
    aided_pose_req->match_type = match_type;

    if(!set_aided_pose_client_->wait_for_service(5s)) {
        RCLCPP_WARN(MAPCORE_LOG, "setAidedPose service can not connected!");
        return;
    }

    set_aided_pose_client_->async_send_request(aided_pose_req, std::bind(&MapServerCore::aided_pose_callback, this, std::placeholders::_1));

    /**auto response = std::make_shared<sps_common_msgs::srv::AidedPose::Response>();
    service_client_->aided_pose_client(aided_pose_req, response, std::chrono::milliseconds(6000));
    RCLCPP_INFO(MAPCORE_LOG, "setAidedPose response ==> result: %d, description: %s",
                response->result, response->description.c_str());**/
}

void MapServerCore::aided_pose_callback(rclcpp::Client<sps_common_msgs::srv::AidedPose>::SharedFuture response) {
    auto resp_data = response.get();
    RCLCPP_INFO(MAPCORE_LOG, "aidedPoseCallback response ==> result: %d, feedback: %s",
               resp_data->result, resp_data->description.c_str());
}

void MapServerCore::publishMapAll() {
    nav_msgs::msg::OccupancyGrid occMap;
    nav_msgs::msg::OccupancyGrid staticMap;
    cm_msgs::msg::BitMap bitMap;
    sps_common_msgs::msg::VirtualWall virtualWall;
    if(mapDatabase->getOccMap(occMap)){
        RCLCPP_INFO(MAPCORE_LOG, "publishMapAll occMap width:%d, height:%d", occMap.info.width, occMap.info.height);
        for(int i=0; i<3; i++) {
            occ_map_pub_->publish(occMap);
            //map_updates_pub_->publish(occMap);
            rclcpp::sleep_for(50ms);
        }
    } else {
        RCLCPP_WARN(MAPCORE_LOG, "publish_map_data_service no occ map");
    }

    if(mapDatabase->getStaticMap(staticMap)){
        static_occ_map_pub_->publish(staticMap);
        static_map_info_pub_->publish(staticMap.info);
    } else {
        RCLCPP_WARN(MAPCORE_LOG, "publish_map_data_service no static map");
    }

    if(mapDatabase->getBitMap(bitMap)){
        bit_map_pub_->publish(bitMap);
    } else {
        RCLCPP_WARN(MAPCORE_LOG, "publish_map_data_service no bit map");
    }

    if(mapDatabase->getVirtualWall(virtualWall)){
        cm_msgs::msg::Virtualwalls nav2VirtualWalls;
        nav2VirtualWalls.header.frame_id = "map";
        nav2VirtualWalls.header.stamp = rclcpp::Time();
        nav2VirtualWalls.info = staticMap.info;
        nav2VirtualWalls.closed = false;

        auto wall_size = virtualWall.start.size();
        for (int i = 0; i < wall_size; i++) {
            geometry_msgs::msg::Point start, end;
            poseManager->changePixelPointToWordPoint(virtualWall.start[i].x, virtualWall.start[i].y, start);
            poseManager->changePixelPointToWordPoint(virtualWall.end[i].x, virtualWall.end[i].y, end);

            cm_msgs::msg::PointArray point_array;
            point_array.point_array.push_back(start);
            point_array.point_array.push_back(end);
            nav2VirtualWalls.virtualwalls.push_back(point_array);
        }
        RCLCPP_INFO(MAPCORE_LOG, "publishMapAll virtual walls size:%ld", nav2VirtualWalls.virtualwalls.size());
        virtual_walls_pub_->publish(nav2VirtualWalls);
    } else {
        RCLCPP_WARN(MAPCORE_LOG, "publish_map_data_service no virtual wall");
    }

    sps_common_msgs::msg::MapInfo mapInfo;
    mapDatabase->getMapInfo(mapInfo);
    map_info_pub_->publish(mapInfo);

    rclcpp::sleep_for(50ms); //图像数据量比较大，延时一会
}

bool MapServerCore::readRobotPoseFromFile() {
    // load initial pose
    bool valid = false;
    initial_pose_.x = 0.0;
    initial_pose_.y = 0.0;
    initial_pose_.theta = 0.0;
    YAML::Node pose_node;
    posefile_lock_.lock();
    std::string pose_file;
    try {
        pose_file = home_path_ + "/.cur_pose.yaml";
        RCLCPP_INFO(MAPCORE_LOG, "readRobotPoseFromFile pose_file: %s", pose_file.c_str());
        pose_node = YAML::LoadFile(pose_file);
        RCLCPP_INFO(MAPCORE_LOG, "readRobotPoseFromFile LoadFile content: \n%s", Dump(pose_node).c_str());
        RCLCPP_INFO(MAPCORE_LOG, "===========================================================\n\n");

        std::string mapId = mapDatabase->getMapInfo("id");
        std::string mapName = mapDatabase->getMapInfo("name");
        if (pose_node["map_uuid"].as<std::string>() == mapId &&
                pose_node["map_name"].as<std::string>() == mapName) {
            initial_pose_.x = pose_node["robot_pose"][0].as<float>();
            initial_pose_.y = pose_node["robot_pose"][1].as<float>();
            initial_pose_.theta = pose_node["robot_pose"][2].as<float>();
            valid = true;
            RCLCPP_INFO(MAPCORE_LOG, "readRobotPoseFromFile initial pose from pose file: %f, %f, %f",
                    initial_pose_.x, initial_pose_.y, initial_pose_.theta);
        } else {
            RCLCPP_ERROR(MAPCORE_LOG, "readRobotPoseFromFile map uuid & name is differente: %s, %s",
                    pose_node["map_uuid"].as<std::string>().c_str(), mapId.c_str());
        }
    } catch (const YAML::Exception &e) {
        RCLCPP_WARN(MAPCORE_LOG, "readRobotPoseFromFile can't parse pose file: %s\n ERROR: %s %s\n", pose_file.c_str(), e.msg.c_str(), e.what());
    }
    posefile_lock_.unlock();
    return valid;
}

void MapServerCore::writeRobotPose() {
    rclcpp::Rate rate(static_pose_upload_fps_);
    std::string pose_file(home_path_ + "/.cur_pose.yaml");
    // YAML::Node robot_pose; // = YAML::LoadFile(pose_file);

    while (map_running_ && rclcpp::ok()) {
        sps_common_msgs::msg::PixelPose pose;
        if (getRobotPose(pose)) {
            pixel_pose_pub_->publish(pose);

            if (mapDatabase->getMapLoadStatus()) {
                posefile_lock_.lock();
                std::ofstream fout(pose_file);
                if (fout.is_open()) {
                    YAML::Emitter out;
                    out << YAML::BeginMap;
                    out << YAML::Key << "map_uuid";
                    out << YAML::Value << mapDatabase->getMapInfo("id");
                    out << YAML::Key << "map_name";
                    out << YAML::Value << mapDatabase->getMapInfo("name");
                    out << YAML::Key << "map_time";
                    out << YAML::Value << pose.stamp.sec;

                    out << YAML::Key << "robot_pose";
                    out << YAML::Value << YAML::BeginSeq << pose.x << pose.y << pose.theta << YAML::EndSeq;
                    out << YAML::EndMap;

                    fout << out.c_str();
                }
                fout.close();
                sync();
                posefile_lock_.unlock();
            }
        }
        rate.sleep();
    }
}
