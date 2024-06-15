#ifndef MAP_SERVER_CORE_HPP_
#define MAP_SERVER_CORE_HPP_

#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "map_database.hpp"
#include "lift_manager.hpp"
#include "pose_manager.hpp"
#include "service_client.hpp"

#include "sps_common_msgs/srv/sps_load_map.hpp"
#include "sps_common_msgs/srv/get_map_info.hpp"
#include "sps_common_msgs/srv/publish_map_data.hpp"
#include "sps_common_msgs/srv/get_curr_robot_pose.hpp"
#include "sps_common_msgs/srv/change_word_pose_to_pixel_pose.hpp"
#include "sps_common_msgs/srv/change_pixel_pose_to_word_pose.hpp"
#include "sps_common_msgs/srv/query_pose_by_name.hpp"
#include "sps_common_msgs/srv/query_poi_type_by_pose.hpp"
#include "sps_common_msgs/srv/goal_valid_check.hpp"
#include "sps_common_msgs/srv/query_charge_pile_by_name.hpp"
#include "sps_common_msgs/srv/set_default_charge_pile.hpp"
#include "sps_common_msgs/srv/get_default_charge_pile.hpp"
#include "sps_common_msgs/srv/get_nearest_charge_pile.hpp"
#include "sps_common_msgs/srv/publish_initial_pixel_pose_zero.hpp"
#include "sps_common_msgs/srv/get_semantic_area_info.hpp"
#include "sps_common_msgs/srv/get_semantics_areas.hpp"
#include "sps_common_msgs/srv/publish_lift_point.hpp"
#include "sps_common_msgs/srv/get_lift_poi_info.hpp"
#include "sps_common_msgs/srv/query_lift_poi_type.hpp"
#include "sps_common_msgs/srv/point_lift_inside_check.hpp"
#include "sps_common_msgs/srv/get_robot_lift_pose.hpp"
#include "sps_common_msgs/srv/get_robot_map_pose.hpp"
#include "sps_common_msgs/srv/get_lift_check_pixel_point.hpp"
#include "sps_common_msgs/srv/get_lift_out_angle.hpp"
#include "sps_common_msgs/srv/mode.hpp"
#include "sps_common_msgs/srv/update_map.hpp"
#include "sps_common_msgs/srv/aided_pose.hpp"

#include "sps_common_msgs/msg/pixel_pose.hpp"
//#include "cm_msgs/msg/initial_slam_pose.hpp" //使用Ros1版本定位
#include "cm_msgs/msg/virtualwalls.hpp"
//#include "sps_common_msgs/msg/virtualwalls.hpp"
//#include "sps_common_msgs/msg/bit_map.hpp"
#include "cm_msgs/msg/bit_map.hpp"
#include "sps_common_msgs/msg/lift_poi.hpp"
#include "sps_common_msgs/msg/battery_state.hpp"
#include "sps_common_msgs/msg/update_navi_map.hpp"
#include "sps_common_msgs/msg/map_info.hpp"
#include "sps_common_msgs/msg/localization_status.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"


class MapServerCore : public rclcpp::Node
{

public:
    MapServerCore();
    ~MapServerCore();

    typedef enum {
        LOCAL_LOAD_MAP = 0,
        CLOUD_LOAD_MAP = 1
    } LOADMAPTYPE;

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    std::shared_ptr<SeviceClient> service_client_;
    MapDatabase* mapDatabase;
    LiftManager* liftManager;
    PoseManager* poseManager;

    bool map_running_;
    std::mutex map_lock_;
    std::mutex posefile_lock_;
    std::shared_ptr<std::thread> publish_thread_;
    sps_common_msgs::msg::PixelPose  initial_pose_;

    std::string home_path_ = "/home/ginger";

    std::string map_path_;

    geometry_msgs::msg::Pose latest_loc_pose_;
    std::int32_t latest_loc_status_ = 0;
    float latest_loc_confidence_ = 0.0; //sub loc confidence
    double static_pose_upload_fps_ = 1.0;

    // about update navimap
    bool update_map_flag_ = false;
    std::string update_map_uuid_;
    std::string update_map_name_;
    std::string update_map_version_;

    void transforms_init();
    void parameter_init();
    void client_init();
    void service_init();
    void publish_init();
    void subscription_init();

    void writeRobotPose();
    bool readRobotPoseFromFile();

    rclcpp::Client<sps_common_msgs::srv::AidedPose>::SharedPtr set_aided_pose_client_;
    rclcpp::Client<sps_common_msgs::srv::SpsLoadMap>::SharedPtr load_map_client_;
    rclcpp::Client<sps_common_msgs::srv::UpdateMap>::SharedPtr update_map_client_;
    rclcpp::Client<sps_common_msgs::srv::Mode>::SharedPtr mode_service_client_;

    rclcpp::Service<sps_common_msgs::srv::SpsLoadMap>::SharedPtr load_2d_test_map_server_;
    rclcpp::Service<sps_common_msgs::srv::SpsLoadMap>::SharedPtr load_default_map_server_;
    rclcpp::Service<sps_common_msgs::srv::SpsLoadMap>::SharedPtr load_navi_map_server_;
    rclcpp::Service<sps_common_msgs::srv::GetMapInfo>::SharedPtr get_map_info_server_;
    rclcpp::Service<sps_common_msgs::srv::PublishMapData>::SharedPtr publish_map_data_server_;
    rclcpp::Service<sps_common_msgs::srv::GetCurrRobotPose>::SharedPtr get_curr_robot_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::ChangeWordPoseToPixelPose>::SharedPtr change_word_pose_to_pixel_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::ChangePixelPoseToWordPose>::SharedPtr change_pixel_pose_to_word_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::QueryPoseByName>::SharedPtr query_pose_by_name_server_;
    rclcpp::Service<sps_common_msgs::srv::QueryPoiTypeByPose>::SharedPtr query_poi_type_by_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::GoalValidCheck>::SharedPtr goal_valid_check_server_;
    rclcpp::Service<sps_common_msgs::srv::QueryChargePileByName>::SharedPtr query_charge_pile_by_name_server_;
    rclcpp::Service<sps_common_msgs::srv::SetDefaultChargePile>::SharedPtr set_default_charge_pile_server_;
    rclcpp::Service<sps_common_msgs::srv::GetDefaultChargePile>::SharedPtr get_default_charge_pile_server_;
    rclcpp::Service<sps_common_msgs::srv::GetNearestChargePile>::SharedPtr get_nearest_charge_pile_server_;
    rclcpp::Service<sps_common_msgs::srv::PublishInitialPixelPoseZero>::SharedPtr publish_initial_pixel_pose_zero_server_;
    //rclcpp::Service<sps_common_msgs::srv::GetSemanticAreaInfo>::SharedPtr get_semantic_area_info_server_;
    rclcpp::Service<sps_common_msgs::srv::GetSemanticsAreas>::SharedPtr get_semantic_area_info_server_;
    rclcpp::Service<sps_common_msgs::srv::PublishLiftPoint>::SharedPtr publish_lift_point_server_;
    rclcpp::Service<sps_common_msgs::srv::GetLiftPoiInfo>::SharedPtr get_lift_poi_info_server_;
    rclcpp::Service<sps_common_msgs::srv::QueryLiftPoiType>::SharedPtr query_lift_poi_type_server_;
    rclcpp::Service<sps_common_msgs::srv::PointLiftInsideCheck>::SharedPtr point_lift_inside_check_server_;
    rclcpp::Service<sps_common_msgs::srv::GetRobotLiftPose>::SharedPtr get_robot_lift_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::GetRobotMapPose>::SharedPtr get_robot_map_pose_server_;
    rclcpp::Service<sps_common_msgs::srv::GetLiftCheckPixelPoint>::SharedPtr get_lift_check_pixel_point_server_;
    rclcpp::Service<sps_common_msgs::srv::GetLiftOutAngle>::SharedPtr get_lift_out_angle_server_;
    rclcpp::Service<sps_common_msgs::srv::UpdateMap>::SharedPtr update_env_map_server_;
    rclcpp::Service<sps_common_msgs::srv::PublishMapData>::SharedPtr publish_mapping_map_server_;

    void load_2d_test_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response);
    void load_default_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response);
    void load_navi_map_service(sps_common_msgs::srv::SpsLoadMap::Request::SharedPtr request,
                            sps_common_msgs::srv::SpsLoadMap::Response::SharedPtr response);
    void get_map_info_service(sps_common_msgs::srv::GetMapInfo::Request::SharedPtr request,
                            sps_common_msgs::srv::GetMapInfo::Response::SharedPtr response);
    void publish_map_data_service(sps_common_msgs::srv::PublishMapData::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishMapData::Response::SharedPtr response);
    void get_curr_robot_pose_service(sps_common_msgs::srv::GetCurrRobotPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetCurrRobotPose::Response::SharedPtr response);
    void change_word_pose_to_pixel_pose_service(sps_common_msgs::srv::ChangeWordPoseToPixelPose::Request::SharedPtr request,
                            sps_common_msgs::srv::ChangeWordPoseToPixelPose::Response::SharedPtr response);
    void change_pixel_pose_to_word_pose_service(sps_common_msgs::srv::ChangePixelPoseToWordPose::Request::SharedPtr request,
                            sps_common_msgs::srv::ChangePixelPoseToWordPose::Response::SharedPtr response);
    void query_pose_by_name_service(sps_common_msgs::srv::QueryPoseByName::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryPoseByName::Response::SharedPtr response);
    void query_poi_type_by_pose_service(sps_common_msgs::srv::QueryPoiTypeByPose::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryPoiTypeByPose::Response::SharedPtr response);
    void goal_valid_check_service(sps_common_msgs::srv::GoalValidCheck::Request::SharedPtr request,
                            sps_common_msgs::srv::GoalValidCheck::Response::SharedPtr response);
    void query_charge_pile_by_name_service(sps_common_msgs::srv::QueryChargePileByName::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryChargePileByName::Response::SharedPtr response);
    void set_default_charge_pile_service(sps_common_msgs::srv::SetDefaultChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::SetDefaultChargePile::Response::SharedPtr response);
    void get_default_charge_pile_service(sps_common_msgs::srv::GetDefaultChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::GetDefaultChargePile::Response::SharedPtr response);
    void get_nearest_charge_pile_service(sps_common_msgs::srv::GetNearestChargePile::Request::SharedPtr request,
                            sps_common_msgs::srv::GetNearestChargePile::Response::SharedPtr response);
    void publish_initial_pixel_pose_zero_service(sps_common_msgs::srv::PublishInitialPixelPoseZero::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishInitialPixelPoseZero::Response::SharedPtr response);
    //void get_semantic_area_info_service(sps_common_msgs::srv::GetSemanticAreaInfo::Request::SharedPtr request,
    //                        sps_common_msgs::srv::GetSemanticAreaInfo::Response::SharedPtr response);
    void get_semantic_area_info_service(sps_common_msgs::srv::GetSemanticsAreas::Request::SharedPtr request,
                            sps_common_msgs::srv::GetSemanticsAreas::Response::SharedPtr response);
    void publish_lift_point_service(sps_common_msgs::srv::PublishLiftPoint::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishLiftPoint::Response::SharedPtr response);
    void get_lift_poi_info_service(sps_common_msgs::srv::GetLiftPoiInfo::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftPoiInfo::Response::SharedPtr response);
    void query_lift_poi_type_service(sps_common_msgs::srv::QueryLiftPoiType::Request::SharedPtr request,
                            sps_common_msgs::srv::QueryLiftPoiType::Response::SharedPtr response);
    void point_lift_inside_check_service(sps_common_msgs::srv::PointLiftInsideCheck::Request::SharedPtr request,
                            sps_common_msgs::srv::PointLiftInsideCheck::Response::SharedPtr response);
    void get_robot_lift_pose_service(sps_common_msgs::srv::GetRobotLiftPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetRobotLiftPose::Response::SharedPtr response);
    void get_robot_map_pose_service(sps_common_msgs::srv::GetRobotMapPose::Request::SharedPtr request,
                            sps_common_msgs::srv::GetRobotMapPose::Response::SharedPtr response);
    void get_lift_check_pixel_point_service(sps_common_msgs::srv::GetLiftCheckPixelPoint::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftCheckPixelPoint::Response::SharedPtr response);
    void get_lift_out_angle_service(sps_common_msgs::srv::GetLiftOutAngle::Request::SharedPtr request,
                            sps_common_msgs::srv::GetLiftOutAngle::Response::SharedPtr response);
    void update_env_map_service(sps_common_msgs::srv::UpdateMap::Request::SharedPtr request,
                            sps_common_msgs::srv::UpdateMap::Response::SharedPtr response);
    void publish_mapping_map_service(sps_common_msgs::srv::PublishMapData::Request::SharedPtr request,
                            sps_common_msgs::srv::PublishMapData::Response::SharedPtr response);

    rclcpp::Publisher<sps_common_msgs::msg::PixelPose>::SharedPtr pixel_pose_pub_;
    //rclcpp::Publisher<cm_msgs::msg::InitialSlamPose>::SharedPtr initialpose_pub_; //使用Ros1版本定位
    rclcpp::Publisher<cm_msgs::msg::Virtualwalls>::SharedPtr virtual_walls_pub_;
    rclcpp::Publisher<cm_msgs::msg::BitMap>::SharedPtr bit_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_updates_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_occ_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr static_map_info_pub_;
    rclcpp::Publisher<sps_common_msgs::msg::LiftPoi>::SharedPtr lift_poi_pub_;
    rclcpp::Publisher<sps_common_msgs::msg::UpdateNaviMap>::SharedPtr update_navi_map_pub_;
    rclcpp::Publisher<sps_common_msgs::msg::MapInfo>::SharedPtr map_info_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapping_occ_map_pub_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr update_occ_map_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr update_tag_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr loc_confidence_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::UpdateNaviMap>::SharedPtr update_navi_map_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::LocalizationStatus>::SharedPtr local_status_sub_;

    void pose_mode_callback(rclcpp::Client<sps_common_msgs::srv::Mode>::SharedFuture response);
    void aided_pose_callback(rclcpp::Client<sps_common_msgs::srv::AidedPose>::SharedFuture response);

    void update_occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void update_tag_pose_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void loc_confidence_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void battery_state_callback(const sps_common_msgs::msg::BatteryState::SharedPtr msg);
    void update_navi_map_callback(const sps_common_msgs::msg::UpdateNaviMap::SharedPtr msg);
    void local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg);

    void setAidedPose(const geometry_msgs::msg::Pose& pose, const int pose_type, const int match_type);
    void loadMapResultToResponse(const int result, std::int32_t& loadMapRes, std::string& loadMapDes);
    void localLoadMap(const std::string& mapPath, const std::int32_t& type, std::int32_t& loadMapRes, std::string& loadMapDes);
    void loadNaviMapImpl(const std::string& mapPath, const std::int32_t& type, std::int32_t& loadMapRes, std::string& loadMapDes);
    bool loadDefaultCharger();
    bool getRobotPose(sps_common_msgs::msg::PixelPose &pose);
    bool getRobotPose(geometry_msgs::msg::PoseStamped& cur_robot_pose);
    bool publishInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& cur_pose, int is_optimize); //使用Ros1版本定位
    bool publishInitialPose();
    bool publishLiftPoiForNavi(const std::string& lift_id, double lift_safe_dist);
    bool getInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped &pose);
    bool getLatestlPose(geometry_msgs::msg::PoseWithCovarianceStamped &pose);
    bool getChargePile(geometry_msgs::msg::PoseWithCovarianceStamped &pose);
    int updateNaviMap();
    void publishMapAll();
    void tagPoseModeCall();
    void loadDefaultMap();
};

#endif
