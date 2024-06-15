#ifndef LOCAL_MANAGER_HPP_
#define LOCAL_MANAGER_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "cm_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "service_client/get_charge_pile.hpp"
#include "service_client/get_map_info.hpp"
#include "service_client/load_navi_map.hpp"
#include "service_client/set_aided_pose.hpp"
#include "service_client/set_slam_model_client.hpp"
#include "service_client/publish_mapping_map.hpp"
#include "sps_common_msgs/msg/local_state.hpp"
#include "sps_common_msgs/msg/localization_status.hpp"
#include "sps_common_msgs/msg/initial_slam_state.hpp"
#include "sps_common_msgs/srv/aided_pose.hpp"
#include "sps_common_msgs/srv/change_pixel_pose_to_word_pose.hpp"
#include "sps_common_msgs/srv/change_word_pose_to_pixel_pose.hpp"
#include "sps_common_msgs/srv/get_map_info.hpp"
#include "sps_common_msgs/srv/sps_load_map.hpp"
#include "sps_common_msgs/srv/set_slam_model.hpp"
#include "sps_common_msgs/srv/publish_map_data.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "system_manager/define.h"

namespace naviengine
{

class LocalManager : public rclcpp::Node
{
public:
    LocalManager();
    ~LocalManager();

    static inline std::string toString(double val, int precision = 0);

    bool updateMapInfo();
    std::string getMapInfo(const std::string& key);
    void prepareInitPose(SCENETYPE scene_type);
    ERESULT setWorldPose(const geometry_msgs::msg::Pose& init_pose,
                         const LOCALINITTYPE& pose_type,
                         const SENSORMATCHTYPE& match_type,
                         const SCENETYPE& scene_type);
    ERESULT setSlamModel(const int slam_model);
    void setDepthScanState(const bool &state);
    bool getRobotPose(sps_common_msgs::msg::PixelPose& pose);
    bool getRobotPose(geometry_msgs::msg::Pose& w_pose);
    bool isLocalError();
    bool isInitComplete();
    bool changePixelPoseToWordPose(const sps_common_msgs::msg::PixelPose& p_pose, geometry_msgs::msg::Pose& w_pose);
    bool changeWordPoseToPixelPose(const geometry_msgs::msg::Pose& w_pose, sps_common_msgs::msg::PixelPose& p_pose);

private:
    typedef enum
    {
        SLAM_IDLE = 1300,
        SLAM_MAPPING = 1301,
        SLAM_LOC  = 1302,
        SLAM_MAPPING_PART_INCREMENTAL  = 1303,
        SLAM_MAPPING_ALL_INCREMENTAL_AND_ALIGN = 1304,
        SLAM_MAPPING_ODOM_CALIB  = 1305
    } ESLAMMODEL;

    std::mutex posefile_lock_;
    std::shared_ptr<std::thread> publish_thread_;
    sps_common_msgs::msg::PixelPose initial_pose_;

    geometry_msgs::msg::Pose latest_loc_pose_;
    LOCALTYPE latest_loc_status_ = IDLE;
    float latest_loc_confidence_ = 0.0;  // sub loc confidence
    bool latest_loc_map_update_ = false;

    int initial_state_ = 0;
    SCENETYPE scene_type_ = UNKNOWN;

    int curr_slam_model_ = SLAM_LOC;

    std::string home_path_ = "/home/ginger";
    double static_pose_upload_fps_ = 1.0;

    bool shutdown_ = false;

    float resolution_ = 0.05;
    std::string map_uuid_;
    std::string map_name_;
    float origin_x_ = 0.0;
    float origin_y_ = 0.0;
    int grid_width_ = 0;
    int grid_height_ = 0;

    rclcpp::TimerBase::SharedPtr load_default_map_timer_;

    void parameter_init();
    void publish_init();
    void client_init();
    void subscription_init();

    rclcpp::Publisher<sps_common_msgs::msg::PixelPose>::SharedPtr pixel_pose_pub_;
    rclcpp::Publisher<sps_common_msgs::msg::LocalState>::SharedPtr local_info_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr depth_scan_switch_pub_;

    std::shared_ptr<GetMapInfoServiceClient> get_map_info_client_;
    std::shared_ptr<GetChargePileServiceClient> get_charge_pile_client_;
    std::shared_ptr<SetAidedPoseServiceClient> set_aided_pose_client_;
    std::shared_ptr<LoadNaviMapServiceClient> load_map_client_;
    std::shared_ptr<SetSlamModelServiceClient> set_slam_model_client_;
    std::shared_ptr<PublishMappingMapServiceClient> publish_mapping_map_client_;

    rclcpp::Subscription<sps_common_msgs::msg::LocalizationStatus>::SharedPtr local_status_sub_;
    rclcpp::Subscription<sps_common_msgs::msg::InitialSlamState>::SharedPtr init_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr shutdown_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr update_occ_map_sub_;

    void loadDefaultMapTimerCallback();

    void local_status_callback(const sps_common_msgs::msg::LocalizationStatus::SharedPtr msg);
    void init_state_callback(const sps_common_msgs::msg::InitialSlamState::SharedPtr msg);
    void shutdown_callback(const std_msgs::msg::Int8::SharedPtr msg);
    void update_occ_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    bool loadDefaultMap();
    bool getChargePilePose();
    void setPosePixelZero();
    bool getLocalInfo(sps_common_msgs::msg::LocalState& localInfo);
    bool readRobotPoseFromFile();
    void writeRobotPose();
    void loadMappingMap();
};

}  // namespace naviengine

#endif
