#ifndef SLAM_NODE_H_
#define SLAM_NODE_H_
#include "lidar_slam/SpsSlam.h"
#include "sps_common_msgs/srv/set_slam_model.hpp"
#include "sps_common_msgs/srv/aided_pose.hpp"
#include "sps_common_msgs/srv/sps_load_map.hpp"

#include "rclcpp/rclcpp.hpp"
// #include "ginger_msgs/SetSlamModel.h"
// #include "ginger_msgs/RdmAlarm.h"
// #include "ginger_msgs/TryToReLocalization.h"
// #include "ginger_msgs/InitialSlamPose.h"
#include "std_srvs/srv/trigger.h"
#include "std_msgs/msg/float32.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"



namespace sps {

enum SlamModel {
    MODE_IDLE = 0,
    MODE_MAPPING,
    MODE_LOCALIZATION,
    MODE_MAPPING_PART_INCREMENTAL,
    MODE_MAPPING_ALL_INCREMENTAL_AND_ALIGN,
    MODE_MAPPING_ODOM_CALIB
};
enum MappingModel {
    MAP_CARTOGRAPHER = 0
};

enum LocModel {
    LOC_CARTOGRAPHER = 0
};

/**
 * @enum nav2_map_server::MapMode
 * @brief Describes the relation between image pixel values and map occupancy
 * status (0-100; -1). Lightness refers to the mean of a given pixel's RGB
 * channels on a scale from 0 to 255.
 */
enum class MapMode
{
  /**
   * Together with associated threshold values (occupied and free):
   *   lightness >= occupied threshold - Occupied (100)
   *             ... (anything in between) - Unknown (-1)
   *    lightness <= free threshold - Free (0)
   */
  Trinary,
  /**
   * Together with associated threshold values (occupied and free):
   *   alpha < 1.0 - Unknown (-1)
   *   lightness >= occ_th - Occupied (100)
   *             ... (linearly interpolate to)
   *   lightness <= free_th - Free (0)
   */
  Scale,
  /**
   * Lightness = 0 - Free (0)
   *          ... (linearly interpolate to)
   * Lightness = 100 - Occupied (100)
   * Lightness >= 101 - Unknown
   */
  Raw,
};

static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

struct LoadParameters
{
  std::string image_file_name;
  double resolution{0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};
class SlamNode {

public:
    SlamNode(RosNodeHandlePtr& nh);
    ~SlamNode();

private:
    //ros
    RosNodeHandlePtr private_nh_ = nullptr;
    RosNodeHandlePtr nh_= nullptr;

    RosNodeHandlePtr nh_mapping_ = nullptr;
    RosNodeHandlePtr nh_loc_ = nullptr;

    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_cast_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_ = nullptr;
    std::vector<rclcpp::ServiceBase::SharedPtr> v_servers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> v_sub_;

    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_footprint";

    
    std::shared_ptr<SpsSlam> slam_obj_ = nullptr;
    SlamModel slam_model_ = MODE_IDLE;
    MappingModel mapping_model_ = MAP_CARTOGRAPHER;
    LocModel loc_model_ = LOC_CARTOGRAPHER;
    float loc_confidence_ = 0.0f;
    nav_msgs::msg::OccupancyGrid occ_map_;
    //当前机器人位姿
    geometry_msgs::msg::PoseWithCovarianceStamped cur_pose_;

    //function

    void SetSlamModelService(const std::shared_ptr<sps_common_msgs::srv::SetSlamModel::Request> req, 
                                std::shared_ptr<sps_common_msgs::srv::SetSlamModel::Response> res);

    LoadParameters LoadMapYaml(const std::string & yaml_filename);

    void LoadMapFromFile(const LoadParameters& load_parameters);

    void LoadMap(const std::shared_ptr<sps_common_msgs::srv::SpsLoadMap::Request> req, 
                    std::shared_ptr<sps_common_msgs::srv::SpsLoadMap::Response> res);

    void SetOccMapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_msg);

    void SetAidedPose(const std::shared_ptr<sps_common_msgs::srv::AidedPose::Request> req,
                        std::shared_ptr<sps_common_msgs::srv::AidedPose::Response> res);
    void SetInitPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg);
    // void SetInitPoseSelfCallback(const ginger_msgs::InitialSlamPose& initial_pose_msg);

    void GetRosParams();

    bool StartLocalizationModel(std::string& message);
    bool StartMappingModel(std::string& message, const SlamModel& slam_model);
    void naviTypeCB(const std_msgs::msg::Int32& msg);
    void GetLocConfidence(const std_msgs::msg::Float32& msg);
};

}  //namespace






#endif
