#ifndef MAP_DATABASE_HPP_
#define MAP_DATABASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_structure.h"
#include "lift_poi_process.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "sps_common_msgs/msg/bit_map.hpp"
#include "cm_msgs/msg/bit_map.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"
#include "sps_common_msgs/msg/virtual_wall.hpp"
#include "sps_common_msgs/msg/special_areas.hpp"
#include "sps_common_msgs/msg/map_info.hpp"
#include "sps_common_msgs/msg/semantics_area.hpp"

#include <vector>
#include <mutex>
#include <map>
#include <unordered_map>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

using RosNodeHandlePtr = std::shared_ptr<rclcpp::Node>;
using OccMapPtr = std::shared_ptr<nav_msgs::msg::OccupancyGrid>;
//using BitMapPtr = std::shared_ptr<sps_common_msgs::msg::BitMap>;
using BitMapPtr = std::shared_ptr<cm_msgs::msg::BitMap>;
using TagPosesPtr = std::shared_ptr<std::vector<TagPoi>>;

class MapDatabase
{
private:
    static MapDatabase* m_instance;
    static pthread_mutex_t mutex;
    MapDatabase();
    ~MapDatabase();

    std::string home_path_;
    bool map_loaded_;
    builtin_interfaces::msg::Time load_time_;
    cv::Mat grid_img_;
    double resolution_ = 0.05;
    double threshold_free_ = 0.25;
    double threshold_occupied_ = 0.65;
    std::vector<double> origin_;
    std::string map_uuid_;
    std::string map_name_;
    std::string map_version_;
    std::string map_frame_ = "map";
    std::string base_frame_ = "base_link";
    int map_floor_;
    BitMapPtr bit_map_;
    OccMapPtr occ_map_;
    OccMapPtr static_map_;
    TagPosesPtr tag_poses_;
    sps_common_msgs::msg::VirtualWall virtual_wall_;

    // about map poi
    std::unordered_map<std::string, CommonPoi> all_poi_;// name + poi

    std::unordered_map<std::string, NormalPoi> normal_poi_;// name + poi
    std::unordered_map<std::string, ChargerPoi> charger_poi_; // charger pose: name + poi
    std::unordered_map<std::string, WaitingPoi> charger_waiting_Poi_; // name + poi
    std::unordered_map<std::string, LiftPoi> lift_poi_;  //lift poi: groud_id + poi
    std::unordered_map<std::string, WaitingPoi> lift_waiting_Poi_; // name + poi
    std::unordered_map<std::string, DoorPoi> door_poi_;  //door poi: name + poi
    std::unordered_map<std::string, TempStopPoi> temp_stop_poi_;  //临时停靠点: name + poi
    std::unordered_map<std::string, FoodDeliveryPoi> food_delivery_poi_;  // 送餐: name + poi
    std::unordered_map<std::string, DisinfectionWarehousePoi> disinfection_warehouse_poi_;  // 消毒仓: name + poi

    std::vector<SemanticsArea> semantics_areas_; // semantics area
    std::vector<geometry_msgs::msg::PoseStamped> poses_stamped_;

    LiftPoiProcess* process_lift_poi_ = nullptr;

    int loadGridMap(const std::string& info_file, const std::string& yaml_file, const std::string& image_file);
    bool loadSemanticsPoi(const std::string& poi_file);
    int loadVirtualWall(const std::string& wall_file);
    bool loadSemanticsArea(const std::string& area_file);
    void updateLoadedMapId(const std::string& path, const std::string &uuid);
    bool getWorldPose(const sps_common_msgs::msg::PixelPose &p_pose, geometry_msgs::msg::Pose &w_pose);
    int processLiftPoi(
            const std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>>& entry_poi,
            const std::map<std::string, sps_common_msgs::msg::PixelPose>& out_poi, 
            const std::map<std::string, sps_common_msgs::msg::PixelPose>& inner_poi,
            const std::map<std::string, std::vector<sps_common_msgs::msg::PixelPose>>& corner_poi,
            std::unordered_map<std::string, LiftPoi>& lift_poi);
    void optimizeLiftPoi(std::unordered_map<std::string, LiftPoi>& src_lift_poi);

    double getWorldRadian(const double &pixel_radian) const;
    bool isInRectangle(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2, 
                              const geometry_msgs::msg::Point32& p3, const geometry_msgs::msg::Point32& p4, 
                              const geometry_msgs::msg::Point32& p);
    float getCross(const geometry_msgs::msg::Point32& p1, const geometry_msgs::msg::Point32& p2, 
                          const geometry_msgs::msg::Point32& p);
public:
    static MapDatabase* getInstance();
    void init(const std::string& home_path);
    void clearAllMapData();
    bool getMapLoadStatus();
    int loadTestMap(const std::string& map_path);
    int loadMap(const std::string& map_path, std::string uuid = "");
    std::string getMapInfo(const std::string &key);
    bool getMapInfo(sps_common_msgs::msg::MapInfo &map_info);
    bool getBitMap(cm_msgs::msg::BitMap& bitMap);
    bool getOccMap(nav_msgs::msg::OccupancyGrid& occMap);
    bool getStaticMap(nav_msgs::msg::OccupancyGrid& staticMap);
    bool getVirtualWall(sps_common_msgs::msg::VirtualWall& virtualWall);
    bool getAllPoints(std::unordered_map<std::string, CommonPoi>& all_points);
    bool getChargerPoints(std::unordered_map<std::string, ChargerPoi>& charger_points);
    bool setDefaultChargePile(const std::string &charger_name, const sps_common_msgs::msg::PixelPose &pose);
    bool getSemanticAreaInfo(const double pose_x, const double pose_y, double &area_speed, AreaType &area_type, sps_common_msgs::msg::SpecialAreas &special_areas);
    bool getLiftPoints(std::unordered_map<std::string, LiftPoi>& lift_points);
    bool getPosesStamped(std::vector<geometry_msgs::msg::PoseStamped>& poses_stamped);
    void updateTagPoses(const std::vector<TagPoi> tag_poses);
    bool getSemanticAreaInfo(std::vector<sps_common_msgs::msg::SemanticsArea>& semantics_areas);
    void updateMapDataInMapping(const nav_msgs::msg::OccupancyGrid& occMap, cm_msgs::msg::BitMap& bit_map);
};

#endif
