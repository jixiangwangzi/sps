#ifndef MAP_STRUCTURE_H_
#define MAP_STRUCTURE_H_

#include <vector>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sps_common_msgs/msg/pixel_pose.hpp"

using VecWorldPoints = std::vector<geometry_msgs::msg::Point32>;
using VecWorldPose = std::vector<geometry_msgs::msg::Pose>;
using NamedPixelPoses = std::unordered_map<std::string, sps_common_msgs::msg::PixelPose>;
using vecPixelPoses = std::unordered_map<std::string, std::vector<sps_common_msgs::msg::PixelPose>>;


enum PoiType {
    UNKOWN_TYPE = -1,
    NORMAL_POI = 0,
    CHARGER_POI = 1,
    LIFT_ENTRY_POI = 2,
    LIFT_INNER_POI = 3,
    LIFT_CORNER_POI = 4,
    DOOR_POI = 5,
    RELOCALIZATION_POI = 6, //暂时未使用
    LIFT_OUT_POI = 7,
    LIFT_WAITING_POI = 8,
    CHARGER_WAITING_POI = 9,
    TEM_STOP_POI = 10,  // 临时停靠点
    FOOD_DELIVERY_POI = 11,  // 送餐
    DISINFECTION_WAREHOUSE_POI = 12,  // 消毒仓
    TAG_POI = 13  // 头顶二维码
};

// struct WorldPoint {
//     double x;
//     double y;
//     double z;
//     double yaw;
//     double pitch;
//     double roll;
// }

struct CommonPoi {
    std::string name;
    PoiType poi_type;
    sps_common_msgs::msg::PixelPose pixel_pose;
};

struct NormalPoi {
    std::string name;
    int id;
    sps_common_msgs::msg::PixelPose pixel_pose;
};

struct TempStopPoi {
    std::string name;
    sps_common_msgs::msg::PixelPose pixel_pose;
};

struct FoodDeliveryPoi {
    std::string name;
    sps_common_msgs::msg::PixelPose pixel_pose;
};

struct DisinfectionWarehousePoi {
    std::string name;
    sps_common_msgs::msg::PixelPose pixel_pose;
};

struct ChargerPoi {
    std::string name;
    bool is_default = false;
    sps_common_msgs::msg::PixelPose pixel_pose;

    //
    sps_common_msgs::msg::PixelPose pixel_pose_navi; // 导航的上桩点
};

struct LiftPoi {
    std::string name;
    std::string group_id;               // 电梯id, lift_id
    geometry_msgs::msg::Pose inner_poi;   // 电梯内点,每个电梯1个
    geometry_msgs::msg::Pose out_poi;     // 出电梯点, 每个电梯1个, 分布在电梯门中轴线上, map frame
    VecWorldPose entry_poi;           // 入电梯点, 每个电梯2个, 分布在电梯门两侧, 电梯外面对门, 左, 右排序, map frame
    VecWorldPose door_poi;            // 电梯门上的两个点, 电梯外面对门, 左, 右排序, map frame
    VecWorldPose corner_poi;          // 电梯轿厢四个角点, 电梯外面向门, 左为第一个点, 逆时针排序, map frame
    //
    sps_common_msgs::msg::PixelPose pixel_out_poi;
    sps_common_msgs::msg::PixelPose pixel_inner_poi;
    std::vector<sps_common_msgs::msg::PixelPose> pixel_entry_poi; 
    std::vector<sps_common_msgs::msg::PixelPose> pixel_door_poi; //电梯门上的两个像素点
    std::vector<sps_common_msgs::msg::PixelPose> pixel_corner_poi; // 箱体角点
};

// waiting poi for lift & charger
struct WaitingPoi {
    std::string name;
    geometry_msgs::msg::Pose waiting_poi;
    sps_common_msgs::msg::PixelPose pixel_waiting_poi;
};

struct DoorPoi {
    std::string name;
    int id; //
    VecWorldPoints door_poi; // 从外侧面对门，左，右排序, map frame
};

struct TagPoi {
    std::string id;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

enum AreaType {
    UNDEFINED_AREA           = -1,  // 未定义区域
    GENERAL_SPEED_LIMIT_AREA =  0,  // 一般限速区
    STEEP_SLOPE_AREA         =  1,  // 陡坡区域
    CLOSE_SONAR_AREA         =  2,  // 关闭超声区
    LONG_CORRIDOR_AREA       =  3,  // 长走廊区
    NARROW_CHANNEL_AREA      =  4,  // 窄通道区
    SPECIAL_INDUCTION_AREA   =  5,  // 特殊感应去
    NOISE_AREA               =  6   // 噪点区
};

struct SemanticsArea {
    std::string name;
    double slope;
    AreaType type;
    double speed;
    double yaw;
    std::vector<geometry_msgs::msg::Point32> points; // pixel frame
};

enum LiftPoiType {
    UNKOWN_POINT = -1,
    INNER_POINT = 0,
    ENTRY_POINT = 1,
    OUT_POINT = 2
};

#endif
