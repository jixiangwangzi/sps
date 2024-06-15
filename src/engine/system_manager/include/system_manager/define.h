

#ifndef _DEFINE_H_
#define _DEFINE_H_

#include <stdio.h>

#include <ctime>

#include "rclcpp/rclcpp.hpp"


typedef char int8;
typedef short int32;
typedef int int64;
typedef unsigned char uint8;
typedef unsigned short uint32;
typedef unsigned int uint64;
typedef float float32;
typedef float float64;
typedef double double32;
typedef double double64;

#ifndef PIXEL_POSE_DEFINED
#define PIXEL_POSE_DEFINED
typedef struct PixelPose
{
    float32 x{0};
    float32 y{0};
    float32 theta{0};
    float32 loc_confidence{0};
    float32 loc_threshold{0};
    int type{-1};
} PixelPose;
#endif

typedef struct
{
    float32 x;
    float32 y;
    float32 z;
} Point;


typedef struct
{
    uint32 seq;
    double32 sec;
    double32 nsec;
    std::string frame_id;
} Header;


typedef struct
{
    float64 vx;
    float64 vy;
    float64 vz;
} Twist;

typedef struct
{
    float32 x;
    float32 y;
    float32 z;
    float32 w;
} Quaternion;



typedef struct
{
    Point position;
    Quaternion orientation;
} Pose;



typedef enum
{
    CLIFF_DETECTED_NONE = 100,
    CLIFF_DETECTED_AHEAD,
    CLIFF_RESUMED_AHEAD,
    CLIFF_DETECTED_BACK,
    CLIFF_RESUMED_BACK
} DropType;

typedef enum
{
    COLLISION_NONE = 110,
    COLLISION_DETECTED,
    COLLISION_RESUMED
} CollisionType;

typedef enum
{
    NO_AUTO_MOVE,
    AUTO_MOVE_START_SUCCESS,
    AUTO_MOVE_START_FAILED,
    AUTO_MOVING,
    AUTO_MOVE_SUCCEED,
    AUTO_MOVE_FAILED,
} AutoMoveState;

typedef enum
{
    EMERGENCY_DOWN = 120,
    EMERGENCY_UP
} EmergencyType;

typedef enum
{
    NONE_DOCK_STATE,
    DOCKING_START,
    UNDOCKING_START,
    CHARGING_START,
    UNDOCKING_SUCCESS,
    NO_DOCK_FOUND,
    DOCKING_FAILED,
    CHARGING_FAILED,
    UNDOCKING_FAILED,
    CANCEL_START,
    CANCEL_SUCCESS,
    CANCEL_FAILED
} DockState;

typedef struct BatteryState
{
    float32 voltage;
    float32 current;
    float32 current_capacity;
    float32 init_capacity;
    float32 dump_energy;
    uint8 cell_type;
    uint8 id;
    uint32 status;
    int8_t chargeset_type;
} BatteryState;

typedef enum HWErrorType
{
    ERROR_HAPPEN = 0,
    ERROR_RESUME
} HWErrorType;
typedef enum
{
    LINE = 0,
    DOCK = 1,
    BOTH = 2
} CHARGE_TYPE;


#ifndef NAVISTATE_DEFINED
#define NAVISTATE_DEFINED
enum class NAVISTATE
{
    NAVI_STATE_IDLE = 0,
    NAVI_STATE_ACTIVE = 11,
    NAVI_STATE_SUCCESS = 12,
    NAVI_STATE_FAILED = 13,
    NAVI_STATE_CANCELED = 14,
    NAVI_STATE_PAUSE = 15,
    NAVI_STATE_EMERGENCY = 16,
    NAVI_STATE_RUNNING = 17
};


enum class NAVIDETAILEDSTATE
{
    NAVI_DETAILED_STATE_IDLE = 0,                       //空闲
    NAVI_DETAILED_STATE_GLOBAL_REACHED = 1,             //到达目标点
    NAVI_DETAILED_STATE_GLOBAL_UNREACHABLE = 2,         //目标点不可达
    NAVI_DETAILED_STATE_FINISHED = 3,                   //导航结束
    NAVI_DETAILED_STATE_FOLLOWING_PATH = 4,             //跟线中
    NAVI_DETAILED_STATE_GLOBAL_PLAN_FAILED = 5,         //规划路径失败
    NAVI_DETAILED_STATE_GLOBAL_PLANNING = 6,            //规划路径中
    NAVI_DETAILED_STATE_AVOIDING_OBSTACLE = 7,          //避障中
    NAVI_DETAILED_STATE_AVOIDING_OBSTACLE_WAITING = 8,  //避障等待中
    NAVI_DETAILED_STATE_AVOIDING_OBSTACLE_FAILED = 9,   //避障失败
    NAVI_DETAILED_STATE_BLOCKED = 10                    //导航阻塞
};



#endif

typedef enum
{
    // General
    E_OK = 0,
    E_FALSE = -9000,
    E_BADSTATE = -9001,
    E_NOTSUPPORT = -9002,
    E_UNKOWN = -9003,
    E_REPEAT = -9004,
    E_NOSESSION = -9005,
    // StartBuildMap
    E_STARTBUILDMAP_SUCCESS = E_OK,
    E_STARTBUILDMAP_FAILED = -201,
    E_STARTBUILDMAP_REPEAT = -202,
    // StopBuildMap
    E_STOPBUILDMAP_SUCCESS = E_OK,
    E_STOPBUILDMAP_FAILED = -9201,
    E_STOPBUILDMAP_REPEAT = -9202,
    // PauseBuildMap
    E_PAUSEBUILDMAP_SUCCESS = E_OK,
    E_PAUSEBUILDMAP_FAILED = -9301,
    E_PAUSEBUILDMAP_REPEAT = -9302,
    // ResumeBuildMap
    E_RESUMEBUILDMAP_SUCCESS = E_OK,
    E_RESUMEBUILDMAP_FAILED = -9401,
    E_RESUMEBUILDMAP_REPEAT = -9402,
    // SaveMap
    E_SAVEMAP_SUCCESS = E_OK,
    E_SAVEMAP_FAILED = -301,
    E_SAVEMAP_MAPPING_NOT_START = -302,
    E_SAVEMAP_PATH_INVALID = -303,
    E_SAVEMAP_NO_DATA = -304,
    // LoadMap
    E_LOADMAP_SUCCESS = E_OK,
    E_LOADMAP_FAILED = -401,
    E_LOADMAP_MODE_ERROR = -402,
    E_LOADMAP_NO_MAP_PNG = -403,
    E_LOADMAP_NO_MAP_YAML = -404,
    E_LOADMAP_MAP_PNG_INVALID = -405,
    E_LOADMAP_MAP_YAML_INVALID = -406,
    E_LOADMAP_NO_MAP_PBSTREAM = -407,
    E_LOADMAP_MAP_PBSTREAM_UNVALID = -408,
    E_LOADMAP_NO_MAP_PCD = -409,
    E_LOADMAP_MAP_PCD_UNVALID = -410,
    // SetPixelPose & SetWorldPose
    E_SETPOSE_SUCCESS = E_OK,
    E_SETPOSE_FAILED = -501,
    E_SETPOSE_MODE_ERROR = -502,
    E_SETPOSE_POSE_INVALID = -503,
    // CheckPose
    E_CHECKPOSE_SUCCESS = E_OK,
    E_CHECKPOSE_FAILED = -601,
    E_CHECKPOSE_MODE_ERROR = -602,
    // NaviCmd
    E_NAVICMD_SUCCESS = E_OK,
    E_NAVICMD_FAILED = -701,
    E_NAVICMD_MODE_ERROR = -702,
    E_NAVICMD_ID_INVALID = -703,
    E_NAVICMD_NO_ACTIVE_TASK = -704,
    E_NAVICMD_NO_PAUSED_TASK = -705,
    E_NAVICMD_ANOTHER_TASK_RUNNING = -706,
    // HA Cmd
    E_HACMD_SUCCESS = E_OK,
    E_HACMD_FAILED = -801,
    E_HACMD_INVALID = -802,
    // StartNavi
    E_STARTNAVI_SUCCESS = E_OK,
    E_STARTNAVI_FAILED = -1,
    E_STARTNAVI_MODE_ERROR = -2,
    E_STARTNAVI_UNKNOWN_TYPE = -3,
    E_STARTNAVI_NAME_INVALID = -4,
    E_STARTNAVI_POSE_INVALID = -5,
    E_STARTNAVI_PLAN_FAILED = -6,
    E_STARTNAVI_VEL_INVALID = -7,
    E_STARTNAVI_LINE_CHARGING = -8,
    E_STARTNAVI_DOCK_CHARGING = -9,
    E_STARTNAVI_ROBOT_BLOCKED = -10,
    E_STARTNAVI_NO_CHARGER_FOUND = -11,
    E_STARTNAVI_BATTERY_LEVEL_LOW = -12,
    E_STARTNAVI_TASK_CONFLICT = -13,
    E_STARTNAVI_LOCAL_ERROR = -14,
    // StopNavi
    E_STOPNAVI_SUCCESS = E_OK,
    E_STOPNAVI_FAILED = -9501,
    E_STOPNAVI_MODE_ERROR = -9502,
    // PauseNavi
    E_PAUSEAVI_SUCCESS = E_OK,
    E_PAUSENAVI_FAILED = -9601,
    E_PAUSENAVI_MODE_ERROR = -9602,
    // ResumeNavi
    E_RESUMENAVI_SUCCESS = E_OK,
    E_RESUMENAVI_FAILED = -9701,
    E_RESUMENAVI_MODE_ERROR = -9702,
    // ParamData
    E_PARAMDATA_SUCCESS = E_OK,
    E_PARAMDATA_FAILED = -7101,
} ERESULT;

typedef enum
{
    NAVI_NORMAL = 0,
    NAVI_GOINLIFT = 1,
    NAVI_GOCHARGE = 2,
    NAVI_GOOUTLIFT = 3,
    NAVI_IGNORE_ANGLE = 4,
    NAVI_TEMP = 10
} ENAVITYPE;

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

typedef enum
{
    GRID_POSE = 0,
    MAP_POSE = 1,
} POSETYPE;

typedef enum
{
    TYPE_2D = 1,
    TYPE_3D,
    TYPE_VSLAM
} EMAPPINGTYPE;


typedef enum
{
    E_NORMAL_MAPPING_MODE = 1,
    E_PART_INCREMENTAL_MAPPING_MODE = 3,
    E_ALL_INCREMENTAL_AND_ALIGN_MAPPING_MODE = 4
} EMAPPINGMODE;

typedef enum
{
    NONE = 0,
    LIDAR,
    CAMERARGBD,
    ALL
} SENSORMATCHTYPE;

typedef enum
{
    INIT_POSE = 0,
    LOCAL_POSE,
    GLOBAL_POSE,
    PREDICT_POSE
} LOCALINITTYPE;

typedef enum
{
    IDLE = 0,
    INITIALIZING,
    RUNNING,
    WEAK,
    LOST,
    BLOCKED
} LOCALTYPE;

typedef enum
{
    UNKNOWN = 0,
    CHARGING_PILE,
    LIFT_INTERIOR,
    START_UP_WITH_SAVED_POSE,
    SWITCH_MAP_WITH_SAVED_POSE,
    CLOUD_INSTRUCTION
} SCENETYPE;


typedef enum
{
    IMU = 0,
    BIGLIDAR,
    SMALLLIDAR,
    ODOM,
    ULTRA,
    BUMPER,
    CAMERA,
    BAROMETER
} SENSORTYPE;

typedef enum
{
    CANCEL = 1,
    PAUSE,
    RESUME
} NAVICMD;


typedef struct
{
    std::string task_id;
    ENAVITYPE navi_type;
    NAVISTATE navi_state;
    std::string navi_map;
    std::string navi_path;
    NAVIDETAILEDSTATE navi_detailed_state;
    float32 linear_speed;
    float32 angular_speed;
    float32 navigation_time;
    float32 distance_remaining;
    float32 estimated_time_remaining;
    float32 mileage;
    int recovery_count;
    int number_of_poses_remaining;

} NaviStateInfo;

struct NaviPathInfo
{
    std::string task_id;
    std::string map_id;
    std::string map_name;
    std::string map_type;
    std::string description;
    std::string map_version;
    PixelPose curr_poi;
    PixelPose target_poi;
    std::vector<PixelPose> path;
};

const std::string SPS_NAVI_BT_XML = "/home/ginger/bt_xml/";


typedef std::function<void(NaviStateInfo)> NavStateFun;


#ifndef SPS_LOG_H_
#define SPS_LOG_H_


#define ROS_DEBUG(arg...) RCLCPP_DEBUG(rclcpp::get_logger("naviengine"), ##arg)
#define ROS_INFO(arg...) RCLCPP_INFO(rclcpp::get_logger("naviengine"), ##arg)
#define ROS_WARN(arg...) RCLCPP_WARN(rclcpp::get_logger("naviengine"), ##arg)
#define ROS_ERROR(arg...) RCLCPP_ERROR(rclcpp::get_logger("naviengine"), ##arg)
#define ROS_FATAL(arg...) RCLCPP_FATAL(rclcpp::get_logger("naviengine"), ##arg)

#endif  //


#endif
