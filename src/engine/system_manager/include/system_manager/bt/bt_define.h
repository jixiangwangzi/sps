/*
 * Copyright (c) 2021, Cloudminds, Inc.
 * All rights reserved.
 *
 * author: Ewen Dong
 */

#ifndef _BT_DEFINE_H_
#define _BT_DEFINE_H_

#include "map"
#include "set"
#include "system_manager/define.h"


// define blackboard keys
const std::string BB_INPUT_FAILURE_CODE = "failure_code";
const std::string BB_INPUT_SUCCESS_CODE = "success_code";
const std::string BB_INPUT_ROBOT_STATE = "robot_state";
const std::string BB_INPUT_DISTINCT_EVENT = "distinct_event";
const std::string BB_OUTPUT_ON_RESPONSE_CODE = "on_response_code";
const std::string BB_ROBOT_INITIALIZED = "robot_initialized";
const std::string BB_ROBOT_EVENT = "robot_event";
const std::string BB_ROBOT_EVENT_RESPONSE = "robot_event_response";
const std::string BB_ROBOT_TASK_CTX = "robot_task_ctx";
const std::string BB_ROBOT_TASK_CTX_COMPLETED = "robot_task_ctx_completed";
const std::string BB_ROBOT_START_MAPPING_CTX = "robot_start_mapping_ctx";
const std::string BB_ROBOT_LOAD_MAP_CTX = "robot_load_map_ctx";
const std::string BB_ROBOT_SAVE_MAP_CTX = "robot_save_map_ctx";
const std::string BB_ROBOT_START_NAVI_CTX = "robot_start_navi_ctx";
const std::string BB_ROBOT_NAVI_CTX = "robot_navi_ctx";
const std::string BB_ROBOT_COLLISION_TYPE = "robot_collision_type";
const std::string BB_ROBOT_EMERGENCY_TYPE = "robot_emergency_type";
const std::string BB_ROBOT_HW_ERROR_TYPE = "robot_hw_error_type";
const std::string BB_ROBOT_CHARGING_TYPE = "robot_charging_type";
const std::string BB_ROBOT_TASK_ID_MARKED = "robot_task_id_marked";
const std::string BB_ROBOT_HALF_POINTS_VECTOR = "robot_half_points_vector";

// define blackboard values
// define robot state
const std::string STATE_INIT = "INIT";
const std::string STATE_IDLE = "IDLE";
const std::string STATE_MAPPING = "MAPPING";
const std::string STATE_MOTIONING = "MOTIONING";
const std::string STATE_STOP = "STOP";
const std::string STATE_CHARGING = "CHARGING";
const std::string STATE_ERROR = "ERROR";
const std::string STATE_TELEOPING = "TELEOPING";

// typedef enum
// {
//     INIT = 1,
//     IDLE,
//     MAPPING,
//     MOTIONING,
//     STOP,
//     CHARGING,
//     ERROR,
//     TELEOPING
// } EROBOTSTATE;

// define robot event
const std::string EVENT_INIT_ROBOT = "InitRobot";
const std::string EVENT_START_BUILD_MAP = "StartBuildMap";
const std::string EVENT_STOP_BUILD_MAP = "StopBuildMap";
const std::string EVENT_PAUSE_BUILD_MAP = "PauseBuildMap";
const std::string EVENT_RESUME_BUILD_MAP = "ResumeBuildMap";
const std::string EVENT_SAVE_MAP = "SaveMap";
const std::string EVENT_LOAD_MAP = "LoadMap";
const std::string EVENT_START_NAVI_TO_POSE = "StartNaviToPose";
const std::string EVENT_START_NAVI_THROUGH_POSES = "StartNaviThroughPoses";
const std::string EVENT_SET_NAVI_HALF_POINTS = "SetNaviHalfPoints";
const std::string EVENT_PAUSE_NAVI = "PauseNavi";
const std::string EVENT_RESUME_NAVI = "ResumeNavi";
const std::string EVENT_STOP_NAVI = "StopNavi";
const std::string EVENT_ON_NAVI_ACTIVE = "OnNaviActive";
const std::string EVENT_ON_NAVI_PROGRESS = "OnNaviProgress";
const std::string EVENT_ON_NAVI_PAUSE = "OnNaviPause";
const std::string EVENT_ON_NAVI_CANCELLED = "OnNaviCancelled";
const std::string EVENT_ON_NAVI_DONE = "OnNaviDone";
const std::string EVENT_START_HA = "StartHa";
const std::string EVENT_STOP_HA = "StopHa";
const std::string EVENT_NOTIFY_DROP = "NotifyDropEvent";
const std::string EVENT_NOTIFY_COLLISION = "NotifyCollisionEvent";
const std::string EVENT_NOTIFY_ESTOP = "NotifyEStopEvent";
const std::string EVENT_NOTIFY_CHARGING = "NotifyChargingEvent";
const std::string EVENT_NOTIFY_CRITICAL_HW_ERR = "NotifyCriticalHwErrEvent";


// define priority of state
const int PRIORITY_IDLE = 0;
const int PRIORITY_MAPPING = 10;
const int PRIORITY_MOTIONING = 10;
const int PRIORITY_STOP = 40;
const int PRIORITY_ERROR = 41;
const int PRIORITY_CHARGING = 40;
const int PRIORITY_TELEOPING = 50;


typedef enum
{
    RESUME_FALSE = 0,
    RESUME_TRUE
} ERESUMEEVENT;

class TaskCtx
{
public:
    TaskCtx(const std::string task_id = "") : task_id_(task_id)
    {
    }

public:
    std::string task_id_;
};

class StartNaviCtx : public TaskCtx
{
public:
    StartNaviCtx() : TaskCtx("")
    {
    }

    StartNaviCtx(const std::string task_id,
                 const ENAVITYPE type,
                 const std::string &goal_name,
                 PixelPose goal_pose,
                 const float &goal_tolerance)
        : TaskCtx(task_id), type_(type), goal_name_(goal_name), goal_pose_(goal_pose), goal_tolerance_(goal_tolerance)
    {
        this->task_id_ = task_id;
        this->type_ = type;
        this->goal_name_ = goal_name;
        this->goal_pose_ = goal_pose;
        this->goal_tolerance_ = goal_tolerance;
        this->points_.push_back(goal_pose_);
    }

public:
    ENAVITYPE type_;
    std::string goal_name_;
    PixelPose goal_pose_;
    float goal_tolerance_;
    std::vector<PixelPose> points_;

    void ks()
    {
        goal_pose_ = {};
    }
};

class HeaderTaskCtx : public TaskCtx
{
public:
    HeaderTaskCtx() : TaskCtx("")
    {
    }
    HeaderTaskCtx(const std::string &task_id, const Header &header) : TaskCtx(task_id), header_(header)
    {
    }

public:
    Header header_;
};



class HeaderStartNaviCtx : public HeaderTaskCtx
{
public:
    HeaderStartNaviCtx()
    {
    }

    HeaderStartNaviCtx(const Header &header,
                       const std::string &task_id,
                       ENAVITYPE type,
                       const Pose &start_pose,
                       const Pose &goal_pose,
                       const std::string &map_name,
                       const std::string &path_name,
                       const Twist &twist,
                       const std::vector<Pose> &points,
                       const std::vector<int> &points_type,
                       const std::string &bt_xml_filename,
                       const bool global_coverage,
                       const bool align_angle,
                       const std::string &lift_id = "")
        : HeaderTaskCtx(task_id, header)
    {
        this->type_ = type;
        this->map_name_ = map_name;
        this->path_name_ = path_name;
        this->start_pose_ = start_pose;
        this->goal_pose_ = goal_pose;
        this->twist_ = twist;
        this->bt_xml_filename_ = bt_xml_filename;
        this->global_coverage_ = global_coverage;
        this->align_angle_ = align_angle;
        this->lift_id_ = lift_id;
        this->points_ = points;
        this->points_type_ = points_type;
    }

public:
    ENAVITYPE type_;
    std::string lift_id_;
    std::string map_name_;
    std::string path_name_;
    Pose start_pose_;
    Pose goal_pose_;
    Twist twist_;
    std::string bt_xml_filename_;
    bool global_coverage_;
    bool align_angle_;
    std::vector<Pose> points_;
    std::vector<int> points_type_;
};

class NaviCtx : public TaskCtx
{
public:
    NaviCtx(const std::string &task_id = "",
            const ENAVITYPE navi_type = ENAVITYPE::NAVI_NORMAL,
            const NAVISTATE &navi_state = NAVISTATE::NAVI_STATE_IDLE,
            const std::string &description = "",
            bool active = false,
            const Pose *cur_pose = nullptr)
        : TaskCtx(task_id),
          navi_type_(navi_type),
          navi_state_(navi_state),
          description_(description),
          active_(active),
          cur_pose_(cur_pose)
    {
    }

public:
    ENAVITYPE navi_type_;
    NAVISTATE navi_state_;
    std::string description_;
    bool active_;
    const Pose *cur_pose_;
};

class MapCtx : public TaskCtx
{
public:
    MapCtx(const std::string &task_id, const std::string &map_name) : TaskCtx(task_id), map_name_(map_name)
    {
    }

public:
    std::string map_name_;
};

class StartMappingCtx : public TaskCtx
{
public:
    StartMappingCtx(const std::string &task_id, EMAPPINGTYPE mapping_type, EMAPPINGMODE mapping_mode)
        : TaskCtx(task_id), mapping_type_(mapping_type), mapping_mode_(mapping_mode)
    {
    }

public:
    EMAPPINGMODE mapping_mode_;
    EMAPPINGTYPE mapping_type_;
};

#define LoadMapCtx MapCtx
#define SaveMapCtx MapCtx

class HWErrorCtx
{
public:
    HWErrorCtx(HWErrorType type) : type_(type)
    {
    }

public:
    HWErrorType type_;
};


// engine state define

class EngineState
{
public:
    EngineState(const std::string &stateName,
                int priority,
                std::set<std::string> multi_event_set = std::set<std::string>())
    {
        this->state_name_ = stateName;
        this->priority_ = priority;
        this->multi_event_set_ = multi_event_set;
    }

    EngineState(const std::string &stateName,
                int priority,
                StartNaviCtx taskCtx,
                std::set<std::string> multi_event_set = std::set<std::string>())
    {
        this->state_name_ = stateName;
        this->priority_ = priority;
        this->multi_event_set_ = multi_event_set;
        this->taskCtx_ = taskCtx;
    }


public:
    std::string state_name_;
    int priority_;
    StartNaviCtx taskCtx_;
    std::set<std::string> multi_event_set_;
};

#endif