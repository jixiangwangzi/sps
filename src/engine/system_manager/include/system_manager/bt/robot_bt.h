/*
 * Copyright (c) 2021, Cloudminds, Inc.
 * All rights reserved.
 *
 * author: Ewen Dong
 */

#ifndef _ROBOT_BT_H_
#define _ROBOT_BT_H_

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <system_manager/robot_engine_observer.h>

#include <mutex>
#include <set>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "system_manager/bt/bt_define.h"
#include "system_manager/bt_node_impl.hpp"
#include "system_manager/define.h"
#include "system_manager/local_manager.hpp"
#include "system_manager/robot_ctrl.h"
#include "system_manager/robot_observer.h"
#include "system_manager/task_manager.hpp"
#include "system_manager/utils/timer.h"



namespace naviengine
{
class RobotBt : public IRobotObserver
{
public:
    virtual ~RobotBt();

    RobotBt(const RobotBt &) = delete;

    RobotBt &operator=(const RobotBt &) = delete;

    static RobotBt *GetInstance()
    {
        static RobotBt instance;
        return &instance;
    }

    // Must call before any func call once you get instance
    ERESULT Init(IEngineCtrl &engineCtrl,
                 IRobotEngineObserver &robotEngineObserver,
                 rclcpp::executors::MultiThreadedExecutor &exector,
                 const std::string &bt_xml_filename);
    bool loadBehaviorTree(const std::string &bt_xml_filename);
    // Map Interface
    ERESULT StartBuildMap(const std::string &task_id, EMAPPINGTYPE mapping_type, EMAPPINGMODE mapping_mode);

    ERESULT StopBuildMap(const std::string &task_id);

    ERESULT PauseBuildMap(const std::string &task_id);

    ERESULT ResumeBuildMap(const std::string &task_id);

    ERESULT SaveMap(const std::string &task_id, const std::string &map_name);

    ERESULT LoadMap(const std::string &task_id, const std::string &map_name);

    // Navi Interface
    // ERESULT StartNavi(const std::string &task_id, ENAVITYPE type, const std::string &goal_name, const PixelPose
    // &goal_pose, const float &goal_tolerance);
    ERESULT StartNaviToPose(const Header &header,
                            const std::string &task_id,
                            const std::string &lift_id,
                            ENAVITYPE type,
                            const Pose &pose,
                            const Twist &twist,
                            const std::string &bt_xml_filename,
                            const bool align_angle);

    ERESULT StartNaviThroughPoses(const Header &header,
                                  const std::string &task_id,
                                  ENAVITYPE type,
                                  const std::vector<Pose> &points,
                                  const std::vector<int> &points_type,
                                  const Twist &twist,
                                  const std::string &bt_xml_filename,
                                  const bool align_angle);
    ERESULT StopNavi(const std::string &task_id);

    ERESULT PauseNavi(const std::string &task_id);

    ERESULT ResumeNavi(const std::string &task_id);

    // State Interface
    std::string GetState();

    bool GetTaskInfo(std::string &task_id, ENAVITYPE &navi_type);

    // Normal
    ERESULT CheckPose(const std::string name, PixelPose &pixel_pose);

    ERESULT SetPoseinitialize(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type, PoiType poi_type = UNKOWN_TYPE);

    ERESULT SetSlamModel(const int slam_model);

    ERESULT SetMaxNaviVel(const double linear_max, const double angular_max);

    double GetNaviPosesMileage(const Pose &start_pose, const Pose &target_pose);

    // Event Interface
    ERESULT NotifyDropEvent(DropType type);

    ERESULT NotifyCollisionEvent(CollisionType type);

    ERESULT NotifyEStopEvent(EmergencyType type);

    ERESULT NotifyChargingEvent(const BatteryState &state);

    ERESULT NotifyCriticalHwErrEvent(HWErrorType type);

    // Teleoping
    ERESULT StartHA(const std::string &task_id);

    ERESULT StopHA(const std::string &task_id);

    ERESULT SetDefaultCharger(const std::string &map_id,
                              const std::string &map_name,
                              const std::string &charger_name,
                              const PixelPose &pose);

    ERESULT SetNaviHalfPoints(const std::vector<PixelPose> &points);

    ERESULT ReportNaviPath(const std::string &task_id);

    ERESULT ChargingToDock(const std::string &task_id, const bool cmd);

    ERESULT InDocking(const std::string &task_id);

    ERESULT UnDocking(const std::string &task_id);

    void RobotAlarmHandle(bool alarm_state);

    bool IsLocalError();

    bool IsChargingLine();

public:
    // IRobotObserver
    ERESULT OnInitDone(bool success);

    ERESULT OnUpdateNaviState(const NaviStateInfo &state);

    ERESULT OnUpdateNaviPath(const NaviPathInfo &path);

    ERESULT OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id);

    // ERESULT OnNaviDone(const std::string &task_id, ENAVITYPE type, NAVISTATE state, const std::string &description);

    ERESULT OnNaviState(const std::string &task_id, ENAVITYPE type, NAVISTATE state);

    // ERESULT OnNaviProgress(const std::string &task_id,
    //                        ENAVITYPE type,
    //                        NAVISTATE state,
    //                        const std::string &description,
    //                        const Pose &cur_pose);

    // ERESULT OnNaviCancelled(const std::string &task_id, ENAVITYPE type, NAVISTATE state);

    // ERESULT OnNaviPause(const std::string &task_id, ENAVITYPE type, NAVISTATE state);

private:
    RobotBt();


    void StartNextNavi(const std::string &task_id,
                       ENAVITYPE type,
                       const std::string &goal_name,
                       const PixelPose &goal_pose,
                       const float &goal_tolerance);

public:
    IEngineCtrl *engineCtrl_;
    IRobotEngineObserver *robotEngineObserver_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr blackboard_;
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;
    std::shared_ptr<BTNodeImpl> btNodeImpl_;
    std::shared_ptr<LocalManager> localManager_;
    std::shared_ptr<TaskManager> taskManager_;
#ifdef DEBUG
    // This logger prints state changes on console
    BT::StdCoutLogger *logger_cout_point_ = NULL;

    // This logger saves state changes on file
    BT::FileLogger *logger_file_point_ = NULL;

    // This logger stores the execution time of each node
    BT::MinitraceLogger *logger_minitrace_point_ = NULL;

    // This logger publish status changes using ZeroMQ. Used by Groot
    BT::PublisherZMQ *publisher_zmq_point_ = NULL;
#endif
    std::mutex mutex_;
    CTimer *heartbeat_timer_;
    CTimer *navi_done_timer_;

    // important property
    std::vector<EngineState> combo_state_vector;


//#define CHECK_TASK_ID_AND_RETURN_REPEAT_IF_EXIST(task_id) \
    if(curr_task_id_ == task_id) {\
        blackboard_->set<ERESULT>("robot_event_response", E_REPEAT); \
        return NodeStatus::SUCCESS;  \
    }
#define SAVE_CURRENT_AND_LAST_TASK_ID(name, task_id)                    \
    {                                                                   \
        last_task_id_ = curr_task_id_;                                  \
        curr_task_id_ = task_id;                                        \
        ROS_INFO("Robot added %s, task_id(%s)", name, task_id.c_str()); \
    }

#define CLEAR_CURR_TASK_ID()   \
    {                          \
        curr_task_id_.clear(); \
    }

#define CLEAR_LAST_TASK_ID()   \
    {                          \
        last_task_id_.clear(); \
    }

#define LOGGING_FROM_STATE_VECTOR                                                                  \
    {                                                                                              \
        if (!from_state_vector_.empty())                                                           \
        {                                                                                          \
            ROS_INFO("%s, from_state_vector_.size(%d)", __func__, (int)from_state_vector_.size()); \
        }                                                                                          \
    }
    //    for(auto item:from_state_vector_) \
                ROS_INFO("%s, from_state_vector(%d)", __func__, item); \
        }  \
    }
};

}  // namespace naviengine
#endif
