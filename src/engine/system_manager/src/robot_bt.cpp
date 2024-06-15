
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "system_manager/bt/bt_define.h"
#include "system_manager/bt/robot_bt.h"
#include "system_manager/bt/simple_node.h"
#include "system_manager/bt/start_navi_async_node.h"
#include "system_manager/bt/state_combo_node.h"
#include "system_manager/bt/state_write_off_node.h"
#include "system_manager/robot_ctrl.h"
#include "system_manager/robot_engine_observer.h"
#include "system_manager/robot_observer.h"


#define DEBUG
#define SLAM_STACK_TIMEOUT 5
#define NAVI_DONE_TIMEOUT 3600
using namespace BT;

namespace naviengine
{


RobotBt::RobotBt()
{
    ROS_INFO("RobotBt entry");
    heartbeat_timer_ =
        new CTimer(SLAM_STACK_TIMEOUT, [&]() { ROS_INFO("%s, heartbeat timeout, go wrong here", __func__); });

    navi_done_timer_ =
        new CTimer(NAVI_DONE_TIMEOUT, [&]() { ROS_INFO("%s, navi done timeout, go wrong here", __func__); });

    ROS_INFO("RobotBt exit");
}

RobotBt::~RobotBt()
{
    ROS_INFO("~RobotBt entry");
    if (heartbeat_timer_)
    {
        heartbeat_timer_->stop();
        delete heartbeat_timer_;
        heartbeat_timer_ = NULL;
    }
    if (navi_done_timer_)
    {
        navi_done_timer_->stop();
        delete navi_done_timer_;
        navi_done_timer_ = NULL;
    }

#ifdef DEBUG
    // Delete console logger
    if (logger_cout_point_)
    {
        delete logger_cout_point_;
        logger_cout_point_ = NULL;
    }

#ifdef groot
    // Delete file logger
    if (logger_file_point_)
    {
        delete logger_file_point_;
        logger_file_point_ = NULL;
    }

    // Delete execution logger
    if (logger_minitrace_point_)
    {
        delete logger_minitrace_point_;
        logger_minitrace_point_ = NULL;
    }

    // Delete zmq publisher
    if (publisher_zmq_point_)
    {
        delete publisher_zmq_point_;
        publisher_zmq_point_ = NULL;
    }
#endif
#endif

    ROS_INFO("~RobotBt exit");
}

ERESULT RobotBt::Init(IEngineCtrl &engineCtrl,
                      IRobotEngineObserver &robotEngineObserver,
                      rclcpp::executors::MultiThreadedExecutor &exector,
                      const std::string &bt_xml_filename)
{
    ROS_INFO("RobotBt init entry");
    engineCtrl_ = &engineCtrl;
    robotEngineObserver_ = &robotEngineObserver;
    // robotCtrl_->SetObserver(*this);
    localManager_ = std::make_shared<LocalManager>();
    taskManager_ = std::make_shared<TaskManager>();
    btNodeImpl_ = std::make_shared<BTNodeImpl>(localManager_, taskManager_);
    exector.add_node(btNodeImpl_);
    exector.add_node(localManager_);
    exector.add_node(taskManager_);

    // BehaviorTreeFactory factory;

    BT::PortsList commonPortList = {BT::InputPort<ERESULT>(BB_INPUT_FAILURE_CODE),
                                    BT::InputPort<ERESULT>(BB_INPUT_SUCCESS_CODE),
                                    BT::InputPort<std::string>(BB_INPUT_ROBOT_STATE),
                                    BT::OutputPort<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE)};

    // Conditions
    factory_.registerSimpleCondition("IsProcessingState", IsProcessingState, commonPortList);
    factory_.registerSimpleCondition("IsProcessedState", IsProcessedState, commonPortList);
    factory_.registerSimpleCondition("IsCompletedTask", IsCompletedTask);
    factory_.registerSimpleCondition("IsInitialized", IsInitialized, commonPortList);
    factory_.registerSimpleCondition("IsConflictTask", IsConflictTask, commonPortList);
    // factory_.registerSimpleCondition("IsChargingLine", IsChargingLine, commonPortList);
    factory_.registerSimpleCondition("IsHalfPointsSuccess", IsHalfPointsSuccess);
    factory_.registerSimpleCondition("IsTmpStopHalfPointSuccess", IsTmpStopHalfPointSuccess);

    // Actions
    factory_.registerSimpleAction("ActionInitRobot", ActionInitRobot);
    factory_.registerSimpleAction("ActionLoadMap", ActionLoadMap);
    factory_.registerSimpleAction("ActionSaveMap", ActionSaveMap);

    factory_.registerSimpleAction("ActionStartBuildMap", ActionStartBuildMap);
    factory_.registerSimpleAction("ActionPauseBuildMap", ActionPauseBuildMap);
    factory_.registerSimpleAction("ActionResumeBuildMap", ActionResumeBuildMap);
    factory_.registerSimpleAction("ActionStopBuildMap", ActionStopBuildMap);

    factory_.registerSimpleAction("ActionStartNavi", ActionStartNavi);
    factory_.registerSimpleAction("ActionStartNaviThroughPoses", ActionStartNaviThroughPoses);
    factory_.registerSimpleAction("ActionSetNaviHalfPoints", ActionSetNaviHalfPoints);
    factory_.registerSimpleAction("ActionPauseNavi", ActionPauseNavi);
    factory_.registerSimpleAction("ActionResumeNavi", ActionResumeNavi);
    factory_.registerSimpleAction("ActionCancelNavi", ActionCancelNavi);
    factory_.registerSimpleAction("ActionCancelNaviMarked", ActionCancelNaviMarked);
    factory_.registerSimpleAction("ActionStopNavi", ActionStopNavi);

    factory_.registerSimpleAction("ActionOnNaviDone", ActionOnNaviDone);
    factory_.registerSimpleAction("ActionOnNaviDoneRepeat", ActionOnNaviDoneRepeat);

    // Action node
    factory_.registerNodeType<StateComboNode>("ActionStateCombo");
    factory_.registerNodeType<StateWriteOffNode>("ActionStateWriteOff");
    factory_.registerNodeType<StartNaviAsyncNode>("ActionStartNaviAsync");


    factory_.registerNodeType<SwitchNode<19>>("SwitchEvent");

    default_bt_xml_filename_ = bt_xml_filename;
    tree_ = factory_.createTreeFromFile(bt_xml_filename);
    blackboard_ = tree_.rootBlackboard();
    ROS_INFO("RobotBt createTreeFromFile blackboard_");
#ifdef DEBUG
    // This logger prints state changes on console
    // logger_cout_point_ = new StdCoutLogger(tree_);

//#define groot
#ifdef groot
    // This logger saves state changes on file
    logger_file_point_ = new FileLogger(tree_, "/data/user/ginger/robot_slamengine_ws/Tmp/bt_trace.fbl");

    // This logger stores the execution time of each node
    logger_minitrace_point_ = new MinitraceLogger(tree_, "/data/user/ginger/robot_slamengine_ws/Tmp/bt_trace.json");

    // This logger publish status changes using ZeroMQ. Used by Groot
    publisher_zmq_point_ = new PublisherZMQ(tree_);
#endif
#endif
    printTreeRecursively(tree_.rootNode());

    // EngineState engineState(STATE_IDLE, PRIORITY_IDLE);
    // combo_state_vector.push_back(engineState);

    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_INIT_ROBOT);
    tree_.tickRoot();

    ROS_INFO("RobotBt init exit");
    return E_OK;
}

bool RobotBt::loadBehaviorTree(const std::string &bt_xml_filename)
{
    // Empty filename is default for backward compatibility
    auto filename = bt_xml_filename.empty() ? default_bt_xml_filename_ : bt_xml_filename;

    // Use previous BT if it is the existing one
    if (current_bt_xml_filename_ == filename)
    {
        ROS_INFO("BT will not be reloaded as the given xml is already loaded");
        return true;
    }

    tree_ = factory_.createTreeFromFile(bt_xml_filename);
    blackboard_ = tree_.rootBlackboard();
    ROS_INFO("RobotBt createTreeFromFile blackboard_");

    current_bt_xml_filename_ = filename;
    return true;
}

ERESULT RobotBt::StartBuildMap(const std::string &task_id, EMAPPINGTYPE mapping_type, EMAPPINGMODE mapping_mode)
{
    ROS_INFO("RobotBt::startBuildMap entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_START_BUILD_MAP);

    // StartNaviCtx ctx;
    // ctx.task_id_ = task_id;
    StartMappingCtx ctx(task_id, mapping_type, mapping_mode);

    blackboard_->set<StartMappingCtx>(BB_ROBOT_START_MAPPING_CTX, ctx);
    ROS_INFO("RobotBt::startBuildMap task_id: %s, mapping_type: %d, mapping_mode: %d",
             ctx.task_id_.c_str(),
             ctx.mapping_type_,
             ctx.mapping_mode_);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::startBuildMap exit, res:%d", res);
    return res;
}

ERESULT RobotBt::StopBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotBt::StopBuildMap entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_STOP_BUILD_MAP);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::StopBuildMap exit, res:%d", res);
    return res;
}

ERESULT RobotBt::PauseBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotBt::PauseBuildMap entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_PAUSE_BUILD_MAP);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::PauseBuildMap exit");
    return res;
}

ERESULT RobotBt::ResumeBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotBt::ResumeBuildMap entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_RESUME_BUILD_MAP);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::ResumeBuildMap exit");
    return res;
}

ERESULT RobotBt::SaveMap(const std::string &task_id, const std::string &map_name)
{
    ROS_INFO("RobotBt::SaveMap entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_SAVE_MAP);
    SaveMapCtx mapctx(task_id, map_name);
    blackboard_->set<SaveMapCtx>(BB_ROBOT_SAVE_MAP_CTX, mapctx);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::SaveMap exit");
    return res;
}

ERESULT RobotBt::LoadMap(const std::string &task_id, const std::string &map_name)
{
    ROS_INFO("RobotBt::LoadMap entry");

    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_LOAD_MAP);
    LoadMapCtx mapctx(task_id, map_name);
    blackboard_->set<LoadMapCtx>(BB_ROBOT_LOAD_MAP_CTX, mapctx);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::LoadMap exit, result:%d", res);
    return res;
}

ERESULT RobotBt::StartNaviToPose(const Header &header,
                                 const std::string &task_id,
                                 const std::string &lift_id,
                                 ENAVITYPE type,
                                 const Pose &goal_pose,
                                 const Twist &twist,
                                 const std::string &bt_xml_filename,
                                 const bool align_angle)
{
    ROS_INFO("RobotBt::StartNavi entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_START_NAVI_TO_POSE);
    HeaderStartNaviCtx ctx(header,
                           task_id,
                           type,
                           goal_pose,
                           goal_pose,
                           "",
                           "",
                           twist,
                           std::vector<Pose>(),
                           std::vector<int>(),
                           bt_xml_filename,
                           false,
                           align_angle,
                           lift_id);
    blackboard_->set<HeaderStartNaviCtx>(BB_ROBOT_START_NAVI_CTX, ctx);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::StartNavi exit, result:%d", res);
    return res;
}

ERESULT RobotBt::StartNaviThroughPoses(const Header &header,
                                       const std::string &task_id,
                                       ENAVITYPE type,
                                       const std::vector<Pose> &points,
                                       const std::vector<int> &points_type,
                                       const Twist &twist,
                                       const std::string &bt_xml_filename,
                                       const bool align_angle)
{
    ROS_INFO("RobotBt::StartNavi entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_START_NAVI_THROUGH_POSES);
    Pose goal_pose;
    HeaderStartNaviCtx ctx(
        header, task_id, type, goal_pose, goal_pose, "", "", twist, points, points_type, bt_xml_filename, false, align_angle);
    blackboard_->set<HeaderStartNaviCtx>(BB_ROBOT_START_NAVI_CTX, ctx);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::StartNavi exit, result:%d", res);
    return res;
}

ERESULT RobotBt::StopNavi(const std::string &task_id)
{
    ROS_INFO("RobotBt::StopNavi entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_STOP_NAVI);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);

    ROS_INFO("RobotBt::StopNavi exit, result: %d", res);
    return res;
}

ERESULT RobotBt::PauseNavi(const std::string &task_id)
{
    ROS_INFO("RobotBt::PauseNavi entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_PAUSE_NAVI);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::PauseNavi exit, result: %d", res);
    return res;
}

ERESULT RobotBt::ResumeNavi(const std::string &task_id)
{
    ROS_INFO("RobotBt::ResumeNavi entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_RESUME_NAVI);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::ResumeNavi exit, result: %d", res);
    return res;
}

ERESULT RobotBt::StartHA(const std::string &task_id)
{
    ROS_INFO("RobotBt::StartHA entry, task_id(%s)", task_id.c_str());
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_START_HA);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::StartHA exit, res:%d", res);
    return res;
}

ERESULT RobotBt::StopHA(const std::string &task_id)
{
    ROS_INFO("RobotBt::StopHA entry, task_id(%s)", task_id.c_str());
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_STOP_HA);
    blackboard_->set<TaskCtx>(BB_ROBOT_TASK_CTX, TaskCtx(task_id));
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::StopHA exit, res:%d", res);
    return res;
}

ERESULT RobotBt::SetDefaultCharger(const std::string &map_id,
                                   const std::string &map_name,
                                   const std::string &charger_name,
                                   const PixelPose &pose)
{
    ROS_INFO("RobotBt::SetDefaultCharger entry");
    ERESULT res = E_OK;
    // ERESULT res = robotCtrl_->SetDefaultCharger(map_id, map_name, charger_name, pose);
    ROS_INFO("RobotBt::SetDefaultCharger exit res:%d", res);
    return res;
}

ERESULT RobotBt::SetNaviHalfPoints(const std::vector<PixelPose> &points)
{
    ROS_INFO("RobotBt::SetNaviHalfPoints entry");


    // std::vector<PixelPose> valid_points;
    // for (auto &tmp_navi_point: points)
    // {
    //     if (naviengine::RobotBt::GetInstance()->robotCtrl_->IsGoalValid(tmp_navi_point) != E_OK)
    //     {
    //         ROS_WARN("This tmp goal point {%f, %f, %f} is invalid, pass!", tmp_navi_point.x, tmp_navi_point.y,
    //         tmp_navi_point.theta);
    //         // return E_STARTNAVI_PLAN_FAILED;
    //     }
    //     else {
    //         valid_points.push_back(tmp_navi_point);
    //     }
    // }
    // ROS_INFO("valid_points: %zu", valid_points.size());
    // if (valid_points.empty()) return E_STARTNAVI_PLAN_FAILED;
    // for (const auto &valid_point: valid_points) {
    //     ROS_INFO("===> {%f, %f, %f}", valid_point.x, valid_point.y, valid_point.theta);
    // }

    // std::unique_lock<std::mutex> lock(mutex_);
    // blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_SET_NAVI_HALF_POINTS);
    // blackboard_->set<std::vector<PixelPose>>(BB_ROBOT_HALF_POINTS_VECTOR, valid_points);

    // tree_.tickRoot();

    // ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);

    // ROS_INFO("RobotBt::SetNaviHalfPoints exit res: %d", res);
    return E_OK;
}

ERESULT RobotBt::ReportNaviPath(const std::string &task_id)
{
    ROS_INFO("RobotBt::PublishNaviPath entry");
    ERESULT res = E_OK;
    // ERESULT res = robotCtrl_->ReportNaviPath(task_id);

    ROS_INFO("RobotBt::PublishNaviPath exit res: %d", res);
    return res;
}

ERESULT RobotBt::InDocking(const std::string &task_id)
{
    ROS_INFO("RobotBt::InDocking entry");
    ERESULT res = E_OK;
    //进行为树
    // ERESULT res = robotCtrl_->ReportNaviPath(task_id);

    ROS_INFO("RobotBt::InDocking exit res: %d", res);
    return res;
}

ERESULT RobotBt::UnDocking(const std::string &task_id)
{
    ROS_INFO("RobotBt::UnDocking entry");
    ERESULT res = E_OK;
    //进行为树
    // ERESULT res = robotCtrl_->ReportNaviPath(task_id);

    ROS_INFO("RobotBt::UnDocking exit res: %d", res);
    return res;
}



std::string RobotBt::GetState()
{
    ROS_INFO("RobotBt::GetState entry");
    auto robot_state = RobotBt::GetInstance()->combo_state_vector.begin()->state_name_;
    ROS_INFO("RobotBt::GetState exit, robot_state: %s", robot_state.c_str());
    return robot_state;
}

bool RobotBt::GetTaskInfo(std::string &task_id, ENAVITYPE &navi_type)
{
    ROS_INFO("RobotBt::GetTaskInfo entry");

    task_id = "";
    navi_type = ENAVITYPE::NAVI_NORMAL;

    for (std::vector<EngineState>::iterator iterator = RobotBt::GetInstance()->combo_state_vector.begin();
         iterator != RobotBt::GetInstance()->combo_state_vector.end();
         iterator++)
    {
        if (iterator->state_name_.compare(STATE_MOTIONING) == 0)
        {
            task_id = iterator->taskCtx_.task_id_;
            navi_type = iterator->taskCtx_.type_;

            break;
        }
    }

    ROS_INFO("RobotBt::GetTaskInfo exit, task_id: %s, navi_type: %d", task_id.c_str(), (int)navi_type);
    return true;
}

ERESULT RobotBt::CheckPose(std::string name, PixelPose &pixel_pose)
{
    ROS_INFO("RobotBt::CheckPose entry");
    ERESULT res = E_OK;
    // ERESULT res = robotCtrl_->CheckPose(name, pixel_pose);
    ROS_INFO("RobotBt::CheckPose exit");
    return res;
}

ERESULT RobotBt::SetPoseinitialize(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type, PoiType poi_type)
{
    ROS_INFO("RobotBt::SetPixelPose entry");
    // ERESULT res = robotCtrl_->SetPixelPose(pixel_pose);
    ERESULT res = btNodeImpl_->setWorldPose(world_pose, init_type, match_type, poi_type);
    ROS_INFO("RobotBt::SetPixelPose exit, result:%d", res);
    return res;
}


ERESULT RobotBt::SetMaxNaviVel(const double linear_max, const double angular_max)
{
    ROS_INFO("RobotBt::SetMaxNaviVel entry");
    ERESULT res = btNodeImpl_->setMaxNaviVel(linear_max, angular_max);
    ROS_INFO("RobotBt::SetMaxNaviVel exit");
    return res;
}

double RobotBt::GetNaviPosesMileage(const Pose &start_pose, const Pose &target_pose)
{
    ROS_INFO("RobotBt::GetNaviPosesMileage entry");
    double res = btNodeImpl_->getNaviPosesMileage(start_pose, target_pose);
    ROS_INFO("RobotBt::GetNaviPosesMileage exit");
    return res;
}

ERESULT RobotBt::SetSlamModel(const int slam_model)
{
    ROS_INFO("RobotBt::SetSlamModel entry");
    ERESULT res = btNodeImpl_->setSlamModel(slam_model);
    ROS_INFO("RobotBt::SetSlamModel exit");
    return res;
}

ERESULT RobotBt::NotifyDropEvent(DropType type)
{
    ROS_INFO("RobotBt::NotifyDropEvent entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_NOTIFY_DROP);
    blackboard_->set<DropType>("robot_drop_type", type);
    tree_.tickRoot();
    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::NotifyDropEvent exit");
    return res;
}

ERESULT RobotBt::NotifyCollisionEvent(CollisionType type)
{
    ROS_INFO("RobotBt::NotifyCollisionEvent entry, type(%d)", (int)type);
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_NOTIFY_COLLISION);
    blackboard_->set<CollisionType>(BB_ROBOT_COLLISION_TYPE, type);
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::NotifyCollisionEvent exit, res=%d", res);
    return res;
}

ERESULT RobotBt::NotifyEStopEvent(EmergencyType type)
{
    ROS_INFO("RobotBt::NotifyStopEvent entry");
    std::unique_lock<std::mutex> lock(mutex_);
    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_NOTIFY_ESTOP);
    blackboard_->set<EmergencyType>(BB_ROBOT_EMERGENCY_TYPE, type);
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::NotifyStopEvent exit, res=%d", res);
    return res;
}

ERESULT RobotBt::NotifyChargingEvent(const BatteryState &state)
{
    ROS_INFO("RobotBt::NotifyChargingEvent entry");
    ROS_INFO("RobotBt::NotifyChargingEvent state.chargeset_type ==> %d", state.chargeset_type);
    std::unique_lock<std::mutex> lock(mutex_);

    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_NOTIFY_CHARGING);
    blackboard_->set<int>(BB_ROBOT_CHARGING_TYPE, state.chargeset_type);
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);

    ROS_INFO("RobotBt::NotifyChargingEvent exit, res=%d", res);
    return res;
}

ERESULT RobotBt::NotifyCriticalHwErrEvent(HWErrorType type)
{
    ROS_INFO("RobotBt::NotifyCriticalHwErrEvent entry, type(%d)", (int)type);
    std::unique_lock<std::mutex> lock(mutex_);

    blackboard_->set<std::string>(BB_ROBOT_EVENT, EVENT_NOTIFY_CRITICAL_HW_ERR);
    blackboard_->set<HWErrorType>(BB_ROBOT_HW_ERROR_TYPE, type);
    tree_.tickRoot();

    ERESULT res = blackboard_->get<ERESULT>(BB_ROBOT_EVENT_RESPONSE);
    ROS_INFO("RobotBt::NotifyCriticalHwErrEvent exit, res=%d", res);
    return res;
}

void RobotBt::RobotAlarmHandle(bool alarm_state)
{
    ROS_INFO("RobotBt::RobotAlarmHandle entry, alarm_state(%d)", (int)alarm_state);
    std::unique_lock<std::mutex> lock(mutex_);
    btNodeImpl_->robotAlarmHandle(alarm_state);
    ROS_INFO("RobotBt::RobotAlarmHandle exit");
}

ERESULT RobotBt::OnInitDone(bool success)
{
    ROS_INFO("RobotBt::onInitDone entry, success(%d)", (int)success);

    if (success)
    {
        blackboard_->set<std::string>(BB_ROBOT_INITIALIZED, "OK");
    }

    ROS_INFO("RobotBt::onInitDone exit");

    return E_OK;
}

ERESULT
RobotBt::OnUpdateNaviPath(const NaviPathInfo &path)
{
    ROS_INFO("RobotBt::OnUpdateNaviPath entry");
    ERESULT res;

    res = naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnUpdateNaviPath(path);

    ROS_INFO("RobotBt::OnUpdateNaviPath exit");

    return res;
}

ERESULT
RobotBt::OnUpdateRetentionStatus(const bool &is_tapped, const std::string& task_id)
{
    ROS_INFO("RobotBt::OnUpdateRetentionStatus entry");
    ERESULT res;

    res = naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnUpdateRetentionStatus(is_tapped, task_id);

    ROS_INFO("RobotBt::OnUpdateRetentionStatus exit");

    return res;
}

ERESULT
RobotBt::OnUpdateNaviState(const NaviStateInfo &state)
{
    ROS_INFO("RobotBt::OnUpdateNaviState entry");
    ERESULT res;
    if (state.navi_state == NAVISTATE::NAVI_STATE_SUCCESS || state.navi_state == NAVISTATE::NAVI_STATE_FAILED ||
        state.navi_state == NAVISTATE::NAVI_STATE_CANCELED)
    {
        naviengine::RobotBt::GetInstance()->engineCtrl_->OnUpdateFsmState(state.navi_state, state.task_id);
    }

    res = naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnUpdateNaviState(state);

    ROS_INFO("RobotBt::OnUpdateNaviState exit");

    return res;
}

ERESULT
RobotBt::OnNaviState(const std::string &task_id, ENAVITYPE type, NAVISTATE state)
{
    ROS_INFO("RobotBt::OnNaviState entry");

    ERESULT res = naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnNaviState(task_id, type, state);

    ROS_INFO("RobotBt::OnNaviState exit");

    return res;
}

bool RobotBt::IsLocalError()
{
    return localManager_->isLocalError();
}

bool RobotBt::IsChargingLine()
{
    return btNodeImpl_->IsChargingLine();
}


}  // namespace naviengine
