
#include <stdexcept>

#include "system_manager/bt/robot_bt.h"
#include "system_manager/robot_naviengine.h"



namespace naviengine
{

RobotNaviEngine::RobotNaviEngine()
{
    robotBt_ = RobotBt::GetInstance();
    naviEngineNode_ = std::make_shared<NaviEngineNode>(robotBt_);

    fsm = std::make_shared<Fsm>();
    fsm->Init();
}

RobotNaviEngine::~RobotNaviEngine()
{
    ROS_INFO("~RobotNaviEngine() entry");
    delete robotBt_;
    ROS_INFO("~RobotNaviEngine() exit");
}


ERESULT RobotNaviEngine::Init(IRobotEngineObserver &robotEngineObserver,
                              rclcpp::executors::MultiThreadedExecutor &exector)
{
    ROS_INFO("RobotNaviEngine::Init entry");
    ERESULT res = robotBt_->Init(*this, robotEngineObserver, exector, fsm->DefaultXml());
    exector.add_node(naviEngineNode_);
    fsm->StateTransform(EVENT_INIT_SUCCESS);
    ROS_INFO("RobotNaviEngine::Init exit");
    return res;
}



ERESULT RobotNaviEngine::StartBuildMap(const std::string &task_id, EMAPPINGTYPE mapping_type, EMAPPINGMODE mapping_mode)
{
    ROS_INFO("RobotNaviEngine::StartBuildMap entry");
    //不进行为树直接调用建图接口
    ERESULT res = robotBt_->StartBuildMap(task_id, mapping_type, mapping_mode);
    ROS_INFO("RobotNaviEngine::StartBuildMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::StopBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::StopBuildMap entry");
    ERESULT res = robotBt_->StopBuildMap(task_id);
    ROS_INFO("RobotNaviEngine::StopBuildMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::PauseBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::PauseBuildMap entry");
    ERESULT res = robotBt_->PauseBuildMap(task_id);
    ROS_INFO("RobotNaviEngine::PauseBuildMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::ResumeBuildMap(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::ResumeBuildMap entry");
    ERESULT res = robotBt_->ResumeBuildMap(task_id);
    ROS_INFO("RobotNaviEngine::ResumeBuildMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SaveMap(const std::string &task_id, const std::string &map_name)
{
    ROS_INFO("RobotNaviEngine::SaveMap entry");
    ERESULT res;
    //不进行为树直接调用建图接口
    // ERESULT res = robotBt_->SaveMap(task_id, map_name);
    ROS_INFO("RobotNaviEngine::SaveMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::UpdateMap(const std::string &task_id, const std::string &map_name)
{
    ROS_INFO("RobotNaviEngine::UpdateMap entry");
    ERESULT res;
    //不进行为树直接调用定位接口

    ROS_INFO("RobotNaviEngine::UpdateMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::LoadMap(const std::string &task_id, const std::string &map_name)
{
    ROS_INFO("RobotNaviEngine::LoadMap entry");
    ERESULT res;
    if (fsm->IdleState())
    {
        robotBt_->loadBehaviorTree(fsm->IdleStateXml());
        res = robotBt_->LoadMap(task_id, map_name);
    }
    else if (fsm->DockingState())
    {
        robotBt_->loadBehaviorTree(fsm->DockingStateXml());
        res = robotBt_->LoadMap(task_id, map_name);
    }
    else
    {
        res = E_LOADMAP_MODE_ERROR;
    }


    ROS_INFO("RobotNaviEngine::LoadMap exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::StartNaviToPose(const Header &header,
                                         const std::string &task_id,
                                         const std::string &lift_id,
                                         ENAVITYPE type,
                                         const Pose &pose,
                                         const Twist &twist,
                                         const std::string &bt_xml_filename,
                                         const bool align_angle)
{
    ROS_INFO("RobotNaviEngine::StartNaviToPose entry");
    ERESULT res;

    res = StartNaviToPoseIfTransformState();
    if (res != E_OK)
    {
        return res;
    }

    if (naviEngineNode_->IsCompletedTask(task_id))
    {
        res = robotBt_->robotEngineObserver_->OnNaviState(task_id, type, NAVISTATE::NAVI_STATE_SUCCESS);
        ROS_INFO("RobotNaviEngine::StartNaviToPose tast complete res:%d", res);
        return res;
    }

    if (fsm->StateTransform(EVENT_START_TASK))
    {
        robotBt_->loadBehaviorTree(fsm->WorkingStateXml());
        res = robotBt_->StartNaviToPose(header, task_id, lift_id, type, pose, twist, SetNaviXml(bt_xml_filename), align_angle);
    }
    else
    {
        res = E_STARTNAVI_MODE_ERROR;
    }
    ROS_INFO("RobotNaviEngine::StartNaviToPose exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::StartNaviThroughPoses(const Header &header,
                                               const std::string &task_id,
                                               ENAVITYPE type,
                                               const std::vector<Pose> &points,
                                               const std::vector<int> &points_type,
                                               const Twist &twist,
                                               const std::string &bt_xml_filename,
                                               const bool align_angle)
{
    ROS_INFO("RobotNaviEngine::StartNaviThroughPoses entry");
    ERESULT res;

    res = StartNaviIfTransformState();
    if (res != E_OK)
    {
        return res;
    }

    if (fsm->StateTransform(EVENT_START_TASK))
    {
        robotBt_->loadBehaviorTree(fsm->WorkingStateXml());
        res = robotBt_->StartNaviThroughPoses(
            header, task_id, type, points, points_type, twist, SetNaviXml(bt_xml_filename), align_angle);
    }
    else
    {
        res = E_STARTNAVI_MODE_ERROR;
    }
    ROS_INFO("RobotNaviEngine::StartNaviThroughPoses exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::StartNaviToPoseIfTransformState()
{
    ERESULT res = E_OK;

    res = StartNaviIfTransformState();
    if (res != E_OK)
    {
        return res;
    }

    if (fsm->WorkingState())
    {
        res = E_STARTNAVI_TASK_CONFLICT;
        ROS_INFO("RobotNaviEngine::StartNaviToPoseTransformState tast conflict res:%d", res);
        return res;
    }
    return res;
}

ERESULT RobotNaviEngine::StartNaviIfTransformState()
{
    ERESULT res = E_OK;

    if(naviEngineNode_->IsAlarmState())
    {
        res = E_STARTNAVI_ROBOT_BLOCKED;
        ROS_INFO("RobotNaviEngine::StartNaviTransformState IsAlarmState res:%d", res);
        return res;
    }

    if (robotBt_->IsLocalError())
    {
        res = E_STARTNAVI_LOCAL_ERROR;
        ROS_INFO("RobotNaviEngine::StartNaviTransformState IsLocalError res:%d", res);
        return res;
    }

    if (robotBt_->IsChargingLine())
    {
        res = E_STARTNAVI_LINE_CHARGING;
        ROS_INFO("RobotNaviEngine::StartNaviTransformState IsChargingLine res:%d", res);
        return res;
    }
    return res;
}

ERESULT RobotNaviEngine::StopNavi(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::StopNavi entry");
    ERESULT res = robotBt_->StopNavi(task_id);
    ROS_INFO("RobotNaviEngine::StopNavi exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::PauseNavi(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::PauseNavi entry");
    if(naviEngineNode_->IsAlarmState())
    {
        ROS_INFO("RobotNaviEngine::PauseNavi IsAlarmState");
        return E_STARTNAVI_MODE_ERROR;
    }

    ERESULT res = robotBt_->PauseNavi(task_id);
    ROS_INFO("RobotNaviEngine::PauseNavi exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::ResumeNavi(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::ResumeNavi entry");
    if(naviEngineNode_->IsAlarmState())
    {
        ROS_INFO("RobotNaviEngine::ResumeNavi IsAlarmState");
        return E_STARTNAVI_MODE_ERROR;
    }

    ERESULT res = robotBt_->ResumeNavi(task_id);
    ROS_INFO("RobotNaviEngine::ResumeNavi exit res:%d", res);
    return res;
}

// State Interface
SPSSTATE RobotNaviEngine::GetState()
{
    // ROS_INFO("RobotNaviEngine::GetState entry");
    SPSSTATE robot_state = fsm->GetState();
    // ROS_INFO("RobotNaviEngine::GetState exit SPSSTATE: %d", robot_state);
    return robot_state;
}

ERESULT RobotNaviEngine::CheckPose(std::string name, PixelPose &pixel_pose)
{
    ROS_INFO("RobotNaviEngine::CheckPose entry");
    return robotBt_->CheckPose(name, pixel_pose);
}


ERESULT RobotNaviEngine::SetPoseinitialize(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type, PoiType poi_type)
{
    ROS_INFO("RobotNaviEngine::SetPoseinitialize entry");
    ERESULT res = robotBt_->SetPoseinitialize(world_pose, match_type, init_type, poi_type);
    ROS_INFO("RobotNaviEngine::ResumeNavi exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SetLocalization(const Pose &world_pose, SENSORMATCHTYPE match_type, LOCALINITTYPE init_type)
{
    ROS_INFO("RobotNaviEngine::SetLocalization entry");
    ERESULT res;
    if (fsm->StateTransform(EVENT_START_TASK))
    {
        robotBt_->loadBehaviorTree(fsm->WorkingStateXml());
        res = robotBt_->SetPoseinitialize(world_pose, match_type, init_type);
        //转圈
        //转圈完成通知定位
    }
    else
    {
        res = E_SETPOSE_MODE_ERROR;
    }
    ROS_INFO("RobotNaviEngine::SetLocalization exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SetParamData(const std::string &name, const std::string &data)
{
    ROS_INFO("RobotNaviEngine::SetParamData entry");
    ERESULT res;
    //不进行为树
    return res;
}

ERESULT RobotNaviEngine::StartMove(const Twist &twist, const bool collision_check)
{
    ROS_INFO("RobotNaviEngine::StartMove entry");
    ERESULT res;
    res = naviEngineNode_->StartMove(twist, collision_check);
    ROS_INFO("RobotNaviEngine::StartMove exit res:%d", res);
    return res;
}


ERESULT RobotNaviEngine::StopMove()
{
    ROS_INFO("RobotNaviEngine::StopMove entry");
    ERESULT res;
    res = naviEngineNode_->StopMove();
    ROS_INFO("RobotNaviEngine::StopMove exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SwitchSensorData(SENSORTYPE sensor_type, const bool state)
{
    ROS_INFO("RobotNaviEngine::SetParamData entry");
    ERESULT res;
    //不进行为树
    return res;
}


ERESULT RobotNaviEngine::SwitchSensorEnable(SENSORTYPE sensor_type, const bool state)
{
    ROS_INFO("RobotNaviEngine::SetParamData entry");
    ERESULT res;
    //不进行为树
    return res;
}

ERESULT RobotNaviEngine::SetSlamModel(const int slam_model)
{
    ROS_INFO("RobotNaviEngine::SetSlamModel entry");
    ERESULT res = robotBt_->SetSlamModel(slam_model);
    return res;
}


ERESULT RobotNaviEngine::ChangePixelToWord(const PixelPose &pixel_pose, Pose &pose)
{
    ROS_INFO("RobotNaviEngine::ChangePixelToWord entry");
    ERESULT res;

    res = naviEngineNode_->ChangePixelToWord(pixel_pose, pose);
    ROS_INFO("RobotNaviEngine::ChangePixelToWord exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::QueryPointType(const PixelPose &pixel_pose, PoiType &poi_type)
{
    ROS_INFO("RobotNaviEngine::QueryPointType entry");
    ERESULT res;

    res = naviEngineNode_->QueryPointType(pixel_pose, poi_type);
    ROS_INFO("RobotNaviEngine::QueryPointType exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::QueryLiftPointType(const PixelPose &pixel_pose, PoiType &poi_type, std::string &lift_id)
{
    ROS_INFO("RobotNaviEngine::QueryLiftPointType entry");
    ERESULT res;

    res = naviEngineNode_->QueryLiftPointType(pixel_pose, poi_type, lift_id);
    ROS_INFO("RobotNaviEngine::QueryLiftPointType exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SetMaxNaviVel(const double linear_max, const double angular_max)
{
    ROS_INFO("RobotNaviEngine::SetMaxNaviVel entry");
    return robotBt_->SetMaxNaviVel(linear_max, angular_max);
}

double RobotNaviEngine::GetNaviPosesMileage(const Pose &start_pose, const Pose &target_pose)
{
    ROS_INFO("RobotNaviEngine::GetNaviPosesMileage entry");
    return robotBt_->GetNaviPosesMileage(start_pose, target_pose);
}


ERESULT RobotNaviEngine::StartHA(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::StartHA entry");
    ERESULT res = robotBt_->StartHA(task_id);
    ROS_INFO("RobotNaviEngine::StartHA exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::StopHA(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::StopHA entry");
    ERESULT res = robotBt_->StopHA(task_id);
    ROS_INFO("RobotNaviEngine::StopHA exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SetDefaultCharger(const std::string &map_id,
                                           const std::string &map_name,
                                           const std::string &charger_name,
                                           const PixelPose &pose)
{
    ROS_INFO("RobotNaviEngine::SetDefaultCharger entry");
    ERESULT res = robotBt_->SetDefaultCharger(map_id, map_name, charger_name, pose);
    ROS_INFO("RobotNaviEngine::SetDefaultCharger exit res:%d", res);
    return res;
}

ERESULT RobotNaviEngine::SetNaviHalfPoints(const std::vector<PixelPose> &points)
{
    ROS_INFO("RobotNaviEngine::SetNaviHalfPoints entry");
    ERESULT res = robotBt_->SetNaviHalfPoints(points);
    ROS_INFO("RobotNaviEngine::SetNaviHalfPoints exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::ReportNaviPath(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::PublishNaviPath entry");
    ERESULT res = robotBt_->ReportNaviPath(task_id);
    ROS_INFO("RobotNaviEngine::PublishNaviPath exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::ChargingToDock(const std::string &task_id, const bool cmd)
{
    ROS_INFO("RobotNaviEngine::ChargingToDock entry");
    ERESULT res;
    if (cmd)
    {
        res = InDocking(task_id);
    }
    else
    {
        res = UnDocking(task_id);
    }

    ROS_INFO("RobotNaviEngine::ChargingToDock exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::InDocking(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::InDocking entry");
    ERESULT res;

    if (fsm->StateTransform(EVENT_IN_DOCKING))
    {
        robotBt_->loadBehaviorTree(fsm->DockingStateXml());
        res = robotBt_->InDocking(task_id);
    }
    else
    {
        res = E_SETPOSE_MODE_ERROR;
    }


    ROS_INFO("RobotNaviEngine::InDocking exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::UnDocking(const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine::UnDocking entry");
    ERESULT res;

    if (fsm->DockingState())
    {
        res = robotBt_->UnDocking(task_id);
    }
    else
    {
        res = E_SETPOSE_MODE_ERROR;
    }


    ROS_INFO("RobotNaviEngine::UnDocking exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::NaviCmd(const std::string &task_id, const NAVICMD cmd)
{
    ROS_INFO("RobotNaviEngine::NaviCmd entry");
    ERESULT res;
    if (fsm->WorkingState())
    {
        res = NaviCmdInner(task_id, cmd);
    }
    else
    {
        res = E_SETPOSE_MODE_ERROR;
    }



    ROS_INFO("RobotNaviEngine::NaviCmd exit res: %d", res);
    return res;
}

ERESULT RobotNaviEngine::NaviCmdInner(const std::string &task_id, const NAVICMD cmd)
{
    ROS_INFO("RobotNaviEngine::NaviCmdInner entry");
    ERESULT res;

    if (cmd == CANCEL)
    {
        res = robotBt_->StopNavi(task_id);
    }
    else if (cmd == PAUSE)
    {
        res = robotBt_->PauseNavi(task_id);
    }
    else
    {
        res = robotBt_->ResumeNavi(task_id);
    }


    ROS_INFO("RobotNaviEngine::NaviCmdInner exit res: %d", res);
    return res;
}


ERESULT RobotNaviEngine::NotifyDropEvent(DropType type)
{
    ROS_INFO("RobotNaviEngine::NotifyDropEvent entry");
    robotBt_->NotifyDropEvent(type);
    return E_OK;
}

ERESULT RobotNaviEngine::NotifyCollisionEvent(CollisionType type)
{
    ROS_INFO("RobotNaviEngine::NotifyCollisionEvent entry");
    robotBt_->NotifyCollisionEvent(type);
    return E_OK;
}

ERESULT RobotNaviEngine::NotifyEStopEvent(EmergencyType type)
{
    ROS_INFO("RobotNaviEngine::NotifyStopEvent entry");
    robotBt_->NotifyEStopEvent(type);
    return E_OK;
}

ERESULT RobotNaviEngine::NotifyChargingEvent(const BatteryState &state)
{
    // return E_OK;
    // ROS_INFO("RobotNaviEngine::NotifyChargingEvent entry");
    static int8_t last_chargeset_type = -1;
    // ROS_INFO("RobotNaviEngine::NotifyChargingEvent ===> last_chargeset_type: %hhd, now chargeset_type: %hhd",
    // last_chargeset_type, state.chargeset_type);
    if (state.chargeset_type != last_chargeset_type)
    {
        robotBt_->NotifyChargingEvent(state);
    }
    last_chargeset_type = state.chargeset_type;
    return E_OK;
}

ERESULT RobotNaviEngine::NotifyCriticalHwErrEvent(HWErrorType type)
{
    ROS_INFO("RobotNaviEngine::NotifyCriticalHwErrEvent entry");
    robotBt_->NotifyCriticalHwErrEvent(type);
    return E_OK;
}

bool RobotNaviEngine::GetTaskInfo(std::string &task_id, ENAVITYPE &navi_type)
{
    ROS_INFO("RobotNaviEngine::GetTaskInfo entry");
    robotBt_->GetTaskInfo(task_id, navi_type);
    return true;
}

std::string RobotNaviEngine::SetNaviXml(const std::string &bt_xml_filename)
{
    return SPS_NAVI_BT_XML + bt_xml_filename;
}

ERESULT RobotNaviEngine::OnUpdateFsmState(NAVISTATE state, const std::string &task_id)
{
    ROS_INFO("RobotNaviEngine OnNaviDone entry");
    ERESULT res;
    switch (state)
    {
        case NAVISTATE::NAVI_STATE_SUCCESS:
            naviEngineNode_->SetNaviTaskId(task_id);
        case NAVISTATE::NAVI_STATE_CANCELED:

            if (fsm->StateTransform(EVENT_TASK_FINISHED))
            {
                robotBt_->loadBehaviorTree(fsm->IdleStateXml());
            }
            else
            {
                res = E_SETPOSE_MODE_ERROR;
            }
            break;
        case NAVISTATE::NAVI_STATE_FAILED:
            // if (fsm->StateTransform(EVENT_TASK_ERROR))
            if (fsm->StateTransform(EVENT_TASK_FINISHED))
            {
                // robotBt_->loadBehaviorTree(fsm->ErrorStateXml());
                robotBt_->loadBehaviorTree(fsm->IdleStateXml());
            }
            else
            {
                res = E_SETPOSE_MODE_ERROR;
            }
            break;


        default:
            break;
    }

    ROS_INFO("RobotNaviEngine OnNaviDone exit\n\n");
    return res;
}


}  // namespace naviengine
