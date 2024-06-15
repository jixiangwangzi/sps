
#include "rclcpp/rclcpp.hpp"
#include "system_manager/bt/bt_define.h"
#include "system_manager/bt/robot_bt.h"
#include "system_manager/bt/simple_node.h"



namespace naviengine
{
template <typename T>
bool GetBB(BT::TreeNode &self, std::string key, T &value)
{
    return self.config().blackboard->get<T>(key, value);
}

template <typename T>
void SetBB(BT::TreeNode &self, std::string key, T &value)
{
    self.config().blackboard->set<T>(key, value);
}

std::string GetRobotState(BT::TreeNode &self)
{
    return RobotBt::GetInstance()->combo_state_vector.begin()->state_name_;
}

std::string GetTaskIdByRobotState(BT::TreeNode &self, const std::string &robot_state)
{
    ROS_INFO("GetTaskIdByRobotState entry");
    for (std::vector<EngineState>::iterator iterator = RobotBt::GetInstance()->combo_state_vector.begin();
         iterator != RobotBt::GetInstance()->combo_state_vector.end();
         iterator++)
    {
        if (iterator->state_name_.compare(robot_state) == 0)
        {
            std::string task_id = iterator->taskCtx_.task_id_;

            ROS_INFO("GetTaskIdByRobotState exit with task_id=%s", task_id.c_str());
            return task_id;
        }
    }

    ROS_INFO("GetTaskIdByRobotState entry with empty");
    return "";
}

void SetEventResponse(BT::TreeNode &self, ERESULT res)
{
    self.config().blackboard->set<ERESULT>(BB_ROBOT_EVENT_RESPONSE, res);
}


BT::NodeStatus IsProcessingState(BT::TreeNode &self)
{
    ROS_INFO("IsProcessingState entry");
    std::string real_robot_state = GetRobotState(self);

    ERESULT failure_code;
    ERESULT success_code;
    std::string robot_state;
    self.getInput(BB_INPUT_FAILURE_CODE, failure_code);
    self.getInput(BB_INPUT_SUCCESS_CODE, success_code);
    self.getInput(BB_INPUT_ROBOT_STATE, robot_state);
    ROS_INFO("Allow state: %s, now robot state: %s", robot_state.c_str(), real_robot_state.c_str());

    char a = ',';

    std::vector<std::string> strvec;

    std::string::size_type pos1, pos2;
    pos2 = robot_state.find(a);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        strvec.push_back(robot_state.substr(pos1, pos2 - pos1));

        pos1 = pos2 + 1;
        pos2 = robot_state.find(a, pos1);
    }
    strvec.push_back(robot_state.substr(pos1));

    if (std::find(strvec.begin(), strvec.end(), real_robot_state) != strvec.end())
    {
        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, success_code);
        ROS_INFO("allow robot state: %s ", real_robot_state.c_str());

        return BT::NodeStatus::SUCCESS;
    }

    self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);

    ROS_INFO("robot state: %s disallow", real_robot_state.c_str());
    ROS_INFO("IsProcessingState exit, state=%s", robot_state.c_str());
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsProcessedState(BT::TreeNode &self)
{
    ROS_INFO("IsProcessedState entry");

    ERESULT failure_code;
    ERESULT success_code;
    std::string robot_state;
    self.getInput(BB_INPUT_FAILURE_CODE, failure_code);
    self.getInput(BB_INPUT_SUCCESS_CODE, success_code);
    self.getInput(BB_INPUT_ROBOT_STATE, robot_state);

    char a = ',';

    std::vector<std::string> strvec;

    std::string::size_type pos1, pos2;
    pos2 = robot_state.find(a);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        strvec.push_back(robot_state.substr(pos1, pos2 - pos1));

        pos1 = pos2 + 1;
        pos2 = robot_state.find(a, pos1);
    }
    strvec.push_back(robot_state.substr(pos1));

    std::vector<EngineState> &combo_state_vector = RobotBt::GetInstance()->combo_state_vector;

    std::vector<EngineState>::iterator iterator;
    for (iterator = combo_state_vector.begin(); iterator != combo_state_vector.end(); iterator++)
    {
        if (std::find(strvec.begin(), strvec.end(), (*iterator).state_name_) != strvec.end())
        {
            self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, success_code);

            return BT::NodeStatus::SUCCESS;
        }
    }

    self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);

    ROS_INFO("IsProcessedState exit, state=%s", robot_state.c_str());
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsCompletedTask(BT::TreeNode &self)
{
    ROS_INFO("IsCompletedTask entry");

    HeaderStartNaviCtx startNaviCtx;
    GetBB<HeaderStartNaviCtx>(self, BB_ROBOT_START_NAVI_CTX, startNaviCtx);

    NaviCtx ctx_completed;
    GetBB<NaviCtx>(self, BB_ROBOT_TASK_CTX_COMPLETED, ctx_completed);

    ROS_INFO("IsCompletedTask task_id_=%s, completed_task_id_=%s",
             startNaviCtx.task_id_.c_str(),
             ctx_completed.task_id_.c_str());

    if (startNaviCtx.type_ == NAVI_GOCHARGE)
    {
        if (!startNaviCtx.task_id_.empty() && !ctx_completed.task_id_.empty() &&
            startNaviCtx.task_id_ == ctx_completed.task_id_)
        {
            ROS_INFO("IsCompletedTask exit with repeated go charge task");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("IsCompletedTask exit, GOCHARGE task startNaviCtx.task_id_=%s", startNaviCtx.task_id_.c_str());

            return BT::NodeStatus::FAILURE;
        }
    }

    if (!startNaviCtx.task_id_.empty() && !ctx_completed.task_id_.empty() &&
        startNaviCtx.task_id_ == ctx_completed.task_id_)
    {
        ROS_INFO("IsCompletedTask exit with repeated task");

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        NaviCtx ctx_tmp;
        SetBB<NaviCtx>(self, BB_ROBOT_TASK_CTX_COMPLETED, ctx_tmp);

        ROS_INFO("IsCompletedTask exit with new task");

        return BT::NodeStatus::FAILURE;
    }
}


BT::NodeStatus IsConflictTask(BT::TreeNode &self)
{
    ROS_INFO("IsConflictTask entry");

    ERESULT success_code;
    ERESULT failure_code;
    std::string robot_state;
    self.getInput(BB_INPUT_SUCCESS_CODE, success_code);
    self.getInput(BB_INPUT_FAILURE_CODE, failure_code);
    self.getInput(BB_INPUT_ROBOT_STATE, robot_state);

    HeaderStartNaviCtx startNaviCtx;
    GetBB<HeaderStartNaviCtx>(self, BB_ROBOT_START_NAVI_CTX, startNaviCtx);

    std::string running_task_id = GetTaskIdByRobotState(self, STATE_MOTIONING);

    if (startNaviCtx.task_id_.compare(running_task_id) == 0)
    {
        ROS_INFO("IsConflictTask exit, NOT Conflict NORMAL task startNaviCtx.task_id_=%s",
                 startNaviCtx.task_id_.c_str());

        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        if (startNaviCtx.type_ == NAVI_GOCHARGE)
        {
            ROS_INFO("IsConflictTask exit, NOT Conflict CHARGE task startNaviCtx.task_id_=%s",
                     startNaviCtx.task_id_.c_str());

            self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);
            return BT::NodeStatus::FAILURE;
        }

        ROS_INFO("IsConflictTask exit, Conflict task startNaviCtx.task_id_=%s, running_task_id=%s",
                 startNaviCtx.task_id_.c_str(),
                 running_task_id.c_str());

        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, success_code);
        return BT::NodeStatus::SUCCESS;
    }
}

BT::NodeStatus IsInitialized(BT::TreeNode &self)
{
    ROS_INFO("IsInitialized entry");
    ERESULT failure_code;
    self.getInput(BB_INPUT_FAILURE_CODE, failure_code);

    std::string robot_initialized;
    GetBB(self, BB_ROBOT_INITIALIZED, robot_initialized);

    ROS_INFO("IsInitialized exit, robot_initialized=%s", robot_initialized.c_str());

    if (robot_initialized.empty())
    {
        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::SUCCESS;
    }
}


BT::NodeStatus IsChargingLine(BT::TreeNode &self)
{
    ROS_INFO("IsChargingLine entry");
    ERESULT failure_code;
    ERESULT success_code;
    self.getInput(BB_INPUT_FAILURE_CODE, failure_code);
    self.getInput(BB_INPUT_SUCCESS_CODE, success_code);



    bool isChargingLine = naviengine::RobotBt::GetInstance()->btNodeImpl_->IsChargingLine();

    ROS_INFO("IsChargingLine exit, IsChargingLine=%d", isChargingLine);

    if (isChargingLine)
    {
        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, success_code);
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        self.setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, failure_code);
        return BT::NodeStatus::FAILURE;
    }
}


BT::NodeStatus IsHalfPointsSuccess(BT::TreeNode &self)
{
    ROS_INFO("IsHalfPointsSuccess entry");
    NaviCtx naviCtx;
    GetBB(self, BB_ROBOT_NAVI_CTX, naviCtx);

    if (NAVISTATE::NAVI_STATE_SUCCESS == naviCtx.navi_state_)
    {
        for (std::vector<EngineState>::iterator iterator = RobotBt::GetInstance()->combo_state_vector.begin();
             iterator != RobotBt::GetInstance()->combo_state_vector.end();
             iterator++)
        {
            if (iterator->state_name_.compare(STATE_MOTIONING) == 0)
            {
                if (iterator->taskCtx_.points_.size() > 0)
                {
                    iterator->taskCtx_.points_.erase(iterator->taskCtx_.points_.begin());
                }

                if (iterator->taskCtx_.points_.size() > 0)
                {
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }
            }
        }
    }

    ROS_INFO("IsHalfPointsSuccess exit");

    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsTmpStopHalfPointSuccess(BT::TreeNode &self)
{
    ROS_INFO("IsTmpStopHalfPointSuccess entry");
    NaviCtx naviCtx;
    GetBB(self, BB_ROBOT_NAVI_CTX, naviCtx);

    if (NAVISTATE::NAVI_STATE_SUCCESS == naviCtx.navi_state_)
    {
        for (auto &combo_state_item : RobotBt::GetInstance()->combo_state_vector)
        {
            if (combo_state_item.state_name_ == STATE_MOTIONING)
            {
                ROS_INFO("combo_state_item.taskCtx_.points_: %zu", combo_state_item.taskCtx_.points_.size());
                if (combo_state_item.taskCtx_.points_.empty())
                {
                    return BT::NodeStatus::FAILURE;
                }
                else
                {
                    if (combo_state_item.taskCtx_.points_[0].type == 100)
                    {
                        combo_state_item.taskCtx_.points_.erase(combo_state_item.taskCtx_.points_.begin());
                        return BT::NodeStatus::SUCCESS;
                    }
                    else
                    {
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
        }
    }

    ROS_INFO("IsTmpStopHalfPointSuccess exit");

    return BT::NodeStatus::FAILURE;
}

//-------------- Action -------------------

BT::NodeStatus ActionInitRobot(BT::TreeNode &self)
{
    ROS_INFO("ActionInitRobot entry");
    SetBB(self, BB_ROBOT_INITIALIZED, "OK");

    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->Init();
    ERESULT res = E_OK;

    SetEventResponse(self, res);

    ROS_INFO("ActionInitRobot exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionLoadMap(BT::TreeNode &self)
{
    ROS_INFO("ActionLoadMap entry");

    LoadMapCtx loadMapCtx("", "");
    GetBB(self, BB_ROBOT_LOAD_MAP_CTX, loadMapCtx);
    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->loadMap(loadMapCtx.map_name_);

    SetEventResponse(self, res);

    ROS_INFO("ActionLoadMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionSaveMap(BT::TreeNode &self)
{
    ROS_INFO("ActionSaveMap entry");

    SaveMapCtx saveMapCtx("", "");
    GetBB(self, BB_ROBOT_SAVE_MAP_CTX, saveMapCtx);

    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->SaveMap(saveMapCtx.task_id_, saveMapCtx.map_name_);
    ERESULT res = E_OK;
    SetEventResponse(self, res);

    ROS_INFO("ActionSaveMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionStartBuildMap(BT::TreeNode &self)
{
    ROS_INFO("ActionStartBuildMap entry");

    // TaskCtx taskCtx("");
    StartMappingCtx ctx("", TYPE_2D, E_NORMAL_MAPPING_MODE);
    // GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);
    GetBB(self, BB_ROBOT_START_MAPPING_CTX, ctx);
    ERESULT res = E_OK;
    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->StartBuildMap(ctx.task_id_, ctx.mapping_type_,
    // ctx.mapping_mode_);

    SetEventResponse(self, res);

    ROS_INFO("ActionStartBuildMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionPauseBuildMap(BT::TreeNode &self)
{
    ROS_INFO("ActionPauseBuildMap entry");

    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->PauseBuildMap(taskCtx.task_id_);
    ERESULT res = E_OK;
    SetEventResponse(self, res);

    ROS_INFO("ActionPauseBuildMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionResumeBuildMap(BT::TreeNode &self)
{
    ROS_INFO("ActionResumeBuildMap entry");

    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->ResumeBuildMap(taskCtx.task_id_);
    ERESULT res = E_OK;
    SetEventResponse(self, res);

    ROS_INFO("ActionResumeBuildMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionStopBuildMap(BT::TreeNode &self)
{
    ROS_INFO("ActionStopBuildMap entry");

    TaskCtx taskCtx;
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    // ERESULT res = naviengine::RobotBt::GetInstance()->robotCtrl_->StopBuildMap(taskCtx.task_id_);
    ERESULT res = E_OK;
    SetEventResponse(self, res);

    ROS_INFO("ActionStopBuildMap exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionStartNavi(BT::TreeNode &self)
{
    ROS_INFO("ActionStartNavi entry");

    HeaderStartNaviCtx startNaviCtx;
    GetBB(self, BB_ROBOT_START_NAVI_CTX, startNaviCtx);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->startNaviToPose(startNaviCtx.header_,
                                                                                   startNaviCtx.task_id_,
                                                                                   startNaviCtx.lift_id_,
                                                                                   startNaviCtx.type_,
                                                                                   startNaviCtx.goal_pose_,
                                                                                   startNaviCtx.twist_,
                                                                                   startNaviCtx.bt_xml_filename_,
                                                                                   startNaviCtx.align_angle_);

    SetEventResponse(self, res);

    ROS_INFO("ActionStartNavi exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionStartNaviThroughPoses(BT::TreeNode &self)
{
    ROS_INFO("ActionStartNaviThroughPoses entry");

    HeaderStartNaviCtx startNaviCtx;
    GetBB(self, BB_ROBOT_START_NAVI_CTX, startNaviCtx);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->startNaviThroughPoses(startNaviCtx.header_,
                                                                                         startNaviCtx.task_id_,
                                                                                         startNaviCtx.type_,
                                                                                         startNaviCtx.points_,
                                                                                         startNaviCtx.points_type_,
                                                                                         startNaviCtx.twist_,
                                                                                         startNaviCtx.bt_xml_filename_,
                                                                                         startNaviCtx.align_angle_);

    SetEventResponse(self, res);

    ROS_INFO("ActionStartNaviThroughPoses exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionSetNaviHalfPoints(BT::TreeNode &self)
{
    ROS_INFO("ActionSetNaviHalfPoints entry");

    std::vector<PixelPose> points;
    GetBB(self, BB_ROBOT_HALF_POINTS_VECTOR, points);

    for (std::vector<EngineState>::iterator iterator = RobotBt::GetInstance()->combo_state_vector.begin();
         iterator != RobotBt::GetInstance()->combo_state_vector.end();
         iterator++)
    {
        if (iterator->state_name_.compare(STATE_MOTIONING) == 0)
        {
            points.push_back(iterator->taskCtx_.goal_pose_);
            iterator->taskCtx_.points_ = points;

            ROS_INFO("ActionSetNaviHalfPoints fill with %d points", (int)points.size());
            break;
        }
    }

    ROS_INFO("ActionSetNaviHalfPoints exit");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ActionPauseNavi(BT::TreeNode &self)
{
    ROS_INFO("ActionPauseNavi entry");
    rclcpp::Rate(0.5).sleep();
    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->pauseNavi(taskCtx.task_id_);

    SetEventResponse(self, res);

    ROS_INFO("ActionPauseNavi exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionResumeNavi(BT::TreeNode &self)
{
    ROS_INFO("ActionResumeNavi entry");

    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->resumeNavi(taskCtx.task_id_);

    SetEventResponse(self, res);

    ROS_INFO("ActionResumeNavi exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionStopNavi(BT::TreeNode &self)
{
    ROS_INFO("ActionStopNavi entry");

    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->stopNavi(taskCtx.task_id_);

    SetEventResponse(self, res);

    ROS_INFO("ActionStopNavi exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionCancelNavi(BT::TreeNode &self)
{
    ROS_INFO("ActionCancelNavi entry");

    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);

    std::string running_task_id;

    if (taskCtx.task_id_.empty())
    {
        running_task_id = GetTaskIdByRobotState(self, STATE_MOTIONING);
    }
    else
    {
        running_task_id = taskCtx.task_id_;
    }

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->stopNavi(running_task_id);

    SetEventResponse(self, res);

    ROS_INFO("ActionCancelNavi exit,running_task_id=%s, ERESULT=%d", running_task_id.c_str(), res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionCancelNaviMarked(BT::TreeNode &self)
{
    ROS_INFO("ActionCancelNaviMarked entry");

    TaskCtx taskCtx;
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);
    std::string running_task_id;

    if (taskCtx.task_id_.empty())
    {
        running_task_id = GetTaskIdByRobotState(self, STATE_MOTIONING);
    }
    else
    {
        running_task_id = taskCtx.task_id_;
    }

    SetBB(self, BB_ROBOT_TASK_ID_MARKED, running_task_id);

    ERESULT res = naviengine::RobotBt::GetInstance()->btNodeImpl_->stopNavi(running_task_id);

    SetEventResponse(self, res);

    ROS_INFO("ActionCancelNaviMarked exit,running_task_id=%s, ERESULT=%d", running_task_id.c_str(), res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionOnNaviDone(BT::TreeNode &self)
{
    ROS_INFO("ActionOnNaviDone entry");

    NaviCtx naviCtx;
    GetBB(self, BB_ROBOT_NAVI_CTX, naviCtx);

    if (naviCtx.navi_state_ == NAVISTATE::NAVI_STATE_SUCCESS)
    {
        SetBB(self, BB_ROBOT_TASK_CTX_COMPLETED, naviCtx);
    }
    TaskCtx taskCtx("");
    GetBB(self, BB_ROBOT_TASK_CTX, taskCtx);
    if (taskCtx.task_id_ == naviCtx.task_id_)
    {
        TaskCtx taskCtxCompleted("");
        SetBB(self, BB_ROBOT_TASK_CTX, taskCtxCompleted);
    }

    ERESULT res = E_OK;  //= naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnNaviDone(naviCtx.task_id_,
                         // naviCtx.navi_type_, naviCtx.navi_state_, naviCtx.description_);

    SetEventResponse(self, res);

    ROS_INFO("ActionOnNaviDone exit navi task id: %s，type: %d, state: %d, ERESULT=%d",
             naviCtx.task_id_.c_str(),
             (int)naviCtx.navi_type_,
             (int)naviCtx.navi_state_,
             res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus ActionOnNaviDoneRepeat(BT::TreeNode &self)
{
    ROS_INFO("ActionOnNaviDoneRepeat entry");

    NaviCtx naviCtx;
    GetBB<NaviCtx>(self, BB_ROBOT_TASK_CTX_COMPLETED, naviCtx);

    ERESULT res = E_OK;
    // ERESULT res = naviengine::RobotBt::GetInstance()->robotEngineObserver_->OnNaviDone(
    //     naviCtx.task_id_, naviCtx.navi_type_, NAVISTATE::NAVI_STATE_SUCCESS, naviCtx.description_);

    SetEventResponse(self, res);

    ROS_INFO("ActionOnNaviDoneRepeat exit, ERESULT=%d", res);

    if (E_OK == res)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}
}  // namespace naviengine
