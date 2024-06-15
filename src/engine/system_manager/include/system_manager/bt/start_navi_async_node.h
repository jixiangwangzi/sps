//
// Created by bing on 21-12-28.
//

#ifndef ROBOT_NAVIENGINE_START_NAVI_ASYNC_NODE_H
#define ROBOT_NAVIENGINE_START_NAVI_ASYNC_NODE_H

#include <system_manager/bt/bt_define.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robot_bt.h"

namespace naviengine
{
class StartNaviAsyncNode : public BT::SyncActionNode
{
public:
    StartNaviAsyncNode(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config){};

    BT::NodeStatus tick() override
    {
        ROS_INFO("StartNaviAsyncNode entry");

        std::thread start_navi_to_next_tmp_point(&naviengine::StartNaviAsyncNode::StartNavi, this);
        start_navi_to_next_tmp_point.detach();

        setOutput<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE, E_OK);

        ROS_INFO("StartNaviAsyncNode exists with FAILURE");
        return BT::NodeStatus::SUCCESS;
    };


    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<ERESULT>(BB_OUTPUT_ON_RESPONSE_CODE)};
    }

private:
    void StartNavi()
    {
        ROS_INFO("StartNaviAsyncNode StartNavi entry");

        for (std::vector<EngineState>::iterator iterator = RobotBt::GetInstance()->combo_state_vector.begin();
             iterator != RobotBt::GetInstance()->combo_state_vector.end();
             iterator++)
        {
            // STOP state has two fire way
            if (iterator->state_name_ == STATE_MOTIONING)
            {
                StartNaviCtx &startNaviCtx = iterator->taskCtx_;

                PixelPose pose_ = startNaviCtx.points_.begin().operator*();

                ENAVITYPE type_ = startNaviCtx.points_.size() > 1 ? ENAVITYPE::NAVI_TEMP : startNaviCtx.type_;
                std::string goal_name_ = startNaviCtx.points_.size() > 1 ? "" : startNaviCtx.goal_name_;

                ERESULT res;  // = RobotBt::GetInstance()->robotCtrl_->StartNavi(startNaviCtx.task_id_, type_,
                              // goal_name_, pose_, startNaviCtx.goal_tolerance_);

                // if(E_OK != res)
                // {
                //     RobotBt::GetInstance()->robotEngineObserver_->OnNaviDone(startNaviCtx.task_id_, ENAVITYPE
                //     ::NAVI_NORMAL, NAVISTATE::NAVI_STATE_FAILED, "");
                // }

                ROS_INFO("StartNaviAsyncNode StartNavi exists res=%d", res);
                break;
            }
        }
    }
};

}  // namespace naviengine

#endif  // ROBOT_NAVIENGINE_START_NAVI_ASYNC_NODE_H
