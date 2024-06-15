//
// Created by bing on 21-11-17.
//

#ifndef ROBOT_NAVIENGINE_STATE_COMBO_NODE_H
#define ROBOT_NAVIENGINE_STATE_COMBO_NODE_H

#include <system_manager/bt/bt_define.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "algorithm"
#include "set"
#include "robot_bt.h"

namespace naviengine
{
    class StateComboNode : public BT::SyncActionNode
    {
    public:

        StateComboNode(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
        {};

        BT::NodeStatus tick() override
        {
            std::string robot_state;
            std::string distinct_event;
            getInput(BB_INPUT_ROBOT_STATE, robot_state);
            getInput(BB_INPUT_DISTINCT_EVENT, distinct_event);

            std::vector<EngineState> &combo_state_vector = RobotBt::GetInstance()->combo_state_vector;

            for (std::vector<EngineState>::iterator iterator = combo_state_vector.begin(); iterator != combo_state_vector.end(); iterator++)
            {
                // STOP state has two fire way
                if (iterator->state_name_ == robot_state)
                {
                    if (!distinct_event.empty())
                    {
                        iterator->multi_event_set_.insert(distinct_event);
                    }

                    ROS_INFO("StateCombo matched exists state=%s , distinct_event=%s", robot_state.c_str(), distinct_event.c_str());

                    // success when already exist
                    return BT::NodeStatus::SUCCESS;
                }
            }

            // STATE_INIT , STATE_IDLE not needed
            if (STATE_MAPPING == robot_state)
            {
                StartNaviCtx taskCtx;
                config().blackboard->get(BB_ROBOT_START_NAVI_CTX, taskCtx);

                EngineState engineState(STATE_MAPPING, PRIORITY_MAPPING, taskCtx);

                combo_state_vector.push_back(engineState);
            }
            else if (STATE_MOTIONING == robot_state)
            {
                StartNaviCtx startNaviCtx;
                config().blackboard->get(BB_ROBOT_START_NAVI_CTX, startNaviCtx);

                EngineState engineState(STATE_MOTIONING, PRIORITY_MOTIONING, startNaviCtx);

                combo_state_vector.push_back(engineState);
            }
            else if (STATE_CHARGING == robot_state)
            {
                EngineState engineState(STATE_CHARGING, PRIORITY_CHARGING);

                combo_state_vector.push_back(engineState);
            }
            else if (STATE_STOP == robot_state)
            {
                EngineState engineState(STATE_STOP, PRIORITY_STOP);

                engineState.multi_event_set_.insert(distinct_event);

                combo_state_vector.push_back(engineState);
            }
            else if (STATE_ERROR == robot_state)
            {
                EngineState engineState(STATE_ERROR, PRIORITY_ERROR);

                combo_state_vector.push_back(engineState);
            }
            else if (STATE_TELEOPING == robot_state)
            {
                EngineState engineState(STATE_TELEOPING, PRIORITY_TELEOPING);

                combo_state_vector.push_back(engineState);
            }


            std::sort(combo_state_vector.begin(), combo_state_vector.end(), _compare);


            for (std::vector<EngineState>::iterator iterator = combo_state_vector.begin(); iterator != combo_state_vector.end(); iterator++)
            {
                ROS_INFO("Combo State Order : %s", iterator->state_name_.c_str());
            }

            return BT::NodeStatus::SUCCESS;
        };

        static BT::PortsList providedPorts()
        {
            return {
                    BT::InputPort<std::string>(BB_INPUT_ROBOT_STATE),
                    BT::InputPort<std::string>(BB_INPUT_DISTINCT_EVENT)
            };
        }

        static bool _compare(const EngineState &e1, const EngineState &e2)
        {
            return e1.priority_ > e2.priority_;
        }


    };

}

#endif //ROBOT_NAVIENGINE_STATE_COMBO_NODE_H
