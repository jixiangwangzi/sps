//
// Created by bing on 21-11-25.
//

#ifndef ROBOT_NAVIENGINE_STATE_WRITE_OFF_NODE_H
#define ROBOT_NAVIENGINE_STATE_WRITE_OFF_NODE_H

#include <system_manager/bt/bt_define.h>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "algorithm"
#include "set"

namespace naviengine
{
    class StateWriteOffNode : public BT::SyncActionNode
    {
    public:

        StateWriteOffNode(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config)
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
                if (iterator->state_name_ == robot_state)
                {
                    ROS_INFO("StateWriteOff matched state=%s", robot_state.c_str());

                    if (!distinct_event.empty() && iterator->multi_event_set_.size() > 0)
                    {
                        ROS_INFO("StateWriteOff erasing distinct_event=%s", distinct_event.c_str());

                        iterator->multi_event_set_.erase(distinct_event);
                    }

                    if (iterator->multi_event_set_.size() == 0)
                    {
                        ROS_INFO("StateWriteOff erasing state=%s", robot_state.c_str());

                        combo_state_vector.erase(iterator);
                    }
                    break;
                }
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
    };

}


#endif //ROBOT_NAVIENGINE_STATE_WRITE_OFF_NODE_H
