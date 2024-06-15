//
// Created by bing on 21-11-18.
//

#ifndef ROBOT_NAVIENGINE_SIMPLE_NODE_H
#define ROBOT_NAVIENGINE_SIMPLE_NODE_H

#include "system_manager/define.h"
#include <behaviortree_cpp_v3/behavior_tree.h>

namespace BT
{
    template<>
    inline ERESULT convertFromString(StringView str)
    {
        ERESULT res;
        res = (ERESULT) convertFromString<int>(str.data());
        return res;
    }
}


namespace naviengine
{


// State Condition
    BT::NodeStatus IsProcessingState(BT::TreeNode &self);

    BT::NodeStatus IsProcessedState(BT::TreeNode &self);

    BT::NodeStatus IsCompletedTask(BT::TreeNode &self);

    BT::NodeStatus IsConflictTask(BT::TreeNode &self);

    BT::NodeStatus IsInitialized(BT::TreeNode &self);

    BT::NodeStatus IsChargingLine(BT::TreeNode &self);

    BT::NodeStatus IsHalfPointsSuccess(BT::TreeNode &self);

    BT::NodeStatus IsTmpStopHalfPointSuccess(BT::TreeNode &self);
// Action
    BT::NodeStatus ActionInitRobot(BT::TreeNode &self);

    BT::NodeStatus ActionLoadMap(BT::TreeNode &self);

    BT::NodeStatus ActionSaveMap(BT::TreeNode &self);

    BT::NodeStatus ActionStartBuildMap(BT::TreeNode &self);

    BT::NodeStatus ActionPauseBuildMap(BT::TreeNode &self);

    BT::NodeStatus ActionResumeBuildMap(BT::TreeNode &self);

    BT::NodeStatus ActionStopBuildMap(BT::TreeNode &self);

    BT::NodeStatus ActionStartNavi(BT::TreeNode &self);

    BT::NodeStatus ActionStartNaviThroughPoses(BT::TreeNode &self);

    BT::NodeStatus ActionSetNaviHalfPoints(BT::TreeNode &self);

    BT::NodeStatus ActionPauseNavi(BT::TreeNode &self);

    BT::NodeStatus ActionResumeNavi(BT::TreeNode &self);

    BT::NodeStatus ActionStopNavi(BT::TreeNode &self);

    BT::NodeStatus ActionCancelNavi(BT::TreeNode &self);

    BT::NodeStatus ActionCancelNaviMarked(BT::TreeNode &self);

    BT::NodeStatus ActionOnNaviDone(BT::TreeNode &self);

    BT::NodeStatus ActionOnNaviDoneRepeat(BT::TreeNode &self);
}

#endif //ROBOT_NAVIENGINE_SIMPLE_NODE_H
