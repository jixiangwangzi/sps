/*
* Copyright (c) 2022, Cloudminds, Inc.
* All rights reserved.
*
* author: ShiLiang Li
* Created Date: 2022-10-31
*/

#include <string>

#include "nav2_behavior_tree/plugins/control/main_sequence.hpp"

namespace nav2_behavior_tree
{

MainSequence::MainSequence(const std::string & name)
: BT::ControlNode::ControlNode(name, {})
{
}

MainSequence::MainSequence(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus MainSequence::tick()
{
  const auto num_children = children_nodes_.size();

  if (status() == BT::NodeStatus::IDLE){
    first_time_ = true; 
		index_ = 0;
  }

  setStatus(BT::NodeStatus::RUNNING);

	for (std::size_t i = index_; i < num_children; i++){
		TreeNode * child_node = children_nodes_[i];
		const BT::NodeStatus child_status = child_node->executeTick();
		switch (child_status) {
			case BT::NodeStatus::SUCCESS:
				{
					if (i == 0) {
						index_ = 1;
					}
					break;
				}
			case BT::NodeStatus::FAILURE:
				{
					if (i == 0) {
						halt();
						return BT::NodeStatus::FAILURE;
					}
					if (i == 1) {
						std::cout << "*****" << std::endl;
						ControlNode::haltChild(1);
						ControlNode::haltChild(2);
						index_ = 0;
						first_time_ = true;
					}
					return BT::NodeStatus::RUNNING;
				}

			case BT::NodeStatus::RUNNING:
				{
					if (i == 1 && !first_time_) {
						break;
					}
					if (i == 2) {
						first_time_ = false;
					}
					return BT::NodeStatus::RUNNING;
				}

			default:
				{
					throw BT::LogicError("Invalid status return from BT node");
				}
		}
	}
	halt();
	return BT::NodeStatus::SUCCESS;
}

void MainSequence::halt()
{
  ControlNode::halt();
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::MainSequence>("MainSequence");
}
