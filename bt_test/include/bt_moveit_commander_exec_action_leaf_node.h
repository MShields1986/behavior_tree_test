#pragma once

#include "ros/ros.h"
#include "noether_client/MoveItExecute.h"

#include "behaviortree_cpp_v3/action_node.h"


class MoveItCommanderExecuteService : public BT::SyncActionNode
{
  public:
    MoveItCommanderExecuteService(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
