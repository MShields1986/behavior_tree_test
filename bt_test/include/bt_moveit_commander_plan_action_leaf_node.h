#pragma once

#include "ros/ros.h"
#include "noether_client/MoveItPlan.h"

#include "behaviortree_cpp_v3/action_node.h"


class MoveItCommanderPlanService : public BT::SyncActionNode
{
  public:
    MoveItCommanderPlanService(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string topic_name_;
};
