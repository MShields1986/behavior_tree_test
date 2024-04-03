#pragma once

#include "ros/ros.h"

#include "behaviortree_cpp_v3/action_node.h"


class Sleep : public BT::SyncActionNode
{
  public:
    Sleep(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    double sleep_time_;

};
