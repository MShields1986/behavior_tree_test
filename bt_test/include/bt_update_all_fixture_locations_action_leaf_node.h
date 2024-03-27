#pragma once

#include "ros/ros.h"

#include "fixture_tracker/UpdateFixturePose.h"

#include "behaviortree_cpp_v3/action_node.h"



class UpdateAllFixtureLocations : public BT::SyncActionNode
{
  public:
    UpdateAllFixtureLocations(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    fixture_tracker::UpdateFixturePose srv_;
    std::string service_name_;

};
