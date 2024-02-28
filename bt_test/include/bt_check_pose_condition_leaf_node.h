#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "behaviortree_cpp_v3/action_node.h"



class CheckPose : public BT::SyncActionNode
{
  public:
    CheckPose(const std::string &name, const BT::NodeConfiguration &config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

};
