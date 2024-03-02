#pragma once

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "behaviortree_cpp_v3/action_node.h"

class TfToPose : public BT::SyncActionNode
{
  public:
    TfToPose(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
