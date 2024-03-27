#pragma once

#include "ros/ros.h"
#include "iiwa_handler/PoseArrayInputAction.h"

#include "behaviortree_cpp_v3/action_node.h"


class IiiwaHandlerSplineMotionAction : public BT::SyncActionNode
{
  public:
    IiiwaHandlerSplineMotionAction(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string topic_name_;
    actionlib::SimpleActionClient<iiwa_handler::PoseArrayInputAction> spline_motion_client_;
};
