#pragma once

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "iiwa_msgs/MoveToJointPositionAction.h"
#include "iiwa_msgs/JointPosition.h"

#include "behaviortree_cpp_v3/action_node.h"

#include "iiwajointposition_bb_parser.h"



class IiwaToJointPosition : public BT::SyncActionNode
{
  public:
    IiwaToJointPosition(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string action_name_;
    iiwa_msgs::MoveToJointPositionGoal joint_position_goal_;
    actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> joint_position_client_;
};
