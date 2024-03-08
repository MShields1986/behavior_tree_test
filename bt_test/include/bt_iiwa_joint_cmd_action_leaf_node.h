#pragma once

#include "ros/ros.h"
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
    iiwa_msgs::JointPosition m_joints;
    ros::Publisher m_pub;
};
