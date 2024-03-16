#include "bt_iiwa_joint_cmd_action_leaf_node.h"



IiwaToJointPosition::IiwaToJointPosition(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList IiwaToJointPosition::providedPorts()
{
  return {BT::InputPort<iiwa_msgs::JointPosition>("joints")};
}


BT::NodeStatus IiwaToJointPosition::tick()
{
  auto res = getInput<iiwa_msgs::JointPosition>("joints");
 
  if( !res )
  {
    throw BT::RuntimeError("IiwaToJointPosition | error reading port [joints]:", res.error());
    return BT::NodeStatus::FAILURE;
  }

  m_joints = res.value();
  ROS_INFO_STREAM("IiwaToJointPosition | joints input: " << m_joints);

  ros::NodeHandle n;
  m_pub = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1000, true);
  m_pub.publish(m_joints);
  ros::spinOnce();
  // TODO: Consider instead of holding for 4 sec could hold until arm vel reaches zero
  ros::Duration(4.0).sleep();
  return BT::NodeStatus::SUCCESS;
}
