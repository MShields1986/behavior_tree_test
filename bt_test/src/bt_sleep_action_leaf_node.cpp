#include "bt_sleep_action_leaf_node.h"


Sleep::Sleep(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)

{
}


BT::PortsList Sleep::providedPorts()
{
  return {
    BT::InputPort<double>("sleep_time")
         };
}


BT::NodeStatus Sleep::tick()
{
  auto res = getInput<double>("sleep_time");

  if (!res)
  {
    throw BT::RuntimeError("TfToPose | error reading port [sleep_time]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  m_sleep_time = res.value();
  ROS_INFO_STREAM("Sleep | sleeping for " << m_sleep_time << " seconds");
  ros::Duration(m_sleep_time).sleep();
  return BT::NodeStatus::SUCCESS;
}
