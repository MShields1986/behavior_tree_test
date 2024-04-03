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

  sleep_time_ = res.value();
  ROS_INFO_STREAM("Sleep | sleeping for " << sleep_time_ << " seconds");
  ros::Duration(sleep_time_).sleep();
  return BT::NodeStatus::SUCCESS;
}
