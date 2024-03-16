#include "bt_view_plan_action_leaf_node.h"


ViewPlan::ViewPlan(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList ViewPlan::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id")
         };
}


BT::NodeStatus ViewPlan::tick()
{
  auto res = getInput<std::string>("frame_id");

  if (!res)
  {
    throw BT::RuntimeError("TfToPose | error reading port [frame_id]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  std::string frame_id = res.value();

  //setOutput<geometry_msgs::PoseArray>("poses", poses);
  //ROS_INFO_STREAM("TfToPose | poses output: " << poses);

  return BT::NodeStatus::SUCCESS;
}
