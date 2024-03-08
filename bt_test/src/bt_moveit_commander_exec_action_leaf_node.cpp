#include "bt_moveit_commander_exec_action_leaf_node.h"


MoveItCommanderExecuteService::MoveItCommanderExecuteService(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList MoveItCommanderExecuteService::providedPorts()
{
  return {};/*
    BT::InputPort<std::string>("pose_array_topic_name")
         };*/
}


BT::NodeStatus MoveItCommanderExecuteService::tick()
{
  /*
  auto res = getInput<std::string>("pose_array_topic_name");

  if (!res)
  {
    throw BT::RuntimeError("MoveItCommanderExecuteService | error reading port [pose_array_topic_name]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  m_topic_name = res.value();
  */
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<noether_client::MoveItExecute>("/iiwa/moveit_execute");
  noether_client::MoveItExecute srv;
  srv.request.enable = true;
  if (client.call(srv))
  {
    ROS_INFO_STREAM("Service call successful: " << std::noboolalpha << srv.response.success);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_ERROR("Failed to call service /iiwa/moveit_execute");
    return BT::NodeStatus::FAILURE;
  }
}
