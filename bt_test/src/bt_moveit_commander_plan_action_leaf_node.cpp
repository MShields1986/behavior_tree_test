#include "bt_moveit_commander_plan_action_leaf_node.h"


MoveItCommanderPlanService::MoveItCommanderPlanService(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList MoveItCommanderPlanService::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_array_topic_name")
         };
}


BT::NodeStatus MoveItCommanderPlanService::tick()
{
  auto res = getInput<std::string>("pose_array_topic_name");

  if (!res)
  {
    throw BT::RuntimeError("MoveItCommanderPlanService | error reading port [pose_array_topic_name]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  m_topic_name = res.value();

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<noether_client::MoveItPlan>("/iiwa/moveit_plan");
  noether_client::MoveItPlan srv;
  srv.request.topic_name = m_topic_name;
  if (client.call(srv))
  {
    ROS_INFO("Service call successful: " + srv.response.success);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_ERROR("Failed to call service /iiwa/moveit_plan");
    return BT::NodeStatus::FAILURE;
  }
}
