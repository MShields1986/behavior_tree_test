#include "bt_tf_to_pose_action_leaf_node.h"


TfToPose::TfToPose(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config),
      m_tfListener(m_tfBuffer)
{
}


BT::PortsList TfToPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("frame_id"),
    BT::OutputPort<geometry_msgs::PoseStamped>("pose")
         };
}


BT::NodeStatus TfToPose::tick()
{
  auto res = getInput<std::string>("frame_id");

  if (!res)
  {
    throw BT::RuntimeError("TfToPose | error reading port [frame_id]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  m_frame_id = res.value();

  try{
      m_transformStamped = m_tfBuffer.lookupTransform("map", m_frame_id, ros::Time(0), ros::Duration(3.0));
      }
  catch (tf2::TransformException& ex) {
      ROS_INFO_STREAM("TfToPose | could not find transform map to " + m_frame_id + ": " + ex.what());
      return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::PoseStamped pose;
  //pose.header.stamp           = ros::Time::now();
  pose.header.frame_id        = "map";
  pose.pose.position.x        = m_transformStamped.transform.translation.x;
  pose.pose.position.y        = m_transformStamped.transform.translation.y;
  pose.pose.position.z        = m_transformStamped.transform.translation.z;
  pose.pose.orientation.x     = m_transformStamped.transform.rotation.x;
  pose.pose.orientation.y     = m_transformStamped.transform.rotation.y;
  pose.pose.orientation.z     = m_transformStamped.transform.rotation.z;
  pose.pose.orientation.w     = m_transformStamped.transform.rotation.w;

  setOutput<geometry_msgs::PoseStamped>("pose", pose);
  ROS_INFO_STREAM("TfToPose | pose output: " << pose);

  return BT::NodeStatus::SUCCESS;
}
