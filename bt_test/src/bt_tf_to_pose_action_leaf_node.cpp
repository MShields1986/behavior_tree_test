#include "bt_tf_to_pose_action_leaf_node.h"


TfToPose::TfToPose(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config),
      tfListener_(tfBuffer_)
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

  frame_id_ = res.value();

  try{
      transformStamped_ = tfBuffer_.lookupTransform("map", frame_id_, ros::Time(0), ros::Duration(3.0));
      }
  catch (tf2::TransformException& ex) {
      ROS_INFO_STREAM("TfToPose | could not find transform map to " + frame_id_ + ": " + ex.what());
      return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::PoseStamped pose;
  //pose.header.stamp           = ros::Time::now();
  pose.header.frame_id        = "map";
  pose.pose.position.x        = transformStamped_.transform.translation.x;
  pose.pose.position.y        = transformStamped_.transform.translation.y;
  pose.pose.position.z        = transformStamped_.transform.translation.z;
  pose.pose.orientation.x     = transformStamped_.transform.rotation.x;
  pose.pose.orientation.y     = transformStamped_.transform.rotation.y;
  pose.pose.orientation.z     = transformStamped_.transform.rotation.z;
  pose.pose.orientation.w     = transformStamped_.transform.rotation.w;

  setOutput<geometry_msgs::PoseStamped>("pose", pose);
  ROS_INFO_STREAM("TfToPose | pose output: " << pose);

  return BT::NodeStatus::SUCCESS;
}
