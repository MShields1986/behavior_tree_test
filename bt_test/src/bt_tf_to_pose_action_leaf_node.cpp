#include "bt_tf_to_pose_action_leaf_node.h"


TfToPose::TfToPose(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
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
  BT::Optional<std::string> msg = getInput<std::string>("frame_id");

  if (!msg)
  {
    throw BT::RuntimeError("Missing required input[frame_id]: ", msg.error());
    return BT::NodeStatus::FAILURE;
  }

  std::string frame_id = msg.value();

  // TODO: Be good to get the instantiation out of here
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;

  try{
      transformStamped = tfBuffer.lookupTransform("map", frame_id, ros::Time(0), ros::Duration(3.0));
      ROS_INFO_STREAM("Current pose...");
      ROS_INFO_STREAM(transformStamped);
      }
  catch (tf2::TransformException &ex) {
      ROS_INFO_STREAM("Could not find transform map to " + frame_id + ": " + ex.what());
      return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp           = ros::Time::now();
  pose.header.frame_id        = frame_id;
  pose.pose.position.x        = transformStamped.transform.translation.x;
  pose.pose.position.y        = transformStamped.transform.translation.y;
  pose.pose.position.z        = transformStamped.transform.translation.z;
  pose.pose.orientation.x     = transformStamped.transform.rotation.x;
  pose.pose.orientation.y     = transformStamped.transform.rotation.y;
  pose.pose.orientation.z     = transformStamped.transform.rotation.z;
  pose.pose.orientation.w     = transformStamped.transform.rotation.w;

  setOutput<geometry_msgs::PoseStamped>("pose", pose);

  return BT::NodeStatus::SUCCESS;
}
