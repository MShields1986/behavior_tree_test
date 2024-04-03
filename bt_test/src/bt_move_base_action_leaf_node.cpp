#include "bt_move_base_action_leaf_node.h"



BaseToGoal::BaseToGoal(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::AsyncActionNode(name, config),
      actionclient_("/kmr/move_base", true)
{
  /*
  while(!_actionclient.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }*/
  ROS_INFO_STREAM("BaseToGoal | move_base action server up");
}


BT::PortsList BaseToGoal::providedPorts()
{
  return {BT::InputPort<geometry_msgs::PoseStamped>("pose")};
}


void BaseToGoal::setGoal()
{
  goal_.target_pose.header.frame_id = pose_.header.frame_id;
  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position.x = pose_.pose.position.x;
  goal_.target_pose.pose.position.y = pose_.pose.position.y;
  goal_.target_pose.pose.orientation.z = pose_.pose.orientation.z;
  goal_.target_pose.pose.orientation.w = pose_.pose.orientation.w;
  //ROS_INFO_STREAM("BaseToGoal | goal set to " << goal_.target_pose);
}


void BaseToGoal::haltGoal()
  {
    actionclient_.cancelGoal();
    ROS_INFO_STREAM("BaseToGoal | move_base goal cancelled");
  }


BT::NodeStatus BaseToGoal::sendGoal()
  {
    ROS_INFO_STREAM("BaseToGoal | sending goal");
    actionclient_.sendGoal(goal_);
    //_actionclient.sendGoal(goal_, &doneCb, &activeCb, &feedbackCb);
    ROS_INFO_STREAM("BaseToGoal | goal sent");

    actionclient_.waitForResult();

    if (actionclient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO_STREAM("BaseToGoal | move_base goal succeeded");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO_STREAM("BaseToGoal | move_base goal failed");
      return BT::NodeStatus::FAILURE;
    }
  }

/*
BaseToGoal::feedbackCb(const MoveBaseFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Got Feedback of length %lu", feedback->sequence.size());
}
*/

// SyncActionNode only return success or failure as they are blocking so this one is AsyncActionNode
BT::NodeStatus BaseToGoal::tick()
{
  auto res = getInput<geometry_msgs::PoseStamped>("pose");
 
  if( !res )
  {
    throw BT::RuntimeError("BaseToGoal | error reading port [pose]:", res.error());
    return BT::NodeStatus::FAILURE;
  }

  pose_ = res.value();
  //ROS_INFO_STREAM("BaseToGoal | pose input: " << pose_);

  setGoal();
  return sendGoal();
}
