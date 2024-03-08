#include "bt_move_base_action_leaf_node.h"



BaseToGoal::BaseToGoal(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::AsyncActionNode(name, config),
      _actionclient("/kmr/move_base", true)
{
  while(!_actionclient.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("BaseToGoal | move_base action server up");
}


BT::PortsList BaseToGoal::providedPorts()
{
  return {BT::InputPort<geometry_msgs::PoseStamped>("pose")};
}


void BaseToGoal::setGoal()
{
  m_goal.target_pose.header.frame_id = m_pose.header.frame_id;
  m_goal.target_pose.header.stamp = ros::Time::now();
  m_goal.target_pose.pose.position.x = m_pose.pose.position.x;
  m_goal.target_pose.pose.position.y = m_pose.pose.position.y;
  m_goal.target_pose.pose.orientation.z = m_pose.pose.orientation.z;
  m_goal.target_pose.pose.orientation.w = m_pose.pose.orientation.w;
  ROS_INFO_STREAM("BaseToGoal | goal set to " << m_goal.target_pose);
}


void BaseToGoal::haltGoal()
  {
    _actionclient.cancelGoal();
    ROS_INFO_STREAM("BaseToGoal | move_base goal cancelled");
  }


BT::NodeStatus BaseToGoal::sendGoal()
  {
    ROS_INFO_STREAM("BaseToGoal | sending goal");
    _actionclient.sendGoal(m_goal);
    //_actionclient.sendGoal(m_goal, &doneCb, &activeCb, &feedbackCb);
    ROS_INFO_STREAM("BaseToGoal | goal sent");

    _actionclient.waitForResult();

    if (_actionclient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
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

  m_pose = res.value();
  ROS_INFO_STREAM("BaseToGoal | pose input: " << m_pose);

  setGoal();
  return sendGoal();
}
