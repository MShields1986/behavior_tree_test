#include "bt_move_base_action_leaf_node.h"


BaseToGoal::BaseToGoal(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::AsyncActionNode(name, config),
      _actionclient("/kmr/move_base", true)
{
  while(!_actionclient.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("move_base action server up");
}


BT::PortsList BaseToGoal::providedPorts()
{
  return {BT::InputPort<std::vector<int>>("goal_pose")};
}


void BaseToGoal::setGoal()
{
  ROS_INFO_STREAM("Setting goal");
  /*
  // TODO: Read port or topic for goal_pose
  BT::Optional<std::vector<int>> msg = getInput<std::vector<int>>("goal_pose");

  if (!msg)
  {
    throw BT::RuntimeError("Missing required input[goal_pose]: ", msg.error());
  }

  ROS_INFO_STREAM("goal_pose from blackboard:");

  for (const auto position : msg.value())
  {
    ROS_INFO_STREAM(position);
  }
  */
  this->goal.target_pose.header.frame_id = "map";
  this->goal.target_pose.header.stamp = ros::Time::now();
  this->goal.target_pose.pose.position.x = 1.0;
  this->goal.target_pose.pose.position.y = 0.0;
  this->goal.target_pose.pose.orientation.z = 0.0;
  this->goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO_STREAM("Goal set");
}


void BaseToGoal::haltGoal()
  {
    _actionclient.cancelGoal();
    ROS_INFO_STREAM("move_base goal cancelled");
  }


BT::NodeStatus BaseToGoal::sendGoal()
  {
    ROS_INFO_STREAM("Sending goal");
    _actionclient.sendGoal(this->goal);
    //_actionclient.sendGoal(this->goal, &doneCb, &activeCb, &feedbackCb);
    ROS_INFO_STREAM("Goal sent");

    _actionclient.waitForResult();

    if (_actionclient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO_STREAM("move_base goal succeeded");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_INFO_STREAM("move_base goal failed");
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
  ROS_INFO_STREAM("BaseToGoal got ticked!");
  /*
  // Read port
  BT::Optional<std::vector<int>> msg = getInput<std::vector<int>>("goal_pose");

  if (!msg)
  {
    throw BT::RuntimeError("Missing required input[goal_pose]: ", msg.error());
    return BT::NodeStatus::FAILURE;
  }
  ROS_INFO_STREAM("goal_pose from blackboard:");

  for (const auto position : msg.value())
  {
    ROS_INFO_STREAM(position);
  }
  */

  this->setGoal();
  return this->sendGoal();
}
