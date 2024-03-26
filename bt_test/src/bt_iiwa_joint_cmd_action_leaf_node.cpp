#include "bt_iiwa_joint_cmd_action_leaf_node.h"



IiwaToJointPosition::IiwaToJointPosition(const std::string& name, const BT::NodeConfiguration& config)
   : BT::SyncActionNode(name, config),
     action_name_("/iiwa/action/move_to_joint_position"),
     joint_position_client_(action_name_, true)
{
    ROS_INFO_STREAM("IiwaToJointPosition | Waiting for external action server " << action_name_ << " to start...");
    joint_position_client_.waitForServer();

    ROS_INFO_STREAM("IiwaToJointPosition | External action server " << action_name_ << " started");
}


BT::PortsList IiwaToJointPosition::providedPorts()
{
    return {BT::InputPort<iiwa_msgs::JointPosition>("joints")};
}


BT::NodeStatus IiwaToJointPosition::tick()
{
    auto res = getInput<iiwa_msgs::JointPosition>("joints");
 
    if( !res )
    {
      throw BT::RuntimeError("IiwaToJointPosition | error reading port [joints]:", res.error());
      return BT::NodeStatus::FAILURE;
    }

    iiwa_msgs::MoveToJointPositionGoal joint_position_goal_;
    joint_position_goal_.joint_position = res.value();

    ROS_INFO_STREAM("IiwaToJointPosition | Sending goal");
    joint_position_client_.sendGoal(joint_position_goal_);
    ROS_INFO_STREAM("IiwaToJointPosition | Waiting 30 seconds for result...");

    bool finished_before_timeout = joint_position_client_.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout)
    {
        ROS_WARN_STREAM("IiwaToJointPosition | Goal timed out");
        return BT::NodeStatus::FAILURE;
    }

    joint_position_client_.getResult();
    if (joint_position_client_.getResult()->success)
    {
        ROS_INFO_STREAM("IiwaToJointPosition | Goal achieved");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_WARN_STREAM("IiwaToJointPosition | Goal failed");
        return BT::NodeStatus::FAILURE;
    }
}
