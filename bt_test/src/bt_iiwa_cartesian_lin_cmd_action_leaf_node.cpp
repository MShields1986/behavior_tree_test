#include "bt_iiwa_cartesian_lin_cmd_action_leaf_node.h"



IiwaToCartesianLinPosition::IiwaToCartesianLinPosition(const std::string& name, const BT::NodeConfiguration& config)
   : BT::SyncActionNode(name, config),
     action_name_("/iiwa/action/move_to_cartesian_pose_lin"),
     cartesian_pose_client_(action_name_, true)
{
    ROS_INFO_STREAM("IiwaToCartesianLinPosition | Waiting for external action server " << action_name_ << " to start...");
    cartesian_pose_client_.waitForServer();

    ROS_INFO_STREAM("IiwaToCartesianLinPosition | External action server " << action_name_ << " started");
}


BT::PortsList IiwaToCartesianLinPosition::providedPorts()
{
    return {BT::InputPort<geometry_msgs::PoseStamped>("pose")};
}


BT::NodeStatus IiwaToCartesianLinPosition::tick()
{
    auto res = getInput<geometry_msgs::PoseStamped>("pose");
 
    if( !res )
    {
      throw BT::RuntimeError("IiwaToCartesianLinPosition | error reading port [pose]:", res.error());
      return BT::NodeStatus::FAILURE;
    }

    iiwa_msgs::MoveToCartesianPoseGoal cartesian_pose_goal_;
    cartesian_pose_goal_.cartesian_pose.poseStamped = res.value();

    ROS_INFO_STREAM("IiwaToCartesianLinPosition | Sending goal");
    cartesian_pose_client_.sendGoal(cartesian_pose_goal_);
    ROS_INFO_STREAM("IiwaToCartesianLinPosition | Waiting 30 seconds for result...");

    bool finished_before_timeout = cartesian_pose_client_.waitForResult(ros::Duration(30.0));
    if (!finished_before_timeout)
    {
        ROS_WARN_STREAM("IiwaToCartesianLinPosition | Goal timed out");
        return BT::NodeStatus::FAILURE;
    }

    cartesian_pose_client_.getResult();
    if (cartesian_pose_client_.getResult()->success)
    {
        ROS_INFO_STREAM("IiwaToCartesianLinPosition | Goal achieved");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        ROS_WARN_STREAM("IiwaToCartesianLinPosition | Goal failed");
        return BT::NodeStatus::FAILURE;
    }
}
