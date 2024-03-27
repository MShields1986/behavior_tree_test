#include "bt_iiwa_handler_spline_motion_action_leaf_node.h"


IiiwaHandlerSplineMotionAction::IiiwaHandlerSplineMotionAction(const std::string& name, const BT::NodeConfiguration& config)
     : 
      BT::SyncActionNode(name, config),
      spline_motion_client_("/iiwa/action/move_along_spline", true)
{
}


BT::PortsList IiiwaHandlerSplineMotionAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("pose_array_topic_name")
         };
}


BT::NodeStatus IiiwaHandlerSplineMotionAction::tick()
{
  auto res = getInput<std::string>("pose_array_topic_name");

  if (!res)
  {
    throw BT::RuntimeError("IiiwaHandlerSplineMotionAction | error reading port [pose_array_topic_name]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  topic_name_ = res.value();

  iiwa_handler::PoseArrayInputGoal spline_motion_goal_;
  spline_motion_goal_.topic_name = topic_name_;

  ROS_INFO_STREAM("IiiwaHandlerSplineMotionAction | Sending goal");
  spline_motion_client_.sendGoal(spline_motion_goal_);
  ROS_INFO_STREAM("IiiwaHandlerSplineMotionAction | Waiting 30 seconds for result...");

  bool finished_before_timeout = spline_motion_client_.waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout)
  {
      ROS_WARN_STREAM("IiiwaHandlerSplineMotionAction | Goal timed out");
      return BT::NodeStatus::FAILURE;
  }

  spline_motion_client_.getResult();
  if (spline_motion_client_.getResult()->success)
  {
      ROS_INFO_STREAM("IiiwaHandlerSplineMotionAction | Goal achieved");
      return BT::NodeStatus::SUCCESS;
  }
  else
  {
      ROS_WARN_STREAM("IiiwaHandlerSplineMotionAction | Goal failed");
      return BT::NodeStatus::FAILURE;
  }
}
