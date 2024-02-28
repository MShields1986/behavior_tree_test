#include "bt_check_pose_condition_leaf_node.h"



//CheckPose::CheckPose(const std::string& name, const BT::NodeConfiguration& config) : 
CheckPose::CheckPose(const std::string &name, const BT::NodeConfiguration &config) : 
      BT::AsyncActionNode(name, config),
      _actionclient("move_base", true)
{
  while(!_actionclient.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO_STREAM("Waiting for the move_base action server to come up");
  }
  ROS_INFO_STREAM("move_base action server up");
}

