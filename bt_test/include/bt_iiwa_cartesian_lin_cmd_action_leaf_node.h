#pragma once

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"

#include "geometry_msgs/PoseStamped.h"
#include "iiwa_msgs/MoveToCartesianPoseAction.h"

#include "behaviortree_cpp_v3/action_node.h"

#include "posestamped_bb_parser.h"



class IiwaToCartesianLinPosition : public BT::SyncActionNode
{
  public:
    IiwaToCartesianLinPosition(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string action_name_;
    iiwa_msgs::MoveToCartesianPoseGoal cartesian_pose_goal_;
    actionlib::SimpleActionClient<iiwa_msgs::MoveToCartesianPoseAction> cartesian_pose_client_;
};
