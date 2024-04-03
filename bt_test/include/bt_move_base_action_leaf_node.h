#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "behaviortree_cpp_v3/action_node.h"

#include "posestamped_bb_parser.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class BaseToGoal : public BT::AsyncActionNode
{
  public:
    BaseToGoal(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

    void setGoal();
    void haltGoal();
    BT::NodeStatus sendGoal();

  private:
    geometry_msgs::PoseStamped pose_;
    MoveBaseClient actionclient_;
    move_base_msgs::MoveBaseGoal goal_;

};
