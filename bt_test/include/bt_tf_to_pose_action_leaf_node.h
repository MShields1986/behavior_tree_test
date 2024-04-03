#pragma once

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "behaviortree_cpp_v3/action_node.h"

#include "posestamped_bb_parser.h"


class TfToPose : public BT::SyncActionNode
{
  public:
    TfToPose(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string frame_id_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    geometry_msgs::TransformStamped transformStamped_;
};
