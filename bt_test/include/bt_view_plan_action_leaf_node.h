#pragma once

#include "ros/ros.h"
//#include "geometry_msgs/PoseArray.h"

#include "view_planner/PlanViews.h"

#include "behaviortree_cpp_v3/action_node.h"


class ViewPlan : public BT::SyncActionNode
{
  public:
    ViewPlan(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

  private:
    std::string m_working_volume_origin;
    float m_camera_standoff;
    float m_points_x;
    float m_points_y;

    //geometry_msgs::PoseArray m_view_poses;

};
