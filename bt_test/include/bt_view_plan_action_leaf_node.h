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
    void processStringPort(std::string& port_value, const std::string& port_name);
    void processFloatPort(float& port_value, const std::string& port_name);

    view_planner::PlanViews srv_;
    std::string service_name_;
    std::string working_volume_origin_;
    float camera_standoff_;
    float points_x_;
    float points_y_;

    //geometry_msgs::PoseArray m_view_poses;

};
