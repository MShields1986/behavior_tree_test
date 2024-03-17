#include "bt_view_plan_action_leaf_node.h"


ViewPlan::ViewPlan(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList ViewPlan::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name"),
    BT::InputPort<std::string>("working_volume_origin"),
    BT::InputPort<float>("camera_standoff"),
    BT::InputPort<float>("points_x"),
    BT::InputPort<float>("points_y")
         };
}


void ViewPlan::processStringPort(std::string& port_value, const std::string& port_name)
{
  auto res = getInput<std::string>(port_name);

  if (!res)
  {
    throw BT::RuntimeError("ViewPlan | error reading port [", port_name, "]: ", res.error());
    // TODO: Handle this being returned properly
    //return BT::NodeStatus::FAILURE;
  }

  port_value = res.value();
}


void ViewPlan::processFloatPort(float& port_value, const std::string& port_name)
{
  auto res = getInput<float>(port_name);

  if (!res)
  {
    throw BT::RuntimeError("ViewPlan | error reading port [", port_name, "]: ", res.error());
    //return BT::NodeStatus::FAILURE;
  }

  port_value = res.value();
}


BT::NodeStatus ViewPlan::tick()
{
  processStringPort(service_name_, "service_name");
  processStringPort(working_volume_origin_, "working_volume_origin");
  processFloatPort(camera_standoff_, "camera_standoff");
  processFloatPort(points_x_, "points_x");
  processFloatPort(points_y_, "points_y");

  ROS_INFO_STREAM("ViewPlan | service_name: " << service_name_);
  ROS_INFO_STREAM("ViewPlan | working_volume_origin: " << working_volume_origin_);
  ROS_INFO_STREAM("ViewPlan | camera_standoff: " << camera_standoff_);
  ROS_INFO_STREAM("ViewPlan | points_x: " << points_x_);
  ROS_INFO_STREAM("ViewPlan | points_y: " << points_y_);

  ros::NodeHandle nh_;
  ros::ServiceClient client_ = nh_.serviceClient<view_planner::PlanViews>(service_name_);
  srv_.request.working_volume_origin = working_volume_origin_;
  srv_.request.camera_standoff = camera_standoff_;
  srv_.request.points_x = points_x_;
  srv_.request.points_y = points_y_;

  if (client_.call(srv_))
  {
    //setOutput<geometry_msgs::PoseArray>("poses", poses);
    //ROS_INFO_STREAM("TfToPose | poses output: " << poses);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return BT::NodeStatus::FAILURE;
  }
}
