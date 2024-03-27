#include "bt_update_all_fixture_locations_action_leaf_node.h"


UpdateAllFixtureLocations::UpdateAllFixtureLocations(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config)
{
}


BT::PortsList UpdateAllFixtureLocations::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name")
         };
}


BT::NodeStatus UpdateAllFixtureLocations::tick()
{
  auto res = getInput<std::string>("service_name");

  if (!res)
  {
    throw BT::RuntimeError("UpdateAllFixtureLocations | error reading port [service_name]: ", res.error());
    return BT::NodeStatus::FAILURE;
  }

  service_name_ = res.value();

  ROS_INFO_STREAM("UpdateAllFixtureLocations | service_name: " << service_name_);

  ros::NodeHandle nh_;
  ros::ServiceClient client_ = nh_.serviceClient<fixture_tracker::UpdateFixturePose>(service_name_);
  srv_.request.update = true;

  if (client_.call(srv_))
  {
    ROS_INFO_STREAM("UpdateAllFixtureLocations | Called service " << service_name_);
    if (srv_.response.success)
    {
      ROS_INFO_STREAM("UpdateAllFixtureLocations | Successfully updated all fixture locations");
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      ROS_ERROR_STREAM("UpdateAllFixtureLocations | Failed to update all fixture locations");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR_STREAM("UpdateAllFixtureLocations | Failed to call service " << service_name_);
    return BT::NodeStatus::FAILURE;
  }
}
