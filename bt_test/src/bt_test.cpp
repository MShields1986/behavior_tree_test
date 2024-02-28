#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

using namespace std::chrono_literals;
using namespace BT;


// Code based on tutorial video here: https://www.youtube.com/watch?v=4PUiDmD5dkg
// Repo here: https://github.com/thehummingbird/robotics_demos/tree/main/behavior_trees

// Creating leaf nodes in three different ways

// Inheritance
class ApproachObject : public SyncActionNode
{
public:
  explicit ApproachObject(const std::string& name) : SyncActionNode(name, {})
  {
  }

  // Synchronous action nodes only return success or failure as they are blocking
  NodeStatus tick() override
  {
    ROS_INFO_STREAM("Approach Object: " << this->name());

    std::this_thread::sleep_for(5s);
    return NodeStatus::SUCCESS;
  }
};

// Adapting a pre-existing function
NodeStatus CheckBattery()
{
  ROS_INFO_STREAM("Checking Battery...");

  std::this_thread::sleep_for(2s);
  ROS_INFO_STREAM("Battery OK");
  return NodeStatus::SUCCESS;
}


// Adapting a pre-existing class method
class GripperInterface
{
public:
  // Constructor

  GripperInterface() : _open(true)
  {
  }

  NodeStatus open() {
    ROS_INFO_STREAM("Opening Gripper");
    std::this_thread::sleep_for(1s);
    _open = true;
    ROS_INFO_STREAM("Gripper Open");
    return NodeStatus::SUCCESS;
  }

  NodeStatus close() {
    ROS_INFO_STREAM("Closing Gripper");
    std::this_thread::sleep_for(1s);
    _open = false;
    ROS_INFO_STREAM("Gripper Closed");
    return NodeStatus::SUCCESS;
  }

private:
  bool _open;

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bt_test");
  ros::NodeHandle nh;

  std::string package_path(ros::package::getPath("bt_test"));
  std::string file("bt_test.xml");
  std::string file_path(package_path + "/bt_xml/" + file);
  ROS_INFO_STREAM("BT XML Path: " << file_path);

  BehaviorTreeFactory factory;

  // Register leaf nodes
  factory.registerNodeType<ApproachObject>("ApproachObject");

  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));

  // Create tree
  auto tree = factory.createTreeFromFile(file_path);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  // Execute tree
  tree.tickRoot();

  ROS_INFO_STREAM("BT Done!");
  return 0;
}
