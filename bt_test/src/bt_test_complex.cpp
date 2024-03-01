#include "iostream"
#include "chrono"
#include "vector"

#include "ros/ros.h"
#include "ros/package.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace std::chrono_literals;

// Code based on tutorial video here: https://www.youtube.com/watch?v=T_Q57-audMk
// Repo here: https://github.com/thehummingbird/robotics_demos/tree/main/behavior_trees

// Find ball sub-tree
//   Condition leaf node
BT::NodeStatus ballFound()
{
  ROS_INFO_STREAM("Ball not found");
  return BT::NodeStatus::FAILURE;
}

//   Action leaf node
class FindBall : public BT::SyncActionNode
{
public:
  explicit FindBall(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<std::vector<int>>("ball_location")};
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    std::vector<int> ballLocation{1, 2, 3};

    // Write to port
    BT::TreeNode::setOutput("ball_location", ballLocation);
    ROS_INFO_STREAM("Ball Found");
    return BT::NodeStatus::SUCCESS;
  }
};

// Approach ball sub-tree
//   Condition leaf node
BT::NodeStatus ballClose(BT::TreeNode& self)
{
  // Read port
  BT::Optional<std::vector<int>> msg = self.getInput<std::vector<int>>("ball_location");

  if (!msg)
  {
    throw BT::RuntimeError("Missing required input[ball_location]: ", msg.error());
  }

  ROS_INFO_STREAM("Positions from blackboard:");

  /*
  for (const auto position : msg.value())
  {
    ROS_INFO_STREAM(position);
  }
  */
  ROS_INFO_STREAM("----------------------------");
  ROS_INFO_STREAM(msg.value()[0]);
  ROS_INFO_STREAM(msg.value()[1]);
  ROS_INFO_STREAM(msg.value()[2]);
  ROS_INFO_STREAM("----------------------------");

  ROS_INFO_STREAM("Ball not close");
  return BT::NodeStatus::FAILURE;
}

//   Action leaf node
class ApproachBall : public BT::SyncActionNode
{
public:
  explicit ApproachBall(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::vector<int>>("ball_location")};
  }

  BT::NodeStatus tick() override
  {
    // Read port
    BT::Optional<std::vector<int>> msg = getInput<std::vector<int>>("ball_location");

    if (!msg)
    {
      throw BT::RuntimeError("Missing required input[ball_location]: ", msg.error());
    }

    ROS_INFO_STREAM("Positions from blackboard:");

    for (const auto position : msg.value())
    {
      ROS_INFO_STREAM(position);
    }

    std::this_thread::sleep_for(3s);
    ROS_INFO_STREAM("Ball Approached");
    return BT::NodeStatus::SUCCESS;
  }
};

// Grasp ball sub-tree
//   Condition leaf node
BT::NodeStatus ballGrasped()
{
  ROS_INFO_STREAM("Ball not grasped");
  return BT::NodeStatus::FAILURE;
}

//   Action leaf node
class GraspBall : public BT::SyncActionNode
{
public:
  explicit GraspBall(const std::string& name)
      : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    ROS_INFO_STREAM("Ball grasped");
    return BT::NodeStatus::SUCCESS;
  }
};


// Approach bin sub-tree
//   Condition leaf node
BT::NodeStatus binClose()
{
  ROS_INFO_STREAM("Bin not close");
  return BT::NodeStatus::FAILURE;
}

//   Action leaf node
class ApproachBin : public BT::SyncActionNode
{
public:
  explicit ApproachBin(const std::string& name)
      : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    ROS_INFO_STREAM("Bin approached");
    return BT::NodeStatus::SUCCESS;
  }
};

// Place ball sub-tree
//   Condition leaf node
BT::NodeStatus ballPlaced()
{
  ROS_INFO_STREAM("Ball not placed");
  return BT::NodeStatus::FAILURE;
}

//   Action leaf node
class PlaceBall : public BT::SyncActionNode
{
public:
  explicit PlaceBall(const std::string& name)
      : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    std::this_thread::sleep_for(3s);
    ROS_INFO_STREAM("Ball placed");
    return BT::NodeStatus::SUCCESS;
  }
};

// Human fallback sub-tree
//   Action leaf node
class AskForHelp : public BT::SyncActionNode
{
public:
  explicit AskForHelp(const std::string& name)
      : BT::SyncActionNode(name, {})
  {
  }

  BT::NodeStatus tick() override
  {
    ROS_INFO_STREAM("Asking for help. Waiting for 10 seconds here");
    std::this_thread::sleep_for(10s);
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bt_test_complex");
  ros::NodeHandle nh;

  std::string package_path(ros::package::getPath("bt_test"));
  std::string file("bt_test_complex.xml");
  std::string file_path(package_path + "/bt_xml/" + file);
  ROS_INFO_STREAM("BT XML Path: " << file_path);

  BT::BehaviorTreeFactory factory;

  factory.registerSimpleCondition("BallFound", std::bind(ballFound));
  factory.registerNodeType<FindBall>("FindBall");

  BT::PortsList ports = {BT::InputPort<std::vector<int>>("ball_location")};
  factory.registerSimpleCondition("BallClose", ballClose, ports);
  factory.registerNodeType<ApproachBall>("ApproachBall");

  factory.registerSimpleCondition("BallGrasped", std::bind(ballGrasped));
  factory.registerNodeType<GraspBall>("GraspBall");

  factory.registerSimpleCondition("BinClose", std::bind(binClose));
  factory.registerNodeType<ApproachBin>("ApproachBin");

  factory.registerSimpleCondition("BallPlaced", std::bind(ballPlaced));
  factory.registerNodeType<PlaceBall>("PlaceBall");

  factory.registerNodeType<AskForHelp>("AskForHelp");

  // Create Tree
  auto tree = factory.createTreeFromFile(file_path);

  // Create a logger
  BT::StdCoutLogger logger_cout(tree);

  // Create a link to Groot
  // Groot: https://github.com/BehaviorTree/Groot
  BT::PublisherZMQ publisher_zmq(tree);

  // execute the tree
  tree.tickRoot();

  ROS_INFO_STREAM("BT Done!");
  return 0;
}