#include "chrono"

#include "ros/ros.h"
#include "ros/package.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "bt_move_base_action_leaf_node.h"
#include "bt_tf_to_pose_action_leaf_node.h"


using namespace std::chrono_literals;



const double max_pos_error = 0.01;
const double max_rot_error = 0.044; // ~5 degs

bool pose_ok(double a, double b)
{
   return std::fabs(a - b) < max_pos_error;
}


bool rot_ok(double a, double b)
{
   return std::fabs(a - b) < max_rot_error;
}



// Adapting a pre-existing function
BT::NodeStatus CheckPose(BT::TreeNode& self)
{
    // Read goal pose from port
    BT::Optional<std::vector<double>> msg = self.getInput<std::vector<double>>("fixture_1_base_location");

    if (!msg)
    {
        throw BT::RuntimeError("Missing required input[fixture_1_base_location]: ", msg.error());
    }

    ROS_INFO_STREAM("Checking Pose x: " << msg.value()[0] << " y: " << msg.value()[1] << " z: " << msg.value()[2]);

    // TODO: Be good to get the instantiation out of here
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try{
        transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
        //transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(3.0));
        ROS_INFO_STREAM("Current pose...");
        ROS_INFO_STREAM(transformStamped);

        if (pose_ok(transformStamped.transform.translation.x, msg.value()[0]) &&
            pose_ok(transformStamped.transform.translation.y, msg.value()[1]) &&
            pose_ok(transformStamped.transform.translation.z, msg.value()[2]) &&
            rot_ok(transformStamped.transform.rotation.z, 0.0) &&
            rot_ok(transformStamped.transform.rotation.w, 1.0))
        {
            ROS_INFO_STREAM("Pose OK");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO_STREAM("Pose NOK");
            return BT::NodeStatus::FAILURE;
        }
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT transform map to base_link: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

}



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
        ROS_INFO_STREAM("Asking for help. Waiting for 10 seconds");
        ros::Duration(10.0).sleep();
        return BT::NodeStatus::SUCCESS;
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bt_test");
  ros::NodeHandle nh;

  std::string package_path(ros::package::getPath("bt_test"));
  std::string file("tree.xml");
  std::string file_path(package_path + "/bt_xml/" + file);
  ROS_INFO_STREAM("BT XML Path: " << file_path);

  BT::BehaviorTreeFactory factory;

  BT::PortsList ports = {BT::InputPort<std::vector<double>>("fixture_1_base_location")};

  // Register leaf nodes
  factory.registerNodeType<AskForHelp>("AskForHelp");
  //factory.registerNodeType<CheckPose>("CheckPose");
  factory.registerSimpleCondition("CheckPose", CheckPose, ports);
  factory.registerNodeType<BaseToGoal>("BaseToGoal");
  factory.registerNodeType<TfToPose>("TfToPose");

  // Create Tree
  auto tree = factory.createTreeFromFile(file_path);

  // Create a logger
  BT::StdCoutLogger logger_cout(tree);

  // Create a link to Groot
  // Groot: https://github.com/BehaviorTree/Groot
  BT::PublisherZMQ publisher_zmq(tree);

  // execute the tree
  //tree.tickRoot();

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ROS_INFO_STREAM("Behavior Tree Finished!");
  return 0;
}
