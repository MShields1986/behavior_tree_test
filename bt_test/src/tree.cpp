#include "chrono"
#include "string"

#include "ros/ros.h"
#include "ros/package.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "bt_iiwa_handler_spline_motion_action_leaf_node.h"
#include "bt_iiwa_cartesian_cmd_action_leaf_node.h"
#include "bt_iiwa_cartesian_lin_cmd_action_leaf_node.h"
#include "bt_iiwa_joint_cmd_action_leaf_node.h"
#include "bt_move_base_action_leaf_node.h"
#include "bt_moveit_commander_plan_action_leaf_node.h"
#include "bt_moveit_commander_exec_action_leaf_node.h"
#include "bt_sleep_action_leaf_node.h"
#include "bt_tf_to_pose_action_leaf_node.h"
#include "bt_update_all_fixture_locations_action_leaf_node.h"
#include "bt_view_plan_action_leaf_node.h"

#include "iiwajointposition_bb_parser.h"
#include "posestamped_bb_parser.h"


using namespace std::chrono_literals;


// TODO: Get these in the param server
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



// CheckPose Condition Leaf Node
BT::NodeStatus CheckPose(BT::TreeNode& self)
{
    auto pose_res = self.getInput<geometry_msgs::PoseStamped>("pose");
    if( !pose_res )
    {
        throw BT::RuntimeError("CheckPose | error reading port [pose]:", pose_res.error());
        return BT::NodeStatus::FAILURE;
    }

    auto frame_res = self.getInput<std::string>("child_frame");
    if( !frame_res )
    {
        throw BT::RuntimeError("CheckPose | error reading port [child_frame]:", frame_res.error());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::PoseStamped pose = pose_res.value();
    std::string child_frame = frame_res.value();
    ROS_INFO_STREAM("CheckPose | child_frame: " << child_frame << "pose input: " << pose);

    // TODO: Be good to get the instantiation out of here
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    try{
        transformStamped = tfBuffer.lookupTransform(pose.header.frame_id, child_frame, ros::Time(0), ros::Duration(3.0));
        //transformStamped = tfBuffer.lookupTransform(pose.header.frame_id, child_frame, ros::Time::now(), ros::Duration(3.0));

        if (pose_ok(transformStamped.transform.translation.x, pose.pose.position.x) &&
            pose_ok(transformStamped.transform.translation.y, pose.pose.position.y) &&
            pose_ok(transformStamped.transform.translation.z, pose.pose.position.z) &&
            rot_ok(transformStamped.transform.rotation.x, pose.pose.orientation.x) &&
            rot_ok(transformStamped.transform.rotation.y, pose.pose.orientation.y) &&
            rot_ok(transformStamped.transform.rotation.z, pose.pose.orientation.z) &&
            rot_ok(transformStamped.transform.rotation.w, pose.pose.orientation.w))
        {
            ROS_INFO_STREAM("CheckPose | pose OK");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO_STREAM("CheckPose | pose NOK");
            return BT::NodeStatus::FAILURE;
        }
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("CheckPose | could not look up required transform : %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

}


// Human Fallback Action Leaf Node
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

  std::string file;
  while (!nh.hasParam("/bt_file"))
  {
    ROS_INFO_STREAM_THROTTLE(10, "Waiting for parameter /bt_file");
  }
  nh.getParam("/bt_file", file);
  ROS_INFO_STREAM("Read in parameter /bt_file = " << file);

  std::string package_path(ros::package::getPath("bt_test"));
  std::string file_path(package_path + "/bt_xml/" + file);
  ROS_INFO_STREAM("BT XML Path: " << file_path);

  BT::BehaviorTreeFactory factory;

  BT::PortsList ports = {
    BT::InputPort<geometry_msgs::PoseStamped>("pose"),
    BT::InputPort<geometry_msgs::PoseStamped>("child_frame")
    };

  // Register leaf nodes
  factory.registerNodeType<AskForHelp>("AskForHelp");
  factory.registerSimpleCondition("CheckPose", CheckPose, ports);
  factory.registerNodeType<Sleep>("Sleep");
  
  factory.registerNodeType<BaseToGoal>("BaseToGoal");
  factory.registerNodeType<TfToPose>("TfToPose");

  factory.registerNodeType<UpdateAllFixtureLocations>("UpdateAllFixtureLocations");
  factory.registerNodeType<ViewPlan>("ViewPlan");
  
  factory.registerNodeType<MoveItCommanderPlanService>("MoveItCommanderPlanService");
  factory.registerNodeType<MoveItCommanderExecuteService>("MoveItCommanderExecuteService");

  factory.registerNodeType<IiiwaHandlerSplineMotionAction>("IiiwaHandlerSplineMotionAction");

  factory.registerNodeType<IiwaToCartesianPosition>("IiwaToCartesianPosition");
  factory.registerNodeType<IiwaToCartesianLinPosition>("IiwaToCartesianLinPosition");
  factory.registerNodeType<IiwaToJointPosition>("IiwaToJointPosition");

  // Create Tree
  //auto tree = factory.createTreeFromFile(file_path);

  factory.registerBehaviorTreeFromFile(file_path);
  auto tree = factory.createTree("MainTree");

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
