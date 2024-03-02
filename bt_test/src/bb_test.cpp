#include "iostream"
#include "chrono"
#include "vector"

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseStamped.h"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"

#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using namespace std::chrono_literals;


// Template specialization to converts a string to PoseStamped
namespace BT
{
    template <> inline geometry_msgs::PoseStamped convertFromString(StringView str)
    {
        // We expect a frame_id real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 8)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            geometry_msgs::PoseStamped output;
            output.header.frame_id        = convertFromString<std::string>(parts[0]);
            output.pose.position.x        = convertFromString<double>(parts[1]);
            output.pose.position.y        = convertFromString<double>(parts[2]);
            output.pose.position.z        = convertFromString<double>(parts[3]);
            output.pose.orientation.x     = convertFromString<double>(parts[4]);
            output.pose.orientation.y     = convertFromString<double>(parts[5]);
            output.pose.orientation.z     = convertFromString<double>(parts[6]);
            output.pose.orientation.w     = convertFromString<double>(parts[7]);
            return output;
        }
    }
} // end namespace BT



//   Action leaf node
class GetHomePose: public BT::SyncActionNode
{
public:
    GetHomePose(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name,config)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<geometry_msgs::PoseStamped>("home") };
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp           = ros::Time::now();
        pose.header.frame_id        = "map";
        pose.pose.position.x        = 1.0;
        pose.pose.position.y        = 0.0;
        pose.pose.position.z        = 0.0;
        pose.pose.orientation.x     = 0.0;
        pose.pose.orientation.y     = 0.0;
        pose.pose.orientation.z     = 0.0;
        pose.pose.orientation.w     = 1.0;

      setOutput<geometry_msgs::PoseStamped>("home", pose);

      return BT::NodeStatus::SUCCESS;
    }
};


class PrintTargetPose: public BT::SyncActionNode
{
  public:
    PrintTargetPose(const std::string& name, const BT::NodeConfiguration& config):
        BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<geometry_msgs::PoseStamped>("target") };
    }
      
    BT::NodeStatus tick() override
    {
      auto res = getInput<geometry_msgs::PoseStamped>("target");
      if( !res )
      {
        throw BT::RuntimeError("error reading port [target]:", res.error());
      }

      geometry_msgs::PoseStamped target = res.value();
      ROS_INFO_STREAM("Header: " + target.header.frame_id);
      printf("Target positions: [ %.1f, %.1f , %.1f ]\n", target.pose.position.x, target.pose.position.y, target.pose.position.z);

      return BT::NodeStatus::SUCCESS;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bt_test");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<GetHomePose>("GetHomePose");
    factory.registerNodeType<PrintTargetPose>("PrintTargetPose");
    
    auto tree = factory.createTreeFromFile(
        ros::package::getPath("bt_test") + "/bt_xml/bb_test.xml");
    
    BT::StdCoutLogger logger_cout(tree);
    BT::PublisherZMQ publisher_zmq(tree);

    tree.tickRoot();

    return 0;
}