#pragma once

#include "geometry_msgs/PoseStamped.h"
#include "behaviortree_cpp_v3/action_node.h"

// Template specialization to converts a string to PoseStamped
namespace BT
{
    template <> inline geometry_msgs::PoseStamped convertFromString(StringView str)
    {
        // We expect a frame_id followed by real numbers separated by semicolons
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