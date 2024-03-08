#pragma once

#include "iiwa_msgs/JointPosition.h"
#include "behaviortree_cpp_v3/action_node.h"

// Template specialization to converts a string to JointPosition
namespace BT
{
    template <> inline iiwa_msgs::JointPosition convertFromString(StringView str)
    {
        // We expect a real numbers (degrees) separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            iiwa_msgs::JointPosition output;
            output.position.a1        = convertFromString<double>(parts[0]) * 3.1415926535 / 180;
            output.position.a2        = convertFromString<double>(parts[1]) * 3.1415926535 / 180;
            output.position.a3        = convertFromString<double>(parts[2]) * 3.1415926535 / 180;
            output.position.a4        = convertFromString<double>(parts[3]) * 3.1415926535 / 180;
            output.position.a5        = convertFromString<double>(parts[4]) * 3.1415926535 / 180;
            output.position.a6        = convertFromString<double>(parts[5]) * 3.1415926535 / 180;
            output.position.a7        = convertFromString<double>(parts[6]) * 3.1415926535 / 180;
            return output;
        }
    }
} // end namespace BT