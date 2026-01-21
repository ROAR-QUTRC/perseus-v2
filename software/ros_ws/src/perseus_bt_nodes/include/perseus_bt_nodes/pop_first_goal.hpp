#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace perseus_bt_nodes
{

    class PopFirstGoal : public BT::SyncActionNode
    {
    public:
        PopFirstGoal(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("input_goals"),
                BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("output_goals")};
        }

        BT::NodeStatus tick() override
        {
            std::vector<geometry_msgs::msg::PoseStamped> goals;

            if (!getInput("input_goals", goals))
            {
                throw BT::RuntimeError("PopFirstGoal: missing input [input_goals]");
            }

            // If there are goals, remove the first one
            if (!goals.empty())
            {
                goals.erase(goals.begin());
            }

            // Set the output with remaining goals
            setOutput("output_goals", goals);
            return BT::NodeStatus::SUCCESS;
        }
    };

}  // namespace perseus_bt_nodes
