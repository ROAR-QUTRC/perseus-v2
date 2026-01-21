#pragma once

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace perseus_bt_nodes
{

    class GetGoalFromGoals : public BT::SyncActionNode
    {
    public:
        GetGoalFromGoals(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
                BT::InputPort<int>("index"),
                BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
        }

        BT::NodeStatus tick() override
        {
            std::vector<geometry_msgs::msg::PoseStamped> goals;
            int index = 0;

            if (!getInput("goals", goals))
            {
                throw BT::RuntimeError("GetGoalFromGoals: missing input [goals]");
            }
            if (!getInput("index", index))
            {
                throw BT::RuntimeError("GetGoalFromGoals: missing input [index]");
            }

            if (index < 0 || static_cast<size_t>(index) >= goals.size())
            {
                // Important: returning FAILURE is useful to stop a RepeatUntilFailure loop.
                return BT::NodeStatus::FAILURE;
            }

            setOutput("goal", goals[static_cast<size_t>(index)]);
            return BT::NodeStatus::SUCCESS;
        }
    };

}  // namespace perseus_bt_nodes
